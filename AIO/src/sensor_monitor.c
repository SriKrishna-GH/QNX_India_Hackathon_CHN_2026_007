#include "common.h"
#include <fcntl.h>
#include <devctl.h>
#include <hw/i2c.h>

#define I2C_DEV   "/dev/i2c1"   // check this name on the pi
#define IMU_ADDR  0x68          // mpu6050, AD0 low
#define GPIO_BASE 0xFE200000
#define GPIO_IRQ  145      // gpio bank 0 vector, not sure, check on the pi
#define TRIG 23
#define ECHO 24

#define PULSE_TICK  (_PULSE_CODE_MINAVAIL + 20)    // timer: next ping
#define PULSE_ECHO  (_PULSE_CODE_MINAVAIL + 21)    // irq: echo pin changed
#define PULSE_IMU   (_PULSE_CODE_MINAVAIL + 22)    // timer: read imu

#define SHAKE_G  0.3       // jump bigger than this between samples = shaking
#define REST_MS  1000      // no shaking for this long = rest

volatile uint32_t *gpio;
uint64_t cycles_per_ms;
int imu_fail_after = 0;    // "imufail" arg, imu counts as failed after 20 reports
int i2c_fd = -1;
int sup_coid, sup_silent = 0;
uint64_t last_sup = 0;     // last alive pulse from the supervisor
double acc[3];             // last accel in g

int imu_write(uint8_t reg, uint8_t value)    // write 1 byte to a register, 0 = ok
{
    struct { i2c_send_t hdr; uint8_t buf[2]; } m;    // header then data

    m.hdr.slave.addr = IMU_ADDR;
    m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.len = 2;
    m.hdr.stop = 1;
    m.buf[0] = reg;
    m.buf[1] = value;
    return devctl(i2c_fd, DCMD_I2C_SEND, &m, sizeof(m.hdr) + 2, NULL);    // i2c via the resource manager
}

int imu_read(uint8_t reg, uint8_t *data, int len)    // read up to 6 bytes from reg, 0 = ok
{
    struct { i2c_sendrecv_t hdr; uint8_t buf[6]; } m;

    m.hdr.slave.addr = IMU_ADDR;
    m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1;
    m.hdr.recv_len = len;
    m.hdr.stop = 1;
    m.buf[0] = reg;                        // received bytes overwrite this
    if (devctl(i2c_fd, DCMD_I2C_SENDRECV, &m, sizeof(m.hdr) + len, NULL) != EOK)
        return -1;
    memcpy(data, m.buf, len);
    return 0;
}

void imu_init(void)    // wake the imu, it starts asleep
{
    uint8_t who = 0;

    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd == -1 || imu_write(0x6B, 0) != EOK || imu_read(0x75, &who, 1) != 0)
        printf("[SENSOR] IMU not found (%s, address 0x%02X): imu_ok will stay 0\n", I2C_DEV, IMU_ADDR);
    else
        printf("[SENSOR] MPU-6050 found, WHO_AM_I = 0x%02X\n", who);
}

int send_to_supervisor(message_t *msg)    // if sup restarted, reconnect and try once more
{
    int r = MsgSend(sup_coid, msg, sizeof(*msg), NULL, 0);

    if (r == -1) {
        name_close(sup_coid);
        sup_coid = find_supervisor();
        r = MsgSend(sup_coid, msg, sizeof(*msg), NULL, 0);
    }
    return r;
}

void report(message_t *msg, int ok, double distance_cm)    // print and send to sup
{
    if (msg->imu_ok)
        printf("[IMU] ax=%.2f ay=%.2f az=%.2f g\n", acc[0], acc[1], acc[2]);

    if (ok)
        printf("Distance: %.2f cm\n", distance_cm);
    else
        printf("[SENSOR] no valid reading\n");

    msg->seq++;
    msg->ultrasonic_ok = ok;
    msg->distance_cm = distance_cm;
    if (imu_fail_after && msg->seq > imu_fail_after)
        msg->imu_ok = 0;                   // fake failure for the demo

    if (send_to_supervisor(msg) == -1)
        printf("[SENSOR] supervisor communication failure\n");
}

int main(int argc, char *argv[])
{
    message_t msg = {0};
    struct _pulse pulse;
    struct sigevent echo_event;
    name_attach_t *attach;
    int chid, coid, irq_id, level;
    int waiting = 0, got_rise = 0, echo_count = 0, silent_pings = 0, prev_ok, shaking, i;
    uint64_t t_rise = 0, t_now, last_shake = 0;
    double time_ms, distance_cm, g, d;
    uint8_t raw[6];

    set_priority(15, "sensor_monitor");
    ThreadCtl(_NTO_TCTL_IO, 0);

    uintptr_t gpio_addr = mmap_device_io(0xA0, GPIO_BASE);
    if (gpio_addr == MAP_DEVICE_FAILED) {
        printf("could not map gpio\n");
        return 1;
    }
    gpio = (volatile uint32_t *) gpio_addr;

    gpio[2] = (gpio[2] & ~(7 << 9)) | (1 << 9);   // trig output, echo stays input
    gpio[19] |= 1 << ECHO;                        // event on rising edge
    gpio[22] |= 1 << ECHO;                        // and falling edge
    gpio[16] = 1 << ECHO;                         // clear old events
    cycles_per_ms = SYSPAGE_ENTRY(qtime)->cycles_per_sec / 1000;

    attach = name_attach(NULL, "sensor_monitor", 0);    // named so sup can send us its alive pulse
    if (attach == NULL) {
        printf("[SENSOR] name_attach failed\n");
        return 1;
    }
    chid = attach->chid;
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&echo_event, coid, 15, PULSE_ECHO, 0);    // irq comes as a pulse on our channel
    irq_id = InterruptAttachEvent(GPIO_IRQ, &echo_event, 0);
    if (irq_id == -1) {
        printf("[SENSOR] InterruptAttachEvent(%d) failed: %s\n", GPIO_IRQ, strerror(errno));
        return 1;
    }

    sup_coid = find_supervisor();
    if (sup_coid == -1) {
        printf("[SENSOR] no safety_supervisor found\n");
        return 1;
    }

    msg.type = MSG_SENSOR_HEALTH;
    imu_init();
    if (argc > 1 && strcmp(argv[1], "imufail") == 0)
        imu_fail_after = 20;

    set_timer(make_timer(coid, 15, PULSE_TICK), 200000000ULL, 200000000ULL);    // ping every 200ms, sleeps in MsgReceive between
    set_timer(make_timer(coid, 15, PULSE_IMU), 20000000ULL, 20000000ULL);       // imu every 20ms to catch vibration

    while (1) {
        MsgReceive(chid, &pulse, sizeof(pulse), NULL);

        if (pulse.code == PULSE_IMU) {
            prev_ok = msg.imu_ok;                     // first sample has nothing to compare to
            msg.imu_ok = (imu_read(0x3B, raw, 6) == 0);    // accel x y z
            for (i = 0; msg.imu_ok && i < 3; i++) {
                g = (int16_t)((raw[2 * i] << 8) | raw[2 * i + 1]) / 16384.0;
                d = g - acc[i];
                if (prev_ok && (d > SHAKE_G || d < -SHAKE_G))
                    last_shake = ClockCycles();       // shaking
                acc[i] = g;
            }
            shaking = last_shake && (ClockCycles() - last_shake) / cycles_per_ms < REST_MS;
            if (shaking && !msg.shaking) {            // just started, tell sup right now
                msg.shaking = 1;
                printf("[SENSOR] VIBRATION detected\n");
                send_to_supervisor(&msg);
            }
            msg.shaking = shaking;
        } else if (pulse.code == PULSE_SUP_ALIVE) {
            last_sup = ClockCycles();
        } else if (pulse.code == PULSE_TICK) {
            if (last_sup && (ClockCycles() - last_sup) / cycles_per_ms > 500 && !sup_silent) {
                printf("[SENSOR] supervisor silent for 500 ms\n");
                sup_silent = 1;
            } else if (last_sup && (ClockCycles() - last_sup) / cycles_per_ms <= 500)
                sup_silent = 0;
            if (waiting) {                          // last ping got no echo
                report(&msg, 0, -1);
                if (echo_count == 0 && ++silent_pings == 10)
                    printf("[SENSOR] no echo interrupt at all yet, is GPIO_IRQ %d right for this board?\n", GPIO_IRQ);
            }
            got_rise = 0;
            waiting = 1;
            gpio[7] = 1 << TRIG;                    // 10us trigger
            nanospin_ns(10000);
            gpio[10] = 1 << TRIG;
        } else if (pulse.code == PULSE_ECHO) {
            t_now = ClockCycles();                  // time first, width = fall - rise
            gpio[16] = 1 << ECHO;                   // clear or it fires again
            level = (gpio[13] >> ECHO) & 1;         // high = rising, low = falling
            InterruptUnmask(0, irq_id);             // kernel masked it, unmask
            echo_count++;

            if (level) {                            // rising
                t_rise = t_now;
                got_rise = 1;
            } else if (waiting && got_rise) {       // falling
                waiting = 0;
                time_ms = (double)(t_now - t_rise) / (double)cycles_per_ms;
                distance_cm = (time_ms * 1000 * 0.0343) / 2;
                report(&msg, distance_cm > 0 && distance_cm < 400, distance_cm);
            }
        }
    }

    return 0;
}
