#include "common.h"

#define MAX_SPEED           2.0     // limits are only here, nav just asks
#define MAX_STEERING        25.0
#define MIN_SAFE_DISTANCE   10.0

#define GPIO_BASE 0xFE200000
#define LED_PIN   17
#define GREEN_PIN 27       // green = normal, red = recovery

#define PULSE_STALE_CHECK     (_PULSE_CODE_MINAVAIL + 10)
#define PULSE_RECOVERY_TIMER  (_PULSE_CODE_MINAVAIL + 11)
#define PULSE_WATCHDOG        (_PULSE_CODE_MINAVAIL + 12)
#define PULSE_REST_TIMEOUT    (_PULSE_CODE_MINAVAIL + 13)   // no rest in time

#define REST_TIME_NS  5000000000ULL    // 5s to come to rest

message_t last_sensor;             // only main thread writes these, so no mutex
int have_sensor_report = 0, have_nav_contact = 0, in_recovery = 0;
int shake_hold = 0, locked = 0;      // waiting for rest / locked for good
int nav_coid = -1, sensor_coid = -1, rest_timer;
int approved_count = 0, rejected_count = 0;
double last_latency_us = 0.0, max_jitter_us = 0.0;
uint64_t last_sensor_time, last_nav_time, last_stale_time, cycles_per_ms;
char events[5][80];        // last 5, newest at the end

volatile int loop_count = 0;       // main loop +1 each wakeup, watchdog reads it

int main_coid, recovery_chid, recovery_coid;    // set in main before the threads start

void log_event(char *text)
{
    int i;

    for (i = 0; i < 4; i++)
        strcpy(events[i], events[i + 1]);
    strcpy(events[4], text);
    printf("[SUPERVISOR] %s\n", text);
}

void trigger_recovery(char *reason)
{
    char text[80];

    in_recovery = 1;
    MsgSendPulse(recovery_coid, 24, PULSE_ENTER_SAFE, 0);    // do the safe state first, 24 = recovery prio
    sprintf(text, "SafetyEvent: %s -> entering recovery", reason);
    log_event(text);                                         // print after, its slow
}

void tell(int *coid, char *name, int prio, int code)    // pulse to a program found by name
{
    if (*coid == -1 || MsgSendPulse(*coid, prio, code, 0) == -1) {    // reconnect if it fails
        *coid = name_open(name, 0);
        if (*coid != -1)
            MsgSendPulse(*coid, prio, code, 0);
    }
}

void tell_nav(int code)
{
    tell(&nav_coid, "navigation", 10, code);    // 10 = nav prio
}

int check_nav(message_t *nav)       // 0 = ok, else reason
{
    if (locked)
        return 6;                   // locked
    if (in_recovery)
        return 4;                   // in recovery
    if (nav->speed > MAX_SPEED)
        return 1;                   // too fast
    if (nav->steering > MAX_STEERING || nav->steering < -MAX_STEERING)
        return 2;                   // too much steering
    if (!have_sensor_report || !last_sensor.imu_ok || !last_sensor.ultrasonic_ok ||
        last_sensor.distance_cm <= MIN_SAFE_DISTANCE)
        return 3;                   // sensor bad or no report
    return 0;
}

void *recovery_thread(void *arg)    // leds + watchdog
{
    struct _pulse pulse;
    volatile uint32_t *gpio;
    int hold_timer, watchdog_timer, last_count = 0, led_on = 0;

    set_priority(24, "recovery-mgr");
    ThreadCtl(_NTO_TCTL_IO, 0);    // io privilege is per thread

    uintptr_t gpio_addr = mmap_device_io(0xA0, GPIO_BASE);
    if (gpio_addr == MAP_DEVICE_FAILED) {
        printf("[RECOVERY] could not map gpio\n");
        exit(1);
    }
    gpio = (volatile uint32_t *) gpio_addr;
    gpio[1] = (gpio[1] & ~(7 << 21)) | (1 << 21);   // red as output
    gpio[10] = 1 << LED_PIN;                        // red off
    gpio[2] = (gpio[2] & ~(7 << 21)) | (1 << 21);   // green as output
    gpio[7] = 1 << GREEN_PIN;                       // green on

    hold_timer = make_timer(recovery_coid, 24, PULSE_RECOVERY_TIMER);

    watchdog_timer = make_timer(recovery_coid, 24, PULSE_WATCHDOG);     // own channel+timer so it still works if main hangs
    set_timer(watchdog_timer, 500000000ULL, 500000000ULL);              // check every 500ms

    printf("[RECOVERY] Recovery Manager ready, priority 24\n");

    while (1) {
        MsgReceive(recovery_chid, &pulse, sizeof(pulse), NULL);

        if (pulse.code == PULSE_ENTER_SAFE) {
            gpio[7] = 1 << LED_PIN;                     // led first, print later
            gpio[10] = 1 << GREEN_PIN;
            set_timer(hold_timer, 3000000000ULL, 0);    // hold 3s
            if (!led_on)                                // sup keeps re-sending this while shaking
                printf("[RECOVERY] entering safe state -- LED ON\n");
            led_on = 1;
        } else if (pulse.code == PULSE_RECOVERY_TIMER) {
            led_on = 0;
            gpio[10] = 1 << LED_PIN;
            gpio[7] = 1 << GREEN_PIN;
            MsgSendPulse(main_coid, 25, PULSE_RECOVERY_DONE, 0);
            printf("[RECOVERY] hold complete -- LED OFF\n");
        } else if (pulse.code == PULSE_WATCHDOG) {
            if (loop_count == last_count) {             // main didnt move = stuck
                gpio[7] = 1 << LED_PIN;
                gpio[10] = 1 << GREEN_PIN;
                set_timer(hold_timer, 3000000000ULL, 0);
                printf("[RECOVERY] WATCHDOG: supervisor main loop is stuck -- LED ON\n");
            }
            last_count = loop_count;
        }
    }
    return NULL;
}

void *cli_thread(void *arg)    // type commands, ask main with MsgSend
{
    message_t q = {0};
    char reply[512];

    set_priority(5, "cli");
    printf("[CLI] commands: status | health | history | hang (demo) | quit\n");

    while (1) {
        printf("supervisor> ");
        fflush(stdout);
        if (scanf("%15s", q.cmd) == EOF)
            break;

        if (strcmp(q.cmd, "quit") == 0) {
            printf("[CLI] shutting down\n");
            exit(0);
        }

        q.type = MSG_CLI_QUERY;
        if (MsgSend(main_coid, &q, sizeof(q), reply, sizeof(reply)) == -1)
            printf("[CLI] request failed\n");
        else
            printf("%s\n", reply);
    }
    return NULL;
}

int main(void)
{
    message_t msg;
    struct _pulse *pulse = (struct _pulse *) &msg;    // pulse comes in the same buffer
    struct sched_param sp;
    name_attach_t *attach;
    pthread_t tid;
    char reply[512], text[80];
    int rcvid, reason, ok, i;
    uint64_t now, t_start;
    double interval_us, jitter_us;

    set_priority(25, "supervisor-main");

    attach = name_attach(NULL, "safety_supervisor", 0);
    if (attach == NULL) {
        printf("name_attach failed\n");
        return 1;
    }
    main_coid = ConnectAttach(0, 0, attach->chid, _NTO_SIDE_CHANNEL, 0);
    cycles_per_ms = SYSPAGE_ENTRY(qtime)->cycles_per_sec / 1000;
    rest_timer = make_timer(main_coid, 25, PULSE_REST_TIMEOUT);    // rest watchdog

    recovery_chid = ChannelCreate(0);      // own channel for recovery, we use pulses
    recovery_coid = ConnectAttach(0, 0, recovery_chid, _NTO_SIDE_CHANNEL, 0);

    pthread_create(&tid, NULL, recovery_thread, NULL);    // docs say pthread_create not ThreadCreate
    pthread_create(&tid, NULL, cli_thread, NULL);

    set_timer(make_timer(main_coid, 25, PULSE_STALE_CHECK), 200000000ULL, 200000000ULL);    // stale check every 200ms, prio 25

    printf("\n[SUPERVISOR] QNX Safety Supervisor started, priority 25\n");

    while (1) {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        loop_count++;                 // watchdog looks at this
        if (rcvid == -1)
            continue;

        if (rcvid == 0) {             // 0 = pulse
            if (pulse->code == PULSE_HEARTBEAT) {
                last_nav_time = ClockCycles();
                have_nav_contact = 1;
            } else if (pulse->code == PULSE_NAV_EXIT) {
                have_nav_contact = 0;    // left on purpose, not a fault
                log_event("navigation exited normally");
            } else if (pulse->code == PULSE_STALE_CHECK) {
                now = ClockCycles();

                if (last_stale_time != 0) {    // jitter = how far from 200ms
                    interval_us = (now - last_stale_time) * 1000.0 / cycles_per_ms;
                    jitter_us = (interval_us > 200000.0) ? interval_us - 200000.0 : 200000.0 - interval_us;
                    if (jitter_us > max_jitter_us)
                        max_jitter_us = jitter_us;
                }
                last_stale_time = now;

                tell_nav(PULSE_SUP_ALIVE);    // sent from this loop, so a hung sup goes silent
                tell(&sensor_coid, "sensor_monitor", 15, PULSE_SUP_ALIVE);

                if (shake_hold || locked || !have_sensor_report)     // keep red on, no sensor = not safe
                    MsgSendPulse(recovery_coid, 24, PULSE_ENTER_SAFE, 0);

                if (have_nav_contact && (now - last_nav_time) / cycles_per_ms > 500) {
                    trigger_recovery("navigation went silent");
                    have_nav_contact = 0;
                }
                if (have_sensor_report && (now - last_sensor_time) / cycles_per_ms > 500) {
                    trigger_recovery("sensor monitor went silent");
                    tell_nav(PULSE_NAV_STOP);     // no sensor data, stop nav too
                    have_sensor_report = 0;
                }
            } else if (pulse->code == PULSE_REST_TIMEOUT) {
                if (shake_hold) {             // still shaking, lock it
                    shake_hold = 0;
                    locked = 1;
                    tell_nav(PULSE_NAV_LOCK);
                    log_event("no rest within 5 s -> navigation LOCKED (restart to reset)");
                }
            } else if (pulse->code == PULSE_RECOVERY_DONE) {
                log_event("recovery hold complete, back to normal");
                in_recovery = 0;
            }
        } else if (msg.type == MSG_NAV_CMD) {
            t_start = ClockCycles();
            SchedGet(0, 0, &sp);      // curpriority = prio we run at now

            reason = check_nav(&msg);
            ok = (reason == 0);
            MsgReply(rcvid, EOK, &ok, sizeof(ok));

            last_latency_us = (ClockCycles() - t_start) * 1000.0 / cycles_per_ms;
            last_nav_time = t_start;
            have_nav_contact = 1;

            if (ok) {
                approved_count++;
                sprintf(text, "nav seq=%d APPROVED (%.1fus, ran at prio %d)", msg.seq, last_latency_us, sp.sched_curpriority);
                log_event(text);
            } else {
                rejected_count++;
                sprintf(text, "nav seq=%d REJECTED reason=%d (%.1fus, ran at prio %d)", msg.seq, reason, last_latency_us, sp.sched_curpriority);
                log_event(text);
                if (reason < 4)       // 4-6 are already stopped
                    trigger_recovery("nav command rejected");
            }
        } else if (msg.type == MSG_SENSOR_HEALTH) {
            last_sensor = msg;
            have_sensor_report = 1;
            last_sensor_time = ClockCycles();
            MsgReply(rcvid, EOK, NULL, 0);

            if (msg.shaking && !shake_hold && !locked) {       // shaking: stop, start rest timer
                shake_hold = 1;
                set_timer(rest_timer, REST_TIME_NS, 0);
                tell_nav(PULSE_NAV_SLOW);                      // only slow down, stop comes with the rest timer
                log_event("vibration detected -> vehicle slowing down, waiting for rest");
            } else if (!msg.shaking && shake_hold) {           // calm again
                shake_hold = 0;
                set_timer(rest_timer, 0, 0);
                tell_nav(PULSE_NAV_RESUME);
                log_event("vehicle at rest, back to full speed");
            }
        } else if (msg.type == MSG_CLI_QUERY) {
            if (strcmp(msg.cmd, "status") == 0) {
                sprintf(reply, "mode=%s approved=%d rejected=%d last_latency=%.1fus max_jitter=%.0fus",
                        locked ? "LOCKED" : shake_hold ? "SHAKING" : in_recovery ? "RECOVERY" : !have_sensor_report ? "NO SENSOR" : "NORMAL", approved_count, rejected_count, last_latency_us, max_jitter_us);
                max_jitter_us = 0.0;    // resets each status
            } else if (strcmp(msg.cmd, "hang") == 0) {
                while (1)               // demo only, fake a stuck loop
                    ;
            } else if (strcmp(msg.cmd, "health") == 0) {
                if (have_sensor_report)
                    sprintf(reply, "imu_ok=%d ultrasonic_ok=%d distance_cm=%.1f",
                            last_sensor.imu_ok, last_sensor.ultrasonic_ok, last_sensor.distance_cm);
                else
                    strcpy(reply, "no sensor report received yet");
            } else if (strcmp(msg.cmd, "history") == 0) {
                reply[0] = '\0';
                for (i = 0; i < 5; i++) {
                    if (events[i][0] != '\0') {
                        strcat(reply, events[i]);
                        strcat(reply, "\n");
                    }
                }
                if (reply[0] == '\0')
                    strcpy(reply, "no events logged yet");
            } else {
                sprintf(reply, "unknown command: %s", msg.cmd);
            }
            MsgReply(rcvid, EOK, reply, strlen(reply) + 1);
        } else {
            MsgError(rcvid, ENOSYS);
        }
    }

    return 0;
}
