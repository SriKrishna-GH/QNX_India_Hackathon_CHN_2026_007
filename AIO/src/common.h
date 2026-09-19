#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <sys/syspage.h>

#define MSG_NAV_CMD        1
#define MSG_SENSOR_HEALTH  2
#define MSG_CLI_QUERY      3

typedef struct {                   // one struct for every msg
    int    type;                   // MSG_*
    int    seq;
    double speed, steering;        // nav
    double distance_cm;            // sensor
    int    imu_ok, ultrasonic_ok;  // sensor
    int    shaking;                // 1 = vibration in the last sec
    char   cmd[16];                // cli command
} message_t;

#define PULSE_HEARTBEAT       (_PULSE_CODE_MINAVAIL + 0)   // nav -> sup, still alive
#define PULSE_NAV_EXIT        (_PULSE_CODE_MINAVAIL + 1)   // nav -> sup, normal exit
#define PULSE_ENTER_SAFE      (_PULSE_CODE_MINAVAIL + 2)   // sup -> recovery
#define PULSE_RECOVERY_DONE   (_PULSE_CODE_MINAVAIL + 3)   // recovery -> sup
#define PULSE_DISTANCE        (_PULSE_CODE_MINAVAIL + 4)   // nav distance timer
#define PULSE_NAV_STOP        (_PULSE_CODE_MINAVAIL + 5)   // sup -> nav, stop
#define PULSE_NAV_LOCK        (_PULSE_CODE_MINAVAIL + 6)   // sup -> nav, stop and lock
#define PULSE_SUP_ALIVE       (_PULSE_CODE_MINAVAIL + 9)  // sup -> nav + sensor, sup main loop still runs
#define PULSE_NAV_SLOW        (_PULSE_CODE_MINAVAIL + 7)   // sup -> nav, vibration, slow down
#define PULSE_NAV_RESUME      (_PULSE_CODE_MINAVAIL + 8)   // sup -> nav, calm again, full speed

void set_priority(int prio, char *name)
{
    struct sched_param p;
    p.sched_priority = prio;
    SchedSet(0, 0, SCHED_FIFO, &p);              // qnx prio
    ThreadCtl(_NTO_TCTL_NAME, name);             // name shows in pidin
}

int make_timer(int coid, int prio, int pulse_code)
{
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, prio, pulse_code, 0);   // receiver runs at prio while it handles this
    return TimerCreate(CLOCK_MONOTONIC, &ev);           // sends the pulse to coid
}

void set_timer(int timer, uint64_t first_ns, uint64_t repeat_ns)
{
    struct _itimer it;
    memset(&it, 0, sizeof(it));
    it.nsec = first_ns;                          // first fire
    it.interval_nsec = repeat_ns;                // then repeat, 0 = one shot, both 0 = stop
    TimerSettime(timer, 0, &it, NULL);
}

int find_supervisor(void)
{
    uint64_t wait_ns = 100000000ULL;   // 100ms
    int coid = name_open("safety_supervisor", 0);
    int tries = 0;

    while (coid == -1 && tries < 20) {   // try for ~2s
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_NANOSLEEP, NULL, &wait_ns, NULL);   // kernel sleep, no spinning
        coid = name_open("safety_supervisor", 0);
        tries++;
    }
    return coid;
}

#endif
