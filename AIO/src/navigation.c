#include "common.h"

int chid;             // our channel: distance ticks + stop/lock from sup
int distance = 0;
int stopped = 0, locked = 0, slowed = 0;    // set by sup pulses
uint64_t cycles_per_ms, last_sup;    // last_sup = last sign of life from the supervisor
int sup_coid, hb_timer;

void wait_for_distance(int stop_at)    // +1 per tick until stop_at or sup stops us
{
    struct _pulse pulse;
    int skip = 0;
    while (distance < stop_at && !stopped) {
        MsgReceive(chid, &pulse, sizeof(pulse), NULL);
        if (pulse.code == PULSE_DISTANCE) {
            if (slowed && (skip++ % 2) == 0)    // slowed = every 2nd tick, half speed
                continue;
            distance++;
            printf("[NAV] Distance = %d%s\n", distance, slowed ? " (slow)" : "");
        } else if (pulse.code == PULSE_NAV_SLOW) {
            slowed = 1;
            printf("[NAV] vibration -> slowing down\n");
        } else if (pulse.code == PULSE_NAV_RESUME) {
            slowed = 0;
            printf("[NAV] calm again -> full speed\n");
        } else if (pulse.code == PULSE_NAV_STOP) {
            stopped = 1;
        } else if (pulse.code == PULSE_NAV_LOCK) {
            stopped = 1;
            locked = 1;
        } else if (pulse.code == PULSE_SUP_ALIVE) {
            last_sup = ClockCycles();
        } else if (pulse.code == _PULSE_CODE_COIDDEATH) {    // the supervisor's channel is gone
            printf("RED - supervisor died, stopping\n");
            stopped = 1;
        }
        if ((ClockCycles() - last_sup) / cycles_per_ms > 500 && !stopped) {    // no sign of life
            printf("RED - supervisor silent for 500 ms, stopping\n");
            stopped = 1;
        }
    }
}

int ask(char *prompt)    // read a number, bad text = 0
{
    char input[32];
    printf("%s", prompt);
    fflush(stdout);
    fgets(input, sizeof(input), stdin);
    return atoi(input);
}

void reconnect(void)    // sup restarted, connect again + redo the heartbeat timer
{
    name_close(sup_coid);
    TimerDestroy(hb_timer);
    sup_coid = find_supervisor();
    hb_timer = make_timer(sup_coid, 25, PULSE_HEARTBEAT);
    set_timer(hb_timer, 200000000, 200000000);
}

int ask_supervisor(message_t *nav)    // 1 = approved
{
    int approved = 0, r = -1, err = 0, tries;
    uint64_t t0, timeout_ns;

    for (tries = 0; tries < 2; tries++) {
        timeout_ns = 50000000ULL;     // 50ms deadline, kernel enforces it
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeout_ns, NULL);
        t0 = ClockCycles();
        r = MsgSend(sup_coid, nav, sizeof(*nav), &approved, sizeof(approved));
        err = errno;                  // grab errno now
        if (r != -1 || err == ETIMEDOUT || tries == 1)
            break;
        reconnect();                  // not a timeout, maybe sup restarted, try again
    }

    if (r == -1) {
        approved = 0;             // no answer = not approved
        if (err == ETIMEDOUT)
            printf("RED - Supervisor missed the 50 ms deadline\n");
        else
            printf("RED - Supervisor communication failure\n");
    } else {
        printf("[NAV] safety check round trip = %.1f us\n", (ClockCycles() - t0) * 1000.0 / cycles_per_ms);
        if (approved)
            printf("GREEN - ACTION ALLOWED\n");
        else
            printf("RED - ACTION OVERRIDDEN\n");
    }
    return approved;
}

int main(void)
{
    message_t nav = {0};
    name_attach_t *attach;
    int coid, choice, target, checkpoint, approved, seq = 0;
    char *names[] = {"", "MOVE FORWARD", "TURN RIGHT", "TURN LEFT"};   // index = menu choice
    double steering[] = {0.0, 0.0, 20.0, -20.0};

    set_priority(10, "navigation");
    cycles_per_ms = SYSPAGE_ENTRY(qtime)->cycles_per_sec / 1000;

    attach = name_attach(NULL, "navigation", 0);    // named so sup can find us
    if (attach == NULL) {
        printf("[NAV] name_attach failed\n");
        return 1;
    }
    chid = attach->chid;
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);

    sup_coid = find_supervisor();
    if (sup_coid == -1) {
        printf("[NAV] no safety_supervisor found\n");
        return 1;
    }

    set_timer(make_timer(coid, 10, PULSE_DISTANCE), 500000000, 500000000);    // distance +1 every 500ms

    hb_timer = make_timer(sup_coid, 25, PULSE_HEARTBEAT);    // only shows the process is alive, prio 25 = sup handles it at 25
    set_timer(hb_timer, 200000000, 200000000);               // every 200ms

    printf("\n[NAVIGATION] QNX Navigation started\n");

    while (1) {
        printf("\n1 - MOVE FORWARD\n2 - TURN RIGHT\n3 - TURN LEFT\n0 - EXIT\n");
        choice = ask("Enter instruction: ");

        if (choice == 0)
            break;
        if (choice < 1 || choice > 3) {
            printf("Invalid instruction\n");
            continue;
        }
        if (locked) {
            printf("RED - NAVIGATION LOCKED by the supervisor (vehicle did not come to rest)\n");
            continue;
        }

        target = ask("Enter target distance: ");
        if (target <= distance) {
            printf("RED - Target distance already passed\n");
            continue;
        }
        printf("\n[NAV] Current distance: %d\n[NAV] Target distance : %d\n", distance, target);

        checkpoint = (target > 5) ? (target - 5) : 0;    // check 5 before the target
        stopped = 0;
        last_sup = ClockCycles();    // grace period: wait for the first alive pulse
        wait_for_distance(checkpoint);

        approved = 0;
        if (!stopped) {    // at checkpoint, ask sup
            nav.type = MSG_NAV_CMD;
            nav.seq = ++seq;
            nav.speed = slowed ? 0.5 : 1.0;
            nav.steering = steering[choice];

            printf("[NAV] Decision: %s\n", names[choice]);
            printf("\n[NAV] Checkpoint reached at distance %d (5 units before target)\n", distance);
            printf("[NAV] speed = %.2f m/s, steering = %.2f deg\n", nav.speed, nav.steering);
            printf("[NAV -> SUPERVISOR] Safety check...\n");
            approved = ask_supervisor(&nav);
        }

        if (approved)
            wait_for_distance(target);    // rest of the way

        if (stopped)
            printf("RED - VEHICLE STOPPED by the supervisor (vibration or no sensor data)%s\n", locked ? ", NAVIGATION LOCKED" : "");
        else if (approved)
            printf("[NAV] Target reached at distance %d\n", distance);
        else
            printf("[NAV] Maneuver stopped, not continuing to target\n");

        distance = 0;    // new cycle
        printf("[NAV] Distance reset to 0\n");
    }

    set_timer(hb_timer, 0, 0);                        // stop heartbeat first
    MsgSendPulse(sup_coid, -1, PULSE_NAV_EXIT, 0);    // then say we left on purpose
    return 0;
}
