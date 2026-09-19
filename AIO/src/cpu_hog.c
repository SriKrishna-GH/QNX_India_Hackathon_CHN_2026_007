#include "common.h"

int hog_prio = 16;     // usage: cpu_hog [prio] [seconds]
uint64_t stop_at;

void *hog_thread(void *arg)
{
    set_priority(hog_prio, "cpu_hog");
    while (ClockCycles() < stop_at)
        ;                              // just spin
    return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t tid[8];
    int i, seconds = 10, n = _syspage_ptr->num_cpu;    // one hog per core

    if (argc > 1)
        hog_prio = atoi(argv[1]);
    if (argc > 2)
        seconds = atoi(argv[2]);
    if (n > 8)
        n = 8;

    stop_at = ClockCycles() + (uint64_t) seconds * SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    printf("[HOG] %d threads at priority %d for %d s\n", n, hog_prio, seconds);

    for (i = 0; i < n; i++)
        pthread_create(&tid[i], NULL, hog_thread, NULL);
    for (i = 0; i < n; i++)
        pthread_join(tid[i], NULL);

    printf("[HOG] done\n");
    return 0;
}
