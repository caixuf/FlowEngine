#ifndef FLOWENGINE_COMPAT_WIN_SCHED_H
#define FLOWENGINE_COMPAT_WIN_SCHED_H

#include <windows.h>

#define SCHED_OTHER 0
#define SCHED_FIFO  1

struct sched_param {
    int sched_priority;
};

static inline int sched_yield(void) {
    SwitchToThread();
    return 0;
}

#endif /* FLOWENGINE_COMPAT_WIN_SCHED_H */
