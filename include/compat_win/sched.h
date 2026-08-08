#ifndef FLOWENGINE_COMPAT_WIN_SCHED_H
#define FLOWENGINE_COMPAT_WIN_SCHED_H

/* MinGW-w64 (winpthreads) already ships a real <sched.h>. Prefer it so we do
 * not redefine struct sched_param / sched_yield against the toolchain headers.
 * MSVC has no sched.h — keep the small SwitchToThread shim below. */
#if defined(__MINGW32__) && !defined(_MSC_VER)
#include_next <sched.h>
#else

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

#endif /* !MinGW */

#endif /* FLOWENGINE_COMPAT_WIN_SCHED_H */
