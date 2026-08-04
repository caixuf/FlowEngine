#ifndef FLOWENGINE_COMPAT_WIN_SYS_TIME_H
#define FLOWENGINE_COMPAT_WIN_SYS_TIME_H

#include <time.h>
#include <stdint.h>
#include <winsock2.h>

#if !defined(_TIMEVAL_DEFINED) && !defined(_WINSOCK2API_)
#define _TIMEVAL_DEFINED
struct timeval {
    long tv_sec;
    long tv_usec;
};
#endif

static inline int gettimeofday(struct timeval* tv, void* tz) {
    (void)tz;
    if (!tv) return -1;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    tv->tv_sec = (long)ts.tv_sec;
    tv->tv_usec = ts.tv_nsec / 1000;
    return 0;
}

#endif /* FLOWENGINE_COMPAT_WIN_SYS_TIME_H */
