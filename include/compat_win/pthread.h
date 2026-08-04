#ifndef FLOWENGINE_COMPAT_WIN_PTHREAD_H
#define FLOWENGINE_COMPAT_WIN_PTHREAD_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <process.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef HANDLE pthread_t;
typedef SRWLOCK pthread_mutex_t;
typedef CONDITION_VARIABLE pthread_cond_t;

typedef struct {
    int pshared;
    int robust;
} pthread_mutexattr_t;

typedef struct {
    int pshared;
    int clock_id;
} pthread_condattr_t;

typedef struct {
    size_t stack_size;
    int detach_state;
} pthread_attr_t;

#define PTHREAD_MUTEX_INITIALIZER SRWLOCK_INIT
#define PTHREAD_COND_INITIALIZER CONDITION_VARIABLE_INIT
#define PTHREAD_PROCESS_PRIVATE 0
#define PTHREAD_PROCESS_SHARED  1
#define PTHREAD_MUTEX_ROBUST    0
#define PTHREAD_CREATE_JOINABLE 0
#define PTHREAD_CREATE_DETACHED 1

static inline int pthread_mutex_init(pthread_mutex_t* m, const pthread_mutexattr_t* attr) {
    (void)attr;
    InitializeSRWLock(m);
    return 0;
}
static inline int pthread_mutex_destroy(pthread_mutex_t* m) {
    (void)m;
    return 0;
}
static inline int pthread_mutex_lock(pthread_mutex_t* m) {
    AcquireSRWLockExclusive(m);
    return 0;
}
static inline int pthread_mutex_unlock(pthread_mutex_t* m) {
    ReleaseSRWLockExclusive(m);
    return 0;
}
static inline int pthread_mutex_trylock(pthread_mutex_t* m) {
    return TryAcquireSRWLockExclusive(m) ? 0 : EBUSY;
}

static inline int pthread_mutexattr_init(pthread_mutexattr_t* a) {
    if (a) { a->pshared = PTHREAD_PROCESS_PRIVATE; a->robust = 0; }
    return 0;
}
static inline int pthread_mutexattr_destroy(pthread_mutexattr_t* a) { (void)a; return 0; }
static inline int pthread_mutexattr_setpshared(pthread_mutexattr_t* a, int pshared) {
    if (a) a->pshared = pshared;
    return 0;
}
static inline int pthread_mutexattr_setrobust(pthread_mutexattr_t* a, int robust) {
    if (a) a->robust = robust;
    return 0;
}
static inline int pthread_mutex_consistent(pthread_mutex_t* m) { (void)m; return 0; }

static inline int pthread_cond_init(pthread_cond_t* c, const pthread_condattr_t* attr) {
    (void)attr;
    InitializeConditionVariable(c);
    return 0;
}
static inline int pthread_cond_destroy(pthread_cond_t* c) { (void)c; return 0; }
static inline int pthread_cond_signal(pthread_cond_t* c) {
    WakeConditionVariable(c);
    return 0;
}
static inline int pthread_cond_broadcast(pthread_cond_t* c) {
    WakeAllConditionVariable(c);
    return 0;
}

static inline DWORD flow_abs_timespec_to_timeout_ms(const struct timespec* abstime) {
    if (!abstime) return INFINITE;
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    int64_t sec = (int64_t)abstime->tv_sec - (int64_t)now.tv_sec;
    int64_t nsec = (int64_t)abstime->tv_nsec - (int64_t)now.tv_nsec;
    int64_t ms = sec * 1000 + nsec / 1000000;
    if (ms <= 0) return 0;
    return ms > 0xFFFFFFFEll ? 0xFFFFFFFEu : (DWORD)ms;
}

static inline int pthread_cond_timedwait(pthread_cond_t* c, pthread_mutex_t* m,
                                         const struct timespec* abstime) {
    DWORD timeout = flow_abs_timespec_to_timeout_ms(abstime);
    BOOL ok = SleepConditionVariableSRW(c, m, timeout, 0);
    if (ok) return 0;
    return GetLastError() == ERROR_TIMEOUT ? ETIMEDOUT : EINVAL;
}
static inline int pthread_cond_wait(pthread_cond_t* c, pthread_mutex_t* m) {
    return SleepConditionVariableSRW(c, m, INFINITE, 0) ? 0 : EINVAL;
}
static inline int pthread_condattr_init(pthread_condattr_t* a) {
    if (a) { a->pshared = PTHREAD_PROCESS_PRIVATE; a->clock_id = CLOCK_REALTIME; }
    return 0;
}
static inline int pthread_condattr_destroy(pthread_condattr_t* a) { (void)a; return 0; }
static inline int pthread_condattr_setpshared(pthread_condattr_t* a, int pshared) {
    if (a) a->pshared = pshared;
    return 0;
}
static inline int pthread_condattr_setclock(pthread_condattr_t* a, int clock_id) {
    if (a) a->clock_id = clock_id;
    return 0;
}

static inline int pthread_attr_init(pthread_attr_t* a) {
    if (a) { a->stack_size = 0; a->detach_state = PTHREAD_CREATE_JOINABLE; }
    return 0;
}
static inline int pthread_attr_destroy(pthread_attr_t* a) { (void)a; return 0; }
static inline int pthread_attr_getstacksize(const pthread_attr_t* a, size_t* out) {
    if (!out) return EINVAL;
    *out = a ? a->stack_size : 0;
    return 0;
}
static inline int pthread_attr_setstacksize(pthread_attr_t* a, size_t size) {
    if (a) a->stack_size = size;
    return 0;
}
static inline int pthread_attr_setdetachstate(pthread_attr_t* a, int state) {
    if (a) a->detach_state = state;
    return 0;
}
static inline int pthread_attr_setschedpolicy(pthread_attr_t* a, int policy) {
    (void)a; (void)policy;
    return 0;
}
static inline int pthread_attr_setschedparam(pthread_attr_t* a, const struct sched_param* param) {
    (void)a; (void)param;
    return 0;
}

typedef struct {
    void* (*start)(void*);
    void* arg;
} flow_pthread_start_t;

static unsigned __stdcall flow_pthread_trampoline(void* p) {
    flow_pthread_start_t* s = (flow_pthread_start_t*)p;
    void* (*start)(void*) = s->start;
    void* arg = s->arg;
    free(s);
    start(arg);
    return 0;
}

static inline int pthread_create(pthread_t* thread, const pthread_attr_t* attr,
                                 void* (*start)(void*), void* arg) {
    if (!thread || !start) return EINVAL;
    flow_pthread_start_t* s = (flow_pthread_start_t*)malloc(sizeof(*s));
    if (!s) return ENOMEM;
    s->start = start;
    s->arg = arg;
    unsigned tid = 0;
    size_t stack_size = attr ? attr->stack_size : 0;
    if (stack_size < 8u * 1024u * 1024u) stack_size = 8u * 1024u * 1024u;
    uintptr_t h = _beginthreadex(NULL, (unsigned)stack_size,
                                 flow_pthread_trampoline, s, 0, &tid);
    if (!h) { free(s); return errno ? errno : EINVAL; }
    *thread = (HANDLE)h;
    if (attr && attr->detach_state == PTHREAD_CREATE_DETACHED) CloseHandle(*thread);
    return 0;
}
static inline int pthread_join(pthread_t thread, void** value_ptr) {
    (void)value_ptr;
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    return 0;
}
static inline int pthread_detach(pthread_t thread) {
    CloseHandle(thread);
    return 0;
}
static inline pthread_t pthread_self(void) {
    return GetCurrentThread();
}
static inline int pthread_equal(pthread_t a, pthread_t b) {
    return GetThreadId(a) == GetThreadId(b);
}
static inline int pthread_setname_np(pthread_t thread, const char* name) {
    (void)thread; (void)name;
    return 0;
}
static inline int pthread_getname_np(pthread_t thread, char* name, size_t len) {
    (void)thread;
    if (name && len) name[0] = '\0';
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOWENGINE_COMPAT_WIN_PTHREAD_H */
