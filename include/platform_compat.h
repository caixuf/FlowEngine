#ifndef FLOWENGINE_PLATFORM_COMPAT_H
#define FLOWENGINE_PLATFORM_COMPAT_H

/* =============================================================================
 * platform_compat.h — 跨平台兼容层(macOS ⇄ Linux)
 *
 * 设计原则(与用户约定):
 *   - 能平替的用宏统一(如线程命名签名差异)→ 调用点零改动;
 *   - macOS 实现不了的降级成无害兜底(如 robust mutex)→ no-op,不破坏语义。
 *
 * 本头由 CMake 在 APPLE 平台通过 `-include` 全局强制包含(见 CMakeLists.txt),
 * 因此绝大多数源文件无需手动 #include。所有内容都包在 __APPLE__ 守卫内,
 * Linux 上完全透明、零影响。
 *
 * 第三方(cJSON / Eigen)不调用下列符号,force-include 对其无副作用。
 * ========================================================================== */

#if defined(__APPLE__)

#include <pthread.h>

/* ── [1] 线程命名 ─────────────────────────────────────────────────────────
 * Linux:  int pthread_setname_np(pthread_t thread, const char *name);  // 2 参
 * macOS:  int pthread_setname_np(const char *name);                    // 1 参
 *         (macOS 只能命名"当前线程",无法命名任意线程句柄)
 *
 * 全项目 35 处调用均为 `pthread_setname_np(pthread_self(), "xxx")`,即命名当前
 * 线程 —— 语义与 macOS 单参版本完全一致。用透明宏把 2 参调用重映射到 1 参,
 * 调用点无需任何改动。
 *
 * 顺序要点:内联包装函数必须在宏定义"之前"出现,此时 pthread_setname_np 仍指向
 * 真实的 libc 单参函数;宏定义之后的所有 2 参调用才被改写。 */
static inline int flow_pthread_setname(const char* name) {
    return pthread_setname_np(name);
}
#define pthread_setname_np(thread, name) flow_pthread_setname(name)

/* ── [2] Robust mutex ─────────────────────────────────────────────────────
 * macOS 的 pthreads 不支持 robust mutex(无 pthread_mutexattr_setrobust /
 * pthread_mutex_consistent / PTHREAD_MUTEX_ROBUST)。
 *
 * 降级为无害 no-op:mutex 退化为普通 PTHREAD_PROCESS_SHARED 锁。影响仅限
 * --multi 多进程 IPC 模式下"持锁进程崩溃后自动标记 inconsistent 并恢复"这一
 * 崩溃自愈能力;默认单进程 dlopen demo 根本不触发该路径。共享状态本身另有
 * seq 号自愈(见 ipc_channel.c),锁弱化不影响正常收发。 */
#ifndef PTHREAD_MUTEX_ROBUST
#define PTHREAD_MUTEX_ROBUST 0
#endif
#define pthread_mutexattr_setrobust(attr, robustness) (0)
#define pthread_mutex_consistent(mutex)               (0)

/* ── [3] condvar 时钟选择 ──────────────────────────────────────────────────
 * macOS 不提供 pthread_condattr_setclock():进程共享条件变量的等待时钟固定为
 * CLOCK_REALTIME,无法切到 CLOCK_MONOTONIC。降级为无害 no-op ——
 * ipc_channel.c 的 IPC_COND_CLOCK 已在 __APPLE__ 下相应选用 CLOCK_REALTIME,
 * 等待方 deadline 与条件变量时钟保持一致,timedwait 语义正确。 */
#define pthread_condattr_setclock(attr, clock_id) (0)

/* ── [4] CPU pause / spin-loop hint ────────────────────────────────────────
 * 依赖 flowcoro(第三方)在自旋等待里用了 x86 专属内建 __builtin_ia32_pause()。
 * Apple Silicon(arm64)无此内建,编译直接报 undeclared identifier。用宏平替:
 *   - arm64 → "yield" 指令(等价的自旋让步提示);
 *   - x86_64 Mac → 保留真实 x86 pause(定义成空宏会退化,故仅 arm 重定义)。
 * 本头 force-include 进 flowcoro TU(add_subdirectory 在 -include 之后),对其生效。 */
#if defined(__aarch64__) || defined(__arm64__)
#define __builtin_ia32_pause() __asm__ __volatile__("yield" ::: "memory")
#endif

/* ── [5] accept4 / SOCK_NONBLOCK / SOCK_CLOEXEC ────────────────────────────
 * Linux 扩展 accept4() 一步完成 accept + 设置 O_NONBLOCK/FD_CLOEXEC;macOS 只有
 * accept()。用 accept() + fcntl 平替。SOCK_NONBLOCK/SOCK_CLOEXEC 在 macOS 无定义,
 * 定义成高位哨兵值(flowcoro 仅在 accept4 的 flags 里用,socket() 不 OR 它们,
 * 故不会污染 socket type)。 */
#include <sys/socket.h>
#include <fcntl.h>
#ifndef SOCK_NONBLOCK
#define SOCK_NONBLOCK 0x40000000
#endif
#ifndef SOCK_CLOEXEC
#define SOCK_CLOEXEC  0x20000000
#endif
static inline int flow_accept4(int fd, struct sockaddr* addr,
                               socklen_t* len, int flags) {
    int c = accept(fd, addr, len);
    if (c < 0) return -1;
    if (flags & SOCK_NONBLOCK) {
        int fl = fcntl(c, F_GETFL);
        if (fl != -1) fcntl(c, F_SETFL, fl | O_NONBLOCK);
    }
    if (flags & SOCK_CLOEXEC) {
        int fd2 = fcntl(c, F_GETFD);
        if (fd2 != -1) fcntl(c, F_SETFD, fd2 | FD_CLOEXEC);
    }
    return c;
}
#define accept4(fd, addr, len, flags) flow_accept4((fd), (addr), (len), (flags))

/* ── [6] 线程栈大小 ────────────────────────────────────────────────────────
 * Linux 次级线程默认栈 8MB;macOS 次级线程默认只有 512KB。项目里若干节点线程
 * (如 perception 的 DBSCAN)在单个栈帧上放了大数组 —— dbscan_run 里
 * `int nb[131072]`(512KB)+ 栈上的 grid(next_point[131072] 512KB),再叠加
 * expand_cluster 的 seeds/nb —— 单帧 >1MB。Linux 8MB 栈吃得下,macOS 512KB
 * 直接栈溢出 SIGSEGV(crash 落在 dbscan_run 入口,far 地址在 sp 下方 ~512KB)。
 *
 * 平替:用透明宏包裹 pthread_create,在 macOS 上把栈下限抬到 8MB,对齐 Linux
 * 默认值。调用点零改动;显式传了更大栈的 attr 予以尊重(只抬高、不压低)。
 * 顺序同 [1]:包装函数在宏定义之前,此时 pthread_create 仍指向真实 libc 符号。 */
#include <stddef.h>
#define FLOW_MIN_THREAD_STACK (8u * 1024u * 1024u)  /* 8MB,= Linux 默认 */
static inline int flow_pthread_create(pthread_t* thread,
                                      const pthread_attr_t* attr,
                                      void* (*start)(void*), void* arg) {
    pthread_attr_t local;
    if (attr) local = *attr;                 /* 继承调用者设置(detachstate 等) */
    else      pthread_attr_init(&local);
    size_t cur = 0;
    if (pthread_attr_getstacksize(&local, &cur) != 0 || cur < FLOW_MIN_THREAD_STACK)
        pthread_attr_setstacksize(&local, FLOW_MIN_THREAD_STACK);
    int rc = pthread_create(thread, &local, start, arg);
    pthread_attr_destroy(&local);
    return rc;
}
#define pthread_create(thread, attr, start, arg) \
    flow_pthread_create((thread), (attr), (start), (arg))

#endif /* __APPLE__ */

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include <io.h>
#include <process.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

static inline int flow_win_clock_gettime(int clock_id, struct timespec* ts) {
    if (!ts) return -1;
    if (clock_id == CLOCK_MONOTONIC) {
        static LARGE_INTEGER freq;
        LARGE_INTEGER now;
        if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&now);
        ts->tv_sec = (time_t)(now.QuadPart / freq.QuadPart);
        ts->tv_nsec = (long)(((now.QuadPart % freq.QuadPart) * 1000000000LL) / freq.QuadPart);
        return 0;
    }

    FILETIME ft;
    ULARGE_INTEGER uli;
    GetSystemTimePreciseAsFileTime(&ft);
    uli.LowPart = ft.dwLowDateTime;
    uli.HighPart = ft.dwHighDateTime;
    uint64_t ns100 = uli.QuadPart - 116444736000000000ULL;
    ts->tv_sec = (time_t)(ns100 / 10000000ULL);
    ts->tv_nsec = (long)((ns100 % 10000000ULL) * 100ULL);
    return 0;
}
#define clock_gettime(clock_id, ts) flow_win_clock_gettime((clock_id), (ts))

#define getpid   _getpid
#define mkdir(path, mode) _mkdir(path)
#define access   _access
#define unlink   _unlink
#define strdup   _strdup
#define strcasecmp  _stricmp
#define strncasecmp _strnicmp
#define strtok_r(str, delim, saveptr) strtok_s((str), (delim), (saveptr))
#ifndef F_OK
#define F_OK 0
#endif
typedef unsigned int useconds_t;

static inline struct tm* flow_win_localtime_r(const time_t* t, struct tm* out) {
    return (out && localtime_s(out, t) == 0) ? out : NULL;
}
#define localtime_r(t, out) flow_win_localtime_r((t), (out))

static inline int flow_win_setenv(const char* name, const char* value, int overwrite) {
    if (!overwrite) {
        size_t len = 0;
        getenv_s(&len, NULL, 0, name);
        if (len > 0) return 0;
    }
    return _putenv_s(name, value ? value : "");
}
#define setenv(name, value, overwrite) flow_win_setenv((name), (value), (overwrite))

static inline char* flow_win_strcasestr(const char* haystack, const char* needle) {
    if (!haystack || !needle) return NULL;
    if (!*needle) return (char*)haystack;
    size_t nlen = strlen(needle);
    for (const char* p = haystack; *p; ++p) {
        if (_strnicmp(p, needle, nlen) == 0) return (char*)p;
    }
    return NULL;
}
#define strcasestr(haystack, needle) flow_win_strcasestr((haystack), (needle))

#ifndef __builtin_expect
#define __builtin_expect(expr, value) (expr)
#endif
#ifndef __builtin_prefetch
#define __builtin_prefetch(addr, ...) ((void)0)
#endif
#ifndef __builtin_ia32_pause
#define __builtin_ia32_pause() YieldProcessor()
#endif

static inline int flow_win_readlink(const char* path, char* buf, size_t bufsz) {
    (void)path;
    if (!buf || bufsz == 0) return -1;
    DWORD n = GetModuleFileNameA(NULL, buf, (DWORD)bufsz);
    if (n == 0 || n >= bufsz) return -1;
    return (int)n;
}
#define readlink(path, buf, bufsz) flow_win_readlink((path), (buf), (bufsz))

static inline void flow_win_sleep_seconds(unsigned int seconds) {
    Sleep(seconds * 1000U);
}
static inline void flow_win_usleep(unsigned int usec) {
    Sleep((usec + 999U) / 1000U);
}
static inline int flow_win_nanosleep(const struct timespec* req, struct timespec* rem) {
    (void)rem;
    if (!req) return -1;
    uint64_t ms = (uint64_t)req->tv_sec * 1000ULL + (uint64_t)((req->tv_nsec + 999999L) / 1000000L);
    Sleep((DWORD)ms);
    return 0;
}
#define sleep(sec)   (flow_win_sleep_seconds((unsigned int)(sec)), 0)
#define usleep(usec) flow_win_usleep((unsigned int)(usec))
#define nanosleep(req, rem) flow_win_nanosleep((req), (rem))

/* posix_memalign: map to _aligned_malloc; <stdlib.h> already included above */
#ifndef posix_memalign
static inline int flow_win_posix_memalign(void** memptr, size_t alignment, size_t size) {
    *memptr = _aligned_malloc(size, alignment);
    return *memptr ? 0 : 12; /* 12 = ENOMEM */
}
#define posix_memalign(memptr, alignment, size) \
    flow_win_posix_memalign((memptr), (alignment), (size))
#endif

/* ── 线程栈大小（对齐 macOS 段 [6]）────────────────────────────────────
 * Windows 次级线程默认栈约 1MB；本项目节点线程在栈上放了大缓冲
 * （monitor scene_entities_json[64KB]、DBSCAN 网格等），FlowCoro + 嵌套帧
 * 易触发 STATUS_STACK_OVERFLOW (0xC00000FD)。抬到 8MB 对齐 Linux 默认。
 * MinGW 走真实 winpthreads；MSVC 走 compat_win/pthread.h（_beginthreadex
 * 亦已设 8MB 下限，此包装双保险）。 */
#include <pthread.h>
#ifndef FLOW_MIN_THREAD_STACK
#define FLOW_MIN_THREAD_STACK (8u * 1024u * 1024u)
#endif
static inline int flow_win_pthread_create(pthread_t* thread,
                                          const pthread_attr_t* attr,
                                          void* (*start)(void*), void* arg) {
    pthread_attr_t local;
    if (attr) local = *attr;
    else      pthread_attr_init(&local);
    size_t cur = 0;
    if (pthread_attr_getstacksize(&local, &cur) != 0 || cur < FLOW_MIN_THREAD_STACK)
        pthread_attr_setstacksize(&local, FLOW_MIN_THREAD_STACK);
    int rc = pthread_create(thread, &local, start, arg);
    pthread_attr_destroy(&local);
    return rc;
}
#define pthread_create(thread, attr, start, arg) \
    flow_win_pthread_create((thread), (attr), (start), (arg))

#endif /* _WIN32 */

#endif /* FLOWENGINE_PLATFORM_COMPAT_H */
