#ifndef FLOWENGINE_COMPAT_SYS_EPOLL_H
#define FLOWENGINE_COMPAT_SYS_EPOLL_H

/* =============================================================================
 * sys/epoll.h — macOS 平替：用 kqueue 实现 Linux epoll 子集
 *
 * 设计原则（与用户约定）：能平替的用「其他」平替。macOS 无 epoll，用底层
 * kqueue/kevent 提供等价语义。仅在 APPLE 平台通过 `-Iinclude/compat` 抢在系统
 * 头之前被 `#include <sys/epoll.h>` 命中（macOS 系统本就无此头，无冲突）。
 * Linux 上此文件永不参与编译，行为零变化。
 *
 * 覆盖 flowcoro/net 实际用到的子集：
 *   epoll_create1 / epoll_ctl(ADD|MOD|DEL) / epoll_wait / struct epoll_event
 *   EPOLLIN / EPOLLOUT / EPOLLERR / EPOLLHUP / EPOLLET / EPOLL_CLOEXEC
 *
 * 语义映射：
 *   epoll_fd            → kqueue()
 *   EPOLLIN/OUT         → EVFILT_READ / EVFILT_WRITE（两个独立 kevent）
 *   EPOLLET            → EV_CLEAR（边沿触发）
 *   epoll_event.data   → 存入 kevent.udata，wait 时原样回填
 *   EPOLLERR/HUP       → kevent 的 EV_EOF / EV_ERROR 回报到 events
 * ========================================================================== */

#if !defined(__APPLE__)
#error "compat/sys/epoll.h is macOS-only; on Linux the system <sys/epoll.h> must be used"
#endif

#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 事件位（值与 Linux 一致不重要，仅需自洽）──
 * 用 #define 而非 enum：EPOLLET/EPOLLONESHOT 取高位 (1u<<31/30) 超出 int enum
 * 底层类型范围，C++ 下会编译报错；宏是无类型的 uint 常量，安全。 */
#define EPOLLIN      0x001u
#define EPOLLPRI     0x002u
#define EPOLLOUT     0x004u
#define EPOLLERR     0x008u
#define EPOLLHUP     0x010u
#define EPOLLRDHUP   0x2000u
#define EPOLLET      (1u << 31)
#define EPOLLONESHOT (1u << 30)

/* epoll_create1 flags */
#ifndef EPOLL_CLOEXEC
#define EPOLL_CLOEXEC 02000000
#endif

/* epoll_ctl ops */
#define EPOLL_CTL_ADD 1
#define EPOLL_CTL_DEL 2
#define EPOLL_CTL_MOD 3

typedef union epoll_data {
    void*    ptr;
    int      fd;
    uint32_t u32;
    uint64_t u64;
} epoll_data_t;

struct epoll_event {
    uint32_t     events;   /* EPOLLIN 等位组合 */
    epoll_data_t data;
};

/* ── 实现：kqueue 平替 ─────────────────────────────────────────────────────
 * 全部 static inline，避免多 TU 重复符号；无状态，data 载荷靠 kevent.udata。 */

static inline int epoll_create1(int flags) {
    int kq = kqueue();
    if (kq < 0) return -1;
    if (flags & EPOLL_CLOEXEC) {
        int fl = fcntl(kq, F_GETFD);
        if (fl != -1) fcntl(kq, F_SETFD, fl | FD_CLOEXEC);
    }
    return kq;
}

static inline int epoll_create(int size) {
    (void)size;
    return epoll_create1(0);
}

static inline int epoll_ctl(int epfd, int op, int fd, struct epoll_event* ev) {
    struct kevent kev[2];
    int n = 0;
    unsigned short flags = 0;

    if (op == EPOLL_CTL_ADD || op == EPOLL_CTL_MOD) {
        flags = EV_ADD;
        if (ev && (ev->events & EPOLLET)) flags |= EV_CLEAR;
        if (ev && (ev->events & EPOLLONESHOT)) flags |= EV_ONESHOT;
    } else if (op == EPOLL_CTL_DEL) {
        flags = EV_DELETE;
    } else {
        errno = EINVAL;
        return -1;
    }

    /* udata 携带 epoll_data（存 ptr；若用户只用 fd，则回填时用户可读 .fd） */
    void* udata = NULL;
    uint32_t want = ev ? ev->events : 0;
    if (ev) {
        /* 把整个 epoll_data 压进指针宽度（union 最大 8 字节，指针 64 位可容纳） */
        udata = ev->data.ptr;
    }

    if (op == EPOLL_CTL_DEL) {
        /* 删除读写两个 filter；忽略 ENOENT（未注册的那个） */
        EV_SET(&kev[0], fd, EVFILT_READ,  EV_DELETE, 0, 0, NULL);
        EV_SET(&kev[1], fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
        (void)kevent(epfd, kev, 2, NULL, 0, NULL);
        return 0;
    }

    /* MOD 语义：先按需增删。这里简化为按 want 重新注册读写 filter。 */
    if (want & EPOLLIN) {
        EV_SET(&kev[n++], fd, EVFILT_READ, flags, 0, 0, udata);
    } else if (op == EPOLL_CTL_MOD) {
        EV_SET(&kev[n++], fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (want & EPOLLOUT) {
        EV_SET(&kev[n++], fd, EVFILT_WRITE, flags, 0, 0, udata);
    } else if (op == EPOLL_CTL_MOD) {
        EV_SET(&kev[n++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (n == 0) return 0;

    int r = kevent(epfd, kev, n, NULL, 0, NULL);
    if (r < 0) {
        /* MOD 下删除不存在的 filter 会 ENOENT，视为成功 */
        if (op == EPOLL_CTL_MOD && errno == ENOENT) return 0;
        return -1;
    }
    return 0;
}

static inline int epoll_wait(int epfd, struct epoll_event* events,
                             int maxevents, int timeout_ms) {
    if (maxevents <= 0) { errno = EINVAL; return -1; }

    struct kevent* kev = (struct kevent*)calloc((size_t)maxevents, sizeof(struct kevent));
    if (!kev) { errno = ENOMEM; return -1; }

    struct timespec ts, *pts = NULL;
    if (timeout_ms >= 0) {
        ts.tv_sec  = timeout_ms / 1000;
        ts.tv_nsec = (long)(timeout_ms % 1000) * 1000000L;
        pts = &ts;
    }

    int n = kevent(epfd, NULL, 0, kev, maxevents, pts);
    if (n < 0) { free(kev); return -1; }

    for (int i = 0; i < n; i++) {
        uint32_t e = 0;
        if (kev[i].filter == EVFILT_READ)  e |= EPOLLIN;
        if (kev[i].filter == EVFILT_WRITE) e |= EPOLLOUT;
        if (kev[i].flags & EV_EOF)   e |= EPOLLHUP;
        if (kev[i].flags & EV_ERROR) e |= EPOLLERR;
        events[i].events   = e;
        events[i].data.ptr = kev[i].udata;
    }
    free(kev);
    return n;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOWENGINE_COMPAT_SYS_EPOLL_H */
