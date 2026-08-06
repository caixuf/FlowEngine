#ifndef FLOWENGINE_COMPAT_WIN_SYS_EPOLL_H
#define FLOWENGINE_COMPAT_WIN_SYS_EPOLL_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <winsock2.h>
#include <sys/socket.h>

/* wingdi.h (pulled in by windows.h) defines ERROR=0, READ, WRITE etc. as macros;
   undef them to avoid conflicts with enum member names in C++ code. */
#ifdef ERROR
#undef ERROR
#endif
#ifdef READ
#undef READ
#endif
#ifdef WRITE
#undef WRITE
#endif

#define EPOLLIN      0x001u
#define EPOLLPRI     0x002u
#define EPOLLOUT     0x004u
#define EPOLLERR     0x008u
#define EPOLLHUP     0x010u
#define EPOLLRDHUP   0x2000u
#define EPOLLET      (1u << 31)
#define EPOLLONESHOT (1u << 30)
#define EPOLL_CLOEXEC 0x80000

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
    uint32_t events;
    epoll_data_t data;
};

typedef struct {
    int fd;
    uint32_t events;
    epoll_data_t data;
    int active;
} flow_epoll_item;

typedef struct {
    flow_epoll_item items[FD_SETSIZE];
} flow_epoll_set;

#define FLOW_WIN_EPOLL_MAX 64
static flow_epoll_set* flow_epoll_sets[FLOW_WIN_EPOLL_MAX];

static inline int epoll_create1(int flags) {
    (void)flags;
    flow_winsock_init_once();
    for (int i = 1; i < FLOW_WIN_EPOLL_MAX; ++i) {
        if (!flow_epoll_sets[i]) {
            flow_epoll_sets[i] = (flow_epoll_set*)calloc(1, sizeof(flow_epoll_set));
            return flow_epoll_sets[i] ? i : -1;
        }
    }
    errno = EMFILE;
    return -1;
}

static inline int epoll_create(int size) {
    (void)size;
    return epoll_create1(0);
}

static inline int epoll_ctl(int epfd, int op, int fd, struct epoll_event* ev) {
    if (epfd <= 0 || epfd >= FLOW_WIN_EPOLL_MAX || !flow_epoll_sets[epfd]) {
        errno = EBADF;
        return -1;
    }
    flow_epoll_set* set = flow_epoll_sets[epfd];
    int free_idx = -1;
    int idx = -1;
    for (int i = 0; i < FD_SETSIZE; ++i) {
        if (set->items[i].active && set->items[i].fd == fd) idx = i;
        if (!set->items[i].active && free_idx < 0) free_idx = i;
    }
    if (op == EPOLL_CTL_DEL) {
        if (idx >= 0) memset(&set->items[idx], 0, sizeof(set->items[idx]));
        return 0;
    }
    if (!ev) { errno = EINVAL; return -1; }
    if (idx < 0) idx = free_idx;
    if (idx < 0) { errno = ENOSPC; return -1; }
    set->items[idx].fd = fd;
    set->items[idx].events = ev->events;
    set->items[idx].data = ev->data;
    set->items[idx].active = 1;
    return 0;
}

static inline int epoll_wait(int epfd, struct epoll_event* events,
                             int maxevents, int timeout_ms) {
    if (epfd <= 0 || epfd >= FLOW_WIN_EPOLL_MAX || !flow_epoll_sets[epfd] ||
        !events || maxevents <= 0) {
        errno = EINVAL;
        return -1;
    }
    flow_epoll_set* set = flow_epoll_sets[epfd];
    fd_set rfds;
    fd_set wfds;
    fd_set efds;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_ZERO(&efds);
    for (int i = 0; i < FD_SETSIZE; ++i) {
        if (!set->items[i].active) continue;
        SOCKET s = (SOCKET)set->items[i].fd;
        if (set->items[i].events & EPOLLIN) FD_SET(s, &rfds);
        if (set->items[i].events & EPOLLOUT) FD_SET(s, &wfds);
        FD_SET(s, &efds);
    }
    struct timeval tv;
    struct timeval* ptv = NULL;
    if (timeout_ms >= 0) {
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        ptv = &tv;
    }
    int rc = select(0, &rfds, &wfds, &efds, ptv);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    int out = 0;
    for (int i = 0; i < FD_SETSIZE && out < maxevents; ++i) {
        if (!set->items[i].active) continue;
        SOCKET s = (SOCKET)set->items[i].fd;
        uint32_t e = 0;
        if (FD_ISSET(s, &rfds)) e |= EPOLLIN;
        if (FD_ISSET(s, &wfds)) e |= EPOLLOUT;
        if (FD_ISSET(s, &efds)) e |= EPOLLERR;
        if (!e) continue;
        events[out].events = e;
        events[out].data = set->items[i].data;
        ++out;
    }
    return out;
}

static inline int epoll_close(int epfd) {
    if (epfd <= 0 || epfd >= FLOW_WIN_EPOLL_MAX) return -1;
    free(flow_epoll_sets[epfd]);
    flow_epoll_sets[epfd] = NULL;
    return 0;
}

#endif /* FLOWENGINE_COMPAT_WIN_SYS_EPOLL_H */
