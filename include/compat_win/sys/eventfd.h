#ifndef FLOWENGINE_COMPAT_WIN_SYS_EVENTFD_H
#define FLOWENGINE_COMPAT_WIN_SYS_EVENTFD_H

#include <stdint.h>
#include <errno.h>
#include <winsock2.h>
#include <sys/socket.h>

typedef uint64_t eventfd_t;

#ifndef EFD_NONBLOCK
#define EFD_NONBLOCK 0x800
#endif
#ifndef EFD_CLOEXEC
#define EFD_CLOEXEC 0x80000
#endif
#ifndef EFD_SEMAPHORE
#define EFD_SEMAPHORE 0x1
#endif

typedef struct {
    SOCKET r;
    SOCKET w;
} flow_eventfd_pair;

#define FLOW_WIN_EVENTFD_MAX 256
static flow_eventfd_pair flow_win_eventfds[FLOW_WIN_EVENTFD_MAX];

static inline void flow_eventfd_init_table(void) {
    static int init = 0;
    if (init) return;
    for (int i = 0; i < FLOW_WIN_EVENTFD_MAX; ++i) {
        flow_win_eventfds[i].r = INVALID_SOCKET;
        flow_win_eventfds[i].w = INVALID_SOCKET;
    }
    init = 1;
}

static inline SOCKET flow_eventfd_write_socket(int fd) {
    flow_eventfd_init_table();
    for (int i = 0; i < FLOW_WIN_EVENTFD_MAX; ++i)
        if ((int)flow_win_eventfds[i].r == fd) return flow_win_eventfds[i].w;
    return INVALID_SOCKET;
}

static inline int eventfd(unsigned int initval, int flags) {
    (void)initval;
    flow_winsock_init_once();
    flow_eventfd_init_table();

    SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) { errno = flow_wsa_errno(); return -1; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(listener, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR ||
        listen(listener, 1) == SOCKET_ERROR) {
        errno = flow_wsa_errno();
        closesocket(listener);
        return -1;
    }
    int len = sizeof(addr);
    if (getsockname(listener, (struct sockaddr*)&addr, &len) == SOCKET_ERROR) {
        errno = flow_wsa_errno();
        closesocket(listener);
        return -1;
    }
    SOCKET w = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (w == INVALID_SOCKET || connect(w, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        errno = flow_wsa_errno();
        if (w != INVALID_SOCKET) closesocket(w);
        closesocket(listener);
        return -1;
    }
    SOCKET r = accept(listener, NULL, NULL);
    closesocket(listener);
    if (r == INVALID_SOCKET) {
        errno = flow_wsa_errno();
        closesocket(w);
        return -1;
    }
    if (flags & EFD_NONBLOCK) {
        u_long mode = 1;
        ioctlsocket(r, FIONBIO, &mode);
        ioctlsocket(w, FIONBIO, &mode);
    }
    for (int i = 0; i < FLOW_WIN_EVENTFD_MAX; ++i) {
        if (flow_win_eventfds[i].r == INVALID_SOCKET) {
            flow_win_eventfds[i].r = r;
            flow_win_eventfds[i].w = w;
            return (int)r;
        }
    }
    closesocket(r);
    closesocket(w);
    errno = EMFILE;
    return -1;
}

static inline int eventfd_write(int fd, eventfd_t value) {
    (void)value;
    SOCKET w = flow_eventfd_write_socket(fd);
    if (w == INVALID_SOCKET) { errno = EBADF; return -1; }
    char c = 1;
    int rc = send(w, &c, 1, 0);
    if (rc == SOCKET_ERROR) {
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK) return 0;
        errno = flow_wsa_errno();
        return -1;
    }
    return 0;
}

static inline int eventfd_read(int fd, eventfd_t* value) {
    char buf[64];
    eventfd_t total = 0;
    for (;;) {
        int n = recv((SOCKET)fd, buf, sizeof(buf), 0);
        if (n > 0) { total += (eventfd_t)n; if (n < (int)sizeof(buf)) break; continue; }
        if (n == 0) break;
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK) break;
        errno = flow_wsa_errno();
        return -1;
    }
    if (value) *value = total;
    return 0;
}

#endif /* FLOWENGINE_COMPAT_WIN_SYS_EVENTFD_H */
