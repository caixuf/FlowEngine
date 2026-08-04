#ifndef FLOWENGINE_COMPAT_WIN_SYS_SOCKET_H
#define FLOWENGINE_COMPAT_WIN_SYS_SOCKET_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdint.h>
#include <errno.h>

typedef int socklen_t;
typedef intptr_t ssize_t;

#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0
#endif

static inline int flow_wsa_errno(void) {
    switch (WSAGetLastError()) {
        case WSAEWOULDBLOCK: return EWOULDBLOCK;
        case WSAEINTR: return EINTR;
        case WSAECONNRESET: return ECONNRESET;
        case WSAECONNREFUSED: return ECONNREFUSED;
        case WSAEADDRINUSE: return EADDRINUSE;
        default: return EIO;
    }
}

static inline void flow_winsock_init_once(void) {
    static volatile LONG done = 0;
    if (InterlockedCompareExchange(&done, 1, 0) == 0) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    }
}

static inline int flow_socket(int af, int type, int protocol) {
    flow_winsock_init_once();
    SOCKET s = socket(af, type, protocol);
    if (s == INVALID_SOCKET) { errno = flow_wsa_errno(); return -1; }
    return (int)s;
}
static inline int flow_bind(int s, const struct sockaddr* name, socklen_t namelen) {
    int rc = bind((SOCKET)s, name, namelen);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return rc;
}
static inline int flow_listen(int s, int backlog) {
    int rc = listen((SOCKET)s, backlog);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return rc;
}
static inline int flow_accept(int s, struct sockaddr* addr, socklen_t* addrlen) {
    SOCKET c = accept((SOCKET)s, addr, addrlen);
    if (c == INVALID_SOCKET) { errno = flow_wsa_errno(); return -1; }
    return (int)c;
}
static inline int flow_connect(int s, const struct sockaddr* name, socklen_t namelen) {
    int rc = connect((SOCKET)s, name, namelen);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return rc;
}
static inline ssize_t flow_send(int s, const void* buf, size_t len, int flags) {
    int rc = send((SOCKET)s, (const char*)buf, (int)len, flags);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return (ssize_t)rc;
}
static inline ssize_t flow_recv(int s, void* buf, size_t len, int flags) {
    int rc = recv((SOCKET)s, (char*)buf, (int)len, flags);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return (ssize_t)rc;
}
static inline ssize_t flow_recvfrom(int s, void* buf, size_t len, int flags,
                                    struct sockaddr* from, socklen_t* fromlen) {
    int rc = recvfrom((SOCKET)s, (char*)buf, (int)len, flags, from, fromlen);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return (ssize_t)rc;
}
static inline ssize_t flow_sendto(int s, const void* buf, size_t len, int flags,
                                  const struct sockaddr* to, socklen_t tolen) {
    int rc = sendto((SOCKET)s, (const char*)buf, (int)len, flags, to, tolen);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return (ssize_t)rc;
}
static inline int flow_setsockopt(int s, int level, int optname, const void* optval, socklen_t optlen) {
    int rc = setsockopt((SOCKET)s, level, optname, (const char*)optval, optlen);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return rc;
}
static inline int flow_shutdown(int s, int how) {
    int rc = shutdown((SOCKET)s, how);
    if (rc == SOCKET_ERROR) { errno = flow_wsa_errno(); return -1; }
    return rc;
}

#define socket    flow_socket
#define bind      flow_bind
#define listen    flow_listen
#define accept    flow_accept
#define connect   flow_connect
#define send      flow_send
#define recv      flow_recv
#define recvfrom  flow_recvfrom
#define sendto    flow_sendto
#define setsockopt flow_setsockopt
#define shutdown  flow_shutdown

#endif /* FLOWENGINE_COMPAT_WIN_SYS_SOCKET_H */
