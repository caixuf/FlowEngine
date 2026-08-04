#ifndef FLOWENGINE_COMPAT_WIN_POLL_H
#define FLOWENGINE_COMPAT_WIN_POLL_H

#include <winsock2.h>

#ifndef POLLIN
#define POLLIN  0x0001
#endif
#ifndef POLLOUT
#define POLLOUT 0x0004
#endif

static inline int poll(struct pollfd* fds, unsigned long nfds, int timeout) {
    return WSAPoll((WSAPOLLFD*)fds, nfds, timeout);
}

#endif /* FLOWENGINE_COMPAT_WIN_POLL_H */
