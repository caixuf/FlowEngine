#ifndef FLOWENGINE_COMPAT_WIN_FCNTL_H
#define FLOWENGINE_COMPAT_WIN_FCNTL_H

#include <winsock2.h>
#include <io.h>

#ifndef O_RDONLY
#define O_RDONLY 0x0000
#endif
#ifndef O_WRONLY
#define O_WRONLY 0x0001
#endif
#ifndef O_RDWR
#define O_RDWR   0x0002
#endif
#ifndef O_CREAT
#define O_CREAT  0x0100
#endif
#ifndef O_TRUNC
#define O_TRUNC  0x0200
#endif
#ifndef O_APPEND
#define O_APPEND 0x0008
#endif
#ifndef O_NOCTTY
#define O_NOCTTY 0
#endif
#ifndef F_GETFL
#define F_GETFL 3
#endif
#ifndef F_SETFL
#define F_SETFL 4
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x4000
#endif

#define open _open

static inline int flow_win_fcntl(int fd, int cmd, int flags) {
    if (cmd == F_GETFL) return 0;
    if (cmd == F_SETFL) {
        u_long mode = (flags & O_NONBLOCK) ? 1UL : 0UL;
        if (ioctlsocket((SOCKET)fd, FIONBIO, &mode) == 0) return 0;
        return 0;
    }
    return -1;
}

#define fcntl flow_win_fcntl

#endif /* FLOWENGINE_COMPAT_WIN_FCNTL_H */
