#ifndef FLOWENGINE_COMPAT_WIN_UNISTD_H
#define FLOWENGINE_COMPAT_WIN_UNISTD_H

#include <io.h>
#include <process.h>
#include <direct.h>
#include <stdint.h>
#include <stddef.h>
#include <winsock2.h>
#include <windows.h>

#ifndef STDIN_FILENO
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#endif

typedef intptr_t ssize_t;
typedef int pid_t;

#ifndef _SC_NPROCESSORS_ONLN
#define _SC_NPROCESSORS_ONLN 1
#endif

static inline long sysconf(int name) {
    if (name == _SC_NPROCESSORS_ONLN) {
        SYSTEM_INFO si;
        GetSystemInfo(&si);
        return (long)si.dwNumberOfProcessors;
    }
    return -1;
}

#define getpid _getpid
#define access _access
#define unlink _unlink
#define dup2   _dup2
static inline int read(int fd, void* buf, unsigned int len) {
    int n = recv((SOCKET)fd, (char*)buf, (int)len, 0);
    if (n >= 0) return n;
    return _read(fd, buf, len);
}
static inline int write(int fd, const void* buf, unsigned int len) {
    int n = send((SOCKET)fd, (const char*)buf, (int)len, 0);
    if (n >= 0) return n;
    return _write(fd, buf, len);
}

static inline int close(int fd) {
    if (fd >= 0 && closesocket((SOCKET)fd) == 0) return 0;
    return _close(fd);
}

#ifndef sleep
#define sleep(sec)   (flow_win_sleep_seconds((unsigned int)(sec)), 0)
#endif
#ifndef usleep
#define usleep(usec) flow_win_usleep((unsigned int)(usec))
#endif

#endif /* FLOWENGINE_COMPAT_WIN_UNISTD_H */
