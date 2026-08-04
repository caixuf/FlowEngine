#ifndef FLOWENGINE_COMPAT_WIN_SYS_IOCTL_H
#define FLOWENGINE_COMPAT_WIN_SYS_IOCTL_H

#include <errno.h>

static inline int ioctl(int fd, unsigned long request, ...) {
    (void)fd; (void)request;
    errno = ENOSYS;
    return -1;
}

#endif /* FLOWENGINE_COMPAT_WIN_SYS_IOCTL_H */
