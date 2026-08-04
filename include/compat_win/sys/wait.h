#ifndef FLOWENGINE_COMPAT_WIN_SYS_WAIT_H
#define FLOWENGINE_COMPAT_WIN_SYS_WAIT_H

#define WNOHANG 1
#define WIFEXITED(status) 1
#define WEXITSTATUS(status) (status)

static inline int waitpid(int pid, int* status, int options) {
    (void)pid; (void)status; (void)options;
    return -1;
}

#endif /* FLOWENGINE_COMPAT_WIN_SYS_WAIT_H */
