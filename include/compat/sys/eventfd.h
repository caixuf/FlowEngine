#ifndef FLOWENGINE_COMPAT_SYS_EVENTFD_H
#define FLOWENGINE_COMPAT_SYS_EVENTFD_H

/* =============================================================================
 * sys/eventfd.h — macOS 平替：用 self-pipe 实现 Linux eventfd 子集
 *
 * macOS 无 eventfd（也无 <sys/eventfd.h>）。flowcoro/net 仅把 eventfd 当作
 * 「可加入 epoll/kqueue 监听的线程唤醒 fd」用（wakeup_fd），语义简单：
 *   - eventfd_write(fd, 1)  → 让监听方 readable
 *   - eventfd_read(fd,&v)   → 清空并读回累计计数
 * 用一对 pipe(2) 平替：write 端写 1 字节，read 端读空。计数语义用「读到的字节数」
 * 近似（flowcoro 只判 readable / 清空，不依赖精确累加值）。
 *
 * 仅在 APPLE 平台通过 `-Iinclude/compat` 命中；Linux 用系统 eventfd，零变化。
 * ========================================================================== */

#if !defined(__APPLE__)
#error "compat/sys/eventfd.h is macOS-only; on Linux the system eventfd must be used"
#endif

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t eventfd_t;

/* eventfd flags（值仅需自洽） */
#ifndef EFD_SEMAPHORE
#define EFD_SEMAPHORE 0x0001
#endif
#ifndef EFD_CLOEXEC
#define EFD_CLOEXEC   02000000
#endif
#ifndef EFD_NONBLOCK
#define EFD_NONBLOCK  04000
#endif

/* macOS 无法让单个 fd 同时可读可写做计数器，用 pipe。为让「同一个 int fd」既能
 * 被 epoll(kqueue) 监听、又能 read/write，这里返回 pipe 的读端，并把写端记到一个
 * 小型静态映射表里。flowcoro 的用法是：epoll 监听该 fd（读端），eventfd_write 触发。 */

#define FLOW_EVENTFD_MAX 256
struct flow_eventfd_pair { int rfd; int wfd; };
static struct flow_eventfd_pair flow_eventfd_table[FLOW_EVENTFD_MAX];
static int flow_eventfd_table_init = 0;

static inline void flow_eventfd_table_ensure(void) {
    if (flow_eventfd_table_init) return;
    for (int i = 0; i < FLOW_EVENTFD_MAX; i++) {
        flow_eventfd_table[i].rfd = -1;
        flow_eventfd_table[i].wfd = -1;
    }
    flow_eventfd_table_init = 1;
}

static inline int flow_eventfd_writefd(int rfd) {
    flow_eventfd_table_ensure();
    for (int i = 0; i < FLOW_EVENTFD_MAX; i++)
        if (flow_eventfd_table[i].rfd == rfd) return flow_eventfd_table[i].wfd;
    return -1;
}

static inline int eventfd(unsigned int initval, int flags) {
    (void)initval;
    flow_eventfd_table_ensure();
    int fds[2];
    if (pipe(fds) != 0) return -1;

    if (flags & EFD_NONBLOCK) {
        int fl0 = fcntl(fds[0], F_GETFL); if (fl0 != -1) fcntl(fds[0], F_SETFL, fl0 | O_NONBLOCK);
        int fl1 = fcntl(fds[1], F_GETFL); if (fl1 != -1) fcntl(fds[1], F_SETFL, fl1 | O_NONBLOCK);
    }
    if (flags & EFD_CLOEXEC) {
        int fd0 = fcntl(fds[0], F_GETFD); if (fd0 != -1) fcntl(fds[0], F_SETFD, fd0 | FD_CLOEXEC);
        int fd1 = fcntl(fds[1], F_GETFD); if (fd1 != -1) fcntl(fds[1], F_SETFD, fd1 | FD_CLOEXEC);
    }

    for (int i = 0; i < FLOW_EVENTFD_MAX; i++) {
        if (flow_eventfd_table[i].rfd == -1) {
            flow_eventfd_table[i].rfd = fds[0];
            flow_eventfd_table[i].wfd = fds[1];
            return fds[0];   /* 返回读端作为「eventfd」；写靠表里的写端 */
        }
    }
    /* 表满：回退——关掉写端，读端仍可用但不可写唤醒 */
    close(fds[1]);
    return fds[0];
}

static inline int eventfd_write(int fd, eventfd_t value) {
    int wfd = flow_eventfd_writefd(fd);
    if (wfd < 0) { errno = EBADF; return -1; }
    /* 写 value 个字节里的 1 个即可让对端 readable；语义上写 1 字节代表一次唤醒 */
    (void)value;
    uint8_t one = 1;
    ssize_t w = write(wfd, &one, 1);
    if (w < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; /* 管道已满=已可读 */
        return -1;
    }
    return 0;
}

static inline int eventfd_read(int fd, eventfd_t* value) {
    uint8_t buf[64];
    ssize_t total = 0, r;
    /* 尽量读空（非阻塞时读到 EAGAIN 为止），累计字节数近似 counter 值 */
    while ((r = read(fd, buf, sizeof(buf))) > 0) {
        total += r;
        if (r < (ssize_t)sizeof(buf)) break;
    }
    if (total == 0 && r < 0 &&
        !(errno == EAGAIN || errno == EWOULDBLOCK)) {
        return -1;
    }
    if (value) *value = (eventfd_t)(total > 0 ? total : 0);
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOWENGINE_COMPAT_SYS_EVENTFD_H */
