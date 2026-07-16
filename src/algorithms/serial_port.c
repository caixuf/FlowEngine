/**
 * serial_port.c — 串口抽象层实现（POSIX termios）
 *
 * 仅 Linux 实现。非 Linux 提供 stub 返回 NULL/-1，让驱动节点能编译。
 */

#include "serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef __linux__
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>

struct SerialPort {
    int fd;
    /* 行读取缓冲：termios read 是流式的，一次 read 可能读到多行或半行，
     * 用 ring buffer 缓存剩余字节，保证 read_line 能按 '\n' 切分。 */
    char    rbuf[2048];
    size_t  rlen;   /* rbuf 中已缓存的有效字节数 */
};

/* 波特率常量映射 */
static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
#ifdef B230400
        case 230400:  return B230400;
#endif
#ifdef B460800
        case 460800:  return B460800;
#endif
#ifdef B921600
        case 921600:  return B921600;
#endif
        default:      return B9600;  /* 未知波特率降级到 9600，调用方应检查 */
    }
}
#endif /* __linux__ */

SerialPort* serial_open(const char* dev, int baud) {
#ifdef __linux__
    if (!dev) { errno = EINVAL; return NULL; }
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return NULL;

    /* 取消 O_NONBLOCK，改用 select 做超时控制 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return NULL; }

    /* 配置 raw 模式（cfmakeraw 等价） */
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;   /* 非阻塞 read，由 select 控制超时 */
    tty.c_cc[VTIME] = 0;
    tty.c_cflag &= ~PARENB;   /* 无校验 */
    tty.c_cflag &= ~CSTOPB;   /* 1 停止位 */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       /* 8 数据位 */
    tty.c_cflag |= CLOCAL | CREAD;  /* 启用接收，无 modem 控制 */

    cfsetispeed(&tty, baud_to_speed(baud));
    cfsetospeed(&tty, baud_to_speed(baud));

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return NULL; }
    tcflush(fd, TCIOFLUSH);  /* 清空内核缓冲区里的脏数据 */

    SerialPort* sp = (SerialPort*)calloc(1, sizeof(SerialPort));
    if (!sp) { close(fd); errno = ENOMEM; return NULL; }
    sp->fd = fd;
    sp->rlen = 0;
    return sp;
#else
    (void)dev; (void)baud;
    errno = ENOSYS;
    return NULL;
#endif
}

int serial_read_line(SerialPort* sp, char* buf, size_t bufsize, int timeout_ms) {
#ifdef __linux__
    if (!sp || sp->fd < 0 || !buf || bufsize < 2) { errno = EINVAL; return -1; }

    size_t out = 0;
    while (out < bufsize - 1) {
        /* 先从缓存里找 '\n' */
        while (sp->rlen > 0) {
            char c = sp->rbuf[0];
            sp->rlen--;
            memmove(sp->rbuf, sp->rbuf + 1, sp->rlen);  /* 弹出首字节 */
            buf[out++] = c;
            if (c == '\n' || out >= bufsize - 1) {
                buf[out] = '\0';
                return (int)out;
            }
        }
        /* 缓存空，select 等待数据 */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sp->fd, &rfds);
        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        int rv = select(sp->fd + 1, &rfds, NULL, NULL, timeout_ms > 0 ? &tv : NULL);
        if (rv < 0) {
            if (errno == EINTR) continue;  /* 被信号中断，重试 */
            return -1;
        }
        if (rv == 0) {
            /* 超时：若已读到部分数据则返回，否则返回 0 */
            if (out > 0) { buf[out] = '\0'; return (int)out; }
            return 0;
        }
        /* 有数据，读入缓存 */
        ssize_t n = read(sp->fd, sp->rbuf, sizeof(sp->rbuf));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return -1;
        }
        if (n == 0) return -1;  /* 设备关闭 */
        sp->rlen = (size_t)n;
    }
    buf[out] = '\0';
    return (int)out;
#else
    (void)sp; (void)buf; (void)bufsize; (void)timeout_ms;
    return -1;
#endif
}

int serial_read(SerialPort* sp, void* buf, size_t bufsize, int timeout_ms) {
#ifdef __linux__
    if (!sp || sp->fd < 0 || !buf || bufsize == 0) { errno = EINVAL; return -1; }
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sp->fd, &rfds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int rv = select(sp->fd + 1, &rfds, NULL, NULL, timeout_ms > 0 ? &tv : NULL);
    if (rv < 0) { return errno == EINTR ? 0 : -1; }
    if (rv == 0) return 0;
    ssize_t n = read(sp->fd, buf, bufsize);
    if (n < 0) return (errno == EINTR || errno == EAGAIN) ? 0 : -1;
    return (int)n;
#else
    (void)sp; (void)buf; (void)bufsize; (void)timeout_ms;
    return -1;
#endif
}

int serial_write(SerialPort* sp, const void* data, size_t len) {
#ifdef __linux__
    if (!sp || sp->fd < 0 || !data) { errno = EINVAL; return -1; }
    ssize_t n = write(sp->fd, data, len);
    if (n < 0) return -1;
    return (int)n;
#else
    (void)sp; (void)data; (void)len;
    return -1;
#endif
}

void serial_flush(SerialPort* sp) {
#ifdef __linux__
    if (sp && sp->fd >= 0) {
        tcflush(sp->fd, TCIOFLUSH);
        sp->rlen = 0;
    }
#else
    (void)sp;
#endif
}

void serial_close(SerialPort* sp) {
#ifdef __linux__
    if (sp) {
        if (sp->fd >= 0) close(sp->fd);
        free(sp);
    }
#else
    (void)sp;
#endif
}
