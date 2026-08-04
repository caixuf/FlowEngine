#ifndef FLOWENGINE_COMPAT_WIN_TERMIOS_H
#define FLOWENGINE_COMPAT_WIN_TERMIOS_H

#include <errno.h>

typedef unsigned int tcflag_t;
typedef unsigned char cc_t;
typedef unsigned int speed_t;

struct termios {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_cc[32];
};

#define TCSANOW 0
#define TCIOFLUSH 0
#define ECHO 0x0008
#define ECHOE 0x0010
#define ECHOK 0x0020
#define ECHONL 0x0040
#define ICANON 0x0002
#define ISIG 0x0001
#define IXON 0x0200
#define IXOFF 0x0400
#define ICRNL 0x0100
#define INLCR 0x0040
#define IGNCR 0x0080
#define OPOST 0x0001
#define TCSAFLUSH 2
#define VMIN 6
#define VTIME 5

static inline int tcgetattr(int fd, struct termios* t) { (void)fd; (void)t; errno = ENOSYS; return -1; }
static inline int tcsetattr(int fd, int actions, const struct termios* t) { (void)fd; (void)actions; (void)t; errno = ENOSYS; return -1; }
static inline int tcflush(int fd, int queue_selector) { (void)fd; (void)queue_selector; return 0; }
static inline void cfmakeraw(struct termios* t) { (void)t; }
static inline int cfsetispeed(struct termios* t, speed_t speed) { (void)t; (void)speed; return 0; }
static inline int cfsetospeed(struct termios* t, speed_t speed) { (void)t; (void)speed; return 0; }

#endif /* FLOWENGINE_COMPAT_WIN_TERMIOS_H */
