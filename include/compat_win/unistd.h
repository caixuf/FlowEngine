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

/* ssize_t / pid_t:MSVC 的 CRT 头不提供这两个 POSIX 类型,需自备。
 * mingw-w64 的 <sys/types.h> 已定义二者,重复 typedef 会 "conflicting types"。
 * 故仅在 MSVC 下自定义;mingw 直接用工具链头。 */
#if defined(_MSC_VER)
typedef intptr_t ssize_t;
typedef int pid_t;
#else
#include <sys/types.h>
#endif

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
/* socket 感知的 read/write/close:Windows 上 socket 句柄不能用 CRT _read/_write/
 * _close 直接操作,需先走 recv/send/closesocket,失败再回退到文件描述符 CRT 调用
 * (monitor_server 的 HTTP 客户端 fd 就是 SOCKET,serial/pipe 则是普通 fd)。
 *
 * 不能定义名为 read/write/close 的 static inline 函数 —— MSVC <io.h> 与
 * mingw <process.h>/<io.h> 均已声明同名 POSIX 函数,重定义会冲突。改为定义
 * flow_win_* 实现 + 宏改写调用点:宏在系统头 include 之后展开,把 read(...) 调用
 * 重写到 flow_win_read(...),两种工具链都无符号冲突。 */
static inline int flow_win_read(int fd, void* buf, unsigned int len) {
    int n = recv((SOCKET)fd, (char*)buf, (int)len, 0);
    if (n >= 0) return n;
    return _read(fd, buf, len);
}
static inline int flow_win_write(int fd, const void* buf, unsigned int len) {
    int n = send((SOCKET)fd, (const char*)buf, (int)len, 0);
    if (n >= 0) return n;
    return _write(fd, buf, len);
}
static inline int flow_win_close(int fd) {
    if (fd >= 0 && closesocket((SOCKET)fd) == 0) return 0;
    return _close(fd);
}
#define read(fd, buf, len)  flow_win_read((fd), (buf), (len))
#define write(fd, buf, len) flow_win_write((fd), (buf), (len))
#define close(fd)           flow_win_close((fd))

#ifndef sleep
#define sleep(sec)   (flow_win_sleep_seconds((unsigned int)(sec)), 0)
#endif
#ifndef usleep
#define usleep(usec) flow_win_usleep((unsigned int)(usec))
#endif

/* strtok_r is POSIX; MSVC provides strtok_s with compatible signature */
#ifndef strtok_r
#define strtok_r strtok_s
#endif

#endif /* FLOWENGINE_COMPAT_WIN_UNISTD_H */
