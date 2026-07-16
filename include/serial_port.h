/**
 * serial_port.h — 串口（UART）抽象层
 *
 * 封装 POSIX termios，为 GPS/IMU/LiDAR 等串口传感器驱动提供统一接口。
 * 核心能力：打开 /dev/ttyUSB* /dev/ttyAMA* 等串口设备、配置波特率/校验、
 * 按行读取（NMEA 等行协议友好）、超时控制。
 *
 * 仅 Linux 可用（依赖 termios）。非 Linux 环境提供桩函数返回 NULL/-1，
 * 让驱动节点能编译过（运行时降级为无数据）。
 *
 * 典型用法（GPS NMEA 读取）:
 *   SerialPort* sp = serial_open("/dev/ttyUSB0", 9600);
 *   if (!sp) { ... 打开失败处理 ... }
 *   char line[256];
 *   while (running) {
 *       int n = serial_read_line(sp, line, sizeof(line), 1000);  // 1s 超时
 *       if (n > 0) {
 *           GpsData gps;
 *           nmea_parse_line(&parser, line, &gps);
 *       }
 *   }
 *   serial_close(sp);
 */

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 串口句柄（ opaque，实现见 serial_port.c ） ── */
typedef struct SerialPort SerialPort;

/* ── 波特率枚举（覆盖常见传感器用到的） ── */
typedef enum {
    SERIAL_BAUD_9600    = 9600,
    SERIAL_BAUD_19200   = 19200,
    SERIAL_BAUD_38400   = 38400,
    SERIAL_BAUD_57600   = 57600,
    SERIAL_BAUD_115200  = 115200,
    SERIAL_BAUD_230400  = 230400,
    SERIAL_BAUD_460800  = 460800,
    SERIAL_BAUD_921600  = 921600,
} SerialBaud;

/* ── API ─────────────────────────────────────────────────── */

/**
 * 打开串口设备并配置为 raw 模式（8N1，无流控）。
 * @param dev       设备路径，如 "/dev/ttyUSB0"、"/dev/ttyAMA0"
 * @param baud      波特率（SerialBaud 枚举）
 * @return 串口句柄，失败返回 NULL（errno 保留失败原因）
 */
SerialPort* serial_open(const char* dev, int baud);

/**
 * 读取一行（以 '\n' 结尾）。NMEA/GPGGA 等 ASCII 行协议友好。
 * 阻塞最多 timeout_ms 毫秒，读到 '\n' 或 buffer 满立即返回。
 * @param sp          串口句柄
 * @param buf         输出缓冲区
 * @param bufsize     缓冲区大小
 * @param timeout_ms  超时（毫秒），<=0 表示阻塞直到读到数据
 * @return >0 实际读取字节数（含 '\n'，不含末尾 '\0'）；0 超时；<0 错误/设备关闭
 */
int serial_read_line(SerialPort* sp, char* buf, size_t bufsize, int timeout_ms);

/**
 * 读取裸字节（不按行）。
 * @return >0 实际读取字节数；0 超时；<0 错误
 */
int serial_read(SerialPort* sp, void* buf, size_t bufsize, int timeout_ms);

/**
 * 写入数据。
 * @return >0 实际写入字节数；<0 错误
 */
int serial_write(SerialPort* sp, const void* data, size_t len);

/**
 * 刷新串口缓冲区（丢弃未读数据）。
 */
void serial_flush(SerialPort* sp);

/**
 * 关闭串口并释放资源。传入 NULL 安全。
 */
void serial_close(SerialPort* sp);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PORT_H */
