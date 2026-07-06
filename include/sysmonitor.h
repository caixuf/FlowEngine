#ifndef SYSMONITOR_H
#define SYSMONITOR_H

/**
 * @file sysmonitor.h
 * @brief 系统资源监控 API
 *
 * 纯 /proc 读取，无外部依赖。采集：
 *   - 全系统 CPU 利用率（差分）
 *   - 内存（总量/已用/缓存/可用）
 *   - 磁盘 I/O（read_bytes / write_bytes，差分速率）
 *   - 系统负载（1/5/15 分钟）
 *   - 系统运行时间
 *   - 当前进程线程列表（/proc/self/task/）及各线程 CPU 占用（差分）
 *
 * 典型用法：
 *   SysMonitor* sm = sysmonitor_create();
 *   SysMonitorSnapshot snap;
 *   sysmonitor_snapshot(sm, &snap);
 *   // ... use snap ...
 *   sysmonitor_destroy(sm);
 */

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 每线程快照 ─────────────────────────────────────────────── */

#define SYSMON_THREAD_NAME_MAX 32

typedef struct {
    pid_t    tid;                            /**< 线程 ID (gettid) */
    char     name[SYSMON_THREAD_NAME_MAX];   /**< /proc/self/task/<tid>/comm */
    double   cpu_pct;                        /**< 差分 CPU 占用率 (0–100 × N_CPU) */
    char     state;                          /**< 线程状态 (R/S/D/Z/...) */
    uint64_t utime_ticks;                    /**< 累计用户态 ticks (原始) */
    uint64_t stime_ticks;                    /**< 累计内核态 ticks (原始) */
} SysMonitorThreadSnapshot;

/* ── 进程+系统整体快照 ───────────────────────────────────────── */

#define SYSMON_MAX_THREADS 128

typedef struct {
    /* ── CPU ── */
    double   cpu_user_pct;    /**< 用户态 CPU% (全系统，差分) */
    double   cpu_sys_pct;     /**< 内核态 CPU% (全系统，差分) */
    double   cpu_idle_pct;    /**< 空闲 CPU%  (全系统，差分) */
    double   cpu_total_pct;   /**< 总使用率 = user + sys + iowait */
    double   cpu_iowait_pct;  /**< I/O 等待 CPU% */
    int      cpu_count;       /**< 逻辑 CPU 核心数 */

    /* ── 内存 (kB) ── */
    uint64_t mem_total_kb;    /**< 系统总内存 */
    uint64_t mem_used_kb;     /**< 已用内存（total - available） */
    uint64_t mem_free_kb;     /**< 空闲内存 */
    uint64_t mem_cached_kb;   /**< Page cache */
    uint64_t mem_available_kb;/**< 可用内存 */
    double   mem_used_pct;    /**< 内存使用率 % */

    /* ── 进程自身内存 ── */
    uint64_t proc_rss_kb;     /**< 进程 RSS (kB) */
    uint64_t proc_vms_kb;     /**< 进程 VmSize (kB) */

    /* ── 磁盘 I/O (速率, bytes/s) ── */
    double   disk_read_bps;   /**< 磁盘读取速率 */
    double   disk_write_bps;  /**< 磁盘写入速率 */

    /* ── 系统负载 ── */
    double   load1;           /**< 1 分钟平均负载 */
    double   load5;           /**< 5 分钟平均负载 */
    double   load15;          /**< 15 分钟平均负载 */

    /* ── 运行时间 ── */
    double   uptime_sec;      /**< 系统开机时长（秒） */

    /* ── 线程列表 ── */
    int                      thread_count;                    /**< 实际线程数 */
    SysMonitorThreadSnapshot threads[SYSMON_MAX_THREADS];     /**< 各线程快照 */

    /* ── 时间戳 ── */
    uint64_t timestamp_us;    /**< 快照采集时间（单调时钟，微秒） */
} SysMonitorSnapshot;

/* ── 不透明句柄 ─────────────────────────────────────────────── */

typedef struct SysMonitor SysMonitor;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 创建监控实例。
 * 内部保存上一次的 /proc/stat、/proc/diskstats 等计数器，
 * 用于下次差分计算。
 * @return 新实例指针，失败返回 NULL。
 */
SysMonitor* sysmonitor_create(void);

/**
 * 采集一次完整快照，填充 @p out。
 * 首次调用时，CPU/IO 差分均为 0（无历史数据）。
 * @param sm   sysmonitor_create() 返回的实例
 * @param out  输出快照（由调用方提供）
 * @return 0 成功，-1 失败
 */
int sysmonitor_snapshot(SysMonitor* sm, SysMonitorSnapshot* out);

/**
 * 仅采集线程级快照（比完整快照更轻量）。
 * @param sm        实例
 * @param threads   输出数组，长度至少为 @p max_threads
 * @param max_threads  数组容量
 * @return 实际写入的线程数，-1 失败
 */
int sysmonitor_thread_snapshot(SysMonitor* sm,
                               SysMonitorThreadSnapshot* threads,
                               int max_threads);

/**
 * 销毁实例并释放所有内部资源。
 */
void sysmonitor_destroy(SysMonitor* sm);

#ifdef __cplusplus
}
#endif
#endif /* SYSMONITOR_H */
