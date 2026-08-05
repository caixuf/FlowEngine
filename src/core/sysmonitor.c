/**
 * sysmonitor.c — 系统资源监控实现 (纯 /proc 读取，无外部依赖)
 *
 * 采集内容:
 *   /proc/stat        → 全系统 CPU ticks（差分）
 *   /proc/meminfo     → 内存各字段
 *   /proc/diskstats   → 块设备读写扇区（差分）
 *   /proc/loadavg     → 1/5/15 分钟负载
 *   /proc/uptime      → 系统运行时间
 *   /proc/self/status → 进程 RSS / VmSize
 *   /proc/self/task/  → 线程列表
 *   /proc/self/task/<tid>/stat  → 单线程 utime/stime（差分）
 *   /proc/self/task/<tid>/comm  → 线程名
 */

#include "sysmonitor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_host.h>
#include <mach/task.h>
#include <mach/thread_info.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <libproc.h>
#endif

/* ── 内部辅助宏 ──────────────────────────────────────────────── */

#define PROC_STAT_PATH       "/proc/stat"
#define PROC_MEMINFO_PATH    "/proc/meminfo"
#define PROC_DISKSTATS_PATH  "/proc/diskstats"
#define PROC_LOADAVG_PATH    "/proc/loadavg"
#define PROC_UPTIME_PATH     "/proc/uptime"
#define PROC_SELF_STATUS     "/proc/self/status"
#define PROC_SELF_TASK       "/proc/self/task"

/* CPU 差分最小窗口（秒）：monitor 高频采样时，若直接对相邻两次做差分，
 * 低占用线程（1~5%）的 tick 增量会被量化成 0。改为"窗口内累计差分"，
 * 每满一个窗口才提交新基线，保证轻负载线程也显示非 0 的真实占比。 */
#define SYSMON_CPU_WINDOW_S  3.0

/* ── 内部数据结构 ─────────────────────────────────────────────── */

/* /proc/stat 单行 CPU 计数器 */
typedef struct {
    uint64_t user;
    uint64_t nice;
    uint64_t system;
    uint64_t idle;
    uint64_t iowait;
    uint64_t irq;
    uint64_t softirq;
    uint64_t steal;
} CpuRaw;

/* 每线程历史计数器（用于差分） */
typedef struct {
    pid_t    tid;
    uint64_t utime;
    uint64_t stime;
} ThreadRaw;

/* 每进程历史计数器（用于进程 CPU 差分） */
typedef struct {
    pid_t    pid;
    uint64_t utime;
    uint64_t stime;
} ProcRaw;

/* 每进程线程历史计数器（用于进程内线程 CPU 差分，键为 pid+tid） */
typedef struct {
    pid_t    pid;
    pid_t    tid;
    uint64_t utime;
    uint64_t stime;
} PThreadRaw;

/* 进程级线程差分历史容量 */
#define SYSMON_PTH_HIST_MAX (SYSMON_MAX_PROCS * SYSMON_PROC_THREADS)

/* /proc/diskstats 汇总（只关心读/写扇区，所有设备求和） */
typedef struct {
    uint64_t read_sectors;
    uint64_t write_sectors;
} DiskRaw;

/* 不透明实例 */
struct SysMonitor {
    /* 上一次采样的 CPU 计数器 */
    CpuRaw   prev_cpu;
    int      cpu_count;         /* 逻辑核数（sysconf） */
    int      has_prev_cpu;

    /* 上一次磁盘计数器 */
    DiskRaw  prev_disk;
    int      has_prev_disk;

    /* 上一次采样时间（单调时钟，us） */
    uint64_t prev_ts_us;

    /* 线程差分历史 */
    ThreadRaw  thread_hist[SYSMON_MAX_THREADS];
    int        thread_hist_count;
    uint64_t   prev_thread_ts_us;

    /* 进程级差分历史 */
    ProcRaw    proc_hist[SYSMON_MAX_PROCS];
    int        proc_hist_count;
    PThreadRaw pth_hist[SYSMON_PTH_HIST_MAX];
    int        pth_hist_count;
    uint64_t   prev_proc_ts_us;

    /* 单进程模式"以线程作进程"的独立差分历史（避免与 thread_hist 争抢时间戳） */
    ThreadRaw  proc_thread_hist[SYSMON_MAX_THREADS];
    int        proc_thread_hist_count;
    uint64_t   proc_thread_prev_ts_us;
};

/* ── 内部工具函数 ─────────────────────────────────────────────── */

static uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

/* 解析 /proc/stat 第一行 "cpu  ..." */
static int read_cpu_raw(CpuRaw* out) {
#if defined(__APPLE__)
    host_cpu_load_info_data_t cpu;
    mach_msg_type_number_t count = HOST_CPU_LOAD_INFO_COUNT;
    kern_return_t k = host_statistics(mach_host_self(), HOST_CPU_LOAD_INFO,
                                      (host_info_t)&cpu, &count);
    if (k != KERN_SUCCESS) return -1;
    out->user    = cpu.cpu_ticks[CPU_STATE_USER];
    out->nice    = 0;
    out->system  = cpu.cpu_ticks[CPU_STATE_SYSTEM];
    out->idle    = cpu.cpu_ticks[CPU_STATE_IDLE];
    out->iowait  = 0;
    out->irq     = 0;
    out->softirq = 0;
    out->steal   = 0;
    return 0;
#else
    FILE* f = fopen(PROC_STAT_PATH, "r");
    if (!f) return -1;
    char label[16];
    int r = fscanf(f, "%15s %llu %llu %llu %llu %llu %llu %llu %llu",
                   label,
                   (unsigned long long*)&out->user,
                   (unsigned long long*)&out->nice,
                   (unsigned long long*)&out->system,
                   (unsigned long long*)&out->idle,
                   (unsigned long long*)&out->iowait,
                   (unsigned long long*)&out->irq,
                   (unsigned long long*)&out->softirq,
                   (unsigned long long*)&out->steal);
    fclose(f);
    return (r == 9) ? 0 : -1;
#endif
}

/* 计算逻辑 CPU 核数（读 /proc/stat 中 "cpuN" 行数） */
static int count_cpus(void) {
    FILE* f = fopen(PROC_STAT_PATH, "r");
    if (!f) return (int)sysconf(_SC_NPROCESSORS_ONLN);
    int count = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "cpu", 3) == 0 && line[3] >= '0' && line[3] <= '9')
            count++;
    }
    fclose(f);
    return count > 0 ? count : (int)sysconf(_SC_NPROCESSORS_ONLN);
}

/* 计算 CPU% (差分) */
static void calc_cpu_pct(const CpuRaw* prev, const CpuRaw* cur,
                         double* user_pct, double* sys_pct,
                         double* idle_pct, double* iowait_pct,
                         double* total_pct) {
    uint64_t d_user    = cur->user    - prev->user;
    uint64_t d_nice    = cur->nice    - prev->nice;
    uint64_t d_system  = cur->system  - prev->system;
    uint64_t d_idle    = cur->idle    - prev->idle;
    uint64_t d_iowait  = cur->iowait  - prev->iowait;
    uint64_t d_irq     = cur->irq     - prev->irq;
    uint64_t d_softirq = cur->softirq - prev->softirq;
    uint64_t d_steal   = cur->steal   - prev->steal;

    uint64_t total = d_user + d_nice + d_system + d_idle +
                     d_iowait + d_irq + d_softirq + d_steal;
    if (total == 0) {
        *user_pct = *sys_pct = *idle_pct = *iowait_pct = *total_pct = 0.0;
        return;
    }
    double inv = 100.0 / (double)total;
    *user_pct   = (double)(d_user + d_nice) * inv;
    *sys_pct    = (double)(d_system + d_irq + d_softirq) * inv;
    *idle_pct   = (double)d_idle * inv;
    *iowait_pct = (double)d_iowait * inv;
    *total_pct  = (double)(total - d_idle - d_iowait) * inv;
}

/* 一次读取所需全部 meminfo 字段 */
static void read_meminfo(uint64_t* total, uint64_t* free_kb,
                         uint64_t* cached, uint64_t* available) {
#if defined(__APPLE__)
    *total = *free_kb = *cached = *available = 0;
    size_t len = sizeof(uint64_t);
    uint64_t phys = 0;
    if (sysctlbyname("hw.memsize", &phys, &len, NULL, 0) == 0)
        *total = phys / 1024;
    vm_statistics64_data_t vm;
    mach_msg_type_number_t cnt = HOST_VM_INFO64_COUNT;
    if (host_statistics64(mach_host_self(), HOST_VM_INFO64,
                          (host_info64_t)&vm, &cnt) != KERN_SUCCESS)
        return;
    uint64_t page = (uint64_t)getpagesize();
    uint64_t free_bytes = (vm.free_count + vm.inactive_count) * page;
    *free_kb   = free_bytes / 1024;
    *cached    = (vm.inactive_count) * page / 1024;
    *available = (vm.free_count + vm.inactive_count) * page / 1024;
#else
    FILE* f = fopen(PROC_MEMINFO_PATH, "r");
    *total = *free_kb = *cached = *available = 0;
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char key[64];
        uint64_t val = 0;
        /* All memory fields we care about have a "kB" unit suffix */
        if (sscanf(line, "%63s %llu kB", key, (unsigned long long*)&val) == 2) {
            if      (strcmp(key, "MemTotal:")     == 0) *total     = val;
            else if (strcmp(key, "MemFree:")      == 0) *free_kb   = val;
            else if (strcmp(key, "Cached:")       == 0) *cached    = val;
            else if (strcmp(key, "MemAvailable:") == 0) *available = val;
        }
    }
    fclose(f);
#endif
}

/* 读取 /proc/diskstats，对所有物理设备（sda/nvme0n1/vda等）求和 */
static int read_disk_raw(DiskRaw* out) {
#if defined(__APPLE__)
    /* macOS 无统一磁盘计数器，磁盘 I/O 标记不可用 */
    (void)out;
    return -1;
#else
    FILE* f = fopen(PROC_DISKSTATS_PATH, "r");
    if (!f) return -1;
    out->read_sectors = 0;
    out->write_sectors = 0;
    unsigned int major, minor;
    char devname[32];
    uint64_t r_ios, r_merges, r_sectors, r_ticks;
    uint64_t w_ios, w_merges, w_sectors, w_ticks;
    /* 只读需要的字段 (diskstats 字段1-11) */
    while (fscanf(f,
                  " %u %u %31s"
                  " %llu %llu %llu %llu"
                  " %llu %llu %llu %llu",
                  &major, &minor, devname,
                  (unsigned long long*)&r_ios,
                  (unsigned long long*)&r_merges,
                  (unsigned long long*)&r_sectors,
                  (unsigned long long*)&r_ticks,
                  (unsigned long long*)&w_ios,
                  (unsigned long long*)&w_merges,
                  (unsigned long long*)&w_sectors,
                  (unsigned long long*)&w_ticks) == 11) {
        /* 跳过分区（名称以数字结尾且 major 是磁盘 major），
         * 简单策略：名称最后一个字符是字母则认为是整盘 */
        size_t nl = strlen(devname);
        if (nl > 0) {
            char last = devname[nl - 1];
            if (last < '0' || last > '9') {
                out->read_sectors  += r_sectors;
                out->write_sectors += w_sectors;
            }
        }
        /* 跳过行尾剩余字段 */
        char tmp[256];
        if (!fgets(tmp, sizeof(tmp), f)) break;
    }
    fclose(f);
    return 0;
#endif
}

/* 读取 /proc/loadavg */
static void read_loadavg(double* l1, double* l5, double* l15) {
#if defined(__APPLE__)
    double v[3] = { 0.0, 0.0, 0.0 };
    if (getloadavg(v, 3) == 3) { *l1 = v[0]; *l5 = v[1]; *l15 = v[2]; }
    else { *l1 = *l5 = *l15 = 0.0; }
#else
    FILE* f = fopen(PROC_LOADAVG_PATH, "r");
    *l1 = *l5 = *l15 = 0.0;
    if (!f) return;
    if (fscanf(f, "%lf %lf %lf", l1, l5, l15) < 3) {
        *l1 = *l5 = *l15 = 0.0;
    }
    fclose(f);
#endif
}

/* 读取 /proc/uptime (第一字段，秒) */
static double read_uptime(void) {
#if defined(__APPLE__)
    struct timeval boot;
    size_t len = sizeof(boot);
    int mib[2] = { CTL_KERN, KERN_BOOTTIME };
    if (sysctl(mib, 2, &boot, &len, NULL, 0) != 0) return 0.0;
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec - boot.tv_sec;
#else
    FILE* f = fopen(PROC_UPTIME_PATH, "r");
    if (!f) return 0.0;
    double up = 0.0;
    if (fscanf(f, "%lf", &up) < 1) up = 0.0;
    fclose(f);
    return up;
#endif
}

/* 读取进程自身 RSS / VmSize (kB) */
static void read_proc_mem(uint64_t* rss_kb, uint64_t* vms_kb) {
#if defined(__APPLE__)
    *rss_kb = *vms_kb = 0;
    proc_taskinfo ti;
    int sz = proc_pidinfo(getpid(), PROC_PIDTASKINFO, 0, &ti, sizeof(ti));
    if (sz == (int)sizeof(ti)) {
        *rss_kb = (uint64_t)(ti.pti_resident_size / 1024);
        *vms_kb = (uint64_t)(ti.pti_virtual_size / 1024);
    }
#else
    FILE* f = fopen(PROC_SELF_STATUS, "r");
    *rss_kb = *vms_kb = 0;
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char key[32];
        uint64_t val = 0;
        /* Only lines with "kB" suffix carry memory values */
        if (sscanf(line, "%31s %llu kB", key, (unsigned long long*)&val) == 2) {
            if      (strcmp(key, "VmRSS:") == 0) *rss_kb = val;
            else if (strcmp(key, "VmSize:") == 0) *vms_kb = val;
        }
    }
    fclose(f);
#endif
}

/* 读取单个线程 stat: utime/stime/state
 * /proc/[pid]/stat 格式: "pid (comm) state ppid ... utime(14) stime(15) ..."
 * comm 字段用括号括起, 可含空格, 不能用 %s 直接解析; 需跳过最后一个 ')' 后再继续。 */
static int read_thread_stat(pid_t tid, uint64_t* utime, uint64_t* stime, char* state) {
    char path[64];
    snprintf(path, sizeof(path), PROC_SELF_TASK "/%d/stat", (int)tid);
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    if (!fgets(line, sizeof(line), f)) { fclose(f); return -1; }
    fclose(f);

    /* 跳过 pid 字段, 找到最后一个 ')' (comm 字段结束) */
    char* rp = strrchr(line, ')');
    if (!rp) return -1;
    rp++;  /* 指向 ')' 之后的字符 */

    /* 此后格式: " state ppid pgrp session tty tpgid flags
     *            minflt cminflt majflt cmajflt utime stime ..." */
    char st = '?';
    int ppid, pgrp, session, tty, tpgid;
    unsigned int flags;
    uint64_t minflt, cminflt, majflt, cmajflt, ut, st2;
    int r = sscanf(rp,
                   " %c %d %d %d %d %d %u"
                   " %llu %llu %llu %llu"
                   " %llu %llu",
                   &st, &ppid, &pgrp, &session, &tty, &tpgid, &flags,
                   (unsigned long long*)&minflt,
                   (unsigned long long*)&cminflt,
                   (unsigned long long*)&majflt,
                   (unsigned long long*)&cmajflt,
                   (unsigned long long*)&ut,
                   (unsigned long long*)&st2);
    if (r < 13) return -1;
    *utime = ut;
    *stime = st2;
    if (state) *state = st;
    return 0;
}

/* 读取线程名 (/proc/self/task/<tid>/comm) */
static void read_thread_name(pid_t tid, char* name, int maxlen) {
    char path[64];
    snprintf(path, sizeof(path), PROC_SELF_TASK "/%d/comm", (int)tid);
    FILE* f = fopen(path, "r");
    if (!f) { snprintf(name, (size_t)maxlen, "t%d", (int)tid); return; }
    if (!fgets(name, maxlen, f)) snprintf(name, (size_t)maxlen, "t%d", (int)tid);
    fclose(f);
    /* 去掉换行 */
    size_t l = strlen(name);
    if (l > 0 && name[l - 1] == '\n') name[l - 1] = '\0';
}

/* 枚举 /proc/self/task/ 目录，返回线程 tid 数组 */
static int enum_threads(pid_t* tids, int max) {
    DIR* d = opendir(PROC_SELF_TASK);
    if (!d) return 0;
    int count = 0;
    struct dirent* ent;
    while ((ent = readdir(d)) != NULL && count < max) {
        if (ent->d_name[0] < '0' || ent->d_name[0] > '9') continue;
        tids[count++] = (pid_t)atoi(ent->d_name);
    }
    closedir(d);
    return count;
}

/* 在历史数组中查找 tid */
static int find_thread_hist(const ThreadRaw* hist, int n, pid_t tid) {
    for (int i = 0; i < n; i++)
        if (hist[i].tid == tid) return i;
    return -1;
}

/* ── 进程级采集辅助函数（支持任意 pid，供 sysmonitor_proc_snapshot 使用） ── */

/* 读 /proc/<pid>/stat：进程 utime/stime(14/15) + rss 页数(24)。
 * format: "pid (comm) state(3) ... utime(14) stime(15) ... rss(24)"。
 * 返回 0 成功；至少读到 utime/stime 才成功，rss 可选。 */
static int read_proc_stat(pid_t pid, uint64_t* utime, uint64_t* stime,
                          uint64_t* rss_kb) {
#if defined(__APPLE__)
    proc_taskinfo ti;
    int sz = proc_pidinfo(pid, PROC_PIDTASKINFO, 0, &ti, sizeof(ti));
    if (sz != (int)sizeof(ti)) return -1;
    *utime = (uint64_t)ti.pti_total_user;
    *stime = (uint64_t)ti.pti_total_system;
    if (rss_kb) *rss_kb = (uint64_t)(ti.pti_resident_size / 1024);
    return 0;
#else
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/stat", (int)pid);
    FILE* f = fopen(path, "r");
    if (!f) return -1;
    char line[512];
    if (!fgets(line, sizeof(line), f)) { fclose(f); return -1; }
    fclose(f);

    char* rp = strrchr(line, ')');
    if (!rp) return -1;
    rp++;

    char         st;
    int          ppid, pgrp, session, tty, tpgid;
    unsigned int flags;
    uint64_t     minflt, cminflt, majflt, cmajflt, ut, st2, cutime, cstime;
    long         priority, nice, num_threads, itrealvalue, rss_pages;
    uint64_t     starttime, vsize;
    int r = sscanf(rp,
                   " %c %d %d %d %d %d %u"
                   " %llu %llu %llu %llu"
                   " %llu %llu %llu %llu"
                   " %ld %ld %ld %ld %llu %llu %ld",
                   &st, &ppid, &pgrp, &session, &tty, &tpgid, &flags,
                   (unsigned long long*)&minflt,
                   (unsigned long long*)&cminflt,
                   (unsigned long long*)&majflt,
                   (unsigned long long*)&cmajflt,
                   (unsigned long long*)&ut,
                   (unsigned long long*)&st2,
                   (unsigned long long*)&cutime,
                   (unsigned long long*)&cstime,
                   &priority, &nice, &num_threads, &itrealvalue,
                   (unsigned long long*)&starttime,
                   (unsigned long long*)&vsize,
                   &rss_pages);
    if (r < 13) return -1;
    *utime = ut;
    *stime = st2;
    if (rss_kb) {
        if (r >= 22 && rss_pages > 0) {
            uint64_t page_b = (uint64_t)sysconf(_SC_PAGESIZE);
            *rss_kb = (uint64_t)rss_pages * (page_b / 1024);
        } else {
            *rss_kb = 0;
        }
    }
    return 0;
#endif
}

#if !defined(__APPLE__)
/* 读 /proc/<pid>/task/<tid>/stat：utime/stime/state（格式同 /proc/<pid>/stat） */
static int read_task_stat(pid_t pid, pid_t tid, uint64_t* utime,
                          uint64_t* stime, char* state) {
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/task/%d/stat", (int)pid, (int)tid);
    FILE* f = fopen(path, "r");
    if (!f) return -1;
    char line[512];
    if (!fgets(line, sizeof(line), f)) { fclose(f); return -1; }
    fclose(f);

    char* rp = strrchr(line, ')');
    if (!rp) return -1;
    rp++;

    char         st;
    int          ppid, pgrp, session, tty, tpgid;
    unsigned int flags;
    uint64_t     minflt, cminflt, majflt, cmajflt, ut, st2;
    int r = sscanf(rp,
                   " %c %d %d %d %d %d %u"
                   " %llu %llu %llu %llu"
                   " %llu %llu",
                   &st, &ppid, &pgrp, &session, &tty, &tpgid, &flags,
                   (unsigned long long*)&minflt,
                   (unsigned long long*)&cminflt,
                   (unsigned long long*)&majflt,
                   (unsigned long long*)&cmajflt,
                   (unsigned long long*)&ut,
                   (unsigned long long*)&st2);
    if (r < 13) return -1;
    *utime = ut;
    *stime = st2;
    if (state) *state = st;
    return 0;
}

/* 读 /proc/<pid>/task/<tid>/comm 线程名 */
static void read_task_name(pid_t pid, pid_t tid, char* name, int maxlen) {
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/task/%d/comm", (int)pid, (int)tid);
    FILE* f = fopen(path, "r");
    if (!f || !fgets(name, maxlen, f)) {
        if (!f) snprintf(name, (size_t)maxlen, "t%d", (int)tid);
        return;
    }
    fclose(f);
    size_t l = strlen(name);
    if (l > 0 && name[l - 1] == '\n') name[l - 1] = '\0';
}

/* 枚举 /proc/<pid>/task/ 下的线程 tid */
static int enum_task(pid_t pid, pid_t* tids, int max) {
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/task", (int)pid);
    DIR* d = opendir(path);
    if (!d) return 0;
    int count = 0;
    struct dirent* ent;
    while ((ent = readdir(d)) != NULL && count < max) {
        if (ent->d_name[0] < '0' || ent->d_name[0] > '9') continue;
        tids[count++] = (pid_t)atoi(ent->d_name);
    }
    closedir(d);
    return count;
}
#endif /* !__APPLE__ */

/* 在进程历史中查找 pid */
static int find_proc_hist(const ProcRaw* hist, int n, pid_t pid) {
    for (int i = 0; i < n; i++)
        if (hist[i].pid == pid) return i;
    return -1;
}

/* 在进程线程历史中查找 (pid,tid) */
static int find_pth_hist(const PThreadRaw* hist, int n, pid_t pid, pid_t tid) {
    for (int i = 0; i < n; i++)
        if (hist[i].pid == pid && hist[i].tid == tid) return i;
    return -1;
}

/* ── 公开 API ─────────────────────────────────────────────────── */

SysMonitor* sysmonitor_create(void) {
    SysMonitor* sm = (SysMonitor*)calloc(1, sizeof(SysMonitor));
    if (!sm) return NULL;
    sm->cpu_count      = count_cpus();
    sm->has_prev_cpu   = 0;
    sm->has_prev_disk  = 0;
    sm->thread_hist_count = 0;
    sm->prev_ts_us     = mono_us();
    sm->prev_thread_ts_us = sm->prev_ts_us;
    sm->prev_proc_ts_us   = sm->prev_ts_us;
    sm->proc_thread_hist_count = 0;
    sm->proc_thread_prev_ts_us = sm->prev_ts_us;
    return sm;
}

void sysmonitor_destroy(SysMonitor* sm) {
    free(sm);
}

int sysmonitor_snapshot(SysMonitor* sm, SysMonitorSnapshot* out) {
    if (!sm || !out) return -1;
    memset(out, 0, sizeof(*out));

    uint64_t now_us = mono_us();
    double   dt_s   = (now_us > sm->prev_ts_us)
                      ? (double)(now_us - sm->prev_ts_us) / 1e6
                      : 0.0;
    out->timestamp_us = now_us;
    out->cpu_count    = sm->cpu_count;

    /* ── CPU ── */
    CpuRaw cur_cpu;
    if (read_cpu_raw(&cur_cpu) == 0) {
        if (sm->has_prev_cpu) {
            calc_cpu_pct(&sm->prev_cpu, &cur_cpu,
                         &out->cpu_user_pct, &out->cpu_sys_pct,
                         &out->cpu_idle_pct, &out->cpu_iowait_pct,
                         &out->cpu_total_pct);
        }
        sm->prev_cpu     = cur_cpu;
        sm->has_prev_cpu = 1;
    }

    /* ── 内存 ── */
    read_meminfo(&out->mem_total_kb, &out->mem_free_kb,
                 &out->mem_cached_kb, &out->mem_available_kb);
    if (out->mem_total_kb > 0) {
        out->mem_used_kb  = out->mem_total_kb - out->mem_available_kb;
        out->mem_used_pct = 100.0 * (double)out->mem_used_kb
                            / (double)out->mem_total_kb;
    }

    /* ── 进程内存 ── */
    read_proc_mem(&out->proc_rss_kb, &out->proc_vms_kb);

    /* ── 磁盘 I/O ── */
    DiskRaw cur_disk;
    if (read_disk_raw(&cur_disk) == 0) {
        if (sm->has_prev_disk && dt_s > 0.0) {
            /* 512 字节/扇区 */
            uint64_t dr = cur_disk.read_sectors  - sm->prev_disk.read_sectors;
            uint64_t dw = cur_disk.write_sectors - sm->prev_disk.write_sectors;
            out->disk_read_bps  = (double)dr * 512.0 / dt_s;
            out->disk_write_bps = (double)dw * 512.0 / dt_s;
        }
        sm->prev_disk     = cur_disk;
        sm->has_prev_disk = 1;
    }

    /* ── 系统负载 ── */
    read_loadavg(&out->load1, &out->load5, &out->load15);

    /* ── 运行时间 ── */
    out->uptime_sec = read_uptime();

    /* ── 线程列表 ── */
    out->thread_count = sysmonitor_thread_snapshot(sm,
                            out->threads, SYSMON_MAX_THREADS);
    if (out->thread_count < 0) out->thread_count = 0;

    sm->prev_ts_us = now_us;
    return 0;
}

int sysmonitor_thread_snapshot(SysMonitor* sm,
                               SysMonitorThreadSnapshot* threads,
                               int max_threads) {
    if (!sm || !threads || max_threads <= 0) return -1;

    uint64_t now_us = mono_us();
    double   dt_s   = (now_us > sm->prev_thread_ts_us)
                      ? (double)(now_us - sm->prev_thread_ts_us) / 1e6
                      : 0.0;

    long hz = sysconf(_SC_CLK_TCK);
    if (hz <= 0) hz = 100;

    /* 枚举当前线程 */
    pid_t tids[SYSMON_MAX_THREADS];
    int   ntids = enum_threads(tids,
                     max_threads < SYSMON_MAX_THREADS
                         ? max_threads : SYSMON_MAX_THREADS);

    /* 构建新历史数组 */
    ThreadRaw new_hist[SYSMON_MAX_THREADS];
    int       out_count = 0;

    /* 窗口未满时不更新基线历史：只对既有基线做累计差分，
     * 避免高频采样把低占用线程量化成 0%。窗口满才提交新基线。 */
    int commit = (dt_s >= SYSMON_CPU_WINDOW_S);

    for (int i = 0; i < ntids && out_count < max_threads; i++) {
        pid_t tid = tids[i];
        uint64_t ut = 0, st = 0;
        char     state = '?';
        if (read_thread_stat(tid, &ut, &st, &state) != 0) continue;

        SysMonitorThreadSnapshot* snap = &threads[out_count];
        snap->tid        = tid;
        snap->state      = state;
        snap->utime_ticks = ut;
        snap->stime_ticks = st;
        read_thread_name(tid, snap->name, SYSMON_THREAD_NAME_MAX);

        /* 差分 CPU% */
        int hi = find_thread_hist(sm->thread_hist,
                                  sm->thread_hist_count, tid);
        if (hi >= 0 && dt_s > 0.0) {
            uint64_t d_ticks = (ut + st)
                               - (sm->thread_hist[hi].utime
                                  + sm->thread_hist[hi].stime);
            snap->cpu_pct = (double)d_ticks / (double)hz / dt_s * 100.0;
        } else {
            snap->cpu_pct = 0.0;
        }

        /* 仅窗口满时保存到新历史（提交基线） */
        if (commit && out_count < SYSMON_MAX_THREADS) {
            new_hist[out_count].tid   = tid;
            new_hist[out_count].utime = ut;
            new_hist[out_count].stime = st;
        }
        out_count++;
    }

    /* 窗口满才更新历史基线；否则保留旧基线，次数越长累计差分越准 */
    if (commit) {
        int copy_n = out_count < SYSMON_MAX_THREADS ? out_count : SYSMON_MAX_THREADS;
        memcpy(sm->thread_hist, new_hist, (size_t)copy_n * sizeof(ThreadRaw));
        sm->thread_hist_count  = copy_n;
        sm->prev_thread_ts_us  = now_us;
    }

    return out_count;
}

int sysmonitor_proc_snapshot(SysMonitor* sm,
                             const pid_t* pids,
                             const char* const* names,
                             int nprocs,
                             SysMonitorProcSnapshot* out,
                             int max_out) {
    if (!sm || !pids || nprocs <= 0 || !out || max_out <= 0) return -1;
    if (nprocs > SYSMON_MAX_PROCS) nprocs = SYSMON_MAX_PROCS;
    if (max_out > SYSMON_MAX_PROCS) max_out = SYSMON_MAX_PROCS;

    uint64_t now_us = mono_us();
    double   dt_s   = (now_us > sm->prev_proc_ts_us)
                      ? (double)(now_us - sm->prev_proc_ts_us) / 1e6
                      : 0.0;

    long hz = sysconf(_SC_CLK_TCK);
    if (hz <= 0) hz = 100;

    /* 新历史数组（本轮结束后原子替换）。
     * new_pth 可能很大（SYSMON_MAX_PROCS × SYSMON_PROC_THREADS），放堆避免大栈。 */
    ProcRaw new_proc[SYSMON_MAX_PROCS];
    PThreadRaw* new_pth = (PThreadRaw*)malloc(SYSMON_PTH_HIST_MAX * sizeof(PThreadRaw));
    if (!new_pth) return -1;
    int new_proc_n = 0, new_pth_n = 0;

    int written = 0;
    for (int pi = 0; pi < nprocs && written < max_out; pi++) {
        pid_t pid   = pids[pi];
        SysMonitorProcSnapshot* snap = &out[written];
        memset(snap, 0, sizeof(*snap));
        snap->pid   = pid;
        if (names && names[pi]) {
            snprintf(snap->name, sizeof(snap->name), "%s", names[pi]);
        } else {
            /* 读 /proc/<pid>/comm 作为进程名 */
            char p[64];
            snprintf(p, sizeof(p), "/proc/%d/comm", (int)pid);
            FILE* f = fopen(p, "r");
            if (f) {
                if (!fgets(snap->name, sizeof(snap->name), f))
                    snprintf(snap->name, sizeof(snap->name), "pid%d", (int)pid);
                fclose(f);
                size_t l = strlen(snap->name);
                if (l > 0 && snap->name[l - 1] == '\n') snap->name[l - 1] = '\0';
            } else {
                snprintf(snap->name, sizeof(snap->name), "pid%d", (int)pid);
            }
        }

        /* 进程 CPU/RSS（差分） */
        uint64_t ut = 0, st = 0, rss_kb = 0;
        if (read_proc_stat(pid, &ut, &st, &rss_kb) == 0) {
            snap->rss_kb = rss_kb;
            int hi = find_proc_hist(sm->proc_hist, sm->proc_hist_count, pid);
            if (hi >= 0 && dt_s > 0.0) {
                uint64_t d_ticks = (ut + st)
                                   - (sm->proc_hist[hi].utime
                                      + sm->proc_hist[hi].stime);
                snap->cpu_pct = (double)d_ticks / (double)hz / dt_s * 100.0;
            }
            if (new_proc_n < SYSMON_MAX_PROCS) {
                new_proc[new_proc_n].pid   = pid;
                new_proc[new_proc_n].utime = ut;
                new_proc[new_proc_n].stime = st;
                new_proc_n++;
            }
        }

        /* 枚举并采集该进程线程（差分需 (pid,tid) 键） */
#if defined(__APPLE__)
        /* macOS：task_for_pid + task_threads + thread_info（瞬时 cpu_usage，不差分） */
        mach_port_t task;
        if (task_for_pid(mach_task_self(), pid, &task) == KERN_SUCCESS) {
            thread_act_array_t tlist = NULL;
            mach_msg_type_number_t tcnt = 0;
            if (task_threads(task, &tlist, &tcnt) == KERN_SUCCESS) {
                int n = (int)tcnt;
                if (n > SYSMON_PROC_THREADS) n = SYSMON_PROC_THREADS;
                for (int ti = 0; ti < n; ti++) {
                    SysMonitorThreadSnapshot* th = &snap->threads[snap->thread_count];
                    th->tid          = (pid_t)tlist[ti];
                    th->utime_ticks  = 0;
                    th->stime_ticks  = 0;
                    thread_basic_info tb;
                    mach_msg_type_number_t cc = THREAD_BASIC_INFO_COUNT;
                    if (thread_info(tlist[ti], THREAD_BASIC_INFO,
                                    (thread_info_t)&tb, &cc) == KERN_SUCCESS) {
                        th->cpu_pct = (double)tb.cpu_usage / 10.0; /* 千分比→% */
                        th->state   = (tb.flags & TH_FLAGS_IDLE) ? 'I'
                                    : (tb.run_state == TH_STATE_RUNNING) ? 'R' : 'S';
                    }
                    thread_extended_info te;
                    mach_msg_type_number_t ec = THREAD_EXTENDED_INFO_COUNT;
                    if (thread_info(tlist[ti], THREAD_EXTENDED_INFO,
                                    (thread_info_t)&te, &ec) == KERN_SUCCESS
                        && te.pth_name[0]) {
                        snprintf(th->name, sizeof(th->name), "%s", te.pth_name);
                    } else {
                        snprintf(th->name, sizeof(th->name), "thread%d", (int)tlist[ti]);
                    }
                    snap->thread_count++;
                }
                for (mach_msg_type_number_t i = 0; i < tcnt; i++)
                    mach_port_deallocate(mach_task_self(), tlist[i]);
                vm_deallocate(mach_task_self(), (vm_address_t)tlist,
                              tcnt * sizeof(thread_t));
            }
            mach_port_deallocate(mach_task_self(), task);
        }
#else
        pid_t tids[SYSMON_PROC_THREADS];
        int   ntids = enum_task(pid, tids, SYSMON_PROC_THREADS);
        for (int ti = 0; ti < ntids && ti < SYSMON_PROC_THREADS; ti++) {
            pid_t tid = tids[ti];
            uint64_t tut = 0, tst = 0;
            char     tstate = '?';
            if (read_task_stat(pid, tid, &tut, &tst, &tstate) != 0) continue;

            SysMonitorThreadSnapshot* th = &snap->threads[snap->thread_count];
            th->tid        = tid;
            th->state      = tstate;
            th->utime_ticks = tut;
            th->stime_ticks = tst;
            read_task_name(pid, tid, th->name, SYSMON_THREAD_NAME_MAX);

            int hi = find_pth_hist(sm->pth_hist, sm->pth_hist_count, pid, tid);
            if (hi >= 0 && dt_s > 0.0) {
                uint64_t d_ticks = (tut + tst)
                                   - (sm->pth_hist[hi].utime
                                      + sm->pth_hist[hi].stime);
                th->cpu_pct = (double)d_ticks / (double)hz / dt_s * 100.0;
            }

            if (new_pth_n < SYSMON_PTH_HIST_MAX) {
                new_pth[new_pth_n].pid   = pid;
                new_pth[new_pth_n].tid   = tid;
                new_pth[new_pth_n].utime = tut;
                new_pth[new_pth_n].stime = tst;
                new_pth_n++;
            }
            snap->thread_count++;
        }
#endif /* !__APPLE__ */

        written++;
    }

    /* 原子替换历史 */
    memcpy(sm->proc_hist, new_proc, (size_t)new_proc_n * sizeof(ProcRaw));
    sm->proc_hist_count = new_proc_n;
    memcpy(sm->pth_hist, new_pth, (size_t)new_pth_n * sizeof(PThreadRaw));
    sm->pth_hist_count  = new_pth_n;
    sm->prev_proc_ts_us = now_us;

    free(new_pth);
    return written;
}

/* ── 单进程(dlopen)模式：节点即线程，以线程作进程快照 ──────────
 * 单进程模式下所有 AD 节点跑在 flow_launcher 一个进程里，每个节点
 * 是独立命名线程（pthread_setname_np）。把每个节点线程当作一条
 * "进程"输出，即可按节点查看真实 CPU 占用。
 * 过滤调度器/主线程等基础设施线程，只保留业务节点线程。 */
int sysmonitor_proc_thread_snapshots(SysMonitor* sm,
                                     SysMonitorProcSnapshot* out,
                                     int max_out) {
    if (!sm || !out || max_out <= 0) return -1;

    uint64_t now_us = mono_us();
    double   dt_s   = (now_us > sm->proc_thread_prev_ts_us)
                      ? (double)(now_us - sm->proc_thread_prev_ts_us) / 1e6
                      : 0.0;
    long hz = sysconf(_SC_CLK_TCK);
    if (hz <= 0) hz = 100;

    pid_t tids[SYSMON_MAX_THREADS];
    int   ntids = enum_threads(tids, SYSMON_MAX_THREADS);
    if (ntids <= 0) return 0;

    uint64_t prss = 0, pvms = 0;
    read_proc_mem(&prss, &pvms);

    ThreadRaw new_hist[SYSMON_MAX_THREADS];
    int new_n = 0, written = 0;

    /* 窗口未满时不更新基线历史，避免高频采样把轻负载节点量化成 0% */
    int commit = (dt_s >= SYSMON_CPU_WINDOW_S);

    for (int i = 0; i < ntids && written < max_out; i++) {
        pid_t  tid = tids[i];
        uint64_t ut = 0, st = 0;
        char     state = '?';
        if (read_thread_stat(tid, &ut, &st, &state) != 0) continue;

        char name[SYSMON_PROC_NAME_MAX];
        read_thread_name(tid, name, sizeof(name));

        /* 过滤基础设施线程：调度 worker / 主控 / HTTP 服务 */
        if (strcmp(name, "flow_launcher") == 0) continue;
        if (strcmp(name, "sched-mon") == 0) continue;
        if (strcmp(name, "httpd") == 0) continue;

        SysMonitorProcSnapshot* snap = &out[written];
        memset(snap, 0, sizeof(*snap));
        snap->pid           = tid;   /* 用 tid 作唯一 pid，便于前端区分/点选 */
        snprintf(snap->name, sizeof(snap->name), "%s", name);
        snap->rss_kb        = prss;  /* 同进程共享 RSS */

        int hi = find_thread_hist(sm->proc_thread_hist,
                                  sm->proc_thread_hist_count, tid);
        if (hi >= 0 && dt_s > 0.0) {
            uint64_t d_ticks = (ut + st)
                               - (sm->proc_thread_hist[hi].utime
                                  + sm->proc_thread_hist[hi].stime);
            snap->cpu_pct = (double)d_ticks / (double)hz / dt_s * 100.0;
        }

        /* 该节点线程自身作为一条线程 */
        snap->thread_count = 1;
        snap->threads[0].tid    = tid;
        snprintf(snap->threads[0].name, SYSMON_THREAD_NAME_MAX,
                 "%.*s", SYSMON_THREAD_NAME_MAX - 1, name);
        snap->threads[0].cpu_pct = snap->cpu_pct;
        snap->threads[0].state   = state;

        /* 仅窗口满时保存到新历史（提交基线） */
        if (commit && new_n < SYSMON_MAX_THREADS) {
            new_hist[new_n].tid   = tid;
            new_hist[new_n].utime = ut;
            new_hist[new_n].stime = st;
            new_n++;
        }
        written++;
    }

    /* 窗口满才更新历史基线；否则保留旧基线累计差分 */
    if (commit) {
        int copy_n = new_n < SYSMON_MAX_THREADS ? new_n : SYSMON_MAX_THREADS;
        memcpy(sm->proc_thread_hist, new_hist, (size_t)copy_n * sizeof(ThreadRaw));
        sm->proc_thread_hist_count  = copy_n;
        sm->proc_thread_prev_ts_us  = now_us;
    }
    return written;
}
