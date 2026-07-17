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

/* ── 内部辅助宏 ──────────────────────────────────────────────── */

#define PROC_STAT_PATH       "/proc/stat"
#define PROC_MEMINFO_PATH    "/proc/meminfo"
#define PROC_DISKSTATS_PATH  "/proc/diskstats"
#define PROC_LOADAVG_PATH    "/proc/loadavg"
#define PROC_UPTIME_PATH     "/proc/uptime"
#define PROC_SELF_STATUS     "/proc/self/status"
#define PROC_SELF_TASK       "/proc/self/task"

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
};

/* ── 内部工具函数 ─────────────────────────────────────────────── */

static uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

/* 解析 /proc/stat 第一行 "cpu  ..." */
static int read_cpu_raw(CpuRaw* out) {
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
}

/* 读取 /proc/diskstats，对所有物理设备（sda/nvme0n1/vda等）求和 */
static int read_disk_raw(DiskRaw* out) {
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
}

/* 读取 /proc/loadavg */
static void read_loadavg(double* l1, double* l5, double* l15) {
    FILE* f = fopen(PROC_LOADAVG_PATH, "r");
    *l1 = *l5 = *l15 = 0.0;
    if (!f) return;
    if (fscanf(f, "%lf %lf %lf", l1, l5, l15) < 3) {
        *l1 = *l5 = *l15 = 0.0;
    }
    fclose(f);
}

/* 读取 /proc/uptime (第一字段，秒) */
static double read_uptime(void) {
    FILE* f = fopen(PROC_UPTIME_PATH, "r");
    if (!f) return 0.0;
    double up = 0.0;
    if (fscanf(f, "%lf", &up) < 1) up = 0.0;
    fclose(f);
    return up;
}

/* 读取进程自身 RSS / VmSize (kB) */
static void read_proc_mem(uint64_t* rss_kb, uint64_t* vms_kb) {
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

        /* 保存到新历史 */
        if (out_count < SYSMON_MAX_THREADS) {
            new_hist[out_count].tid   = tid;
            new_hist[out_count].utime = ut;
            new_hist[out_count].stime = st;
        }
        out_count++;
    }

    /* 更新历史 */
    int copy_n = out_count < SYSMON_MAX_THREADS ? out_count : SYSMON_MAX_THREADS;
    memcpy(sm->thread_hist, new_hist, (size_t)copy_n * sizeof(ThreadRaw));
    sm->thread_hist_count  = copy_n;
    sm->prev_thread_ts_us  = now_us;

    return out_count;
}
