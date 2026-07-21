/**
 * crash_handler.c — 崩溃信号处理器实现
 *
 * 使用 execinfo.h (backtrace/backtrace_symbols) 打印调用栈，
 * 不依赖 libunwind/glibc 以外的库，保证极低耦合。
 *
 * 编译: 需链接 -rdynamic 以解析符号名（CMakeLists.txt 已配置）。
 */

#define _GNU_SOURCE
#include "crash_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <execinfo.h>
#include <ucontext.h>
#include <sys/syscall.h>

#define BT_DEPTH 64

/* ── 信号名映射 ──────────────────────────────────────────── */

static const char* sig_name(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV";
        case SIGABRT: return "SIGABRT";
        case SIGFPE:  return "SIGFPE";
        case SIGILL:  return "SIGILL";
        case SIGBUS:  return "SIGBUS";
        default:      return "UNKNOWN";
    }
}

/* ── 寄存器 dump (x86_64 / ARM64) ────────────────────────── */

#if defined(__x86_64__)
static void dump_regs(ucontext_t* uc) {
    fprintf(stderr, "  rax=0x%016lx rbx=0x%016lx rcx=0x%016lx rdx=0x%016lx\n",
            uc->uc_mcontext.gregs[REG_RAX], uc->uc_mcontext.gregs[REG_RBX],
            uc->uc_mcontext.gregs[REG_RCX], uc->uc_mcontext.gregs[REG_RDX]);
    fprintf(stderr, "  rsi=0x%016lx rdi=0x%016lx rbp=0x%016lx rsp=0x%016lx\n",
            uc->uc_mcontext.gregs[REG_RSI], uc->uc_mcontext.gregs[REG_RDI],
            uc->uc_mcontext.gregs[REG_RBP], uc->uc_mcontext.gregs[REG_RSP]);
    fprintf(stderr, "  r8 =0x%016lx r9 =0x%016lx r10=0x%016lx r11=0x%016lx\n",
            uc->uc_mcontext.gregs[REG_R8],  uc->uc_mcontext.gregs[REG_R9],
            uc->uc_mcontext.gregs[REG_R10], uc->uc_mcontext.gregs[REG_R11]);
    fprintf(stderr, "  rip=0x%016lx eflags=0x%08lx\n",
            uc->uc_mcontext.gregs[REG_RIP], uc->uc_mcontext.gregs[REG_EFL]);
    fprintf(stderr, "  fault_addr=0x%016lx\n",
            (unsigned long)uc->uc_mcontext.gregs[REG_RIP]);
}
#elif defined(__aarch64__)
static void dump_regs(ucontext_t* uc) {
    fprintf(stderr, "  x0=0x%016lx x1=0x%016lx x2=0x%016lx x3=0x%016lx\n",
            uc->uc_mcontext.regs[0], uc->uc_mcontext.regs[1],
            uc->uc_mcontext.regs[2], uc->uc_mcontext.regs[3]);
    fprintf(stderr, "  x4=0x%016lx x5=0x%016lx x6=0x%016lx x7=0x%016lx\n",
            uc->uc_mcontext.regs[4], uc->uc_mcontext.regs[5],
            uc->uc_mcontext.regs[6], uc->uc_mcontext.regs[7]);
    fprintf(stderr, "  fp=0x%016lx lr=0x%016lx sp=0x%016lx pc=0x%016lx\n",
            uc->uc_mcontext.regs[29], uc->uc_mcontext.regs[30],
            uc->uc_mcontext.sp, uc->uc_mcontext.pc);
    fprintf(stderr, "  fault_addr=0x%016lx\n", uc->uc_mcontext.fault_address);
}
#else
static void dump_regs(ucontext_t* uc) { (void)uc; }
#endif

/* ── 线程信息 ────────────────────────────────────────────── */

static void dump_thread_info(void) {
    char tname[16];
    tname[0] = '\0';
    pthread_getname_np(pthread_self(), tname, sizeof(tname));
    fprintf(stderr, "  thread=%s tid=%ld pid=%d\n",
            tname[0] ? tname : "(unnamed)",
            (long)syscall(SYS_gettid), getpid());
}

/* ── 符号解析（使用 addr2line 获得精确文件名+行号） ──────── */

static void resolve_symbols(void** addrs, int n) {
    char** symbols = backtrace_symbols(addrs, n);
    if (!symbols) {
        fprintf(stderr, "  (backtrace_symbols failed)\n");
        return;
    }

    char self[256];
    ssize_t len = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (len > 0) self[len] = '\0';
    else snprintf(self, sizeof(self), "flow_launcher");

    for (int i = 0; i < n; i++) {
        /* 尝试 addr2line 解析（若有调试符号） */
        char cmd[512];
        snprintf(cmd, sizeof(cmd),
                 "addr2line -e %s -f -p -C 0x%lx 2>/dev/null",
                 self, (unsigned long)addrs[i]);
        FILE* fp = popen(cmd, "r");
        if (fp) {
            char line[512];
            if (fgets(line, sizeof(line), fp)) {
                size_t l = strlen(line);
                if (l > 0 && line[l-1] == '\n') line[l-1] = '\0';
                fprintf(stderr, "  #%-2d %s\n", i, line);
                pclose(fp);
                continue;
            }
            pclose(fp);
        }
        /* fallback: raw backtrace_symbols */
        fprintf(stderr, "  #%-2d %s\n", i, symbols[i]);
    }
    free(symbols);
}

/* ── 核心信号处理 ────────────────────────────────────────── */

static void crash_signal_handler(int sig, siginfo_t* info, void* ctx) {
    /* 先恢复默认处理器，防止递归 */
    signal(sig, SIG_DFL);

    fprintf(stderr, "\n"
            "╔══════════════════════════════════════════════════╗\n"
            "║  FATAL SIGNAL: %-7s                            ║\n",
            sig_name(sig));

    /* Fault address */
    if (sig == SIGSEGV || sig == SIGBUS) {
        fprintf(stderr, "║  fault_addr=0x%016lx                       ║\n",
                (unsigned long)info->si_addr);
    }
    fprintf(stderr,
            "╚══════════════════════════════════════════════════╝\n\n");

    dump_thread_info();

    if (ctx) {
        ucontext_t* uc = (ucontext_t*)ctx;
        fprintf(stderr, "Registers:\n");
        dump_regs(uc);
    }

    fprintf(stderr, "\nBacktrace:\n");
    void* bt[BT_DEPTH];
    int n = backtrace(bt, BT_DEPTH);
    resolve_symbols(bt, n);
    fprintf(stderr, "\n");

    fflush(stderr);

    /* 重新抛出信号，让系统生成 core dump */
    signal(sig, SIG_DFL);
    raise(sig);
}

/* ── 公开 API ────────────────────────────────────────────── */

void crash_handler_install(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = crash_signal_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESETHAND;

    /* 捕获崩溃信号 */
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGFPE,  &sa, NULL);
    sigaction(SIGILL,  &sa, NULL);
    sigaction(SIGBUS,  &sa, NULL);

    /* 忽略 SIGPIPE — write() 返回 EPIPE 即可 */
    signal(SIGPIPE, SIG_IGN);

    fprintf(stderr, "[crash_handler] installed (SIGSEGV SIGABRT SIGFPE SIGILL SIGBUS)\n");
}

void crash_handler_print_backtrace(void) {
    char tname[16];
    tname[0] = '\0';
    pthread_getname_np(pthread_self(), tname, sizeof(tname));
    fprintf(stderr, "[crash_handler] manual backtrace (thread=%s tid=%ld):\n",
            tname[0] ? tname : "?", (long)syscall(SYS_gettid));

    void* bt[BT_DEPTH];
    int n = backtrace(bt, BT_DEPTH);
    resolve_symbols(bt, n);
    fprintf(stderr, "\n");
    fflush(stderr);
}