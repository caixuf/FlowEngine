#ifndef CRASH_HANDLER_H
#define CRASH_HANDLER_H

/**
 * @file crash_handler.h
 * @brief 崩溃信号处理器 — SIGSEGV/SIGABRT/SIGFPE 捕获 + backtrace
 *
 * 注册信号处理器后，崩溃时自动打印：
 *   1. 信号名称和 fault address
 *   2. 寄存器状态 (x86_64 / ARM64)
 *   3. 调用栈 backtrace（符号名 + 偏移）
 *   4. 线程名称
 *
 * 用法：
 *   crash_handler_install();  // 在 main() 开头调用一次
 *
 * 编译依赖：-rdynamic（符号表）或 -g（调试信息），可选 libunwind 提升精度
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 安装崩溃信号处理器。
 * 捕获 SIGSEGV, SIGABRT, SIGFPE, SIGILL, SIGBUS。
 * 多次调用安全（幂等）。
 */
void crash_handler_install(void);

/**
 * 手动触发 backtrace 打印（不崩溃）。
 * 在可疑位置调用可辅助调试，不会终止进程。
 */
void crash_handler_print_backtrace(void);

#ifdef __cplusplus
}
#endif

#endif /* CRASH_HANDLER_H */