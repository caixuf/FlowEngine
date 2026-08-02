/**
 * @file fp_env.h
 * @brief 浮点环境初始化 — 开启 FTZ/DAZ（denormal 清零）。
 *
 * 背景：控制量（如 steer）指数衰减会滑入 subnormal 区（~1e-320），
 * cJSON_Print 会把它原样打进 JSON；glibc strtod_l 解析 subnormal
 * 字符串存在已知断言 bug（strtod_l.c:1496 `numsize == 1 && n < d`），
 * 触发即 SIGABRT 整进程崩溃。
 *
 * 治本：进程入口调用 fp_env_init()，将 MXCSR 的 FTZ(bit15)+DAZ(bit6)
 * 置位，所有浮点运算结果/输入中的 denormal 直接刷成 0——JSON 里永远
 * 不会出现 subnormal 字面量。x86-64 下子线程继承创建者的 MXCSR，
 * 在 main 最早处调用即可覆盖全部线程。
 *
 * 自动驾驶场景中 <1e-308 的量没有物理意义，刷零无副作用。
 */
#ifndef FLOW_FP_ENV_H
#define FLOW_FP_ENV_H

#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#include <pmmintrin.h>
#endif

static inline void fp_env_init(void) {
#if defined(__x86_64__) || defined(__i386__)
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#elif defined(__aarch64__)
    /* FPCR.FZ (bit 24) — ARM64 flush-to-zero */
    unsigned long long fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (1ULL << 24);
    __asm__ __volatile__("msr fpcr, %0" :: "r"(fpcr));
#endif
}

#endif /* FLOW_FP_ENV_H */
