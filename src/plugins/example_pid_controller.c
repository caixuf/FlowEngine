/**
 * example_pid_controller.c — 参考 PID 纵向控制器插件
 *
 * 演示: 如何将算法封装为 FlowEngine 插件
 * 真实项目: 替换此文件中的 PID 为 MPC/LQR 或集成 Apollo Control
 *
 * 编译为 .so:
 *   gcc -shared -fPIC -I include src/plugins/example_pid_controller.c
 *       -o build/lib/libpid_controller.so
 *
 * launcher 加载:
 *   { "name": "pid_control", "plugin": "lib/libpid_controller.so", ... }
 */

#include "algorithm_plugin.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── PID 状态 ────────────────────────────────────────────── */

typedef struct {
    double kp, ki, kd;       /**< PID 参数 */
    double setpoint;          /**< 目标速度 (m/s) */
    double integral;
    double prev_error;
    double output_min;        /**< 输出下限 (刹车) */
    double output_max;        /**< 输出上限 (油门) */
    int    iteration;
} PidState;

/* ── AlgorithmInterface 实现 ─────────────────────────────── */

static const AlgorithmInfo g_info = {
    .name = "PID_LongitudinalControl",
    .version = "1.0.0",
    .type = ALGO_CONTROL,
    .description = "Reference PID longitudinal controller",
    .dependencies = (const char*[]){"libm", NULL},
    .input_topics = (const char*[]){"fusion/localization", NULL},
    .output_topics = (const char*[]){"control/cmd", NULL},
};

static const AlgorithmInfo* pid_get_info(void) { return &g_info; }

static void* pid_init(MessageBus* bus, const char* config, const char** params) {
    (void)bus; (void)params;
    PidState* s = (PidState*)calloc(1, sizeof(PidState));
    /* Default PID 参数 — 可从 config JSON 覆盖 */
    s->kp = 0.5;  s->ki = 0.05; s->kd = 0.1;
    s->setpoint = 33.0;  /* 目标 33 m/s */
    s->output_min = -1.0; /* 全力刹车 */
    s->output_max =  1.0; /* 全力油门 */
    if (config) {
        /* Parse JSON: {"kp":0.5, "ki":0.05, ...} — simplified parsing */
        const char* p = config;
        #define PARSE_FIELD(name, target) \
            p = strstr(config, "\"" name "\":"); \
            if (p) s->target = atof(p + strlen("\"" name "\":") + 1)
        PARSE_FIELD("kp", kp);
        PARSE_FIELD("ki", ki);
        PARSE_FIELD("kd", kd);
        PARSE_FIELD("setpoint", setpoint);
    }
    return s;
}

static int pid_process(void* handle, const void* input_data) {
    PidState* s = (PidState*)handle;
    const char* data = (const char*)input_data;
    if (!data) return -1;

    /* Extract current speed from fusion output:
     * Format: "pos=(x,y) gps=(lat,lon) speed=33.0 dt=XXus" */
    double current_speed = 0;
    const char* sp = strstr(data, "speed=");
    if (sp) current_speed = atof(sp + 6);

    /* PID 计算 */
    double error = s->setpoint - current_speed;
    s->integral += error;
    double derivative = error - s->prev_error;
    s->prev_error = error;

    double output = s->kp * error + s->ki * s->integral + s->kd * derivative;
    /* 限幅 */
    if (output > s->output_max) output = s->output_max;
    if (output < s->output_min) output = s->output_min;
    /* 抗积分饱和 */
    if (output >= s->output_max || output <= s->output_min) s->integral -= error;

    s->iteration++;
    printf("[PID] #%d speed=%.1f target=%.1f error=%.2f output=%.2f\n",
           s->iteration, current_speed, s->setpoint, error, output);
    return 0;
}

static const char* pid_get_state_json(void* handle) {
    static char buf[256];
    PidState* s = (PidState*)handle;
    snprintf(buf, sizeof(buf),
        "{\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"setpoint\":%.1f,\"iteration\":%d}",
        s->kp, s->ki, s->kd, s->setpoint, s->iteration);
    return buf;
}

static int pid_set_param(void* handle, const char* key, const char* value) {
    PidState* s = (PidState*)handle;
    double v = atof(value);
    if (strcmp(key, "kp") == 0)       s->kp = v;
    else if (strcmp(key, "ki") == 0)  s->ki = v;
    else if (strcmp(key, "kd") == 0)  s->kd = v;
    else if (strcmp(key, "setpoint") == 0) s->setpoint = v;
    else return -1;
    return 0;
}

static StateId pid_get_recommended_mode(void* handle) {
    PidState* s = (PidState*)handle;
    /* If speed is very low, recommend LP mode */
    if (s->setpoint < 10.0) return SM_MODE_LP;
    return SM_MODE_ACC;  /* Default: ACC active */
}

static void pid_destroy(void* handle) { free(handle); }

/* ── 导出符号 (dlopen 入口) ──────────────────────────────── */

static AlgorithmInterface g_pid_interface = {
    .get_info            = pid_get_info,
    .initialize          = pid_init,
    .process             = pid_process,
    .get_state_json      = pid_get_state_json,
    .set_param           = pid_set_param,
    .get_recommended_mode = pid_get_recommended_mode,
    .destroy             = pid_destroy,
};

AlgorithmInterface* get_algorithm_interface(void) {
    return &g_pid_interface;
}

const char* get_algorithm_version(void) {
    return "1.0.0";
}
