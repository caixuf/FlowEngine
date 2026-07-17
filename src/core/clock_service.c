#include "clock_service.h"
#include <time.h>
#include <pthread.h>

static pthread_mutex_t g_clock_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool     g_sim_mode   = false;
static uint64_t g_sim_time   = 0;
static uint64_t g_step_us    = 0;

uint64_t clock_now_realtime_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint64_t clock_now_monotonic_wall_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint64_t clock_now_us(void) {
    pthread_mutex_lock(&g_clock_mutex);
    bool sim = g_sim_mode;
    uint64_t t = g_sim_time;
    pthread_mutex_unlock(&g_clock_mutex);

    if (sim) {
        return t;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* Deprecated transition shim: funnel legacy monotonic_us() callers through
 * clock_now_us() so they respect sim-time. Declared deprecated in the header;
 * new code must call clock_now_us() directly. */
uint64_t monotonic_us(void) {
    return clock_now_us();
}

void clock_set_sim_mode(bool enable) {
    pthread_mutex_lock(&g_clock_mutex);
    g_sim_mode = enable;
    pthread_mutex_unlock(&g_clock_mutex);
}

void clock_set_sim_time(uint64_t timestamp_us) {
    pthread_mutex_lock(&g_clock_mutex);
    g_sim_time = timestamp_us;
    pthread_mutex_unlock(&g_clock_mutex);
}

bool clock_is_sim_mode(void) {
    pthread_mutex_lock(&g_clock_mutex);
    bool sim = g_sim_mode;
    pthread_mutex_unlock(&g_clock_mutex);
    return sim;
}

void clock_advance_us(uint64_t delta_us) {
    pthread_mutex_lock(&g_clock_mutex);
    if (g_sim_mode) {
        g_sim_time += delta_us;
    }
    pthread_mutex_unlock(&g_clock_mutex);
}

void clock_set_step_us(uint64_t step_us) {
    pthread_mutex_lock(&g_clock_mutex);
    g_step_us = step_us;
    pthread_mutex_unlock(&g_clock_mutex);
}

uint64_t clock_get_step_us(void) {
    pthread_mutex_lock(&g_clock_mutex);
    uint64_t s = g_step_us;
    pthread_mutex_unlock(&g_clock_mutex);
    return s;
}
