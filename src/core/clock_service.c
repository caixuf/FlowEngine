#include "clock_service.h"
#include <time.h>
#include <pthread.h>

static pthread_mutex_t g_clock_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool     g_sim_mode   = false;
static uint64_t g_sim_time   = 0;

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
