/**
 * bag.c — 消息录制与回放实现
 *
 * 文件格式（纯二进制）：
 *   每条记录 = [timestamp_us(8B) | topic_len(1B) | topic(topic_len B) | data_size(4B) | data(data_size B)]
 */

#include "bag.h"
#include "clock_service.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

/* ─────────────────────────────────────────────────────── */
/* Writer                                                  */
/* ─────────────────────────────────────────────────────── */

struct BagWriter {
    FILE*          fp;
    pthread_mutex_t mutex;
    bool            attached;    /* true when running background subscription */
    MessageBus*     bus;
};

BagWriter* bag_writer_open(const char* path) {
    if (!path) return NULL;
    FILE* fp = fopen(path, "wb");
    if (!fp) return NULL;

    BagWriter* w = (BagWriter*)calloc(1, sizeof(BagWriter));
    if (!w) { fclose(fp); return NULL; }
    w->fp = fp;
    pthread_mutex_init(&w->mutex, NULL);
    return w;
}

int bag_writer_write(BagWriter* w, const Message* msg) {
    if (!w || !msg) return -1;

    pthread_mutex_lock(&w->mutex);

    uint64_t ts       = msg->timestamp_us;
    uint8_t  tlen     = (uint8_t)strnlen(msg->topic, MSG_BUS_MAX_TOPIC_LEN - 1);
    uint32_t dsize    = msg->data_size;

    fwrite(&ts,    sizeof(ts),    1, w->fp);
    fwrite(&tlen,  sizeof(tlen),  1, w->fp);
    fwrite(msg->topic, 1, tlen,    w->fp);
    fwrite(&dsize, sizeof(dsize), 1, w->fp);
    if (dsize > 0) fwrite(msg->data, 1, dsize, w->fp);

    pthread_mutex_unlock(&w->mutex);
    return 0;
}

/* Callback used for auto-recording from bus */
static void bag_record_callback(const Message* msg, void* user_data) {
    BagWriter* w = (BagWriter*)user_data;
    bag_writer_write(w, msg);
}

int bag_writer_attach(BagWriter* w, MessageBus* bus) {
    if (!w || !bus) return -1;
    w->bus = bus;
    w->attached = true;
    return message_bus_subscribe(bus, "*", bag_record_callback, w);
}

void bag_writer_close(BagWriter* w) {
    if (!w) return;
    if (w->attached && w->bus) {
        message_bus_unsubscribe(w->bus, "*", bag_record_callback);
    }
    if (w->fp) {
        fflush(w->fp);
        fclose(w->fp);
    }
    pthread_mutex_destroy(&w->mutex);
    free(w);
}

/* ─────────────────────────────────────────────────────── */
/* Reader                                                  */
/* ─────────────────────────────────────────────────────── */

struct BagReader {
    FILE* fp;
    char  path[512];
};

BagReader* bag_reader_open(const char* path) {
    if (!path) return NULL;
    FILE* fp = fopen(path, "rb");
    if (!fp) return NULL;

    BagReader* r = (BagReader*)calloc(1, sizeof(BagReader));
    if (!r) { fclose(fp); return NULL; }
    r->fp = fp;
    strncpy(r->path, path, sizeof(r->path) - 1);
    return r;
}

void bag_reader_close(BagReader* r) {
    if (!r) return;
    if (r->fp) fclose(r->fp);
    free(r);
}

/* Read one record from the current file position.
 * Returns 1 on success, 0 on EOF, -1 on error. */
static int read_record(FILE* fp, uint64_t* ts_out, Message* msg_out) {
    uint64_t ts;
    if (fread(&ts, sizeof(ts), 1, fp) != 1) return 0; /* EOF */

    uint8_t tlen;
    if (fread(&tlen, sizeof(tlen), 1, fp) != 1) return -1;

    char topic[MSG_BUS_MAX_TOPIC_LEN];
    memset(topic, 0, sizeof(topic));
    if (tlen > 0 && fread(topic, 1, tlen, fp) != tlen) return -1;

    uint32_t dsize;
    if (fread(&dsize, sizeof(dsize), 1, fp) != 1) return -1;

    if (dsize > MSG_BUS_MAX_DATA_SIZE) return -1;

    uint8_t data[MSG_BUS_MAX_DATA_SIZE];
    if (dsize > 0 && fread(data, 1, dsize, fp) != dsize) return -1;

    if (ts_out)  *ts_out = ts;
    if (msg_out) {
        memset(msg_out, 0, sizeof(*msg_out));
        strncpy(msg_out->topic, topic, MSG_BUS_MAX_TOPIC_LEN - 1);
        strncpy(msg_out->sender, "bag_replay", MSG_BUS_MAX_SENDER_LEN - 1);
        msg_out->timestamp_us = ts;
        msg_out->type         = MSG_TYPE_PUBLISH;
        msg_out->data_size    = dsize;
        if (dsize > 0) memcpy(msg_out->data, data, dsize);
    }
    return 1;
}

static void sleep_us(uint64_t us) {
    struct timespec ts;
    ts.tv_sec  = us / 1000000ULL;
    ts.tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
    nanosleep(&ts, NULL);
}

int bag_reader_play_filtered(BagReader* r, MessageBus* bus, float speed,
                             const char* topic_filter,
                             uint64_t start_us, uint64_t end_us) {
    if (!r || !r->fp) return -1;

    rewind(r->fp);

    uint64_t prev_ts     = 0;
    uint64_t wall_start  = 0;  /* wall-clock start (monotonic us) */
    int      count       = 0;
    bool     first       = true;

    Message msg;
    uint64_t ts;

    while (read_record(r->fp, &ts, &msg) == 1) {
        /* time range filter */
        if (start_us > 0 && ts < start_us) continue;
        if (end_us   > 0 && ts > end_us)   break;

        /* topic filter */
        if (topic_filter && strcmp(topic_filter, "*") != 0 &&
            strcmp(topic_filter, "") != 0) {
            if (strcmp(msg.topic, topic_filter) != 0) continue;
        }

        if (first) {
            prev_ts    = ts;
            wall_start = clock_now_us();
            first      = false;
        } else if (speed > 0.0f) {
            uint64_t gap_us  = ts - prev_ts;
            uint64_t delay   = (uint64_t)((double)gap_us / (double)speed);
            if (delay > 0) sleep_us(delay);
            prev_ts = ts;
        }

        /* Update sim clock for bag replay */
        if (clock_is_sim_mode()) {
            clock_set_sim_time(ts);
        }

        if (bus) {
            message_bus_publish(bus, msg.topic, msg.sender, msg.data, msg.data_size);
        } else {
            printf("[bag_play] ts=%llu topic=%s size=%u\n",
                   (unsigned long long)ts, msg.topic, msg.data_size);
        }
        count++;
    }

    (void)wall_start;
    return count;
}

int bag_reader_play(BagReader* r, MessageBus* bus, float speed) {
    return bag_reader_play_filtered(r, bus, speed, NULL, 0, 0);
}

int bag_reader_info(BagReader* r, uint64_t* msg_count, uint64_t* duration_us) {
    if (!r || !r->fp) return -1;

    rewind(r->fp);

    uint64_t count    = 0;
    uint64_t first_ts = 0;
    uint64_t last_ts  = 0;
    bool     first    = true;

    uint64_t ts;
    Message msg;

    while (read_record(r->fp, &ts, &msg) == 1) {
        count++;
        if (first) { first_ts = ts; first = false; }
        last_ts = ts;
    }

    if (msg_count)   *msg_count   = count;
    if (duration_us) *duration_us = (last_ts > first_ts) ? (last_ts - first_ts) : 0;

    rewind(r->fp);
    return 0;
}
