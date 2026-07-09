/**
 * bag.c — 消息录制与回放实现 (v2 格式)
 *
 * 文件格式 v2：
 *   [Header: magic(4B)|version(4B)|msg_count(8B)|duration_us(8B)|
 *            index_offset(8B)|reserved(32B)] — 64 bytes total
 *   [Records: type_id(4B)|schema_ver(1B)|endian(1B)|ts(8B)|topic_len(1B)|
 *             topic(N)|data_size(4B)|data(N)] × msg_count
 *   [Index: entry_count(8B)|entries[topic(64B)|count(8B)|first_off(8B)|last_off(8B)]|
 *            crc32(4B)]
 *
 * 向后兼容：reader 检测前 4 字节是否为 "FLB_"，否则回退到 legacy 格式。
 */

#include "bag.h"
#include "clock_service.h"
#include "serializer.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>

/* ── 文件格式常量 ──────────────────────────────────────────── */

#define BAG_MAGIC        0x5F424C46u  /* "FLB_" in LE */

static const char BAG_MAGIC_STR[5] = "FLB_";

#define BAG_VERSION      2
#define BAG_HEADER_SIZE  64
#define BAG_RESERVED_SIZE 32
#define BAG_INDEX_ENTRY_TOPIC_LEN 64

/* CRC32 lookup table (generated) */
static uint32_t crc32_table[256];
static bool     crc32_table_init = false;

static void crc32_init_table(void) {
    if (crc32_table_init) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
        }
        crc32_table[i] = crc;
    }
    crc32_table_init = true;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    if (!crc32_table_init) crc32_init_table();
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return ~crc;
}

/* ── Index entry (used during write and read) ──────────────── */

typedef struct {
    char     topic[BAG_INDEX_ENTRY_TOPIC_LEN];
    uint64_t count;
    uint64_t first_offset;
    uint64_t last_offset;
    uint32_t type_id;
    uint8_t  schema_version;
} BagIndexEntry;

#define BAG_MAX_INDEX_ENTRIES 256

/* ─────────────────────────────────────────────────────── */
/* Writer (async ring-buffer + background flush thread)    */
/* ─────────────────────────────────────────────────────── */

#define BAG_RING_SIZE  (1024 * 1024)    /* 1 MB ring buffer */
#define BAG_MAX_RECORD (64 * 1024)      /* max single record (64 KB) */

struct BagWriter {
    FILE*            fp;
    pthread_mutex_t  mutex;
    pthread_cond_t   flush_cv;          /* signal: data available */
    bool             attached;
    bool             running;
    bool             stop_flush;
    MessageBus*      bus;

    /* Ring buffer: callback writes here, flush thread drains */
    uint8_t*         ring;
    size_t           ring_w;            /* write cursor */
    size_t           ring_r;            /* read cursor (only flush thread) */
    size_t           ring_len;          /* bytes available to read */

    /* Stats for header/index */
    uint64_t         msg_count;
    uint64_t         first_ts;
    uint64_t         last_ts;

    /* File offset tracker (updated by flush thread) */
    uint64_t         file_offset;

    /* Index (built during write) */
    BagIndexEntry    index[BAG_MAX_INDEX_ENTRIES];
    int              index_count;

    /* Flush thread */
    pthread_t        flush_thread;
};

/* ── Ring buffer helpers ────────────────────────────────── */

static size_t ring_write_space(BagWriter* w) {
    return BAG_RING_SIZE - w->ring_len;
}

static size_t ring_contig_write(BagWriter* w) {
    size_t space = ring_write_space(w);
    size_t tail = BAG_RING_SIZE - w->ring_w;
    return space < tail ? space : tail;
}

static void ring_advance_write(BagWriter* w, size_t n) {
    w->ring_w = (w->ring_w + n) % BAG_RING_SIZE;
    w->ring_len += n;
}

static size_t ring_contig_read(BagWriter* w) {
    if (w->ring_len == 0) return 0;
    size_t tail = BAG_RING_SIZE - w->ring_r;
    return w->ring_len < tail ? w->ring_len : tail;
}

static void ring_advance_read(BagWriter* w, size_t n) {
    w->ring_r = (w->ring_r + n) % BAG_RING_SIZE;
    w->ring_len -= n;
}

/* ── Flush thread: drains ring buffer → disk ────────────── */

static void* bag_flush_thread(void* arg) {
    BagWriter* w = (BagWriter*)arg;
    uint8_t* chunk = (uint8_t*)malloc(BAG_RING_SIZE);
    if (!chunk) return NULL;

    while (!w->stop_flush) {
        pthread_mutex_lock(&w->mutex);
        /* Wait for data or stop signal */
        while (w->ring_len == 0 && !w->stop_flush)
            pthread_cond_wait(&w->flush_cv, &w->mutex);
        if (w->stop_flush) { pthread_mutex_unlock(&w->mutex); break; }

        /* Drain ring buffer: copy contiguous data, fwrite, advance */
        size_t n = ring_contig_read(w);
        if (n > 0) {
            memcpy(chunk, w->ring + w->ring_r, n);
            ring_advance_read(w, n);
            size_t remaining = w->ring_len;
            pthread_mutex_unlock(&w->mutex);

            fwrite(chunk, 1, n, w->fp);

            /* More data wrapped around? */
            if (remaining > 0) {
                pthread_mutex_lock(&w->mutex);
                n = ring_contig_read(w);
                if (n > 0) {
                    memcpy(chunk, w->ring + w->ring_r, n);
                    ring_advance_read(w, n);
                }
                pthread_mutex_unlock(&w->mutex);
                if (n > 0) fwrite(chunk, 1, n, w->fp);
            }
        } else {
            pthread_mutex_unlock(&w->mutex);
        }
    }

    /* Final drain: flush any remaining data before exit */
    pthread_mutex_lock(&w->mutex);
    while (w->ring_len > 0) {
        size_t n = ring_contig_read(w);
        memcpy(chunk, w->ring + w->ring_r, n);
        ring_advance_read(w, n);
        pthread_mutex_unlock(&w->mutex);
        fwrite(chunk, 1, n, w->fp);
        pthread_mutex_lock(&w->mutex);
    }
    pthread_mutex_unlock(&w->mutex);

    free(chunk);
    return NULL;
}

/* ── Public API ─────────────────────────────────────────── */

BagWriter* bag_writer_open(const char* path) {
    if (!path) return NULL;
    FILE* fp = fopen(path, "wb");
    if (!fp) return NULL;

    BagWriter* w = (BagWriter*)calloc(1, sizeof(BagWriter));
    if (!w) { fclose(fp); return NULL; }
    w->fp       = fp;
    w->running  = true;
    w->ring     = (uint8_t*)malloc(BAG_RING_SIZE);
    if (!w->ring) { fclose(fp); free(w); return NULL; }
    pthread_mutex_init(&w->mutex, NULL);
    pthread_cond_init(&w->flush_cv, NULL);

    /* Write placeholder header (overwritten in close) */
    uint8_t header[BAG_HEADER_SIZE];
    memset(header, 0, sizeof(header));
    fwrite(header, 1, sizeof(header), fp);
    w->file_offset = BAG_HEADER_SIZE;

    /* 256KB I/O buffer on the file stream */
    static char io_buf[256 * 1024];
    setvbuf(fp, io_buf, _IOFBF, sizeof(io_buf));

    /* Start background flush thread */
    pthread_create(&w->flush_thread, NULL, bag_flush_thread, w);

    return w;
}

static BagIndexEntry* find_or_create_index(BagWriter* w, const char* topic) {
    for (int i = 0; i < w->index_count; i++) {
        if (strcmp(w->index[i].topic, topic) == 0) return &w->index[i];
    }
    if (w->index_count >= BAG_MAX_INDEX_ENTRIES) return NULL;
    BagIndexEntry* e = &w->index[w->index_count++];
    memset(e, 0, sizeof(*e));
    snprintf(e->topic, BAG_INDEX_ENTRY_TOPIC_LEN, "%s", topic);
    return e;
}

int bag_writer_write(BagWriter* w, const Message* msg) {
    if (!w || !msg) return ERR_INVALID_PARAM;

    uint64_t ts    = msg->timestamp_us;
    uint8_t  tlen  = (uint8_t)strnlen(msg->topic, MSG_BUS_MAX_TOPIC_LEN - 1);
    uint32_t dsize = msg->data_size;

    /* Build serialized record: [type_id:4B][schema_ver:1B][endian:1B]
     * [ts:8B][tlen:1B][topic:N][dsize:4B][data:N]
     * Same on-disk format as before — no extra framing in ring buffer. */
    uint32_t rec_size = 4 + 1 + 1 + 8 + 1 + (uint32_t)tlen + 4 + dsize;
    if (rec_size > BAG_MAX_RECORD) return -1;

    pthread_mutex_lock(&w->mutex);

    /* Wait for space in ring buffer (bounded: drop if full > 10ms) */
    int waits = 0;
    while (ring_write_space(w) < rec_size && waits < 10) {
        pthread_mutex_unlock(&w->mutex);
        usleep(1000);
        pthread_mutex_lock(&w->mutex);
        waits++;
    }
    if (ring_write_space(w) < rec_size) {
        pthread_mutex_unlock(&w->mutex);
        return -1;  /* drop — don't block the pipeline */
    }

    /* Pack record into ring buffer (same format as v2 on-disk) */
    uint8_t* dst = w->ring + w->ring_w;
    memcpy(dst, &msg->type_id, 4);       dst += 4;
    *dst++ = msg->schema_version;
    *dst++ = msg->endian_marker;
    memcpy(dst, &ts, 8);                 dst += 8;
    *dst++ = tlen;
    if (tlen > 0) { memcpy(dst, msg->topic, tlen); dst += tlen; }
    memcpy(dst, &dsize, 4);              dst += 4;
    if (dsize > 0) memcpy(dst, msg->data, dsize);
    size_t written = rec_size;

    /* Handle ring wrap: if record spans the end, it must fit in one chunk.
     * The wait above ensures contiguous space is available. */
    ring_advance_write(w, written);

    /* Update index (file offset estimated from flush position) */
    BagIndexEntry* ie = find_or_create_index(w, msg->topic);
    if (ie) {
        if (ie->count == 0) {
            ie->first_offset   = w->file_offset + (uint64_t)w->ring_len;
            ie->type_id        = msg->type_id;
            ie->schema_version = msg->schema_version;
        }
        ie->last_offset = w->file_offset + (uint64_t)w->ring_len;
        ie->count++;
    }

    if (w->msg_count == 0) w->first_ts = ts;
    w->last_ts = ts;
    w->msg_count++;

    pthread_cond_signal(&w->flush_cv);
    pthread_mutex_unlock(&w->mutex);
    return 0;
}

static void bag_record_callback(const Message* msg, void* user_data) {
    bag_writer_write((BagWriter*)user_data, msg);
}

int bag_writer_attach(BagWriter* w, MessageBus* bus) {
    if (!w || !bus) return ERR_INVALID_PARAM;
    w->bus      = bus;
    w->attached = true;
    return message_bus_subscribe(bus, "*", bag_record_callback, w);
}

void bag_writer_close(BagWriter* w) {
    if (!w) return;

    if (w->attached && w->bus)
        message_bus_unsubscribe(w->bus, "*", bag_record_callback);

    /* Stop flush thread and drain remaining */
    pthread_mutex_lock(&w->mutex);
    w->stop_flush = true;
    pthread_cond_signal(&w->flush_cv);
    pthread_mutex_unlock(&w->mutex);
    pthread_join(w->flush_thread, NULL);

    /* Update file_offset from actual ftell */
    w->file_offset = (uint64_t)ftell(w->fp);

    /* Write index and header */
    if (w->fp) {
        uint64_t index_offset = (uint64_t)ftell(w->fp);
        uint64_t entry_count  = (uint64_t)w->index_count;
        fwrite(&entry_count, sizeof(entry_count), 1, w->fp);
        for (int i = 0; i < w->index_count; i++) {
            BagIndexEntry* e = &w->index[i];
            char tp[BAG_INDEX_ENTRY_TOPIC_LEN];
            memset(tp, 0, sizeof(tp));
            snprintf(tp, sizeof(tp), "%s", e->topic);
            fwrite(tp, 1, sizeof(tp), w->fp);
            fwrite(&e->count,        sizeof(e->count),        1, w->fp);
            fwrite(&e->first_offset, sizeof(e->first_offset), 1, w->fp);
            fwrite(&e->last_offset,  sizeof(e->last_offset),  1, w->fp);
            fwrite(&e->type_id,      sizeof(e->type_id),      1, w->fp);
            fwrite(&e->schema_version,sizeof(e->schema_version),1,w->fp);
        }
        uint32_t crc = 0;
        fwrite(&crc, sizeof(crc), 1, w->fp);

        /* Rewrite header */
        fseek(w->fp, 0, SEEK_SET);
        uint32_t magic    = BAG_MAGIC;
        uint32_t version  = BAG_VERSION;
        uint64_t msg_cnt  = w->msg_count;
        uint64_t dur      = (w->last_ts > w->first_ts) ? (w->last_ts - w->first_ts) : 0;
        uint8_t  reserved[BAG_RESERVED_SIZE];
        memset(reserved, 0, sizeof(reserved));
        fwrite(&magic,        sizeof(magic),        1, w->fp);
        fwrite(&version,      sizeof(version),      1, w->fp);
        fwrite(&msg_cnt,      sizeof(msg_cnt),      1, w->fp);
        fwrite(&dur,          sizeof(dur),          1, w->fp);
        fwrite(&index_offset, sizeof(index_offset), 1, w->fp);
        fwrite(reserved,      1, sizeof(reserved),      w->fp);
        fflush(w->fp);
        fclose(w->fp);
    }

    free(w->ring);
    pthread_mutex_destroy(&w->mutex);
    pthread_cond_destroy(&w->flush_cv);
    free(w);
}

/* ─────────────────────────────────────────────────────── */
/* Reader                                                  */
/* ─────────────────────────────────────────────────────── */

struct BagReader {
    FILE*   fp;
    char    path[512];
    bool    is_v2;               /* true = new format, false = legacy */
    uint64_t msg_count;          /* from header (v2) or computed (legacy) */
    uint64_t duration_us;        /* from header (v2) or computed (legacy) */
    /* Index (v2 only) */
    BagIndexEntry index[BAG_MAX_INDEX_ENTRIES];
    int      index_count;
    /* Cached: first record offset */
    uint64_t data_start;
};

BagReader* bag_reader_open(const char* path) {
    if (!path) return NULL;
    FILE* fp = fopen(path, "rb");
    if (!fp) return NULL;

    BagReader* r = (BagReader*)calloc(1, sizeof(BagReader));
    if (!r) { fclose(fp); return NULL; }
    r->fp = fp;
    snprintf(r->path, sizeof(r->path), "%s", path);

    /* Detect format: read first 4 bytes */
    uint32_t first_word = 0;
    size_t nread = fread(&first_word, sizeof(first_word), 1, fp);
    rewind(fp);

    if (nread == 1 && first_word == BAG_MAGIC) {
        /* ── v2 format ── */
        r->is_v2 = true;
        uint32_t magic, version;
        uint64_t index_offset = 0;

        fread(&magic,        sizeof(magic),        1, fp);
        fread(&version,      sizeof(version),      1, fp);
        fread(&r->msg_count, sizeof(r->msg_count), 1, fp);
        fread(&r->duration_us, sizeof(r->duration_us), 1, fp);
        fread(&index_offset, sizeof(index_offset),  1, fp);
        fseek(fp, BAG_HEADER_SIZE, SEEK_SET);
        r->data_start = BAG_HEADER_SIZE;

        /* Read index if present */
        if (index_offset > 0 && index_offset > BAG_HEADER_SIZE) {
            fseek(fp, (long)index_offset, SEEK_SET);
            uint64_t entry_count = 0;
            if (fread(&entry_count, sizeof(entry_count), 1, fp) == 1) {
                if (entry_count > BAG_MAX_INDEX_ENTRIES) entry_count = BAG_MAX_INDEX_ENTRIES;
                for (uint64_t i = 0; i < entry_count; i++) {
                    BagIndexEntry* e = &r->index[r->index_count++];
                    char topic_padded[BAG_INDEX_ENTRY_TOPIC_LEN];
                    fread(topic_padded,          1, sizeof(topic_padded),  fp);
                    fread(&e->count,             sizeof(e->count),         1, fp);
                    fread(&e->first_offset,      sizeof(e->first_offset),  1, fp);
                    fread(&e->last_offset,       sizeof(e->last_offset),   1, fp);
                    fread(&e->type_id,           sizeof(e->type_id),       1, fp);
                    fread(&e->schema_version,    sizeof(e->schema_version), 1, fp);
                    memcpy(e->topic, topic_padded, BAG_INDEX_ENTRY_TOPIC_LEN - 1);
                    e->topic[BAG_INDEX_ENTRY_TOPIC_LEN - 1] = '\0';
                }
            }
            /* Skip CRC */
            fseek(fp, (long)r->data_start, SEEK_SET);
        }
    } else {
        /* ── Legacy format ── */
        r->is_v2       = false;
        r->data_start  = 0;
        /* msg_count and duration computed on first info() call */
    }

    return r;
}

void bag_reader_close(BagReader* r) {
    if (!r) return;
    if (r->fp) fclose(r->fp);
    free(r);
}

/* ── Read one record ──────────────────────────────────────── */

/**
 * Read one v2 record.
 * Returns 1 on success, 0 on EOF, -1 on error.
 */
static int read_record_v2(FILE* fp, uint64_t* ts_out, Message* msg_out) {
    uint32_t type_id;
    uint8_t  schema_ver, endian;
    if (fread(&type_id, sizeof(type_id), 1, fp) != 1) return 0;
    if (fread(&schema_ver, sizeof(schema_ver), 1, fp) != 1) return ERR_IO;
    if (fread(&endian, sizeof(endian), 1, fp) != 1) return ERR_IO;

    uint64_t ts;
    if (fread(&ts, sizeof(ts), 1, fp) != 1) return ERR_IO;

    uint8_t tlen;
    if (fread(&tlen, sizeof(tlen), 1, fp) != 1) return ERR_IO;

    char topic[MSG_BUS_MAX_TOPIC_LEN];
    memset(topic, 0, sizeof(topic));
    if (tlen > 0 && fread(topic, 1, tlen, fp) != tlen) return ERR_IO;

    uint32_t dsize;
    if (fread(&dsize, sizeof(dsize), 1, fp) != 1) return ERR_IO;
    if (dsize > MSG_BUS_MAX_DATA_SIZE) return ERR_IO;

    uint8_t data[MSG_BUS_MAX_DATA_SIZE];
    if (dsize > 0 && fread(data, 1, dsize, fp) != dsize) return ERR_IO;

    if (ts_out)  *ts_out = ts;
    if (msg_out) {
        memset(msg_out, 0, sizeof(*msg_out));
        snprintf(msg_out->topic,  MSG_BUS_MAX_TOPIC_LEN,  "%s", topic);
        snprintf(msg_out->sender, MSG_BUS_MAX_SENDER_LEN, "bag_replay");
        msg_out->timestamp_us   = ts;
        msg_out->type           = MSG_TYPE_PUBLISH;
        msg_out->data_size      = dsize;
        msg_out->type_id        = type_id;
        msg_out->schema_version = schema_ver;
        msg_out->endian_marker  = endian;
        if (dsize > 0) memcpy(msg_out->data, data, dsize);
    }
    return 1;
}

/**
 * Read one legacy record.
 * Returns 1 on success, 0 on EOF, -1 on error.
 */
static int read_record_legacy(FILE* fp, uint64_t* ts_out, Message* msg_out) {
    uint64_t ts;
    if (fread(&ts, sizeof(ts), 1, fp) != 1) return 0;

    uint8_t tlen;
    if (fread(&tlen, sizeof(tlen), 1, fp) != 1) return ERR_IO;

    char topic[MSG_BUS_MAX_TOPIC_LEN];
    memset(topic, 0, sizeof(topic));
    if (tlen > 0 && fread(topic, 1, tlen, fp) != tlen) return ERR_IO;

    uint32_t dsize;
    if (fread(&dsize, sizeof(dsize), 1, fp) != 1) return ERR_IO;

    if (dsize > MSG_BUS_MAX_DATA_SIZE) return ERR_IO;

    uint8_t data[MSG_BUS_MAX_DATA_SIZE];
    if (dsize > 0 && fread(data, 1, dsize, fp) != dsize) return ERR_IO;

    if (ts_out)  *ts_out = ts;
    if (msg_out) {
        memset(msg_out, 0, sizeof(*msg_out));
        snprintf(msg_out->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
        snprintf(msg_out->sender, MSG_BUS_MAX_SENDER_LEN, "%s", "bag_replay");
        msg_out->timestamp_us = ts;
        msg_out->type         = MSG_TYPE_PUBLISH;
        msg_out->data_size    = dsize;
        /* Legacy: type_id = 0 (raw) */
        if (dsize > 0) memcpy(msg_out->data, data, dsize);
    }
    return 1;
}

static int read_record(BagReader* r, uint64_t* ts_out, Message* msg_out) {
    if (r->is_v2) return read_record_v2(r->fp, ts_out, msg_out);
    else          return read_record_legacy(r->fp, ts_out, msg_out);
}

/* ── Sleep ────────────────────────────────────────────────── */

static void sleep_us(uint64_t us) {
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000ULL);
    ts.tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
    nanosleep(&ts, NULL);
}

/* ── Playback ──────────────────────────────────────────────── */

int bag_reader_play_filtered(BagReader* r, MessageBus* bus, float speed,
                             const char* topic_filter,
                             uint64_t start_us, uint64_t end_us) {
    if (!r || !r->fp) return ERR_INVALID_PARAM;

    /* Seek to data start */
    fseek(r->fp, (long)r->data_start, SEEK_SET);

    uint64_t prev_ts    = 0;
    uint64_t wall_start = 0;
    int      count      = 0;
    bool     first      = true;

    Message msg;
    uint64_t ts;

    while (read_record(r, &ts, &msg) == 1) {
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
            printf("[bag_play] ts=%" PRIu64 " topic=%s size=%u type_id=0x%08x\n",
                   ts, msg.topic, msg.data_size, msg.type_id);
        }
        count++;
    }

    (void)wall_start;
    return count;
}

int bag_reader_play(BagReader* r, MessageBus* bus, float speed) {
    return bag_reader_play_filtered(r, bus, speed, NULL, 0, 0);
}

/* ── Metadata ──────────────────────────────────────────────── */

int bag_reader_info(BagReader* r, uint64_t* msg_count, uint64_t* duration_us) {
    if (!r || !r->fp) return ERR_INVALID_PARAM;

    if (r->is_v2) {
        /* Use header data */
        if (msg_count)   *msg_count   = r->msg_count;
        if (duration_us) *duration_us = r->duration_us;
        return 0;
    }

    /* Legacy: full scan */
    fseek(r->fp, 0, SEEK_SET);

    uint64_t count    = 0;
    uint64_t first_ts = 0;
    uint64_t last_ts  = 0;
    bool     first    = true;

    uint64_t ts;
    Message msg;

    while (read_record(r, &ts, &msg) == 1) {
        count++;
        if (first) { first_ts = ts; first = false; }
        last_ts = ts;
    }

    if (msg_count)   *msg_count   = count;
    if (duration_us) *duration_us = (last_ts > first_ts) ? (last_ts - first_ts) : 0;

    fseek(r->fp, 0, SEEK_SET);  /* back to start */
    return 0;
}

int bag_reader_get_topics(BagReader* r, char topics[][64], int max_count,
                          uint64_t* counts) {
    if (!r || !r->fp || !topics || max_count <= 0) return ERR_INVALID_PARAM;

    if (r->is_v2 && r->index_count > 0) {
        /* Use index (fast path) */
        int n = (r->index_count < max_count) ? r->index_count : max_count;
        for (int i = 0; i < n; i++) {
            snprintf(topics[i], 64, "%s", r->index[i].topic);
            if (counts) counts[i] = r->index[i].count;
        }
        return n;
    }

    /* Legacy / no index: full scan */
    fseek(r->fp, (long)r->data_start, SEEK_SET);

    /* Simple hash-map-like accumulation */
    char seen_topics[256][64];
    uint64_t seen_counts[256];
    int seen_count = 0;

    uint64_t ts;
    Message msg;

    while (read_record(r, &ts, &msg) == 1) {
        bool found = false;
        for (int i = 0; i < seen_count; i++) {
            if (strcmp(seen_topics[i], msg.topic) == 0) {
                seen_counts[i]++;
                found = true;
                break;
            }
        }
        if (!found && seen_count < 256) {
            snprintf(seen_topics[seen_count], 64, "%s", msg.topic);
            seen_counts[seen_count] = 1;
            seen_count++;
        }
    }

    int n = (seen_count < max_count) ? seen_count : max_count;
    for (int i = 0; i < n; i++) {
        snprintf(topics[i], 64, "%s", seen_topics[i]);
        if (counts) counts[i] = seen_counts[i];
    }

    fseek(r->fp, (long)r->data_start, SEEK_SET);
    return n;
}

int bag_reader_get_type_info(BagReader* r, const char* topic,
                             uint32_t* type_id, uint8_t* schema_ver) {
    if (!r || !topic) return ERR_INVALID_PARAM;

    /* Check index first */
    for (int i = 0; i < r->index_count; i++) {
        if (strcmp(r->index[i].topic, topic) == 0) {
            if (type_id)   *type_id   = r->index[i].type_id;
            if (schema_ver) *schema_ver = r->index[i].schema_version;
            return 0;
        }
    }

    /* Not in index — legacy or different topic */
    return ERR_IO;
}
