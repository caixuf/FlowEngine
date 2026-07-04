/**
 * mcap_writer.c — 轻量 MCAP 录制器（JSON 编码）
 *
 * 生成 Foxglove Studio 直接可打开的 .mcap 文件。
 * 只依赖标准 C 库，无需 protobuf/zstd。
 *
 * 用法:
 *   McapWriter* w = mcap_writer_open("demo.mcap");
 *   mcap_writer_register_channel(w, "sensor/lidar", "LidarFrame", lidar_schema_json);
 *   mcap_writer_write_msg(w, ch_id, timestamp_ns, json_data, json_len);
 *   mcap_writer_close(w);
 */

#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <inttypes.h>
#include <time.h>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>  /* htonl, htonll */
#else
/* 简单回退 */
#define htonl(x) ((((x)&0xFF)<<24)|(((x)&0xFF00)<<8)|(((x)&0xFF0000)>>8)|(((x)&0xFF000000)>>24))
static inline uint64_t htonll(uint64_t x) {
    return ((uint64_t)htonl((uint32_t)(x))<<32)|htonl((uint32_t)(x>>32));
}
#endif

#define MCAP_MAGIC "\x89MCAP\r\n\x1a\n"
#define MCAP_MAGIC_LEN 8

#define MCAP_OP_HEADER       1
#define MCAP_OP_FOOTER       2
#define MCAP_OP_SCHEMA       3
#define MCAP_OP_CHANNEL      4
#define MCAP_OP_MESSAGE      5
#define MCAP_OP_CHUNK        6
#define MCAP_OP_CHUNK_INDEX  7
#define MCAP_OP_MESSAGE_INDEX 8
#define MCAP_OP_DATA_END     9

#define MCAP_MAX_CHANNELS 32
#define MCAP_BUF_SIZE (4*1024*1024)

typedef struct {
    uint16_t id;
    char     topic[128];
    char     schema_name[64];
    char     encoding[16];
    uint16_t schema_id;
    uint64_t message_count;
} McapChannel;

typedef struct {
    FILE*    fp;
    char     profile[32];
    uint64_t start_time;       /* ns since epoch */
    uint64_t last_write_offset; /* for summary */

    McapChannel channels[MCAP_MAX_CHANNELS];
    uint16_t channel_count;
    uint16_t schema_count;

    uint8_t  buf[MCAP_BUF_SIZE];  /* 写缓冲 */
    size_t   buf_used;
} McapWriter;

/* ── 小端序写入 ──────────────────────────────────────────── */

static inline void w8(uint8_t* dst, uint8_t v)  { dst[0] = v; }
static inline void w16(uint8_t* dst, uint16_t v) { dst[0]=v; dst[1]=v>>8; }
static inline void w32(uint8_t* dst, uint32_t v) { dst[0]=v;dst[1]=v>>8;dst[2]=v>>16;dst[3]=v>>24; }
static inline void w64(uint8_t* dst, uint64_t v) {
    w32(dst, (uint32_t)v); w32(dst+4, (uint32_t)(v>>32));
}

/* ── 内部: 写一条 MCAP 记录 ─────────────────────────────── */

static int write_record(FILE* fp, uint8_t op, const uint8_t* data, uint32_t len) {
    uint8_t hdr[5];
    hdr[0] = op;
    w32(hdr+1, len);
    if (fwrite(hdr, 5, 1, fp) != 1) return -1;
    if (len > 0 && fwrite(data, 1, len, fp) != len) return -1;
    return 0;
}

/* ── 内部: 写一条 MCAP 记录（两段拼接，减少 malloc）────── */

static int write_record2(FILE* fp, uint8_t op,
                         const uint8_t* d1, uint32_t l1,
                         const uint8_t* d2, uint32_t l2) {
    uint32_t total = l1 + l2;
    uint8_t hdr[5];
    hdr[0] = op;
    w32(hdr+1, total);
    if (fwrite(hdr, 5, 1, fp) != 1) return -1;
    if (l1 > 0 && fwrite(d1, 1, l1, fp) != l1) return -1;
    if (l2 > 0 && fwrite(d2, 1, l2, fp) != l2) return -1;
    return 0;
}

/* ── 内部: 写 Header ─────────────────────────────────────── */

static int write_header(McapWriter* w) {
    /* profile 字符串 */
    char pro[256];
    snprintf(pro, sizeof(pro), "%s", w->profile);

    uint8_t data[512];
    size_t off = 0;
    /* key: profile */
    data[off++] = 7; /* string length */
    memcpy(data+off, "profile", 7); off += 7;
    uint32_t plen = (uint32_t)strlen(w->profile);
    w32(data+off, plen); off += 4;
    memcpy(data+off, w->profile, plen); off += plen;

    return write_record(w->fp, MCAP_OP_HEADER, data, (uint32_t)off);
}

/* ── 内部: 写 Footer ─────────────────────────────────────── */

static int write_footer(McapWriter* w) {
    uint8_t data[20];
    w64(data,     w->last_write_offset); /* summary_start (0 = no summary) */
    w64(data+8,   0);                     /* summary_offset_start */
    w32(data+16,  0);                     /* summary_crc */
    return write_record(w->fp, MCAP_OP_FOOTER, data, 20);
}

/* ── 内部: 写 Schema ─────────────────────────────────────── */

static int write_schema(McapWriter* w, uint16_t id, const char* name,
                        const char* encoding, const char* schema_data) {
    /* Schema 记录: id(2) + name_len+name + enc_len+enc + data_len+data */
    uint16_t nlen = (uint16_t)strlen(name);
    uint16_t elen = (uint16_t)strlen(encoding);
    uint32_t dlen = (uint32_t)strlen(schema_data);
    uint32_t total = 2 + 2 + nlen + 2 + elen + 4 + dlen;

    uint8_t* buf = (uint8_t*)malloc(total);
    if (!buf) return -1;
    size_t off = 0;
    w16(buf+off, id); off += 2;
    w16(buf+off, nlen); off += 2;
    memcpy(buf+off, name, nlen); off += nlen;
    w16(buf+off, elen); off += 2;
    memcpy(buf+off, encoding, elen); off += elen;
    w32(buf+off, dlen); off += 4;
    memcpy(buf+off, schema_data, dlen); off += dlen;

    int rc = write_record(w->fp, MCAP_OP_SCHEMA, buf, (uint32_t)off);
    free(buf);
    return rc;
}

/* ── 内部: 写 Channel 记录 ───────────────────────────────── */

static int write_channel_record(McapWriter* w, McapChannel* ch) {
    uint16_t tlen = (uint16_t)strlen(ch->topic);
    uint32_t total = 2 + 2 + tlen + 2 + 2;
    uint8_t buf[256];
    size_t off = 0;
    w16(buf+off, ch->id); off += 2;
    w16(buf+off, ch->schema_id); off += 2;
    w16(buf+off, tlen); off += 2;
    memcpy(buf+off, ch->topic, tlen); off += tlen;
    w16(buf+off, 1); off += 2; /* message_encoding length = 1 */
    buf[off++] = 'j';          /* 'j' = JSON encoding */
    /* metadata: 0 entries */
    w16(buf+off, 0); off += 2;

    return write_record(w->fp, MCAP_OP_CHANNEL, buf, (uint32_t)off);
}

/* ══════════════════════════════════════════════════════════ */
/* 公共 API                                                    */
/* ══════════════════════════════════════════════════════════ */

McapWriter* mcap_writer_open(const char* path, const char* profile) {
    McapWriter* w = (McapWriter*)calloc(1, sizeof(McapWriter));
    if (!w) return NULL;

    w->fp = fopen(path, "wb");
    if (!w->fp) { free(w); return NULL; }

    /* Magic */
    fwrite(MCAP_MAGIC, MCAP_MAGIC_LEN, 1, w->fp);

    snprintf(w->profile, sizeof(w->profile), "%s", profile ? profile : "x-json");
    write_header(w);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    w->start_time = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    return w;
}

uint16_t mcap_writer_register_channel(McapWriter* w, const char* topic,
                                      const char* schema_name,
                                      const char* schema_json) {
    if (!w || w->channel_count >= MCAP_MAX_CHANNELS) return 0;

    McapChannel* ch = &w->channels[w->channel_count];
    ch->id         = (uint16_t)(w->channel_count + 1);
    ch->schema_id  = (uint16_t)(w->schema_count + 1);

    snprintf(ch->topic, sizeof(ch->topic), "%s", topic);
    snprintf(ch->schema_name, sizeof(ch->schema_name), "%s", schema_name);
    snprintf(ch->encoding, sizeof(ch->encoding), "json");

    /* Write Schema record */
    write_schema(w, ch->schema_id, schema_name, "jsonschema", schema_json);
    w->schema_count++;

    /* Write Channel record */
    write_channel_record(w, ch);

    w->channel_count++;
    return ch->id;
}

int mcap_writer_write_msg(McapWriter* w, uint16_t channel_id,
                          uint64_t log_time_ns, /* 0 表示用当前时间 */
                          const char* json_data, uint32_t json_len) {
    if (!w || channel_id == 0 || channel_id > w->channel_count) return -1;

    if (log_time_ns == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        log_time_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    }

    McapChannel* ch = &w->channels[channel_id - 1];

    /* Message record: channel_id(2) + sequence(4) + log_time(8) + publish_time(8) + data */
    uint32_t seq = (uint32_t)(ch->message_count + 1);
    uint8_t hdr[22];
    w16(hdr,     channel_id);
    w32(hdr+2,   seq);
    w64(hdr+6,   log_time_ns);
    w64(hdr+14,  log_time_ns); /* publish_time = log_time for simplicity */

    int rc = write_record2(w->fp, MCAP_OP_MESSAGE, hdr, 22,
                           (const uint8_t*)json_data, json_len);
    if (rc == 0) {
        ch->message_count++;
        w->last_write_offset = (uint64_t)ftell(w->fp);
    }
    return rc;
}

int mcap_writer_close(McapWriter** pw) {
    if (!pw || !*pw) return -1;
    McapWriter* w = *pw;

    write_footer(w);

    /* Trailing magic */
    fwrite(MCAP_MAGIC, MCAP_MAGIC_LEN, 1, w->fp);

    /* Stats flush */
    fflush(w->fp);
    uint64_t total_bytes = (uint64_t)ftell(w->fp);

    printf("\n[McapWriter] %s: %"PRIu64" bytes, %u channels\n",
           "closed", total_bytes, w->channel_count);
    for (uint16_t i = 0; i < w->channel_count; i++) {
        printf("  channel[%u] \"%s\": %"PRIu64" msgs\n",
               w->channels[i].id, w->channels[i].topic,
               w->channels[i].message_count);
    }

    fclose(w->fp);
    free(w);
    *pw = NULL;
    return 0;
}

/* ── 便捷: 构造简单 JSON 消息 ─────────────────────────────── */

int mcap_writer_write_json(McapWriter* w, uint16_t channel_id,
                           uint64_t log_time_ns,
                           const char* fmt, ...) {
    char json[4096];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(json, sizeof(json), fmt, ap);
    va_end(ap);
    if (len < 0 || (size_t)len >= sizeof(json)) return -1;
    return mcap_writer_write_msg(w, channel_id, log_time_ns, json, (uint32_t)len);
}

/* ── 全局单例（方便各模块共享同一个 mcap 文件）────────────── */

static McapWriter* g_mcap = NULL;

McapWriter* mcap_writer_global(void) { return g_mcap; }

void mcap_writer_set_global(McapWriter* w) { g_mcap = w; }
