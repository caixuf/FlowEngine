/**
 * mcap_reader.c — MCAP file reader implementation.
 *
 * Parses the MCAP binary format:
 *   Magic(8) + [Op(1) Len(4) Data(Len)]* + Footer + Magic(8)
 *
 * Only cares about: Header, Schema, Channel, Message records.
 * Skips: Chunk, ChunkIndex, MessageIndex, DataEnd.
 */

#include "mcap_reader.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MCAP_MAGIC_LEN 8
#define MCAP_MAGIC     "\x89MCAP\r\n\x1a\n"

/* Op codes */
#define MCAP_OP_HEADER       1
#define MCAP_OP_FOOTER       2
#define MCAP_OP_SCHEMA       3
#define MCAP_OP_CHANNEL      4
#define MCAP_OP_MESSAGE      5
#define MCAP_OP_CHUNK        6
#define MCAP_OP_CHUNK_INDEX  7
#define MCAP_OP_MESSAGE_INDEX 8
#define MCAP_OP_DATA_END     9

/* ── 小端序读取 ──────────────────────────────────────────────── */

static inline uint16_t r16(const uint8_t* d) { return (uint16_t)d[0] | ((uint16_t)d[1]<<8); }
static inline uint32_t r32(const uint8_t* d) { return (uint32_t)d[0]|((uint32_t)d[1]<<8)|((uint32_t)d[2]<<16)|((uint32_t)d[3]<<24); }
static inline uint64_t r64(const uint8_t* d) {
    return (uint64_t)r32(d) | ((uint64_t)r32(d+4) << 32);
}

/* ── 读取足够字节 ────────────────────────────────────────────── */

static int read_exact(FILE* fp, uint8_t* buf, size_t n) {
    size_t total = 0;
    while (total < n) {
        size_t got = fread(buf + total, 1, n - total, fp);
        if (got == 0) return -1;
        total += got;
    }
    return 0;
}

/* ── 解析 Header (获取 profile) ───────────────────────────────── */

static int parse_header(const uint8_t* data, uint32_t len, char* profile, size_t prof_sz) {
    uint32_t off = 0;
    while (off < len) {
        if (off + 5 > len) break;
        uint32_t key_len = r32(data + off); off += 4;
        if (off + key_len > len) break;
        /* Only care about "profile" key */
        if (key_len == 7 && memcmp(data + off, "profile", 7) == 0) {
            off += key_len;
            if (off + 4 > len) break;
            uint32_t val_len = r32(data + off); off += 4;
            if (off + val_len > len) break;
            size_t copy = val_len < prof_sz - 1 ? val_len : prof_sz - 1;
            memcpy(profile, data + off, copy);
            profile[copy] = '\0';
            return 0;
        }
        off += key_len;
        if (off + 4 > len) break;
        uint32_t val_len = r32(data + off); off += 4;
        off += val_len;
    }
    return -1;
}

/* ── 解析 Schema ──────────────────────────────────────────────── */

static int parse_schema(const uint8_t* data, uint32_t len,
                         uint16_t* id, char* name, size_t name_sz) {
    if (len < 4) return -1;
    uint32_t off = 2; /* skip id (we read it for verification but don't store) */
    *id = r16(data);
    if (off + 2 > len) return -1;
    uint16_t nlen = r16(data + off); off += 2;
    if (off + nlen > len) return -1;
    size_t copy = nlen < name_sz - 1 ? nlen : name_sz - 1;
    memcpy(name, data + off, copy);
    name[copy] = '\0';
    return 0;
}

/* ── 解析 Channel ─────────────────────────────────────────────── */

static int parse_channel(const uint8_t* data, uint32_t len,
                          McapChannelInfo* ch) {
    if (len < 8) return -1;
    uint32_t off = 0;
    ch->id        = r16(data + off); off += 2;
    ch->schema_id = r16(data + off); off += 2;
    uint16_t tlen = r16(data + off); off += 2;
    if (off + tlen > len) return -1;
    size_t copy = tlen < sizeof(ch->topic) - 1 ? tlen : sizeof(ch->topic) - 1;
    memcpy(ch->topic, data + off, copy);
    ch->topic[copy] = '\0';
    off += tlen;
    /* skip encoding */
    if (off + 2 > len) return -1;
    uint16_t elen = r16(data + off); off += 2;
    off += elen;
    /* metadata pairs */
    if (off + 2 > len) return -1;
    uint16_t meta_pairs = r16(data + off); off += 2;
    for (uint16_t i = 0; i < meta_pairs && off < len; i++) {
        if (off + 4 > len) break;
        uint32_t klen = r32(data + off); off += 4;
        if (off + klen > len) break;
        off += klen;
        if (off + 4 > len) break;
        uint32_t vlen = r32(data + off); off += 4;
        if (off + vlen > len) break;
        off += vlen;
    }
    ch->message_count = 0;
    return 0;
}

/* ══════════════════════════════════════════════════════════ */
/*  Public API                                                */
/* ══════════════════════════════════════════════════════════ */

McapReader* mcap_reader_open(const char* path) {
    FILE* fp = fopen(path, "rb");
    if (!fp) return NULL;

    /* Verify magic */
    uint8_t magic[MCAP_MAGIC_LEN];
    if (read_exact(fp, magic, MCAP_MAGIC_LEN) != 0 ||
        memcmp(magic, MCAP_MAGIC, MCAP_MAGIC_LEN) != 0) {
        fclose(fp); return NULL;
    }

    McapReader* r = (McapReader*)calloc(1, sizeof(McapReader));
    if (!r) { fclose(fp); return NULL; }
    r->fp = fp;
    r->speed = 1.0;
    r->loop = 0;

    /* Get file size */
    fseek(fp, 0, SEEK_END);
    r->file_size = (uint64_t)ftell(fp);
    fseek(fp, MCAP_MAGIC_LEN, SEEK_SET);

    /* Parse all records up to Footer to build channel map */
    char schema_names[MCAP_READER_MAX_CHANNELS][64];
    int schema_count = 0;

    while (1) {
        uint8_t op_buf[5];
        if (read_exact(fp, op_buf, 5) != 0) break;
        uint8_t  op  = op_buf[0];
        uint32_t len = r32(op_buf + 1);

        /* Footer → stop (we've reached the end of the data section) */
        if (op == MCAP_OP_FOOTER) {
            /* skip footer data */
            fseek(fp, (long)len, SEEK_CUR);
            break;
        }

        if (op == MCAP_OP_HEADER && len < 4096) {
            uint8_t* buf = (uint8_t*)malloc(len);
            if (buf && read_exact(fp, buf, len) == 0)
                parse_header(buf, len, r->profile, sizeof(r->profile));
            free(buf);
        }
        else if (op == MCAP_OP_SCHEMA && len < 8192) {
            uint8_t* buf = (uint8_t*)malloc(len);
            if (buf && read_exact(fp, buf, len) == 0 && schema_count < MCAP_READER_MAX_CHANNELS) {
                uint16_t sid;
                parse_schema(buf, len, &sid, schema_names[schema_count], 64);
                schema_count++;
            }
            free(buf);
        }
        else if (op == MCAP_OP_CHANNEL && len < 512) {
            uint8_t* buf = (uint8_t*)malloc(len);
            if (buf && read_exact(fp, buf, len) == 0) {
                if (r->channel_count < MCAP_READER_MAX_CHANNELS) {
                    McapChannelInfo* ch = &r->channels[r->channel_count];
                    parse_channel(buf, len, ch);
                    /* Backfill schema name */
                    if (ch->schema_id > 0 && ch->schema_id <= (uint16_t)schema_count)
                        snprintf(ch->schema_name, sizeof(ch->schema_name),
                                 "%s", schema_names[ch->schema_id - 1]);
                    r->channel_count++;
                }
            }
            free(buf);
        }
        else if (op == MCAP_OP_MESSAGE) {
            /* We've reached the message section */
            r->data_start = (uint64_t)ftell(fp) - 5;  /* rewind to include op header */
            fseek(fp, -(long)5, SEEK_CUR);
            break;
        }
        else {
            /* Skip unknown records (Chunk, Index, etc.) */
            fseek(fp, (long)len, SEEK_CUR);
        }
    }

    /* If we didn't find any messages, rewind to current position */
    if (r->data_start == 0) {
        r->data_start = (uint64_t)ftell(fp);
    }

    return r;
}

int mcap_reader_next(McapReader* r, McapMessage* msg) {
    if (!r || !msg) return -1;

    while (1) {
        uint8_t op_buf[5];
        if (read_exact(r->fp, op_buf, 5) != 0) {
            if (r->loop) {
                mcap_reader_seek_start(r);
                continue;
            }
            return 0; /* EOF */
        }

        uint8_t  op  = op_buf[0];
        uint32_t len = r32(op_buf + 1);

        if (op == MCAP_OP_MESSAGE) {
            if (len < 22) { fseek(r->fp, (long)len, SEEK_CUR); continue; }
            if (len - 22 > MCAP_READER_MAX_MSG_LEN) { fseek(r->fp, (long)len, SEEK_CUR); continue; }

            uint8_t hdr[22];
            if (read_exact(r->fp, hdr, 22) != 0) return -1;

            uint16_t ch_id = r16(hdr);
            msg->channel_id = ch_id;
            msg->sequence    = r32(hdr + 2);
            msg->log_time_ns = r64(hdr + 6);
            msg->publish_time_ns = r64(hdr + 14);

            /* Set base time on first message */
            if (r->base_time_ns == 0) r->base_time_ns = msg->log_time_ns;

            uint32_t data_len = len - 22;
            if (read_exact(r->fp, msg->data, data_len) != 0) return -1;
            msg->data[data_len] = '\0';
            msg->data_len = data_len;

            /* Lookup topic */
            msg->topic[0] = '\0';
            for (int i = 0; i < r->channel_count; i++) {
                if (r->channels[i].id == ch_id) {
                    snprintf(msg->topic, sizeof(msg->topic), "%s", r->channels[i].topic);
                    r->channels[i].message_count++;
                    break;
                }
            }

            return 1;
        }
        else if (op == MCAP_OP_FOOTER) {
            fseek(r->fp, (long)len, SEEK_CUR);
            if (r->loop) {
                mcap_reader_seek_start(r);
                continue;
            }
            return 0;
        }
        else {
            /* Skip non-message records */
            fseek(r->fp, (long)len, SEEK_CUR);
        }
    }
}

int mcap_reader_seek_start(McapReader* r) {
    if (!r) return -1;
    fseek(r->fp, (long)r->data_start, SEEK_SET);
    return 0;
}

int mcap_reader_channel_count(const McapReader* r) {
    return r ? (int)r->channel_count : 0;
}

const McapChannelInfo* mcap_reader_get_channel(const McapReader* r, int idx) {
    if (!r || idx < 0 || idx >= r->channel_count) return NULL;
    return &r->channels[idx];
}

void mcap_reader_close(McapReader* r) {
    if (!r) return;
    if (r->fp) fclose(r->fp);
    free(r);
}
