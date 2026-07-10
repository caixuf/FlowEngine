/**
 * nmea_parser.c — NMEA 0183 GPS 报文解析器实现
 *
 * 见 nmea_parser.h。纯 C，无外部依赖。
 */

#include "nmea_parser.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_MAX_LINE   128
#define NMEA_MAX_FIELDS  32

#define KNOTS_TO_MPS     0.514444    /* 1 knot = 0.514444 m/s */
#define HDOP_TO_METERS   5.0         /* 名义 UERE，用于把 HDOP 折算为 1-sigma 精度 */

/* ── 十六进制字符转值（校验和用）───────────────────────────── */
static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

uint8_t nmea_checksum(const char* sentence) {
    if (!sentence) return 0;
    const char* p = sentence;
    if (*p == '$' || *p == '!') p++;   /* 跳过前导起始符 */
    uint8_t cs = 0;
    for (; *p && *p != '*'; ++p) {
        cs ^= (uint8_t)(*p);
    }
    return cs;
}

/* ── ddmm.mmmm / dddmm.mmmm → 十进制度 ──────────────────────── */
static bool nmea_coord_to_deg(const char* field, char quad, double* out_deg) {
    if (!field || !*field || !out_deg) return false;

    const char* dot = strchr(field, '.');
    /* 度部分 = 小数点前除去最后两位（分）的所有数字 */
    size_t int_len = dot ? (size_t)(dot - field) : strlen(field);
    if (int_len < 3) return false;              /* 至少 1 位度 + 2 位分 */
    size_t deg_len = int_len - 2;

    char deg_buf[8];
    if (deg_len >= sizeof(deg_buf)) return false;
    memcpy(deg_buf, field, deg_len);
    deg_buf[deg_len] = '\0';

    double deg = atof(deg_buf);
    double minutes = atof(field + deg_len);
    double val = deg + minutes / 60.0;

    if (quad == 'S' || quad == 'W' || quad == 's' || quad == 'w') val = -val;
    *out_deg = val;
    return true;
}

/* ── 原地分割字段（保留空字段）─────────────────────────────── */
static int nmea_split(char* buf, char* fields[], int max_fields) {
    int n = 0;
    fields[n++] = buf;
    for (char* p = buf; *p; ++p) {
        if (*p == ',') {
            *p = '\0';
            if (n < max_fields) fields[n++] = p + 1;
            else break;
        }
    }
    return n;
}

void nmea_parser_init(NmeaParser* p) {
    if (!p) return;
    memset(p, 0, sizeof(*p));
}

const GpsData* nmea_parser_last(const NmeaParser* p) {
    return p ? &p->last : NULL;
}

/* 判断 5 字符消息 id（如 GPGGA/GNRMC）后三位是否匹配 type */
static bool msg_type_is(const char* id, const char* type3) {
    size_t len = strlen(id);
    if (len < 3) return false;
    return strncmp(id + (len - 3), type3, 3) == 0;
}

int nmea_parse_line(NmeaParser* p, const char* line, GpsData* out) {
    if (!p || !line) return NMEA_ERR_ARG;

    /* 拷贝并裁剪 CR/LF/空白 */
    char buf[NMEA_MAX_LINE];
    size_t len = 0;
    for (const char* s = line; *s && len < sizeof(buf) - 1; ++s) {
        if (*s == '\r' || *s == '\n') break;
        buf[len++] = *s;
    }
    buf[len] = '\0';

    if (len < 6 || (buf[0] != '$' && buf[0] != '!')) {
        p->sentences_bad++;
        return NMEA_ERR_FORMAT;
    }

    /* 校验和（若语句携带 *HH）*/
    char* star = strchr(buf, '*');
    if (star) {
        int hi = hex_val(star[1]);
        int lo = (star[1] != '\0') ? hex_val(star[2]) : -1;
        if (hi < 0 || lo < 0) {
            p->sentences_bad++;
            return NMEA_ERR_CHECKSUM;
        }
        uint8_t want = (uint8_t)((hi << 4) | lo);
        uint8_t got = nmea_checksum(buf);
        if (want != got) {
            p->sentences_bad++;
            return NMEA_ERR_CHECKSUM;
        }
        *star = '\0';   /* 去掉 *HH，后续按字段解析 */
    }

    char* fields[NMEA_MAX_FIELDS];
    int nf = nmea_split(buf, fields, NMEA_MAX_FIELDS);
    if (nf < 1) {
        p->sentences_bad++;
        return NMEA_ERR_FORMAT;
    }

    const char* id = fields[0] + 1;   /* 跳过 '$' */

    bool updated = false;
    bool no_fix  = false;

    if (msg_type_is(id, "GGA")) {
        /* GGA: 0=id 1=utc 2=lat 3=NS 4=lon 5=EW 6=fix 7=nsat 8=hdop 9=alt ... */
        if (nf < 10) { p->sentences_bad++; return NMEA_ERR_FORMAT; }
        int fix = (fields[6][0] != '\0') ? atoi(fields[6]) : 0;
        if (fix <= 0) {
            no_fix = true;
        } else {
            double lat, lon;
            if (nmea_coord_to_deg(fields[2], fields[3][0], &lat) &&
                nmea_coord_to_deg(fields[4], fields[5][0], &lon)) {
                p->last.latitude  = lat;
                p->last.longitude = lon;
                p->has_position = true;
                updated = true;
            }
            if (fields[8][0] != '\0') {
                double hdop = atof(fields[8]);
                if (hdop > 0.0) p->last.accuracy_m = (float)(hdop * HDOP_TO_METERS);
            }
        }
    } else if (msg_type_is(id, "RMC")) {
        /* RMC: 0=id 1=utc 2=status 3=lat 4=NS 5=lon 6=EW 7=spd(kn) 8=course 9=date ... */
        if (nf < 9) { p->sentences_bad++; return NMEA_ERR_FORMAT; }
        char status = fields[2][0];
        if (status != 'A' && status != 'a') {
            no_fix = true;
        } else {
            double lat, lon;
            if (nmea_coord_to_deg(fields[3], fields[4][0], &lat) &&
                nmea_coord_to_deg(fields[5], fields[6][0], &lon)) {
                p->last.latitude  = lat;
                p->last.longitude = lon;
                p->has_position = true;
                updated = true;
            }
            if (fields[7][0] != '\0') {
                p->last.speed_mps = (float)(atof(fields[7]) * KNOTS_TO_MPS);
                p->has_velocity = true;
                updated = true;
            }
            if (fields[8][0] != '\0') {
                p->last.heading_deg = (float)atof(fields[8]);
                p->has_velocity = true;
                updated = true;
            }
        }
    } else {
        return NMEA_ERR_UNSUPPORTED;
    }

    if (no_fix) {
        p->sentences_ok++;   /* 语句本身有效，只是无定位 */
        return NMEA_ERR_NO_FIX;
    }
    if (!updated) {
        p->sentences_bad++;
        return NMEA_ERR_FORMAT;
    }

    p->sentences_ok++;
    if (out) *out = p->last;
    return NMEA_OK;
}
