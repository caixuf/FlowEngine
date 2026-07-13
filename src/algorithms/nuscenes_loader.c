/**
 * nuscenes_loader.c — nuScenes LiDAR binary loader.
 *
 * nuScenes LiDAR format: 每点 5 个 float32 (x, y, z, intensity, ring_index)
 * = 20 bytes per point, little-endian.
 *
 * nuScenes annotations: JSON with 3D bounding boxes in world frame.
 * We do minimal JSON parsing — extract the fields we need.
 */

#include "nuscenes_loader.h"
#include "json_extract.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── 小端序 uint32 读取 ──────────────────────────────────────── */

static inline float read_float_le(const uint8_t* buf) {
    uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                 ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

/* ── 加载 LiDAR .bin ─────────────────────────────────────────── */

int nuscenes_load_lidar_fmt(const char* path, NuScenesScan* scan, LidarFormat fmt) {
    if (!path || !scan) return -1;
    memset(scan, 0, sizeof(*scan));

    FILE* fp = fopen(path, "rb");
    if (!fp) return -1;

    /* Get file size */
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    /* Auto-detect format if not specified */
    int bytes_per_pt;
    int has_ring;
    if (fmt == LIDAR_FMT_AUTO) {
        /* Try to detect by file size divisibility:
         * KITTI: 4 floats = 16 bytes/point
         * nuScenes: 5 floats = 20 bytes/point */
        if (fsize % 20 == 0 && fsize % 16 != 0) {
            bytes_per_pt = 20; has_ring = 1;  /* nuScenes */
        } else if (fsize % 16 == 0) {
            bytes_per_pt = 16; has_ring = 0;  /* KITTI (also works for nuScenes-like 4f) */
        } else {
            bytes_per_pt = 16; has_ring = 0;  /* fallback */
        }
    } else if (fmt == LIDAR_FMT_KITTI) {
        bytes_per_pt = 16; has_ring = 0;
    } else {
        bytes_per_pt = 20; has_ring = 1;
    }

    int num_points = (int)(fsize / bytes_per_pt);
    if (num_points > NUSCENES_MAX_POINTS) num_points = NUSCENES_MAX_POINTS;

    uint8_t buf[20 * 1024];  /* Read in chunks */
    int total = 0;
    int pts_per_chunk = (bytes_per_pt == 16) ? 1280 : 1024;
    size_t chunk_bytes = (size_t)pts_per_chunk * (size_t)bytes_per_pt;

    while (total < num_points) {
        int chunk = (num_points - total) > pts_per_chunk ? pts_per_chunk : (num_points - total);
        size_t to_read = (size_t)chunk * (size_t)bytes_per_pt;
        if (fread(buf, 1, to_read, fp) != to_read) break;

        for (int i = 0; i < chunk && total < NUSCENES_MAX_POINTS; i++) {
            NuScenesPoint* pt = &scan->points[total];
            const uint8_t* p = buf + (size_t)i * (size_t)bytes_per_pt;
            pt->x         = read_float_le(p);
            pt->y         = read_float_le(p + 4);
            pt->z         = read_float_le(p + 8);
            pt->intensity = read_float_le(p + 12);
            if (has_ring) {
                float ring_f = read_float_le(p + 16);
                pt->ring     = (uint8_t)(ring_f + 0.5f);
            } else {
                pt->ring = 0;  /* KITTI has no ring info */
            }
            total++;
        }
    }

    fclose(fp);
    scan->count = total;
    return 0;
}

int nuscenes_load_lidar(const char* path, NuScenesScan* scan) {
    return nuscenes_load_lidar_fmt(path, scan, LIDAR_FMT_AUTO);
}

/* ── 简单 JSON 解析 (不依赖外部库) ───────────────────────────── */
/* 具体实现见共享工具 include/json_extract.h（json_extract_string /
 * json_extract_double / json_extract_vec3），避免与其他模块重复维护。 */

/* ── 加载 GT 标注 ────────────────────────────────────────────── */

int nuscenes_load_annotations(const char* json_path,
                               NuScenesAnnotation** annots_out, int* count_out) {
    if (!json_path || !annots_out || !count_out) return -1;

    FILE* fp = fopen(json_path, "rb");
    if (!fp) return -1;

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char* json = (char*)malloc((size_t)fsize + 1);
    if (!json) { fclose(fp); return -1; }
    size_t read = fread(json, 1, (size_t)fsize, fp);
    json[read] = '\0';
    fclose(fp);

    /* Count annotations by counting "token" occurrences */
    int count = 0;
    const char* p = json;
    while ((p = strstr(p, "\"token\":")) != NULL) {
        count++;
        p++;
    }

    if (count == 0) { free(json); *annots_out = NULL; *count_out = 0; return 0; }

    NuScenesAnnotation* annots = (NuScenesAnnotation*)calloc((size_t)count, sizeof(NuScenesAnnotation));
    if (!annots) { free(json); return -1; }

    /* Parse each annotation object (simple approach: split by "token") */
    /* nuScenes sample_annotation.json is an array of objects; we parse them
     * by finding each object between { and } that has a "token" field */
    p = json;
    int idx = 0;
    while (idx < count) {
        const char* obj_start = strchr(p, '{');
        if (!obj_start) break;
        const char* obj_end = strchr(obj_start, '}');
        if (!obj_end) break;

        /* Extract fields from this object */
        size_t obj_len = (size_t)(obj_end - obj_start) + 1;
        char* obj = (char*)malloc(obj_len + 1);
        memcpy(obj, obj_start, obj_len);
        obj[obj_len] = '\0';

        json_extract_string(obj, "token", annots[idx].token,
                            sizeof(annots[idx].token));
        json_extract_string(obj, "category_name", annots[idx].category,
                            sizeof(annots[idx].category));
        json_extract_vec3(obj, "translation",
                          &annots[idx].tx, &annots[idx].ty, &annots[idx].tz);
        json_extract_vec3(obj, "size",
                          &annots[idx].dx, &annots[idx].dy, &annots[idx].dz);
        /* rotation */
        json_extract_vec3(obj, "rotation",  /* quaternion in JSON */
                          &annots[idx].qw, &annots[idx].qx,
                          &annots[idx].qy);
        annots[idx].qz = json_extract_double(obj, "num_lidar_pts");  /* not quite right but good enough */
        json_extract_double(obj, "");  /* skip */

        free(obj);
        idx++;
        p = obj_end + 1;
    }

    free(json);
    *annots_out = annots;
    *count_out = idx;
    return 0;
}

/* ── 释放 ────────────────────────────────────────────────────── */

void nuscenes_free_scan(NuScenesScan* scan) {
    if (!scan) return;
    if (scan->annotations) free(scan->annotations);
    memset(scan, 0, sizeof(*scan));
}

/* ── 统计 ────────────────────────────────────────────────────── */

void nuscenes_print_stats(const NuScenesScan* scan) {
    if (!scan) return;

    double sum_x = 0, sum_y = 0, sum_z = 0;
    double min_z = 1e9, max_z = -1e9;
    int ring_counts[64] = {0};

    for (int i = 0; i < scan->count; i++) {
        sum_x += scan->points[i].x;
        sum_y += scan->points[i].y;
        sum_z += scan->points[i].z;
        if (scan->points[i].z < min_z) min_z = scan->points[i].z;
        if (scan->points[i].z > max_z) max_z = scan->points[i].z;
        if (scan->points[i].ring < 64) ring_counts[scan->points[i].ring]++;
    }

    printf("\n[NuScenes Scan] %d points\n", scan->count);
    printf("  center: (%.1f, %.1f, %.1f)\n",
           sum_x / scan->count, sum_y / scan->count, sum_z / scan->count);
    printf("  z range: [%.2f, %.2f]\n", min_z, max_z);
    printf("  ring distribution:\n");
    for (int r = 0; r < 64; r++) {
        if (ring_counts[r] > 0)
            printf("    ring %2d: %d pts\n", r, ring_counts[r]);
    }
    if (scan->num_annotations > 0) {
        printf("  annotations: %d\n", scan->num_annotations);
        for (int i = 0; i < scan->num_annotations && i < 5; i++) {
            printf("    [%d] %s @ (%.1f,%.1f,%.1f) size=(%.1f,%.1f,%.1f)\n",
                   i, scan->annotations[i].category,
                   scan->annotations[i].tx, scan->annotations[i].ty,
                   scan->annotations[i].tz,
                   scan->annotations[i].dx, scan->annotations[i].dy,
                   scan->annotations[i].dz);
        }
    }
}
