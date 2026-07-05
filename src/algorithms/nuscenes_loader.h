/**
 * nuscenes_loader.h — nuScenes LiDAR data loader
 *
 * Reads nuScenes LiDAR binary files (.bin format):
 *   Each point = [x(float), y(float), z(float), intensity(float), ring_index(float)]
 *   Total: 20 bytes per point.
 *
 * Also parses the nuScenes sample_annotation.json for ground-truth 3D boxes
 * to validate DBSCAN clustering accuracy.
 *
 * Usage:
 *   NuScenesScan scan;
 *   nuscenes_load_lidar("path/to/lidar.bin", &scan);
 *   // scan.points[0..scan.count-1] are in sensor coordinate frame
 *   // Filter by ring, run DBSCAN, compare with GT boxes
 *   nuscenes_free_scan(&scan);
 *
 * nuScenes mini download (~4GB):
 *   wget https://www.nuscenes.org/data/v1.0-mini.tgz
 *   tar xzf v1.0-mini.tgz
 */

#ifndef NUSCENES_LOADER_H
#define NUSCENES_LOADER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUSCENES_MAX_POINTS  (256 * 1024)  /* 典型 32线雷达每帧 ~30k-60k 点 */

/* ── 3D 点 (传感器坐标系) ────────────────────────────────────── */
typedef struct {
    float x, y, z;
    float intensity;
    uint8_t ring;        /**< 激光线号 (0-31 for 32-line LiDAR) */
} NuScenesPoint;

/* ── nuScenes 标注框 (世界坐标系) ────────────────────────────── */
typedef struct {
    char    token[64];       /**< 实例 token */
    double  tx, ty, tz;     /**< 中心位置 (m, 世界系) */
    double  dx, dy, dz;     /**< 包围盒尺寸 (长/宽/高, m) */
    double  qw, qx, qy, qz; /**< 四元数旋转 */
    char    category[32];   /**< "vehicle.car", "human.pedestrian", etc. */
    int     num_lidar_pts;  /**< 框内 LiDAR 点数 */
} NuScenesAnnotation;

/* ── 单帧 LiDAR 扫描 ─────────────────────────────────────────── */
typedef struct {
    NuScenesPoint  points[NUSCENES_MAX_POINTS];
    int            count;
    uint64_t       timestamp_us;  /**< 微秒时间戳 */
    int            num_annotations;
    NuScenesAnnotation* annotations; /**< GT 标注 (可选) */
} NuScenesScan;

/* ── LiDAR 格式 ──────────────────────────────────────────────── */
typedef enum {
    LIDAR_FMT_AUTO     = 0,  /**< 自动检测 (by file size) */
    LIDAR_FMT_NUSCENES = 1,  /**< nuScenes: 5f/pt (x,y,z,intensity,ring) = 20B */
    LIDAR_FMT_KITTI    = 2,  /**< KITTI:    4f/pt (x,y,z,intensity) = 16B */
} LidarFormat;

/* ── API ────────────────────────────────────────────────────── */

/** 加载 LiDAR .bin 文件 (自动检测 nuScenes/KITTI 格式) */
int nuscenes_load_lidar(const char* path, NuScenesScan* scan);

/** 加载 LiDAR .bin 文件 (指定格式) */
int nuscenes_load_lidar_fmt(const char* path, NuScenesScan* scan, LidarFormat fmt);

/** 加载 nuScenes sample_annotation.json (GT 标注) */
int nuscenes_load_annotations(const char* json_path,
                               NuScenesAnnotation** annots, int* count);

/** 释放扫描内存 */
void nuscenes_free_scan(NuScenesScan* scan);

/** 统计信息 */
void nuscenes_print_stats(const NuScenesScan* scan);

#ifdef __cplusplus
}
#endif

#endif /* NUSCENES_LOADER_H */
