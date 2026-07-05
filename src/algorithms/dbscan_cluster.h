/**
 * dbscan_cluster.h — DBSCAN Clustering for LiDAR Point Clouds
 *
 * Pipeline:
 *   1. Ground removal  — threshold points by z-height
 *   2. DBSCAN          — density-based spatial clustering
 *   3. Bounding box    — fit axis-aligned box to each cluster
 *   4. Classification  — heuristic: size-based vehicle/pedestrian/unknown
 *
 * All operations are O(n) to O(n²) in the worst case but with
 * a simple grid-acceleration that makes it near O(n log n) for
 * typical LiDAR scans (n ≈ 100–2000 points after ground removal).
 *
 * Usage:
 *   DbscanCluster db;
 *   dbscan_init(&db, eps, min_pts);
 *   int n_clusters = dbscan_run(&db, points, n_points);
 *   for (int i = 0; i < n_clusters; i++) {
 *       const ClusterBounds* b = dbscan_get_cluster(&db, i);
 *       // b->cx, b->cy, b->width, b->length, ...
 *   }
 *
 * Reference:
 *   Ester, Kriegel, et al. "A Density-Based Algorithm for Discovering
 *   Clusters in Large Spatial Databases with Noise." KDD-96.
 */

#ifndef DBSCAN_CLUSTER_H
#define DBSCAN_CLUSTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ────────────────────────────────────────────────────── */
#define DBSCAN_MAX_POINTS      131072 /**< 最大输入点数 (支持 Velodyne HDL-32E 单帧) */
#define DBSCAN_MAX_CLUSTERS    32     /**< 最大聚类数 */

/* ── 3D 点 ───────────────────────────────────────────────────── */
typedef struct {
    float x, y, z;
    float intensity;  /* 可选 */
} Point3D;

/* ── 聚类边界盒 ──────────────────────────────────────────────── */
typedef enum {
    CLS_UNKNOWN    = 0,
    CLS_VEHICLE    = 1,
    CLS_PEDESTRIAN = 2,
    CLS_CYCLIST    = 3,
} ClusterClass;

typedef struct {
    float  cx, cy;          /**< 聚类中心 (x, y) */
    float  width, length;   /**< 包围盒宽、长 (m) */
    float  min_z, max_z;    /**< 高度范围 */
    int    point_count;     /**< 聚类内点数 */
    float  confidence;      /**< 置信度 [0, 1] — 基于点密度 */
    ClusterClass cls;       /**< 类型 */
} ClusterBounds;

/* ── 地面去除模式 ──────────────────────────────────────────── */
typedef enum {
    GROUND_REMOVE_NONE   = 0,  /**< 不去除地面 */
    GROUND_REMOVE_ZCUT   = 1,  /**< 简单 z 阈值 (旧方式) */
    GROUND_REMOVE_RANSAC = 2,  /**< RANSAC 平面拟合 (推荐) */
} GroundRemoveMode;

/* ── DBSCAN 实例 ─────────────────────────────────────────────── */
typedef struct {
    float eps;               /**< 邻域半径 (m) */
    int   min_pts;           /**< 最小邻域点数 */
    float ground_z_thresh;   /**< z 阈值 (ZCUT 模式), 低于此值视为地面 */

    /* ── RANSAC 地面拟合参数 ── */
    GroundRemoveMode ground_mode;     /**< 地面去除模式 (默认 RANSAC) */
    int    ransac_max_iter;           /**< 最大迭代次数 (默认 100) */
    float  ransac_dist_thresh;        /**< 点到平面距离阈值 (m) (默认 0.2) */
    float  ransac_min_inlier_ratio;   /**< 最小内点比例 (默认 0.3) */
    float  ground_plane[4];           /**< 输出: 拟合地面平面 (a,b,c,d) */

    /* 内部工作区 */
    int   labels[DBSCAN_MAX_POINTS];
    int   n_clusters;
    ClusterBounds clusters[DBSCAN_MAX_CLUSTERS];
} DbscanCluster;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 初始化 DBSCAN 聚类器。
 * 默认使用 RANSAC 地面去除, eps=2.0, min_pts=4。
 * @param db         实例
 * @param eps        邻域半径 (m), 典型值: 1.5–3.0 (32线LiDAR)
 * @param min_pts    最小点数, 典型值: 4–8
 */
void dbscan_init(DbscanCluster* db, float eps, int min_pts);

/**
 * 设置地面过滤阈值 (ZCUT 模式)。
 */
void dbscan_set_ground_thresh(DbscanCluster* db, float z_thresh);

/**
 * 配置 RANSAC 地面去除参数。
 * @param max_iter         最大迭代次数 (建议 50-200)
 * @param dist_thresh      内点距离阈值 (m) (建议 0.15-0.3)
 * @param min_inlier_ratio 最小内点比例 (建议 0.2-0.5)
 */
void dbscan_set_ransac(DbscanCluster* db, int max_iter,
                        float dist_thresh, float min_inlier_ratio);

/**
 * 执行完整处理管线: 地面去除 → DBSCAN → 包围盒 → 分类。
 *
 * @param db        实例
 * @param points    输入点云（会被排序，调用者可复制）
 * @param n_points  点数
 * @return          聚类数量 (0 表示无聚类或全部为噪声)
 */
int dbscan_run(DbscanCluster* db, Point3D* points, int n_points);

/**
 * 获取聚类信息。
 * @param idx  0..n_clusters-1
 * @return     聚类边界盒，或 NULL 若 idx 越界
 */
const ClusterBounds* dbscan_get_cluster(const DbscanCluster* db, int idx);

/**
 * 获取聚类数量。
 */
int dbscan_cluster_count(const DbscanCluster* db);

#ifdef __cplusplus
}
#endif

#endif /* DBSCAN_CLUSTER_H */
