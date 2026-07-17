/**
 * dbscan_cluster.c — DBSCAN clustering implementation with
 * grid-acceleration for near-linear runtime on typical LiDAR scans.
 */

#include "dbscan_cluster.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* ── 网格加速结构 ────────────────────────────────────────────── */
/* 将空间划分为 eps×eps 的网格单元，邻居查找只需检查相邻单元 */

#define GRID_MAX_CELLS  256

typedef struct {
    int   cell_x, cell_y;     /* 网格坐标 */
    int   head;               /* 该单元的第一个点索引 */
    int   count;              /* 该单元内的点数 */
} GridCell;

typedef struct {
    float inv_eps;            /* 1/eps */
    int   grid_w, grid_h;     /* 网格维度 */
    float min_x, min_y;       /* 偏移 */
    GridCell cells[GRID_MAX_CELLS];
    int    next_point[DBSCAN_MAX_POINTS]; /* 链表后继 */
} GridIndex;

static int grid_build(GridIndex* g, const Point3D* pts, int n, float eps) {
    if (n < 1) return -1;
    g->inv_eps = 1.0f / eps;

    /* 计算边界 */
    float min_x = pts[0].x, max_x = pts[0].x;
    float min_y = pts[0].y, max_y = pts[0].y;
    for (int i = 1; i < n; i++) {
        if (pts[i].x < min_x) min_x = pts[i].x;
        if (pts[i].x > max_x) max_x = pts[i].x;
        if (pts[i].y < min_y) min_y = pts[i].y;
        if (pts[i].y > max_y) max_y = pts[i].y;
    }

    g->min_x = min_x;
    g->min_y = min_y;
    g->grid_w = (int)((max_x - min_x) * g->inv_eps) + 2;
    g->grid_h = (int)((max_y - min_y) * g->inv_eps) + 2;
    if (g->grid_w < 2) g->grid_w = 2;
    if (g->grid_h < 2) g->grid_h = 2;
    if (g->grid_w * g->grid_h > GRID_MAX_CELLS) {
        g->grid_w = 16; g->grid_h = 16;
    }

    /* 清零网格 */
    int n_cells = g->grid_w * g->grid_h;
    for (int i = 0; i < n_cells; i++) {
        g->cells[i].head = -1;
        g->cells[i].count = 0;
    }

    /* 分配点到网格 */
    for (int i = 0; i < n; i++) {
        int cx = (int)((pts[i].x - min_x) * g->inv_eps);
        int cy = (int)((pts[i].y - min_y) * g->inv_eps);
        if (cx < 0) cx = 0;
        if (cx >= g->grid_w) cx = g->grid_w - 1;
        if (cy < 0) cy = 0;
        if (cy >= g->grid_h) cy = g->grid_h - 1;
        int idx = cy * g->grid_w + cx;
        g->next_point[i] = g->cells[idx].head;
        g->cells[idx].head = i;
        g->cells[idx].count++;
    }

    return 0;
}

/* 检查点 i 的 eps-邻域，将索引写入 neighbors[]，返回数量 */
static int grid_region_query(int* neighbors, int max_nb,
                              const GridIndex* g, const Point3D* pts,
                              int i, float eps) {
    int count = 0;
    const Point3D* pi = &pts[i];
    float eps2 = eps * eps;

    int cx = (int)((pi->x - g->min_x) * g->inv_eps);
    int cy = (int)((pi->y - g->min_y) * g->inv_eps);
    if (cx < 0) cx = 0;
    if (cx >= g->grid_w) cx = g->grid_w - 1;
    if (cy < 0) cy = 0;
    if (cy >= g->grid_h) cy = g->grid_h - 1;

    /* 检查 3×3 邻域 */
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= g->grid_w || ny < 0 || ny >= g->grid_h) continue;
            int cell_idx = ny * g->grid_w + nx;
            int pt_idx = g->cells[cell_idx].head;
            while (pt_idx >= 0) {
                if (count >= max_nb) return count;
                float dx2 = pi->x - pts[pt_idx].x;
                float dy2 = pi->y - pts[pt_idx].y;
                if (dx2 * dx2 + dy2 * dy2 <= eps2)
                    neighbors[count++] = pt_idx;
                pt_idx = g->next_point[pt_idx];
            }
        }
    }
    return count;
}

/* ── 扩张聚类 ────────────────────────────────────────────────── */
static void dbscan_expand(int pt, int cluster_id,
                           int* labels, const Point3D* pts, int n,
                           const GridIndex* grid, float eps, int min_pts) {
    (void)n; /* n not needed: neighborhood bounded by grid cells */
    int seeds[DBSCAN_MAX_POINTS];
    int n_seeds = 0;

    /* 收集 pt 的 eps-邻域 */
    n_seeds = grid_region_query(seeds, DBSCAN_MAX_POINTS, grid, pts, pt, eps);

    labels[pt] = cluster_id;

    for (int s = 0; s < n_seeds; s++) {
        int q = seeds[s];
        if (labels[q] == -2) {      /* 之前标记为噪声 → 边界点 */
            labels[q] = cluster_id;
        }
        if (labels[q] != 0) continue; /* 已访问 */
        labels[q] = cluster_id;

        int nb[DBSCAN_MAX_POINTS];
        int nb_count = grid_region_query(nb, DBSCAN_MAX_POINTS, grid, pts, q, eps);
        if (nb_count >= min_pts) {
            /* 合并邻域 */
            for (int j = 0; j < nb_count; j++) {
                if (n_seeds >= DBSCAN_MAX_POINTS) break;
                seeds[n_seeds++] = nb[j];
            }
        }
    }
}

/* ── 从聚类计算包围盒 ────────────────────────────────────────── */
static void fit_bounding_box(ClusterBounds* cb,
                              const Point3D* pts, const int* indices, int count) {
    if (count == 0) { memset(cb, 0, sizeof(*cb)); return; }

    /* 先用 indices 中第一个点初始化 */
    int first = indices[0];
    float min_x = pts[first].x, max_x = pts[first].x;
    float min_y = pts[first].y, max_y = pts[first].y;
    float min_z = pts[first].z, max_z = pts[first].z;
    float sum_x = 0, sum_y = 0;

    for (int i = 0; i < count; i++) {
        const Point3D* p = &pts[indices[i]];
        if (p->x < min_x) min_x = p->x;
        if (p->x > max_x) max_x = p->x;
        if (p->y < min_y) min_y = p->y;
        if (p->y > max_y) max_y = p->y;
        if (p->z < min_z) min_z = p->z;
        if (p->z > max_z) max_z = p->z;
        sum_x += p->x;
        sum_y += p->y;
    }

    cb->cx      = sum_x / (float)count;
    cb->cy      = sum_y / (float)count;
    cb->width   = max_x - min_x;
    cb->length  = max_y - min_y;
    cb->min_z   = min_z;
    cb->max_z   = max_z;
    cb->point_count = count;

    /* 置信度: 基于点密度 (点数/包围盒面积) */
    float area = cb->width * cb->length;
    float density = (area > 0.01f) ? (float)count / area : (float)count;
    if (density > 50.0f)      cb->confidence = 0.95f;
    else if (density > 20.0f) cb->confidence = 0.80f;
    else if (density > 5.0f)  cb->confidence = 0.50f;
    else                      cb->confidence = 0.30f;

    /* 启发式分类 */
    if (cb->width < 1.5f && cb->length < 1.5f && cb->max_z < 2.0f) {
        cb->cls = CLS_PEDESTRIAN;
    } else if (cb->width < 3.5f && cb->length < 8.0f && cb->max_z < 3.0f) {
        cb->cls = CLS_VEHICLE;
    } else if (cb->width < 1.5f && cb->length < 2.5f) {
        cb->cls = CLS_CYCLIST;
    } else {
        cb->cls = CLS_UNKNOWN;
    }
}

/* ══════════════════════════════════════════════════════════ */
/*  Public API                                                */
/* ══════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════ */
/*  RANSAC 地面平面拟合                                        */
/* ══════════════════════════════════════════════════════════ */

/* 3D vector cross product */
static void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* vector magnitude */
static float norm3(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/* Fit a plane from 3 points. Returns 0 if points are collinear or plane is too vertical. */
static int plane_from_3pts(const Point3D* p1, const Point3D* p2, const Point3D* p3,
                            float plane[4]) {
    float v1[3] = {p2->x - p1->x, p2->y - p1->y, p2->z - p1->z};
    float v2[3] = {p3->x - p1->x, p3->y - p1->y, p3->z - p1->z};
    float n[3];
    cross3(v1, v2, n);
    float mag = norm3(n);
    if (mag < 1e-6f) return 0; /* collinear */

    n[0] /= mag; n[1] /= mag; n[2] /= mag;

    /* Require near-horizontal plane: normal must point mostly upward (z component > 0.7) */
    if (n[2] < 0) { n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2]; }
    if (n[2] < 0.7f) return 0;  /* too tilted */

    plane[0] = n[0]; plane[1] = n[1]; plane[2] = n[2];
    plane[3] = -(n[0]*p1->x + n[1]*p1->y + n[2]*p1->z);
    return 1;
}

/* RANSAC ground plane fitting */
static int ransac_fit_ground(Point3D* points, int n, int max_iter,
                              float dist_thresh, float min_inlier_ratio,
                              float plane_out[4]) {
    if (n < 10) return -1;

    int best_count = 0;
    float best_plane[4] = {0, 0, 1, 0};
    int min_inliers = (int)((float)n * min_inlier_ratio);
    if (min_inliers < 10) min_inliers = 10;

    for (int iter = 0; iter < max_iter; iter++) {
        /* Pick 3 random points across the entire scan — use a stratified
         * approach: take 2 from the lower half (more likely ground) and
         * 1 from random position to anchor the plane */
        int i1 = rand() % (n / 2);          /* lower half — likely ground */
        int i2 = rand() % (n / 2);          /* lower half */
        int i3 = rand() % n;                /* anywhere */
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        /* Ensure points have some spread: at least 1m apart */
        float d12 = (points[i1].x-points[i2].x)*(points[i1].x-points[i2].x)
                  + (points[i1].y-points[i2].y)*(points[i1].y-points[i2].y);
        float d23 = (points[i2].x-points[i3].x)*(points[i2].x-points[i3].x)
                  + (points[i2].y-points[i3].y)*(points[i2].y-points[i3].y);
        if (d12 < 1.0f || d23 < 1.0f) continue;

        float plane[4];
        if (!plane_from_3pts(&points[i1], &points[i2], &points[i3], plane))
            continue;

        /* Quick count — full inlier check only if promising */
        int quick_count = 0;
        for (int j = 0; j < 50 && j < n; j++) {
            int idx = rand() % n;
            float d = plane[0]*points[idx].x + plane[1]*points[idx].y
                    + plane[2]*points[idx].z + plane[3];
            if (d*d <= dist_thresh*dist_thresh) quick_count++;
        }
        if (quick_count < 20) continue; /* unlikely to be the ground plane */

        /* Full inlier count */
        int count = 0;
        for (int i = 0; i < n; i++) {
            float d = plane[0]*points[i].x + plane[1]*points[i].y
                    + plane[2]*points[i].z + plane[3];
            if (d*d <= dist_thresh*dist_thresh) count++;
        }

        if (count > best_count) {
            best_count = count;
            best_plane[0] = plane[0]; best_plane[1] = plane[1];
            best_plane[2] = plane[2]; best_plane[3] = plane[3];
            /* Early exit if enough inliers found */
            if (count >= min_inliers && (float)count / (float)n > 0.5f) break;
        }
    }

    if (best_count < min_inliers) return -1;

    plane_out[0] = best_plane[0]; plane_out[1] = best_plane[1];
    plane_out[2] = best_plane[2]; plane_out[3] = best_plane[3];
    return best_count;
}

/* RANSAC ground removal: filter out inlier points */
static int ransac_remove_ground(Point3D* points, int n,
                                 const float plane[4], float dist_thresh) {
    int nf = 0;
    float thresh2 = dist_thresh * dist_thresh;
    for (int i = 0; i < n; i++) {
        float d = plane[0]*points[i].x + plane[1]*points[i].y
                + plane[2]*points[i].z + plane[3];
        if (d*d > thresh2) {  /* NOT ground → keep */
            if (i != nf) points[nf] = points[i];
            nf++;
        }
    }
    return nf;
}

/* ══════════════════════════════════════════════════════════ */
/*  Public API                                                */
/* ══════════════════════════════════════════════════════════ */

void dbscan_init(DbscanCluster* db, float eps, int min_pts) {
    memset(db, 0, sizeof(*db));
    db->eps = eps;
    db->min_pts = min_pts;
    db->ground_z_thresh = 0.3f;
    /* RANSAC defaults */
    db->ground_mode            = GROUND_REMOVE_RANSAC;
    db->ransac_max_iter        = 100;
    db->ransac_dist_thresh     = 0.2f;
    db->ransac_min_inlier_ratio = 0.3f;
}

void dbscan_set_ground_thresh(DbscanCluster* db, float z_thresh) {
    db->ground_z_thresh = z_thresh;
    db->ground_mode = GROUND_REMOVE_ZCUT;
}

void dbscan_set_ransac(DbscanCluster* db, int max_iter,
                        float dist_thresh, float min_inlier_ratio) {
    db->ground_mode            = GROUND_REMOVE_RANSAC;
    db->ransac_max_iter        = max_iter;
    db->ransac_dist_thresh     = dist_thresh;
    db->ransac_min_inlier_ratio = min_inlier_ratio;
}

int dbscan_run(DbscanCluster* db, Point3D* points, int n_points) {
    if (n_points < 1 || n_points > DBSCAN_MAX_POINTS) return 0;

    /* ── 第1步: 地面去除 ── */
    int nf = n_points;

    if (db->ground_mode == GROUND_REMOVE_RANSAC) {
        /* RANSAC 地面平面拟合 + 移除 */
        float plane[4];
        int n_ground = ransac_fit_ground(points, n_points,
                                          db->ransac_max_iter,
                                          db->ransac_dist_thresh,
                                          db->ransac_min_inlier_ratio,
                                          plane);
        if (n_ground > 0) {
            nf = ransac_remove_ground(points, n_points, plane, db->ransac_dist_thresh);
            db->ground_plane[0] = plane[0]; db->ground_plane[1] = plane[1];
            db->ground_plane[2] = plane[2]; db->ground_plane[3] = plane[3];
        }
        /* Fallback: if RANSAC fails, use robust z-percentile threshold */
        if (n_ground < 10 && n_points > 100) {
            /* Sample z values and find the 30th percentile (≈ ground level) */
            float zsamples[200];
            int zcnt = 0;
            for (int i = 0; i < n_points && zcnt < 200; i += n_points / 200) {
                if (n_points / 200 < 1) break;
                zsamples[zcnt++] = points[i].z;
            }
            /* Simple sort for percentile */
            for (int i = 0; i < zcnt - 1; i++)
                for (int j = i + 1; j < zcnt; j++)
                    if (zsamples[i] > zsamples[j]) {
                        float t = zsamples[i]; zsamples[i] = zsamples[j]; zsamples[j] = t;
                    }
            float z_cut = zsamples[zcnt * 2 / 10];  /* 20th percentile as ground cutoff */
            /* Keep points above ground + margin */
            nf = 0;
            for (int i = 0; i < n_points; i++) {
                if (points[i].z > z_cut + 0.25f) {
                    if (i != nf) points[nf] = points[i];
                    nf++;
                }
            }
            /* Record approximate ground plane for diagnostics */
            db->ground_plane[0] = 0; db->ground_plane[1] = 0;
            db->ground_plane[2] = 1; db->ground_plane[3] = -z_cut;
        }
    } else if (db->ground_mode == GROUND_REMOVE_ZCUT) {
        /* 简单 z 阈值 */
        nf = 0;
        for (int i = 0; i < n_points; i++) {
            if (points[i].z > db->ground_z_thresh) {
                if (i != nf) points[nf] = points[i];
                nf++;
            }
        }
    }
    /* else GROUND_REMOVE_NONE: nf = n_points */

    if (nf < db->min_pts) return 0;

    /* ── 第2步: 构建网格索引 ── */
    GridIndex grid;
    if (grid_build(&grid, points, nf, db->eps) != 0) return 0;

    /* ── 第3步: DBSCAN ── */
    /* labels: 0=未访问, -1=噪声, -2=待确认噪声, >0=聚类ID */
    memset(db->labels, 0, sizeof(int) * nf);

    int cluster_id = 0;
    for (int i = 0; i < nf; i++) {
        if (db->labels[i] != 0) continue;

        int nb[DBSCAN_MAX_POINTS];
        int nb_count = grid_region_query(nb, DBSCAN_MAX_POINTS, &grid, points, i, db->eps);

        if (nb_count < db->min_pts) {
            db->labels[i] = -1;  /* 噪声 */
        } else {
            cluster_id++;
            dbscan_expand(i, cluster_id, db->labels, points, nf,
                          &grid, db->eps, db->min_pts);
        }
    }

    /* ── 第4步: 拟合包围盒 ── */
    /* 收集每个聚类的点索引 (heap allocation — LiDAR scans can be large) */
    int* cluster_pts[DBSCAN_MAX_CLUSTERS];
    int  cluster_counts[DBSCAN_MAX_CLUSTERS];
    int* cluster_buf = (int*)calloc((size_t)cluster_id * (size_t)nf, sizeof(int));

    for (int c = 0; c < cluster_id && c < DBSCAN_MAX_CLUSTERS; c++) {
        cluster_counts[c] = 0;
        cluster_pts[c] = cluster_buf ? cluster_buf + (size_t)c * nf : NULL;
    }

    for (int i = 0; i < nf; i++) {
        int lbl = db->labels[i];
        if (lbl <= 0 || lbl > DBSCAN_MAX_CLUSTERS) continue;
        int ci = lbl - 1;
        if (cluster_pts[ci] && cluster_counts[ci] < nf)
            cluster_pts[ci][cluster_counts[ci]++] = i;
    }

    int valid = 0;
    for (int c = 0; c < cluster_id && c < DBSCAN_MAX_CLUSTERS; c++) {
        if (cluster_counts[c] < db->min_pts) continue;
        fit_bounding_box(&db->clusters[valid], points, cluster_pts[c], cluster_counts[c]);
        if (db->clusters[valid].width < 0.2f && db->clusters[valid].length < 0.2f) continue;
        valid++;
    }
    free(cluster_buf);

    db->n_clusters = valid;
    return valid;
}

const ClusterBounds* dbscan_get_cluster(const DbscanCluster* db, int idx) {
    if (idx < 0 || idx >= db->n_clusters) return NULL;
    return &db->clusters[idx];
}

int dbscan_cluster_count(const DbscanCluster* db) {
    return db->n_clusters;
}
