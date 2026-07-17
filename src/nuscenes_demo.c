/**
 * nuscenes_demo.c — nuScenes LiDAR + DBSCAN 验证工具
 *
 * 加载 nuScenes LiDAR 点云 → DBSCAN 聚类 → 与 GT 标注对比。
 *
 * 用法:
 *   flow_nuscenes_demo <lidar.bin> [annotations.json] [eps] [min_pts]
 *
 * 示例:
 *   flow_nuscenes_demo data/nuscenes/samples/LIDAR_TOP/xxx.bin
 *   flow_nuscenes_demo xxx.bin data/nuscenes/v1.0-mini/sample_annotation.json 2.0 5
 */

#include "dbscan_cluster.h"
#include "nuscenes_loader.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── 比较 DBSCAN 结果与 GT 标注 ──────────────────────────────── */

static void compare_with_gt(const DbscanCluster* db,
                             const NuScenesAnnotation* annots, int n_annots) {
    int nc = dbscan_cluster_count(db);
    printf("\n=== DBSCAN vs Ground Truth ===\n");
    printf("  DBSCAN: %d clusters\n", nc);
    printf("  GT:     %d annotations\n", n_annots);

    int matched = 0;
    int missed  = 0;
    int extra   = 0;

    /* 简单 IoU 匹配 (2D, 距离 < 2m 视为匹配) */
    for (int g = 0; g < n_annots; g++) {
        int found = 0;
        for (int c = 0; c < nc; c++) {
            const ClusterBounds* cb = dbscan_get_cluster(db, c);
            if (!cb) continue;
            double dx = cb->cx - annots[g].tx;
            double dy = cb->cy - annots[g].ty;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < 3.0) { found = 1; break; }  /* 3m threshold */
        }
        if (found) matched++;
        else missed++;
    }

    extra = nc - matched;
    if (extra < 0) extra = 0;

    printf("  Matched: %d / Missed: %d / Extra: %d\n", matched, missed, extra);
    printf("  Detection rate: %.0f%%\n",
           n_annots > 0 ? 100.0 * (double)matched / (double)n_annots : 0.0);
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <lidar.bin> [annotations.json] [eps=2.0] [min_pts=5]\n", argv[0]);
        fprintf(stderr, "  Download nuScenes mini: bash scripts/download_nuscenes_mini.sh\n");
        return 1;
    }

    const char* lidar_path = argv[1];
    const char* annot_path = (argc > 2) ? argv[2] : NULL;
    float eps     = (argc > 3) ? (float)atof(argv[3]) : 2.0f;
    int   min_pts = (argc > 4) ? atoi(argv[4])      : 5;

    log_init(LOG_INFO, NULL);

    /* ── 加载 LiDAR ── */
    LOG_INFO("nuscenes", "Loading LiDAR: %s", lidar_path);
    NuScenesScan scan;
    if (nuscenes_load_lidar(lidar_path, &scan) != 0) {
        fprintf(stderr, "Failed to load %s\n", lidar_path);
        return 1;
    }
    nuscenes_print_stats(&scan);

    /* ── 转为 DBSCAN 格式 ── */
    int n_pts = scan.count;
    if (n_pts > DBSCAN_MAX_POINTS) n_pts = DBSCAN_MAX_POINTS;

    Point3D* pts = (Point3D*)malloc((size_t)n_pts * sizeof(Point3D));
    for (int i = 0; i < n_pts; i++) {
        pts[i].x = scan.points[i].x;
        pts[i].y = scan.points[i].y;
        pts[i].z = scan.points[i].z;
        pts[i].intensity = scan.points[i].intensity;
    }

    /* ── DBSCAN (默认 RANSAC 地面去除) ── */
    DbscanCluster db;
    dbscan_init(&db, eps, min_pts);
    /* RANSAC: 100 iterations, 0.2m distance threshold, >30% inliers */
    dbscan_set_ransac(&db, 100, 0.2f, 0.3f);

    int n_clusters = dbscan_run(&db, pts, n_pts);

    LOG_INFO("nuscenes", "DBSCAN: %d pts → %d clusters (eps=%.1f min_pts=%d)",
             n_pts, n_clusters, eps, min_pts);

    /* ── 打印聚类结果 ── */
    for (int i = 0; i < n_clusters; i++) {
        const ClusterBounds* cb = dbscan_get_cluster(&db, i);
        if (!cb) continue;
        const char* cls_name = "unknown";
        switch (cb->cls) {
            case CLS_VEHICLE:    cls_name = "vehicle";    break;
            case CLS_PEDESTRIAN: cls_name = "pedestrian"; break;
            case CLS_CYCLIST:    cls_name = "cyclist";    break;
            default: break;
        }
        printf("  [%d] %s @ (%.1f,%.1f) size=(%.1f,%.1f) pts=%d conf=%.0f%%\n",
               i, cls_name, cb->cx, cb->cy, cb->width, cb->length,
               cb->point_count, cb->confidence * 100.0f);
    }

    /* ── 加载 GT 标注并对比 ── */
    if (annot_path) {
        NuScenesAnnotation* annots = NULL;
        int n_annots = 0;
        if (nuscenes_load_annotations(annot_path, &annots, &n_annots) == 0 && n_annots > 0) {
            compare_with_gt(&db, annots, n_annots);
            free(annots);
        } else {
            LOG_WARN("nuscenes", "No GT annotations loaded from %s", annot_path);
        }
    }

    /* ── ASCII 俯视图 ── */
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║  Top-Down View (LiDAR + DBSCAN Clusters)         ║\n");
    printf("║  ▶ = ego    █ = vehicle    ● = pedestrian       ║\n");
    printf("║  ○ = cyclist    · = non-ground point             ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    /* 80×40 字符网格, 覆盖 -30..90m 纵向, -20..20m 横向 */
    int gw = 80, gh = 40;
    double xmin = -30, xmax = 90, ymin = -20, ymax = 20;
    double xs = (xmax - xmin) / gw, ys = (ymax - ymin) / gh;
    char grid[40][81];
    memset(grid, ' ', sizeof(grid));

    /* 画地面点 (下采样) */
    for (int i = 0; i < n_pts; i += n_pts / 800 + 1) {
        int gx = (int)((pts[i].x - xmin) / xs);
        int gy = (int)((pts[i].y - ymin) / ys);
        if (gx >= 0 && gx < gw && gy >= 0 && gy < gh)
            grid[gy][gx] = '.';
    }

    /* 画聚类包围盒 */
    for (int i = 0; i < n_clusters; i++) {
        const ClusterBounds* cb = dbscan_get_cluster(&db, i);
        if (!cb || cb->point_count < 5) continue;
        char marker = '?';
        switch (cb->cls) {
            case CLS_VEHICLE:    marker = '#'; break;
            case CLS_PEDESTRIAN: marker = 'O'; break;
            case CLS_CYCLIST:    marker = 'o'; break;
            default:             marker = '+'; break;
        }
        /* 画包围盒边框 */
        int x0 = (int)((cb->cx - cb->length/2 - xmin) / xs);
        int x1 = (int)((cb->cx + cb->length/2 - xmin) / xs);
        int y0 = (int)((cb->cy - cb->width/2 - ymin) / ys);
        int y1 = (int)((cb->cy + cb->width/2 - ymin) / ys);
        if (x0 < 0) x0 = 0;
        if (x1 >= gw) x1 = gw-1;
        if (y0 < 0) y0 = 0;
        if (y1 >= gh) y1 = gh-1;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                grid[y][x] = (y==y0||y==y1||x==x0||x==x1) ? marker : grid[y][x];
        /* 中心标注 */
        int cx = (int)((cb->cx - xmin) / xs), cy = (int)((cb->cy - ymin) / ys);
        if (cx >= 0 && cx < gw && cy >= 0 && cy < gh) grid[cy][cx] = marker;
    }

    /* 自车位置 */
    int ex = (int)((-xmin) / xs), ey = (int)((-ymin) / ys);
    if (ex >= 0 && ex < gw && ey >= 0 && ey < gh) grid[ey][ex] = '>';

    /* 渲染 */
    printf("  y=+20m\n");
    for (int y = gh-1; y >= 0; y--) {
        grid[y][gw] = '\0';
        printf("  %s\n", grid[y]);
    }
    printf("  y=-20m  x=-30m");
    for (int i = 20; i < gw; i += 10) printf("─");
    printf(" x=+90m\n");

    /* ── HTML 导出 ── */
    if (argc > 5 && strcmp(argv[5], "--html") == 0) {
        const char* html_path = argc > 6 ? argv[6] : "/tmp/lidar_view.html";
        FILE* hf = fopen(html_path, "w");
        if (hf) {
            fprintf(hf, "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<title>LiDAR + DBSCAN</title><style>"
                "body{background:#111;color:#eee;font:14px monospace;margin:20px}"
                "canvas{background:#000;border:1px solid #333}"
                ".info{margin:10px 0}</style></head><body>"
                "<h2>KITTI LiDAR + DBSCAN Clusters</h2>"
                "<div class='info'>%d points → %d clusters | "
                "eps=%.1f min_pts=%d | "
                "<span style='color:#0f0'>█ vehicle</span> "
                "<span style='color:#ff0'>● pedestrian</span> "
                "<span style='color:#0ff'>○ cyclist</span></div>"
                "<canvas id='c' width='1000' height='500'></canvas>"
                "<script>"
                "var C=document.getElementById('c'),ctx=C.getContext('2d');"
                "var xmin=%.0f,xmax=%.0f,ymin=%.0f,ymax=%.0f;"
                "function tx(x){return(x-xmin)/(xmax-xmin)*1000;}"
                "function ty(y){return 500-(y-ymin)/(ymax-ymin)*500;}"
                /* 画地面点 (半透明) */
                "ctx.fillStyle='rgba(100,255,100,0.03)';",
                n_pts, n_clusters, eps, min_pts,
                xmin, xmax, ymin, ymax);

            for (int i = 0; i < n_pts; i += n_pts / 2000 + 1)
                fprintf(hf, "ctx.fillRect(tx(%.1f),ty(%.1f),1,1);", pts[i].x, pts[i].y);

            /* 画聚类 */
            const char* colors[] = {"#0f0","#ff0","#0ff","#f80","#f0f","#08f","#8f0","#f00"};
            fprintf(hf, "var clrs=['%s'", colors[0]);
            for (int i = 1; i < 8; i++) fprintf(hf, ",'%s'", colors[i]);
            fprintf(hf, "];");

            for (int i = 0; i < n_clusters && i < 8; i++) {
                const ClusterBounds* cb = dbscan_get_cluster(&db, i);
                if (!cb || cb->point_count < 3) continue;
                fprintf(hf,
                    "ctx.strokeStyle=clrs[%d];ctx.lineWidth=2;"
                    "ctx.strokeRect(tx(%.1f),ty(%.1f),%.0f,%.0f);"
                    "ctx.fillStyle=clrs[%d];ctx.font='11px monospace';"
                    "ctx.fillText('%s(%d)',tx(%.1f)+2,ty(%.1f)-4);",
                    i % 8,
                    cb->cx - cb->length/2, cb->cy + cb->width/2,
                    cb->length / (xmax-xmin) * 1000,
                    -cb->width / (ymax-ymin) * 500,
                    i % 8,
                    cb->cls == CLS_VEHICLE ? "V" : cb->cls == CLS_PEDESTRIAN ? "P" : "C",
                    cb->point_count,
                    cb->cx, cb->cy);
            }
            fprintf(hf, "</script></body></html>");
            fclose(hf);
            printf("\n  HTML exported: %s  (open in browser)\n", html_path);
        }
    }

    /* ── 清理 ── */
    free(pts);
    nuscenes_free_scan(&scan);
    log_shutdown();

    return 0;
}
