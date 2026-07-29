#ifndef SCENARIO_ROUTER_H
#define SCENARIO_ROUTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROUTER_MAX_LANES   256
#define ROUTER_MAX_EDGES   512
#define ROUTER_MAX_PATH    64

/* 车道段 */
typedef struct {
    int     id;             /* 车道段唯一 ID */
    double  start_x;        /* 起点 x (m) */
    double  end_x;          /* 终点 x (m) */
    int     lane_idx;       /* 车道索引 (0=最左) */
    int     road_id;        /* 所在道路段 ID */
    double  speed_limit;    /* 限速 (m/s) */
    double  length;         /* 车道段长度 (m) */
} RouterLane;

/* 拓扑边 */
typedef struct {
    int from_id;            /* 源车道段 ID */
    int to_id;              /* 目标车道段 ID */
    int type;               /* 0=后继, 1=左邻, 2=右邻 */
    double cost;            /* 通行代价 */
} RouterEdge;

/* 路由图 */
typedef struct {
    RouterLane  lanes[ROUTER_MAX_LANES];
    int         lane_count;
    RouterEdge  edges[ROUTER_MAX_EDGES];
    int         edge_count;
} RouterGraph;

/* 路由结果 */
typedef struct {
    int lane_ids[ROUTER_MAX_PATH];   /* 车道段 ID 序列 */
    int count;
    double total_cost;
} RouterPath;

/**
 * 初始化路由图（清空所有车道和边）。
 */
void router_graph_init(RouterGraph* g);

/**
 * 添加车道段。
 */
int router_add_lane(RouterGraph* g, int id, double start_x, double end_x,
                    int lane_idx, int road_id, double speed_limit);

/**
 * 添加拓扑边。type=0(后继), 1(左邻), 2(右邻)。
 * cost 自动按 length + lane_change_penalty 计算。
 */
int router_add_edge(RouterGraph* g, int from_id, int to_id, int type);

/**
 * 自动构建车道拓扑：为同一lane_idx的相邻车道段加后继边，
 * 为同一road_id的相邻lane_idx加左右邻边。
 * lane_change_penalty: 变道惩罚代价（米等效）。
 */
void router_build_topology(RouterGraph* g, double lane_change_penalty);

/**
 * A* 路径搜索。
 *
 * @param g         路由图
 * @param from_id   起点车道段 ID
 * @param to_id     终点车道段 ID
 * @param path      输出路径
 * @return          0=成功, -1=无路可达
 */
int router_astar(const RouterGraph* g, int from_id, int to_id, RouterPath* path);

/**
 * 获取指定位置所在的车道段 ID。
 * 遍历所有车道段，找到包含 x 坐标的车道段。
 * @return 车道段 ID, -1=未找到
 */
int router_lane_at(const RouterGraph* g, int road_id, int lane_idx, double x);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_ROUTER_H */
