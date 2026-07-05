#include "Obstacle.h"
#include <cmath>

using namespace Eigen;
using namespace std;

/* ── line-segment intersection (replaces Qt's QLineF) ──────── */
/* Returns true if segments (a1,a2) and (b1,b2) intersect */
static int orientation(float ax, float ay, float bx, float by, float cx, float cy) {
    float val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
    if (fabsf(val) < 1e-9f) return 0;        /* collinear */
    return (val > 0) ? 1 : 2;                 /* 1=clockwise, 2=counterclockwise */
}

static bool on_segment(float ax, float ay, float bx, float by, float px, float py) {
    return (px <= fmaxf(ax, bx) && px >= fminf(ax, bx) &&
            py <= fmaxf(ay, by) && py >= fminf(ay, by));
}

static bool segments_intersect(float p1x, float p1y, float p2x, float p2y,
                                float q1x, float q1y, float q2x, float q2y) {
    int o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
    int o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
    int o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
    int o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

    if (o1 != o2 && o3 != o4) return true;           /* general case */
    if (o1 == 0 && on_segment(p1x, p1y, p2x, p2y, q1x, q1y)) return true;
    if (o2 == 0 && on_segment(p1x, p1y, p2x, p2y, q2x, q2y)) return true;
    if (o3 == 0 && on_segment(q1x, q1y, q2x, q2y, p1x, p1y)) return true;
    if (o4 == 0 && on_segment(q1x, q1y, q2x, q2y, p2x, p2y)) return true;
    return false;
}

/* ── Obstacle implementation ───────────────────────────────── */

Obstacle::Obstacle(Vector2f first_point, Vector2f second_point, double obstacle_clearance)
{
    Vector2f tmp;
    if (first_point.x() > second_point.x() && first_point.y() > second_point.y()) {
        tmp = first_point;
        first_point = second_point;
        second_point = tmp;
    } else if (first_point.x() < second_point.x() && first_point.y() > second_point.y()) {
        float height = first_point.y() - second_point.y();
        first_point.y() -= height;
        second_point.y() += height;
    } else if (first_point.x() > second_point.x() && first_point.y() < second_point.y()) {
        float length = first_point.x() - second_point.x();
        first_point.x() -= length;
        second_point.x() += length;
    }
    first_point.x() -= obstacle_clearance;
    first_point.y() -= obstacle_clearance;
    second_point.x() += obstacle_clearance;
    second_point.y() += obstacle_clearance;

    bbox.first.x() = first_point.x();
    bbox.first.y() = first_point.y();
    bbox.second.x() = second_point.x();
    bbox.second.y() = second_point.y();
}

bool Obstacle::isSegmentInObstacle(Vector2f &p1, Vector2f &p2)
{
    float length = bbox.second.x() - bbox.first.x();
    float breadth = bbox.second.y() - bbox.first.y();

    /* Check intersection with each of the 4 edges of the bounding box */
    if (segments_intersect(p1.x(), p1.y(), p2.x(), p2.y(),
            bbox.first.x(), bbox.first.y(),
            bbox.first.x() + length, bbox.first.y())) return true;

    if (segments_intersect(p1.x(), p1.y(), p2.x(), p2.y(),
            bbox.first.x(), bbox.first.y(),
            bbox.first.x(), bbox.first.y() + breadth)) return true;

    if (segments_intersect(p1.x(), p1.y(), p2.x(), p2.y(),
            bbox.second.x(), bbox.second.y(),
            bbox.second.x(), bbox.second.y() - breadth)) return true;

    if (segments_intersect(p1.x(), p1.y(), p2.x(), p2.y(),
            bbox.second.x(), bbox.second.y(),
            bbox.second.x() - length, bbox.second.y())) return true;

    return false;
}

bool Obstacle::isPointNearObstacle(Vector2f &p, double radius) {
    double dist_to_ll, dist_to_lr, dist_to_ul, dist_to_ur;
    dist_to_ll = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_lr = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_ul = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    dist_to_ur = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    if (dist_to_ll <= radius || dist_to_lr <= radius ||
        dist_to_ul <= radius || dist_to_ur <= radius ) {
        return true;
    }
    return false;
}
