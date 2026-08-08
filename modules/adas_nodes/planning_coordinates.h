#ifndef FLOWENGINE_PLANNING_COORDINATES_H
#define FLOWENGINE_PLANNING_COORDINATES_H

#include <algorithm>
#include <cmath>
#include <vector>

namespace planning_coord {

struct Projection {
    double s{0.0};
    double d{0.0};
    double ref_x{0.0};
    double ref_y{0.0};
    double heading{0.0};
};

inline double lane_center_d(int lane_idx, int lane_count, double lane_width) {
    return -(lane_idx - (lane_count - 1) * 0.5) * lane_width;
}

inline int nearest_lane(double d, int lane_count, double lane_width,
                        bool own_side_only = true) {
    if (lane_count <= 0 || lane_width <= 0.0) return 0;
    double raw = (-d) / lane_width + (lane_count - 1) * 0.5;
    int lane = static_cast<int>(raw >= 0.0 ? raw + 0.5 : raw - 0.5);
    lane = std::max(own_side_only ? lane_count / 2 : 0, lane);
    return std::min(lane_count - 1, lane);
}

inline int nearest_own_lane(double d, int lane_count, double lane_width) {
    return nearest_lane(d, lane_count, lane_width, true);
}

inline bool project_to_path(double x, double y,
                            const double* ref_x, const double* ref_y,
                            const double* ref_s, int count,
                            Projection& out) {
    if (!ref_x || !ref_y || !ref_s || count < 2) return false;

    double best_dist2 = 1e300;
    bool found = false;
    for (int i = 0; i + 1 < count; ++i) {
        const double vx = ref_x[i + 1] - ref_x[i];
        const double vy = ref_y[i + 1] - ref_y[i];
        const double len2 = vx * vx + vy * vy;
        if (len2 <= 1e-9) continue;

        const double t = std::clamp(
            ((x - ref_x[i]) * vx + (y - ref_y[i]) * vy) / len2, 0.0, 1.0);
        const double px = ref_x[i] + t * vx;
        const double py = ref_y[i] + t * vy;
        const double dx = x - px;
        const double dy = y - py;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 >= best_dist2) continue;

        const double heading = std::atan2(vy, vx);
        out.s = ref_s[i] + t * std::sqrt(len2);
        out.d = -dx * std::sin(heading) + dy * std::cos(heading);
        out.ref_x = px;
        out.ref_y = py;
        out.heading = heading;
        best_dist2 = dist2;
        found = true;
    }
    return found;
}

inline bool quintic_lane_change(double start_d, double target_d,
                                double length, double s, double& out_d) {
    if (length <= 1e-6) return false;
    const double delta = target_d - start_d;
    if (std::fabs(delta) <= 0.2 || std::fabs(delta) >= 8.0) return false;
    const double u = std::clamp(s / length, 0.0, 1.0);
    const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    out_d = start_d + delta * blend;
    return true;
}

}  // namespace planning_coord

#endif
