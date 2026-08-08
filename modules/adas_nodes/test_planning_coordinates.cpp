#include "planning_coordinates.h"

#include <cassert>
#include <cmath>

static bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

int main() {
    assert(near(planning_coord::lane_center_d(0, 4, 3.5), 5.25));
    assert(near(planning_coord::lane_center_d(2, 4, 3.5), -1.75));
    assert(planning_coord::nearest_own_lane(6.0, 4, 3.5) == 2);
    assert(planning_coord::nearest_own_lane(-6.0, 4, 3.5) == 3);
    assert(planning_coord::nearest_lane(1.75, 4, 3.5, false) == 1);

    const double x[] = {0.0, 10.0, 20.0};
    const double y[] = {0.0, 10.0, 20.0};
    const double s[] = {0.0, std::sqrt(200.0), 2.0 * std::sqrt(200.0)};
    planning_coord::Projection p;
    assert(planning_coord::project_to_path(5.0, 7.0, x, y, s, 3, p));
    assert(near(p.d, std::sqrt(2.0)));
    assert(near(p.ref_x, 6.0));
    assert(near(p.ref_y, 6.0));

    double d = 0.0;
    assert(planning_coord::quintic_lane_change(-5.25, -1.75, 50.0, 0.0, d));
    assert(near(d, -5.25));
    assert(planning_coord::quintic_lane_change(-5.25, -1.75, 50.0, 25.0, d));
    assert(near(d, -3.5));
    assert(planning_coord::quintic_lane_change(-5.25, -1.75, 50.0, 50.0, d));
    assert(near(d, -1.75));
    return 0;
}
