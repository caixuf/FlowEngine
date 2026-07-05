/**
 * frenet_bridge.cpp — C wrapper around the open-source Frenet planner.
 * Compiles as C++ but exports a pure C API via extern "C".
 */

#include "frenet_bridge.h"

#include <vector>
#include <cstring>
#include <cmath>

/* Open-source Frenet planner (Apache-2.0) */
#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"

/* Declare run_fot from fot_wrapper.cpp (no header provided upstream) */
extern "C" void run_fot(FrenetInitialConditions*, FrenetHyperparameters*, FrenetReturnValues*);

using namespace std;

struct FrenetHandle {
    FrenetHyperparameters          hp;
    vector<double>                 wx, wy;
    vector<double>                 ox, oy, ow, ol;
    bool                           path_set;
};

extern "C" {

FrenetHandle* frenet_create(double max_speed, double max_accel) {
    FrenetHandle* fh = new FrenetHandle();
    memset(&fh->hp, 0, sizeof(fh->hp));
    fh->hp.max_speed          = max_speed;
    fh->hp.max_accel          = max_accel;
    fh->hp.max_curvature      = 0.3;
    fh->hp.max_road_width_l   = 7.0;
    fh->hp.max_road_width_r   = 7.0;
    fh->hp.d_road_w           = 1.5;
    fh->hp.dt                 = 0.25;
    fh->hp.maxt               = 6.0;
    fh->hp.mint               = 2.0;
    fh->hp.d_t_s              = 2.0;
    fh->hp.n_s_sample         = 3.0;
    fh->hp.obstacle_clearance = 1.0;
    fh->hp.kd                 = 1.0;
    fh->hp.kv                 = 0.5;
    fh->hp.ka                 = 0.3;
    fh->hp.kj                 = 0.1;
    fh->hp.kt                 = 0.3;
    fh->hp.ko                 = 1.5;
    fh->hp.klat               = 0.8;
    fh->hp.klon               = 0.5;
    fh->path_set              = false;
    return fh;
}

void frenet_set_reference_path(FrenetHandle* fh, const double* wx, const double* wy, int n) {
    if (!fh || n < 3) return;
    fh->wx.assign(wx, wx + n);
    fh->wy.assign(wy, wy + n);
    fh->path_set = true;
}

void frenet_set_obstacles(FrenetHandle* fh,
                           const double* ox, const double* oy,
                           const double* ow, const double* ol, int n) {
    if (!fh) return;
    fh->ox.assign(ox, ox + n);
    fh->oy.assign(oy, oy + n);
    fh->ow.assign(ow, ow + n);
    fh->ol.assign(ol, ol + n);
}

int frenet_plan(FrenetHandle* fh,
                 double ego_s, double ego_d, double ego_speed,
                 double target_speed,
                 double* out_s, double* out_d, double* out_speed,
                 int max_pts) {
    if (!fh || !fh->path_set || max_pts < 1) return 0;

    /* Build initial conditions */
    FrenetInitialConditions ic;
    memset(&ic, 0, sizeof(ic));
    ic.s0           = ego_s;
    ic.c_speed      = ego_speed;
    ic.c_d          = ego_d;
    ic.c_d_d        = 0.0;
    ic.c_d_dd       = 0.0;
    ic.target_speed = target_speed;
    ic.wx           = const_cast<double*>(fh->wx.data());
    ic.wy           = const_cast<double*>(fh->wy.data());
    ic.nw           = (int)fh->wx.size();
    ic.o_llx        = nullptr;
    ic.o_lly        = nullptr;
    ic.o_urx        = nullptr;
    ic.o_ury        = nullptr;
    ic.no           = 0;

    /* Add obstacles if any */
    int no = (int)fh->ox.size();
    if (no > 0) {
        /* Allocate temp arrays for obstacle bbox corners */
        static double llx[32], lly[32], urx[32], ury[32];
        int actual = (no > 32) ? 32 : no;
        for (int i = 0; i < actual; i++) {
            llx[i] = fh->ox[i] - fh->ol[i] * 0.5;
            lly[i] = fh->oy[i] - fh->ow[i] * 0.5;
            urx[i] = fh->ox[i] + fh->ol[i] * 0.5;
            ury[i] = fh->oy[i] + fh->ow[i] * 0.5;
        }
        ic.o_llx = llx;
        ic.o_lly = lly;
        ic.o_urx = urx;
        ic.o_ury = ury;
        ic.no    = actual;
    }

    /* Run the planner */
    FrenetReturnValues rv;
    memset(&rv, 0, sizeof(rv));

    run_fot(&ic, &fh->hp, &rv);

    if (!rv.success) return 0;

    /* Extract trajectory points */
    int count = 0;
    for (int i = 0; i < MAX_PATH_LENGTH && count < max_pts; i++) {
        if (isnan(rv.x_path[i])) break;
        out_s[i]     = rv.s[i];
        out_d[i]     = rv.d[i];
        out_speed[i] = rv.speeds[i];
        count++;
    }

    return count;
}

void frenet_destroy(FrenetHandle* fh) {
    delete fh;
}

} /* extern "C" */
