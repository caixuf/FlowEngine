/**
 * frenet_bridge.h — C-callable wrapper for the open-source Frenet planner
 *
 * Usage from C:
 *   FrenetHandle* fh = frenet_create(max_speed_mps, max_accel_mps2);
 *   frenet_set_reference_path(fh, wx, wy, n_pts);
 *   int n = frenet_plan(fh, ego_s, ego_d, ego_speed, target_speed,
 *                       out_s, out_d, out_speed, max_pts);
 *   frenet_destroy(fh);
 */

#ifndef FRENET_BRIDGE_H
#define FRENET_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle to the Frenet planner */
typedef struct FrenetHandle FrenetHandle;

/**
 * Create a Frenet planner instance.
 * @param max_speed   Maximum speed (m/s)
 * @param max_accel   Maximum acceleration (m/s²)
 * @return handle, or NULL on failure
 */
FrenetHandle* frenet_create(double max_speed, double max_accel);

/**
 * Set the reference path (centerline of the road).
 * @param wx, wy  Waypoint arrays (global coordinates)
 * @param n       Number of waypoints ( >= 3)
 */
void frenet_set_reference_path(FrenetHandle* fh, const double* wx, const double* wy, int n);

/**
 * Set obstacles in the scene.
 * @param ox, oy   Obstacle center positions (global, NULL-terminated)
 * @param ow, ol   Obstacle width, length
 * @param n        Number of obstacles
 */
void frenet_set_obstacles(FrenetHandle* fh,
                          const double* ox, const double* oy,
                          const double* ow, const double* ol, int n);

/**
 * Plan an optimal trajectory.
 * @param ego_s       Current longitudinal position along reference path (m)
 * @param ego_d       Current lateral offset from reference path (m)
 * @param ego_speed   Current speed (m/s)
 * @param target_speed Desired cruising speed (m/s)
 * @param out_s       Output: s coordinates along reference path [max_pts]
 * @param out_d       Output: lateral offsets [max_pts]
 * @param out_speed   Output: speeds at each point [max_pts]
 * @param max_pts     Max number of output points
 * @return            Number of waypoints written (0 if planning failed)
 */
int frenet_plan(FrenetHandle* fh,
                double ego_s, double ego_d, double ego_speed,
                double target_speed,
                double* out_s, double* out_d, double* out_speed,
                int max_pts);

/** Destroy the planner and free resources */
void frenet_destroy(FrenetHandle* fh);

#ifdef __cplusplus
}
#endif

#endif /* FRENET_BRIDGE_H */
