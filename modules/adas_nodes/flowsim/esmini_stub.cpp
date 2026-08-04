#include "esminiRMLib.hpp"
#include <cstring>

extern "C" {

int RM_Init(const char* odrFilename) { (void)odrFilename; return -1; }
int RM_Close() { return 0; }
int RM_CreatePosition() { return 1; }
int RM_DeletePosition(int handle) { (void)handle; return 0; }
int RM_CopyPosition(int handle) { (void)handle; return 1; }
int RM_GetNumberOfRoads() { return 0; }
id_t RM_GetIdOfRoadFromIndex(unsigned int index) { (void)index; return -1; }
double RM_GetRoadLength(id_t id) { (void)id; return 0.0; }
const char* RM_GetRoadIdString(id_t road_id) { (void)road_id; return ""; }
int RM_GetRoadNumberOfDrivableLanes(id_t roadId, double s) { (void)roadId; (void)s; return 0; }
int RM_SetLanePosition(int handle, id_t roadId, int laneId, double laneOffset, double s, bool align) {
    (void)handle; (void)roadId; (void)laneId; (void)laneOffset; (void)s; (void)align;
    return -1;
}
int RM_SetWorldXYHPosition(int handle, double x, double y, double h) {
    (void)handle; (void)x; (void)y; (void)h;
    return -1;
}
int RM_PositionMoveForward(int handle, double dist, double junctionSelectorAngle) {
    (void)handle; (void)dist; (void)junctionSelectorAngle;
    return -1;
}
int RM_GetPositionData(int handle, RM_PositionData* data) {
    (void)handle;
    if (data) std::memset(data, 0, sizeof(*data));
    return -1;
}
double RM_GetSpeedLimit(int handle) { (void)handle; return 0.0; }
int RM_GetLaneWidthByRoadId(id_t road_id, int lane_id, double s, double* width) {
    (void)road_id; (void)lane_id; (void)s;
    if (width) *width = 3.5;
    return -1;
}

} // extern "C"
