/**
 * typed_topic.c — topic→type 映射表实现
 *
 * 维护每个 TOPIC_* 常量到其预期消息类型的映射，
 * 供运行时校验使用。
 */

#include "typed_topic.h"
#include "adas_msgs_gen.h" /* 包含所有生成的消息类型头文件 */

/* ══════════════════════════════════════════════════════════════ */
/* Topic → Type 映射表                                           */
/* ══════════════════════════════════════════════════════════════ */

typedef struct {
    const char* topic;
    uint32_t    expected_type_id;
    const char* type_name;
    size_t      expected_size;
} TopicTypeEntry;

static const TopicTypeEntry g_topic_type_map[] = {
    /* Sensor topics */
    { TOPIC_SENSOR_LIDAR,    LIDARFRAME_TYPE_ID,    "LidarFrame",   sizeof(LidarFrame)   },
    { TOPIC_SENSOR_GPS,      GPSDATA_TYPE_ID,       "GpsData",      sizeof(GpsData)      },
    { TOPIC_SENSOR_IMU,      IMUDATA_TYPE_ID,       "ImuData",      sizeof(ImuData)      },
    { TOPIC_SENSOR_POSE,     POSE2D_TYPE_ID,        "Pose2D",       sizeof(Pose2D)       },
    { TOPIC_SENSOR_STEREO,   STEREOFRAME_TYPE_ID,   "StereoFrame",  sizeof(StereoFrame)  },

    /* Perception topics */
    { TOPIC_PERCEPTION_OBSTACLES,         OBSTACLELIST_TYPE_ID,       "ObstacleList",       sizeof(ObstacleList)       },
    { TOPIC_PERCEPTION_OBSTACLES_LIDAR,   OBSTACLELIST_TYPE_ID,       "ObstacleList",       sizeof(ObstacleList)       },
    { TOPIC_PERCEPTION_OBSTACLES_STEREO,  OBSTACLELIST_TYPE_ID,       "ObstacleList",       sizeof(ObstacleList)       },
    /* TrackedObjectList, LaneBoundaryList — no generated header yet, use 0 */
    { TOPIC_PERCEPTION_TRACKED_OBJECTS,   0,  "TrackedObjectList",  0 },
    { TOPIC_PERCEPTION_LANES,             0,  "LaneBoundaryList",   0 },

    /* Fusion topics */
    { TOPIC_FUSION_LOCALIZATION, LOCALIZATION_TYPE_ID,  "Localization",   sizeof(Localization)   },
    { TOPIC_FUSION_LATENCY,      LATENCYREPORT_TYPE_ID, "LatencyReport",  sizeof(LatencyReport)  },

    /* Planning topic */
    { TOPIC_PLANNING_TRAJECTORY, LOCALIZATION_TYPE_ID,  "Localization",   sizeof(Localization)   },

    /* Control topics */
    { TOPIC_CONTROL_CMD,     CONTROLCMD_TYPE_ID,     "ControlCmd",  sizeof(ControlCmd)  },
    { TOPIC_CONTROL_RAW_CMD, CONTROLRAW_TYPE_ID,     "ControlRaw",  sizeof(ControlRaw)  },

    /* Vehicle topic */
    { TOPIC_VEHICLE_STATE,   EGOSTATE_TYPE_ID,       "EgoState",    sizeof(EgoState)    },

    /* Prediction topic — no generated header yet, use 0 */
    { TOPIC_PREDICTION_TRACKS, 0, "PredictionList", 0 },

    /* Sentinel */
    { NULL, 0, NULL, 0 }
};

/* ── Public API ──────────────────────────────────────────────── */

int typed_topic_validate(const char* topic, uint32_t type_id, size_t size) {
    if (!topic) return -1;
    for (const TopicTypeEntry* e = g_topic_type_map; e->topic; e++) {
        if (strcmp(e->topic, topic) == 0) {
            if (e->expected_type_id != 0 && type_id != 0 &&
                e->expected_type_id != type_id) {
                return -1;  /* type_id mismatch */
            }
            if (e->expected_size != 0 && size != e->expected_size) {
                return -1;  /* size mismatch */
            }
            return 0;
        }
    }
    return 0;  /* unknown topic, skip validation */
}

uint32_t typed_topic_type_id(const char* topic) {
    if (!topic) return 0;
    for (const TopicTypeEntry* e = g_topic_type_map; e->topic; e++) {
        if (strcmp(e->topic, topic) == 0) return e->expected_type_id;
    }
    return 0;
}

const char* typed_topic_type_name(const char* topic) {
    if (!topic) return "unknown";
    for (const TopicTypeEntry* e = g_topic_type_map; e->topic; e++) {
        if (strcmp(e->topic, topic) == 0) return e->type_name;
    }
    return "unknown";
}