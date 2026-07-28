#ifndef TOPIC_REGISTRY_H
#define TOPIC_REGISTRY_H

/**
 * @file topic_registry.h
 * @brief 话题注册表 — 编译时校验 topic 名字拼写与 producer/consumer 匹配
 *
 * 所有节点通过 `TOPIC_*` 常量引用 topic 名字，而非硬编码字符串。
 * 若拼错名字（如 TOPIC_SENSOR_LIDAR → TOPIC_SENSOR_LIDRA），
 * 编译器直接报 "undeclared" 错误，杜绝运行时静默失败。
 *
 * 用法：
 *   #include "topic_registry.h"
 *   transport_publish(transport, TOPIC_SENSOR_LIDAR, buf, len);
 *   transport_subscribe(transport, TOPIC_CONTROL_CMD, callback, NULL);
 *
 * 命名约定：
 *   TOPIC_<CATEGORY>_<NAME> → "category/name"
 *   例: TOPIC_SENSOR_LIDAR → "sensor/lidar"
 */

/* ── Sensor topics ──────────────────────────────────────────── */

#define TOPIC_SENSOR_LIDAR    "sensor/lidar"
#define TOPIC_SENSOR_IMU      "sensor/imu"
#define TOPIC_SENSOR_POSE     "sensor/pose"
#define TOPIC_SENSOR_GPS      "sensor/gps"
#define TOPIC_SENSOR_CAMERA   "sensor/camera"
#define TOPIC_SENSOR_STEREO   "sensor/stereo"

/* ── Perception topics ──────────────────────────────────────── */

#define TOPIC_PERCEPTION_OBSTACLES         "perception/obstacles"
#define TOPIC_PERCEPTION_OBSTACLES_LIDAR   "perception/obstacles_lidar"
#define TOPIC_PERCEPTION_OBSTACLES_STEREO  "perception/obstacles_stereo"
#define TOPIC_PERCEPTION_TRACKED_OBJECTS   "perception/tracked_objects"
#define TOPIC_PERCEPTION_LANES             "perception/lanes"
#define TOPIC_PERCEPTION_TRAFFIC_LIGHTS    "perception/traffic_lights"
#define TOPIC_PERCEPTION_TRAVERSABILITY    "perception/traversability"

/* ── Fusion topics ──────────────────────────────────────────── */

#define TOPIC_FUSION_LOCALIZATION  "fusion/localization"
#define TOPIC_FUSION_LATENCY       "fusion/latency"

/* ── Planning topics ────────────────────────────────────────── */

#define TOPIC_PLANNING_TRAJECTORY  "planning/trajectory"

/* ── Control topics ─────────────────────────────────────────── */

#define TOPIC_CONTROL_CMD         "control/cmd"
#define TOPIC_CONTROL_RAW_CMD     "control/raw_cmd"
#define TOPIC_CONTROL_RAW_CMD_TEXT "control/raw_cmd/text"
#define TOPIC_CONTROL_CTE         "control/cte"
#define TOPIC_CONTROL_LDW         "control/ldw"
#define TOPIC_INFERENCE_CONTROL_DELTA "inference/control_delta"

/* ── Vehicle topics ─────────────────────────────────────────── */

#define TOPIC_VEHICLE_STATE       "vehicle/state"

/* ── Prediction topics ──────────────────────────────────────── */

#define TOPIC_PREDICTION_TRACKS   "prediction/tracks"

/* ── Road / Map topics ──────────────────────────────────────── */

#define TOPIC_ROAD_GEOMETRY       "road/geometry"
#define TOPIC_ROAD_REF_PATH       "road/ref_path"
#define TOPIC_ROAD_TRAFFIC_LIGHTS "road/traffic_lights"

/* ── Simulation topics ──────────────────────────────────────── */

#define TOPIC_SIM_TICK            "sim/tick"
#define TOPIC_SIM_COLLISION       "sim/collision"

/* ── Scene topics ───────────────────────────────────────────── */

#define TOPIC_SCENE_FRAME         "scene/frame"

/* ── FlowEngine internal topics ─────────────────────────────── */

#define TOPIC_FLOWENGINE_NODE_INFO "flowengine/node_info"

/* ── Compile-time topic producer/consumer map ──────────────────
 *
 * 每个 topic 记录其 producer(s) 和 consumer(s)。
 * 这里的注释作为文档，CI 可以自动解析验证。
 * 默认拓扑以 config/pipeline.json 为准（仿真闭环），其它驱动/感知
 * 子节点（如 lidar_driver_node、stereo_vision_node、scene_assembler_node）
 * 仅在对应 config/pipeline_*.json 中启用。
 *
 * TOPIC_SENSOR_LIDAR:
 *   PRODUCERS: sensor_model_node [pipeline], lidar_driver_node [drivers]
 *   CONSUMERS: perception_node, fusion_node [pipeline]; slam_node [drivers]
 *
 * TOPIC_SENSOR_IMU:
 *   PRODUCERS: imu_driver_node [drivers]
 *   CONSUMERS: slam_node [drivers]
 *
 * TOPIC_SENSOR_POSE:
 *   PRODUCERS: slam_node [drivers]
 *   CONSUMERS: fusion_node [drivers]
 *
 * TOPIC_SENSOR_GPS:
 *   PRODUCERS: sensor_model_node [pipeline], gps_driver_node [drivers]
 *   CONSUMERS: fusion_node
 *
 * TOPIC_SENSOR_CAMERA:
 *   PRODUCERS: sensor_model_node [pipeline]
 *   CONSUMERS: (none in default pipeline — consumed by perception_fusion_node in pipeline_scene.json)
 *
 * TOPIC_SENSOR_STEREO:
 *   PRODUCERS: stereo_camera_node
 *   CONSUMERS: stereo_vision_node
 *
 * TOPIC_PERCEPTION_OBSTACLES:
 *   PRODUCERS: perception_node [pipeline]; perception_fusion_node, stereo_vision_node [alt pipelines]
 *   CONSUMERS: planning_node, safety_control_node, inference_node, data_recorder_node, learner_node, monitor_node [pipeline]; object_tracker_node [alt]
 *
 * TOPIC_PERCEPTION_TRACKED_OBJECTS:
 *   PRODUCERS: object_tracker_node [alt]
 *   CONSUMERS: scene_assembler_node [alt]
 *
 * TOPIC_PERCEPTION_LANES:
 *   PRODUCERS: lane_detection_node [alt]
 *   CONSUMERS: scene_assembler_node [alt]
 *
 * TOPIC_PERCEPTION_TRAFFIC_LIGHTS:
 *   PRODUCERS: traffic_light_recognition_node [alt]
 *   CONSUMERS: scene_assembler_node [alt]
 *
 * TOPIC_PERCEPTION_TRAVERSABILITY:
 *   PRODUCERS: traversability_node [alt]
 *   CONSUMERS: scene_assembler_node [alt]
 *
 * TOPIC_FUSION_LOCALIZATION:
 *   PRODUCERS: fusion_node
 *   CONSUMERS: planning_node, control_node, safety_control_node, inference_node, data_recorder_node, learner_node, model_ota_node [pipeline]
 *
 * TOPIC_FUSION_LATENCY:
 *   PRODUCERS: fusion_node
 *   CONSUMERS: monitor_node
 *
 * TOPIC_PLANNING_TRAJECTORY:
 *   PRODUCERS: planning_node
 *   CONSUMERS: control_node, inference_node, data_recorder_node, learner_node, monitor_node
 *
 * TOPIC_CONTROL_CMD:
 *   PRODUCERS: safety_control_node [pipeline] (闭环最终输出，限幅后)
 *   CONSUMERS: flowsim_node, inference_node, data_recorder_node, learner_node [pipeline]; actuator_node [real vehicle]
 *
 * TOPIC_CONTROL_RAW_CMD:
 *   PRODUCERS: control_node
 *   CONSUMERS: safety_control_node [pipeline] (订阅关系在代码内建立，未在 pipeline.json 显式声明)
 *
 * TOPIC_VEHICLE_STATE:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: sensor_model_node, perception_node, control_node, monitor_node
 *
 * TOPIC_PREDICTION_TRACKS:
 *   PRODUCERS: prediction_node [alt]
 *   CONSUMERS: scene_assembler_node [alt]
 *
 * TOPIC_ROAD_GEOMETRY:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: planning_node, control_node, monitor_node
 *
 * TOPIC_ROAD_REF_PATH:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: control_node, planning_node
 *
 * TOPIC_ROAD_TRAFFIC_LIGHTS:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: planning_node
 *
 * TOPIC_SIM_TICK:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: (none — internal)
 *
 * TOPIC_SIM_COLLISION:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: (none — internal; monitor 通过日志正则读取)
 *
 * TOPIC_SCENE_FRAME:
 *   PRODUCERS: flowsim_node
 *   CONSUMERS: planning_node, control_node, monitor_node
 *
 * TOPIC_FLOWENGINE_NODE_INFO:
 *   PRODUCERS: all nodes (via node_announce_self)
 *   CONSUMERS: monitor_node
 */

#endif /* TOPIC_REGISTRY_H */