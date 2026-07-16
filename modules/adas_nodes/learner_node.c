/**
 * learner_node.c — 车端增量微调节点 (车端学习闭环 · Stage 3)
 *
 * 订阅 fusion/localization + planning/trajectory（teacher），在车端对已加载的
 * tiny-MLP 进行增量 SGD 微调（在线学习），以资源受限方式低频运行，避免干扰实时
 * 控制链路。定期将更新后的权重保存到磁盘，可通过 model_ota_node 或 modelctl.py
 * 手动 promote 到 C runtime。
 *
 * 设计要点:
 *   1. 在线 SGD: 累积一个滑窗样本缓冲（环形），每次训练步从中随机抽取 mini-batch
 *      执行梯度更新，不阻塞采样/控制链路。
 *   2. 资源受限调度: 采样以 20 Hz 运行（锁存最新特征），训练以 train_hz 运行（默认
 *      0.5 Hz，每 2 秒一次），两者在同一线程中交错，互不干扰。
 *   3. 防灾难性遗忘: 默认 full_finetune=0，只更新顶层 W2/b2；全参数微调需显式开启。
 *   4. 影子保存: 更新后的权重写到 save_path（默认 /tmp/flow_learner_model.txt），
 *      不自动替换 runtime 模型，需 modelctl.py promote 显式提升。
 *   5. 状态上报: 每个训练步发布 learner/status，包含 loss、iteration、buf 利用率等。
 *
 * 数据契约 (与 inference_node/data_recorder 一致):
 *   特征 x[4]  = [ego_v, ego_y, ego_heading, ego_yaw_rate]  (v1, in_dim=4)
 *   特征 x[16] = v1 + front0/1 + control                   (v2, in_dim=16)
 *   标签 y[1]  = planning target_speed                       (模仿 teacher)
 *
 * NodePlugin 接口，编译为 liblearner_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"
#include "tiny_mlp.h"

#include "clock_service.h"
#include <cjson/cJSON.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#define LEARNER_FEAT_DIM     16   /* 最大特征维度 (v2) */
#define LEARNER_BUF_DEFAULT  512  /* 样本环形缓冲默认大小 */
#define LEARNER_BATCH_DEFAULT 32  /* 默认 mini-batch 大小 */

/* ── 单个训练样本 ───────────────────────────────────────────── */

typedef struct {
    float features[LEARNER_FEAT_DIM];
    float label;    /* planning target_speed */
} LearnerSample;

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t         thread;
    volatile int      running;
    volatile int      should_stop;

    ReflectiveStateMachine sm;

    /* 模型 + 互斥（inference 读、train 写，必须加锁） */
    TinyMLP           model;
    pthread_mutex_t   model_mutex;
    char              model_path[256];
    char              save_path[256];

    /* 最新锁存特征（回调写入，train 线程读取） */
    double ego_v, ego_y, ego_heading, ego_yaw_rate;
    double front0_x, front0_y, front0_vx, front0_type, front0_confidence;
    double front1_x, front1_y, front1_vx, front1_type, front1_confidence;
    double ctrl_brake;
    int    ctrl_emergency_stop;
    double planning_target_speed;
    volatile int has_fusion;
    volatile int has_planning;
    volatile int has_obstacles;
    volatile int has_control;

    /* 样本环形缓冲 */
    LearnerSample*  sample_buf;
    int             buf_size;
    int             buf_head;   /* 下一个写入位置（循环） */
    int             buf_count;  /* 当前有效样本数 */
    pthread_mutex_t buf_mutex;

    /* 配置 */
    double cfg_train_hz;
    int    cfg_batch_size;
    float  cfg_lr;
    int    cfg_full_finetune;
    int    cfg_save_interval;  /* 每 N 步保存一次 */

    /* 统计 */
    int   train_step;
    float running_loss;   /* EMA 平滑损失 */
    float loss_alpha;     /* EMA 平滑系数 */
} g;

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    Localization loc;
    if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.ego_v        = loc.v;
        g.ego_y        = loc.y;
        g.ego_heading  = loc.heading;
        g.ego_yaw_rate = loc.yaw_rate;
        g.has_fusion   = 1;
        return;
    }
    /* Fallback: 文本 JSON */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "v");
        if (cJSON_IsNumber(j)) g.ego_v = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "y");
        if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "heading");
        if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate");
        if (cJSON_IsNumber(j)) g.ego_yaw_rate = j->valuedouble;
        cJSON_Delete(root);
    }
    g.has_fusion = 1;
}

static void on_planning(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "target_speed");
        if (cJSON_IsNumber(j))
            g.planning_target_speed = j->valuedouble;
        else {
            j = cJSON_GetObjectItemCaseSensitive(root, "speed");
            if (cJSON_IsNumber(j))
                g.planning_target_speed = j->valuedouble;
        }
        cJSON_Delete(root);
    }
    g.has_planning = 1;
}

static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) == 0) {
        int n = list.count > 2 ? 2 : list.count;
        int fi = 0;
        for (int i = 0; i < n && fi < 2; i++) {
            double type_f = 0.0;
            if (list.obstacles[i].type == 3)      type_f = 1.0;
            else if (list.obstacles[i].type == 4)  type_f = 2.0;
            if (fi == 0) {
                g.front0_x = list.obstacles[i].x;
                g.front0_y = list.obstacles[i].y;
                g.front0_vx = list.obstacles[i].vx;
                g.front0_type = type_f;
                g.front0_confidence = list.obstacles[i].confidence;
            } else {
                g.front1_x = list.obstacles[i].x;
                g.front1_y = list.obstacles[i].y;
                g.front1_vx = list.obstacles[i].vx;
                g.front1_type = type_f;
                g.front1_confidence = list.obstacles[i].confidence;
            }
            fi++;
        }
        if (fi == 0) {
            g.front0_x = 0; g.front0_y = 0; g.front0_vx = 0;
            g.front0_type = 0; g.front0_confidence = 0;
        }
        if (fi <= 1) {
            g.front1_x = 0; g.front1_y = 0; g.front1_vx = 0;
            g.front1_type = 0; g.front1_confidence = 0;
        }
        g.has_obstacles = 1;
        return;
    }
    g.has_obstacles = 1;
}

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    ControlCmd cmd;
    if (ControlCmd_deserialize(&cmd, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.ctrl_brake = cmd.brake;
        g.ctrl_emergency_stop = cmd.emergency_stop ? 1 : 0;
        g.has_control = 1;
        return;
    }
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "brake");
        if (cJSON_IsNumber(j)) g.ctrl_brake = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "mode");
        if (cJSON_IsString(j) && j->valuestring)
            g.ctrl_emergency_stop = (strstr(j->valuestring, "AEB") || strstr(j->valuestring, "BRAKE")) ? 1 : 0;
        cJSON_Delete(root);
    }
    g.has_control = 1;
}

/* ── 特征构建 ─────────────────────────────────────────────────── */

static void build_features_v2(float feat[LEARNER_FEAT_DIM]) {
    feat[0]  = (float)g.ego_v;
    feat[1]  = (float)g.ego_y;
    feat[2]  = (float)g.ego_heading;
    feat[3]  = (float)g.ego_yaw_rate;
    feat[4]  = (float)g.front0_x;
    feat[5]  = (float)g.front0_y;
    feat[6]  = (float)g.front0_vx;
    feat[7]  = (float)g.front0_type;
    feat[8]  = (float)g.front0_confidence;
    feat[9]  = (float)g.front1_x;
    feat[10] = (float)g.front1_y;
    feat[11] = (float)g.front1_vx;
    feat[12] = (float)g.front1_type;
    feat[13] = (float)g.front1_confidence;
    feat[14] = (float)g.ctrl_brake;
    feat[15] = (float)g.ctrl_emergency_stop;
}

/* ── 样本缓冲管理 ──────────────────────────────────────────────── */

static void push_sample(void) {
    if (!g.has_fusion || !g.has_planning) return;

    LearnerSample s;
    memset(&s, 0, sizeof(s));
    build_features_v2(s.features);
    s.label = (float)g.planning_target_speed;

    pthread_mutex_lock(&g.buf_mutex);
    g.sample_buf[g.buf_head] = s;
    g.buf_head = (g.buf_head + 1) % g.buf_size;
    if (g.buf_count < g.buf_size) g.buf_count++;
    pthread_mutex_unlock(&g.buf_mutex);
}

/* ── Mini-batch SGD 训练步 ─────────────────────────────────────── */

static float do_train_step(void) {
    pthread_mutex_lock(&g.buf_mutex);
    int available = g.buf_count;
    int buf_size  = g.buf_size;
    int buf_head  = g.buf_head;
    pthread_mutex_unlock(&g.buf_mutex);

    if (available < 2) return 0.0f;

    int batch = (g.cfg_batch_size < available) ? g.cfg_batch_size : available;
    float total_loss = 0.0f;

    pthread_mutex_lock(&g.model_mutex);
    if (!g.model.loaded) {
        pthread_mutex_unlock(&g.model_mutex);
        return 0.0f;
    }

    /* 确定实际输入维度 */
    int in_dim = g.model.in_dim;
    if (in_dim < 1 || in_dim > LEARNER_FEAT_DIM) in_dim = 4;

    for (int b = 0; b < batch; b++) {
        /* 从环形缓冲随机采样一条 */
        unsigned int ridx = (unsigned int)(rand() % available);
        int actual = (buf_head - available + (int)ridx + buf_size * 2) % buf_size;

        pthread_mutex_lock(&g.buf_mutex);
        LearnerSample s = g.sample_buf[actual];
        pthread_mutex_unlock(&g.buf_mutex);

        /* 截断特征到模型 in_dim */
        float feat[TINY_MLP_MAX_IN];
        memset(feat, 0, sizeof(float) * (size_t)in_dim);
        int copy_dim = (in_dim < LEARNER_FEAT_DIM) ? in_dim : LEARNER_FEAT_DIM;
        for (int i = 0; i < copy_dim; i++) feat[i] = s.features[i];

        float label[1] = { s.label };
        float step_loss = tiny_mlp_sgd_step(&g.model, feat, label,
                                             g.cfg_lr, g.cfg_full_finetune);
        total_loss += step_loss;
    }
    pthread_mutex_unlock(&g.model_mutex);

    return total_loss / (float)batch;
}

/* ── 主训练线程 ─────────────────────────────────────────────────── */

static void* learner_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "learner");

    /* 采样频率 20 Hz，训练频率 cfg_train_hz（通常 0.5~2 Hz） */
    const useconds_t sample_us = 50000u;  /* 50 ms = 20 Hz */
    const double     train_period = g.cfg_train_hz > 0.0
                                    ? 1.0 / g.cfg_train_hz : 2.0;

    double accum = 0.0;
    uint64_t t_last_us = clock_now_us();

    while (!g.should_stop) {
        usleep(sample_us);
        if (g.should_stop) break;

        /* 1. 采样 */
        push_sample();

        /* 2. 检查是否到达训练时刻 */
        uint64_t t_now_us = clock_now_us();
        double dt = (double)(t_now_us - t_last_us) * 1e-6;
        accum += dt;
        t_last_us = t_now_us;
        if (accum < train_period) continue;
        accum = 0.0;

        /* 3. SGD 步 */
        float loss = do_train_step();
        g.train_step++;

        /* EMA 平滑损失 */
        if (g.train_step == 1)
            g.running_loss = loss;
        else
            g.running_loss = g.loss_alpha * loss
                           + (1.0f - g.loss_alpha) * g.running_loss;

        /* 4. 周期性保存权重 */
        if (g.cfg_save_interval > 0 && g.train_step % g.cfg_save_interval == 0) {
            pthread_mutex_lock(&g.model_mutex);
            int ret = tiny_mlp_save(&g.model, g.save_path);
            pthread_mutex_unlock(&g.model_mutex);
            if (ret == 0)
                LOG_INFO("learner", "step=%d loss=%.4f → saved %s",
                         g.train_step, g.running_loss, g.save_path);
            else
                LOG_WARN("learner", "step=%d save failed: %s",
                         g.train_step, g.save_path);
        }

        /* 5. 发布 learner/status */
        {
            pthread_mutex_lock(&g.buf_mutex);
            int buf_cnt = g.buf_count;
            pthread_mutex_unlock(&g.buf_mutex);

            cJSON* s_root = cJSON_CreateObject();
            cJSON_AddNumberToObject(s_root, "step", g.train_step);
            cJSON_AddNumberToObject(s_root, "loss", g.running_loss);
            cJSON_AddNumberToObject(s_root, "lr", g.cfg_lr);
            cJSON_AddNumberToObject(s_root, "buf_count", buf_cnt);
            cJSON_AddNumberToObject(s_root, "buf_size", g.buf_size);
            cJSON_AddNumberToObject(s_root, "full_finetune", g.cfg_full_finetune);
            cJSON_AddNumberToObject(s_root, "model_loaded", g.model.loaded ? 1 : 0);
            cJSON_AddStringToObject(s_root, "save_path", g.save_path);
            char* s = cJSON_PrintUnformatted(s_root);
            transport_publish(g.transport, "learner/status",
                              (const uint8_t*)s, (uint32_t)strlen(s) + 1);
            free(s);
            cJSON_Delete(s_root);
        }

        if (g.train_step % 20 == 1) {
            pthread_mutex_lock(&g.buf_mutex);
            int buf_cnt = g.buf_count;
            pthread_mutex_unlock(&g.buf_mutex);
            LOG_INFO("learner", "step=%d loss=%.4f lr=%.5f buf=%d/%d %s",
                     g.train_step, (double)g.running_loss, (double)g.cfg_lr,
                     buf_cnt, g.buf_size,
                     g.cfg_full_finetune ? "(full)" : "(top-layer)");
        }
    }

    LOG_INFO("learner", "stopped (step=%d loss=%.4f)", g.train_step, (double)g.running_loss);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = {
    "fusion/localization",
    "planning/trajectory",
    "perception/obstacles",
    "control/cmd",
    NULL
};
static const char* s_outputs[] = {
    "learner/status",
    NULL
};

static NodePlugin s_plugin;

static int learner_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.should_stop = 0;

    /* 默认配置 */
    g.cfg_train_hz      = 0.5;
    g.cfg_batch_size    = LEARNER_BATCH_DEFAULT;
    g.cfg_lr            = 0.001f;
    g.cfg_full_finetune = 0;
    g.cfg_save_interval = 50;
    g.buf_size          = LEARNER_BUF_DEFAULT;
    g.loss_alpha        = 0.1f;
    strncpy(g.model_path, "tools/train/model.txt",        sizeof(g.model_path) - 1);
    strncpy(g.save_path,  "/tmp/flow_learner_model.txt",  sizeof(g.save_path)  - 1);

    /* 解析 params_json */
    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(p, "train_hz");
            if (cJSON_IsNumber(j)) g.cfg_train_hz = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lr");
            if (cJSON_IsNumber(j)) g.cfg_lr = (float)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "batch_size");
            if (cJSON_IsNumber(j)) g.cfg_batch_size = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "buffer_size");
            if (cJSON_IsNumber(j)) {
                g.buf_size = j->valueint;
                if (g.buf_size < 16)   g.buf_size = 16;
                if (g.buf_size > 4096) g.buf_size = 4096;
            }
            j = cJSON_GetObjectItemCaseSensitive(p, "full_finetune");
            if (cJSON_IsNumber(j)) g.cfg_full_finetune = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "save_interval");
            if (cJSON_IsNumber(j)) g.cfg_save_interval = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "model_path");
            if (cJSON_IsString(j) && j->valuestring) {
                strncpy(g.model_path, j->valuestring, sizeof(g.model_path) - 1);
                g.model_path[sizeof(g.model_path) - 1] = '\0';
            }
            j = cJSON_GetObjectItemCaseSensitive(p, "save_path");
            if (cJSON_IsString(j) && j->valuestring) {
                strncpy(g.save_path, j->valuestring, sizeof(g.save_path) - 1);
                g.save_path[sizeof(g.save_path) - 1] = '\0';
            }
            cJSON_Delete(p);
        }
    }

    /* 限制 batch 不超过 buffer */
    if (g.cfg_batch_size > g.buf_size) g.cfg_batch_size = g.buf_size;
    if (g.cfg_batch_size < 1)          g.cfg_batch_size = 1;

    /* 分配样本缓冲 */
    g.sample_buf = (LearnerSample*)calloc((size_t)g.buf_size, sizeof(LearnerSample));
    if (!g.sample_buf) return -1;

    pthread_mutex_init(&g.buf_mutex,   NULL);
    pthread_mutex_init(&g.model_mutex, NULL);

    /* 加载初始模型 */
    if (tiny_mlp_load(&g.model, g.model_path) == 0) {
        LOG_INFO("learner", "model loaded from %s (in=%d hid=%d out=%d)",
                 g.model_path, g.model.in_dim, g.model.hid_dim, g.model.out_dim);
    } else {
        LOG_WARN("learner", "no model at %s — will wait until model_ota activates one",
                 g.model_path);
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_planning, NULL);
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);
    transport_subscribe(transport, "control/cmd",          on_control_cmd, NULL);
    transport_advertise(transport, "learner/status", 0x4C524E53u);

    discovery_advertise(discovery, "learner/status", 0x4C524E53u,
                        CAP_PUBLISHER, g.cfg_train_hz);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "learner");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("learner",
             "initialized (train=%.2fHz lr=%.5f batch=%d buf=%d %s → %s)",
             g.cfg_train_hz, (double)g.cfg_lr, g.cfg_batch_size, g.buf_size,
             g.cfg_full_finetune ? "full-finetune" : "top-layer-only",
             g.save_path);
    return 0;
}

static int learner_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, learner_thread, NULL) != 0) return -1;
    LOG_INFO("learner", "started [state=%s]",
             statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void learner_stop(void) {
    g.should_stop = 1;
}

static void learner_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    /* 退出前最后保存一次 */
    if (g.model.loaded && g.train_step > 0) {
        pthread_mutex_lock(&g.model_mutex);
        if (tiny_mlp_save(&g.model, g.save_path) == 0)
            LOG_INFO("learner", "final save → %s (step=%d)", g.save_path, g.train_step);
        pthread_mutex_unlock(&g.model_mutex);
    }
    free(g.sample_buf);
    g.sample_buf = NULL;
    pthread_mutex_destroy(&g.buf_mutex);
    pthread_mutex_destroy(&g.model_mutex);
    LOG_INFO("learner", "cleanup done");
}

static int learner_health(void) {
    return g.model.loaded ? 0 : 1;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "learner",
    .version       = "1.0.0",
    .description   = "On-vehicle incremental SGD fine-tuning (Stage 3)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = learner_init,
    .start         = learner_start,
    .stop          = learner_stop,
    .cleanup       = learner_cleanup,
    .health        = learner_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
