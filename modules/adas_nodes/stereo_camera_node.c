/**
 * stereo_camera_node.c — 双目摄像头驱动节点插件 (OAK-D / ZED / DIY 双目真车输入)
 *
 * 从 OAK-D 双目摄像头读取左图 + 深度图，序列化为 StereoFrame 发布到 sensor/stereo 话题。
 *
 * 双目视觉是 FlowEngine 感知栈的 RGB+深度输入入口：
 *   - 为 perception_node 提供 left JPEG 供 YOLO/检测器跑 2D 检测；
 *   - 为感知/规划提供 depth_data（80×60 降采样深度图，米）做障碍物距离融合；
 *   - baseline_m + fov_deg 让下游能把像素距离反投影到车体坐标系。
 *
 * 工作流程：
 *   OAK-D USB → read_stereo_frame() → StereoFrame → sensor/stereo
 *
 * ── 核心设计——OAK-D 适配作为可替换钩子 ──
 *   本节点是**适配层**，真实 OAK-D SDK（depthai）作为外部库通过钩子接入。
 *   默认未编译 depthai 时，自动降级为 dry-run（生成模拟左图 + 模拟深度图，
 *   走完整 序列化 + 发布 链路，不阻塞流水线其余节点运行）。
 *   用编译宏 HAVE_DEPTHAI 控制是否启用真实 OAK-D 路径（类似 slam_node 的
 *   algo 降级思路、planning_node 的 HAVE_FRENET 思路）。
 *
 *   OAK-D 的核心优势：板载 Movidius NPU 直接算好深度，主机只收结果，零 CPU 负担，
 *   非常适合树莓派/Jetson Nano 等算力受限平台。
 *
 * ── 三种硬件接入方案 ──
 *   方案1（推荐，OAK-D NPU 模式）：
 *     OAK-D 板载 NPU 算深度，主机通过 depthai 库直接拿现成的 left JPEG + depth
 *     frame，主机零 CPU 负担。本节点 HAVE_DEPTHAI 分支即对应此模式。
 *   方案2（OAK-D 原始图 + OpenCV SGBM）：
 *     OAK-D 只出左右原始图（不开 StereoDepth 节点），主机用 OpenCV
 *     StereoSGBM 算深度，CPU 负担重，树莓派上可能跑不到 10fps。
 *   方案3（DIY 双目 USB 摄像头）：
 *     两个普通 USB 摄像头 + 已知基线，V4L2 读两路 + OpenCV SBM/SGBM 算深度，
 *     无需 depthai 库，但需自己处理曝光同步/畸变校正。改 read_stereo_frame()
 *     的 #else 分支为 V4L2 读取即可。
 *
 * ── 集成 depthai 步骤（启用真实 OAK-D） ──
 *   1. 安装 depthai：
 *        pip install depthai               # Python 绑定（开发调试）
 *        或编译 depthai C++ 库             # 部署用
 *   2. CMakeLists.txt 加：
 *        find_package(depthai REQUIRED)
 *        target_link_libraries(stereo_camera_node ${NODE_LINK_LIBS} depthai::core)
 *        target_compile_definitions(stereo_camera_node PRIVATE HAVE_DEPTHAI)
 *   3. 定义 HAVE_DEPTHAI 宏（见上一步 target_compile_definitions）
 *   4. 在 read_stereo_frame() 的 #ifdef HAVE_DEPTHAI 分支填真实读取代码：
 *        a. dai.Pipeline() 创建管道（只在首次，缓存到全局）
 *        b. 配置 MonoLeft + StereoDepth 节点，setOutputDir/AI/ISP
 *        c. startPipeline() → 队列取 latest left frame → JPEG 编码 → 填 left_jpeg
 *        d. 取 depth frame → 降采样到 80×60 → 填 depth_data（单位米）
 *        e. 填 width/height/timestamp_us/left_jpeg_size/depth_count
 *      注意：depthai 是 C++ 库，本 .c 文件无法直接调用 dai::Pipeline，
 *      故通过 extern "C" 桥接函数 oakd_capture_stereo() 接入（桥接实现
 *      见用户自带的 oakd_bridge.cpp，参考 slam_node 的 C++ 桥接思路）。
 *
 * ── JPEG 编码依赖 ──
 *   真实路径：OAK-D 板载 ISP 可直接输出 JPEG（首选），或主机用 libjpeg/turbojpeg
 *   对原始 left 帧编码。libjpeg 为可选依赖，用 HAVE_LIBJPEG 宏保护，未定义时
 *   桥接函数需自行处理编码（或直接取 OAK-D ISP 的 JPEG 输出）。
 *   dry-run 路径：不需要真 JPEG，填一个伪 JPEG 头（SOI/EOI + 随机数据），
 *   left_jpeg_size 设为几百字节，仅供走通发布链路；消费方应检查 left_jpeg_size>0
 *   且用真实 JPEG 解码器校验数据有效性（伪 JPEG 不可解码）。
 *
 * ── 也可作为 DIY 双目节点 ──
 *   不用 depthai 时，把 read_stereo_frame() 的 #else 分支改为 V4L2 读取两路 +
 *   OpenCV SGBM 算深度，结果填到 StereoFrame 即可，订阅/发布/线程框架无需改动。
 *
 * 话题契约：
 *   输入: 无（从 OAK-D USB 设备读）
 *   输出: sensor/stereo (StereoFrame 二进制序列化, type_id=0x669200d2, STEREOFRAME_TYPE_ID)
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "stereo_camera",
 *     "library": "libstereo_camera_node.so",
 *     "params": {
 *       "fps": 10,
 *       "width": 320,
 *       "height": 240,
 *       "baseline_m": 0.075,
 *       "fov_deg": 65.0,
 *       "jpeg_quality": 70,
 *       "enable": true,
 *       "dry_run": false,
 *       "device_id": ""
 *     }
 *   }
 *
 * 树莓派部署前置（OAK-D USB 示例）:
 *   # 插入 OAK-D 后出现 /dev/video*，depthai 库通过 USB 直接枚举，权限不足时:
 *   sudo usermod -aG plugdev $USER
 *   # 多 OAK-D 时用 device_id 指定序列号（params.device_id）
 *   # 验证设备枚举（装好 depthai 后）:
 *   python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
 *
 * 编译依赖: adas_msgs_gen.h (StereoFrame 序列化)，随构建生成；HAVE_DEPTHAI 可选。
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 配置参数（pipeline.json 注入） */
    int    enabled;          /* 总开关，false=不读相机也不发布 */
    int    fps;              /* 采样频率，OAK-D 深度模式通常 10-30fps，默认 10 */
    int    width;            /* 图像宽度（像素），默认 320 */
    int    height;           /* 图像高度（像素），默认 240 */
    double baseline_m;       /* 双目基线长度（米），OAK-D 标准基线 0.075 */
    double fov_deg;          /* 水平视场角（度），OAK-D 水平 FOV 65 */
    int    jpeg_quality;     /* JPEG 压缩质量 1-100，默认 70 */
    int    dry_run;          /* true=只发模拟数据（无硬件或调试） */
    char   device_id[64];    /* OAK-D 设备序列号，多 OAK-D 时指定，默认 "" */

    /* 统计 */
    uint64_t frames_captured;    /* 成功采集的立体帧数 */
    uint64_t frames_failed;      /* 采集失败的帧数 */
    uint64_t stereo_published;   /* 发布到 sensor/stereo 的帧数 */

    /* dry-run 提示只打一次，避免刷屏 */
    int    dry_run_warned;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 stereo_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase taskbase;
} g;

/* ── 硬件适配点：读取一帧立体数据（左图 + 深度图） ───────────
 *
 * ⚠ 这是硬件适配点。社区用户按自己双目硬件方案改本函数即可，无需动其它代码。
 *
 * 三种硬件方案的适配方式：
 *
 *   方案1（OAK-D NPU 模式，推荐）：
 *     OAK-D 板载 NPU 算好深度，主机通过 depthai 库直接拿现成的 left + depth。
 *     零 CPU 负担，适合树莓派。对应 #ifdef HAVE_DEPTHAI 分支，通过 extern "C"
 *     桥接函数 oakd_capture_stereo() 调用 depthai C++ API。
 *
 *   方案2（OAK-D 原始图 + OpenCV SGBM）：
 *     OAK-D 只出左右原始图（不开 StereoDepth），主机用 OpenCV StereoSGBM 算深度。
 *     CPU 负担重，树莓派上可能 <10fps。改本函数：用 depthai 取 MonoLeft+MonoRight
 *     原始帧 → cv::StereoSGBM::compute() → 降采样到 80×60 → 填 depth_data。
 *
 *   方案3（DIY 双目 USB 摄像头）：
 *     两个普通 USB 摄像头 + 已知基线，V4L2 读两路 + OpenCV SBM/SGBM 算深度。
 *     改本函数的 #else 分支：open /dev/video0 + /dev/video1 → VIDIOC_DQBUF 取两帧
 *     → cv::StereoSGBM → 降采样填 depth_data；left JPEG 用 libjpeg 编码。
 *
 * StereoFrame 字段填充约定：
 *   - width/height：原始左图分辨率（如 320×240）
 *   - left_jpeg/left_jpeg_size：左图 JPEG 压缩数据 + 有效字节数
 *   - depth_data/depth_count：80×60=4800 个 float 深度（米），depth_count=4800
 *   - baseline_m/fov_deg：相机标定参数
 *   - timestamp_us：由调用处用 (uint32_t)(clock_now_us() & 0xFFFFFFFFu) 填充
 *
 * @param out  输出 StereoFrame（调用前已 memset=0）
 * @return 0 成功，-1 失败（硬件错误/无数据）
 */
static int read_stereo_frame(StereoFrame* out) {
    if (!out) return -1;
    memset(out, 0, sizeof(*out));

#ifdef HAVE_DEPTHAI
    /* ── 真实 OAK-D 路径（方案1：板载 NPU 算深度） ──
     *
     * depthai 是 C++ 库，本 .c 文件无法直接调用 dai::Pipeline，故通过 extern "C"
     * 桥接函数接入。桥接实现见用户自带的 oakd_bridge.cpp，示例骨架：
     *
     *   extern "C" int oakd_capture_stereo(StereoFrame* out, int jpeg_quality,
     *                                      const char* device_id) {
     *       static dai::Pipeline pipeline;            // 只在首次构建
     *       static std::shared_ptr<dai::Device> device;
     *       if (!device) {
     *           auto cam = pipeline.create<dai::node::ColorCamera>();
     *           auto depth = pipeline.create<dai::node::StereoDepth>();
     *           cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
     *           cam->setIspScale(1, 3);               // 降采样到 640×360
     *           cam->isp.link(depth->rectifiedLeft);  // 左图进深度计算
     *           device = std::make_shared<dai::Device>(pipeline, dai::DeviceInfo(device_id));
     *       }
     *       auto leftQ = device->getOutputQueue("left", 1, false);
     *       auto depthQ = device->getOutputQueue("depth", 1, false);
     *       auto left = leftQ->get<dai::ImgFrame>();   // 取最新左图
     *       auto depth = depthQ->get<dai::ImgFrame>();  // 取最新深度图
     *       // left->getCvFrame() → JPEG 编码（turbojpeg/jpeg_quality）→ out->left_jpeg
     *       // depth->getCvFrame() → 降采样到 80×60 → 转米 → out->depth_data
     *       out->width = 320; out->height = 240;
     *       out->left_jpeg_size = <jpeg 字节数>;
     *       out->depth_count = 4800;
     *       return 0;
     *   }
     *
     * 注：若 OAK-D ISP 直接输出 JPEG（cam->setEncoding(dai::ImgEncoder::JPEG)），
     * 则无需 libjpeg；否则用 HAVE_LIBJPEG 保护的主机端编码。
     */
    extern int oakd_capture_stereo(StereoFrame* out, int jpeg_quality,
                                   const char* device_id);
    if (oakd_capture_stereo(out, g.jpeg_quality, g.device_id) != 0) {
        return -1;
    }
    /* 桥接函数已填好图像/深度字段，补时间戳与标定参数 */
    out->timestamp_us = (uint32_t)(clock_now_us() & 0xFFFFFFFFu);
    out->baseline_m = (float)g.baseline_m;
    out->fov_deg = (float)g.fov_deg;
    return 0;

#else
    /* ── dry-run / 默认降级路径（无 depthai 编译时） ──
     *
     * 生成模拟数据走完整发布链路，便于在无硬件环境下联调下游 perception。
     * 模拟策略：前方有障碍物——图像中心近（~1m），边缘远（~5m）。
     */
    if (!g.dry_run_warned) {
        LOG_WARN("stereo_camera", "未编译 depthai (HAVE_DEPTHAI 未定义)，使用 dry-run "
                 "模拟数据 (左图伪 JPEG + 中心近/边缘远模拟深度)。"
                 "集成真实 OAK-D 步骤见本文件头注释 (pip install depthai + CMakeLists "
                 "find_package(depthai) + 定义 HAVE_DEPTHAI)");
        g.dry_run_warned = 1;
    }

    out->width  = (uint32_t)g.width;
    out->height = (uint32_t)g.height;
    out->baseline_m = (float)g.baseline_m;
    out->fov_deg    = (float)g.fov_deg;

    /* 左图：伪 JPEG（SOI=FFD8 + 随机数据 + EOI=FFD9），不可解码，仅供走通链路。
     * 消费方应检查 left_jpeg_size>0，并用真实 JPEG 解码器校验数据有效性。 */
    {
        uint32_t sz = 200u + (uint32_t)(rand() % 200);  /* 200-400 字节 */
        if (sz > 25600u) sz = 25600u;
        out->left_jpeg[0] = 0xFF; out->left_jpeg[1] = 0xD8;  /* SOI */
        for (uint32_t i = 2; i + 1 < sz; i++) {
            out->left_jpeg[i] = (uint8_t)(rand() & 0xFF);
        }
        out->left_jpeg[sz - 2] = 0xFF; out->left_jpeg[sz - 1] = 0xD9;  /* EOI */
        out->left_jpeg_size = sz;
    }

    /* 深度图：80×60=4800 个 float，中心近(~1m)、边缘远(~5m)，模拟前方有障碍物 */
    {
        const int DW = 80, DH = 60;          /* width/4 × height/4 降采样 */
        const float half_w = (float)DW * 0.5f;
        const float half_h = (float)DH * 0.5f;
        for (int j = 0; j < DH; j++) {
            for (int i = 0; i < DW; i++) {
                float nx = ((float)i - half_w + 0.5f) / half_w;  /* -1..1 */
                float ny = ((float)j - half_h + 0.5f) / half_h;
                float r  = sqrtf(nx * nx + ny * ny);
                if (r > 1.0f) r = 1.0f;
                float depth = 1.0f + 4.0f * r;                   /* 1m(中心) ~ 5m(边缘) */
                depth += (float)((rand() % 201) - 100) / 1000.0f; /* ±0.1m 噪声 */
                out->depth_data[j * DW + i] = depth;
            }
        }
        out->depth_count = (uint32_t)(DW * DH);  /* 4800 */
    }

    out->timestamp_us = (uint32_t)(clock_now_us() & 0xFFFFFFFFu);
    return 0;
#endif
}

/* ── 托管模式主循环：循环 read_stereo_frame → serialize → publish ─
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 stereo_reader_thread 行为等价，只是 should_stop 改由 TaskBase 提供。
 *
 * 真实模式（HAVE_DEPTHAI）：通过桥接函数从 OAK-D 取 left + depth，按 fps 节拍发布。
 * dry-run 模式：无硬件，按 fps 生成模拟立体数据，走完整 publish 链路。
 */
static int stereo_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "stereo_reader");
    long period_us = 1000000L / (g.fps > 0 ? g.fps : 10);

    /* StereoFrame 序列化后固定 44828 字节，线程栈上分配（默认 8MB 栈无压力） */
    uint8_t buf[44828];

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

        StereoFrame frame;
        if (read_stereo_frame(&frame) != 0) {
            g.frames_failed++;
            LOG_WARN("stereo_camera", "read_stereo_frame failed (device error?)");
            continue;
        }
        g.frames_captured++;

        /* 序列化 + 发布到 sensor/stereo */
        size_t len = 0;
        if (StereoFrame_serialize(&frame, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, "sensor/stereo", buf, (uint32_t)len);
            g.stereo_published++;
            /* 周期性日志（相机低频，每 30 帧打一次） */
            if (g.stereo_published % 30 == 1) {
                LOG_INFO("stereo_camera", "stereo #%lu %dx%d jpeg=%uB depth=%u "
                         "baseline=%.3fm fov=%.1f (captured=%lu fail=%lu)",
                         (unsigned long)g.stereo_published,
                         frame.width, frame.height,
                         frame.left_jpeg_size, frame.depth_count,
                         frame.baseline_m, frame.fov_deg,
                         (unsigned long)g.frames_captured,
                         (unsigned long)g.frames_failed);
            }
        } else {
            g.frames_failed++;
            LOG_ERROR("stereo_camera", "StereoFrame_serialize failed (len=%zu)", len);
        }
    }
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface stereo_vtable = {
    .execute = stereo_execute,
};

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { "sensor/stereo", NULL };

static NodePlugin s_plugin;

static int stereo_camera_init(MessageBus* bus, Transport* transport,
                              DiscoveryManager* discovery, Scheduler* scheduler,
                              const char* params_json) {
    (void)bus;
    g.scheduler = scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;

    /* 默认参数 */
    g.enabled      = 1;
    g.fps          = 10;
    g.width        = 320;
    g.height       = 240;
    g.baseline_m   = 0.075;   /* OAK-D 标准基线 */
    g.fov_deg      = 65.0;    /* OAK-D 水平 FOV */
    g.jpeg_quality = 70;
    g.dry_run      = 0;
    g.device_id[0] = '\0';    /* 默认空字符串，多 OAK-D 时用 device_id 指定序列号 */

    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            g.enabled      = 1; if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j)) g.enabled = j->valueint;
            g.fps          = 10; if ((j = cJSON_GetObjectItem(root, "fps")) && cJSON_IsNumber(j)) g.fps = j->valueint;
            g.width        = 320; if ((j = cJSON_GetObjectItem(root, "width")) && cJSON_IsNumber(j)) g.width = j->valueint;
            g.height       = 240; if ((j = cJSON_GetObjectItem(root, "height")) && cJSON_IsNumber(j)) g.height = j->valueint;
            g.baseline_m   = 0.075; if ((j = cJSON_GetObjectItem(root, "baseline_m")) && cJSON_IsNumber(j)) g.baseline_m = j->valuedouble;
            g.fov_deg      = 65.0; if ((j = cJSON_GetObjectItem(root, "fov_deg")) && cJSON_IsNumber(j)) g.fov_deg = j->valuedouble;
            g.jpeg_quality = 70; if ((j = cJSON_GetObjectItem(root, "jpeg_quality")) && cJSON_IsNumber(j)) g.jpeg_quality = j->valueint;
            g.dry_run      = 0; if ((j = cJSON_GetObjectItem(root, "dry_run")) && cJSON_IsNumber(j)) g.dry_run = j->valueint;
            snprintf(g.device_id, sizeof(g.device_id), "%s", ""); if ((j = cJSON_GetObjectItem(root, "device_id")) && cJSON_IsString(j)) snprintf(g.device_id, sizeof(g.device_id), "%s", j->valuestring);
            cJSON_Delete(root);
        }
    }

    /* dry-run 噪声种子 */
    srand((unsigned)time(NULL));

    if (!g.enabled) {
        LOG_INFO("stereo_camera", "disabled by config (enable=0), will not capture");
        return 0;
    }

    /* 检查 HAVE_DEPTHAI 宏：未定义则强制 dry_run 并提示集成方式 */
#ifndef HAVE_DEPTHAI
    if (!g.dry_run) {
        LOG_WARN("stereo_camera", "HAVE_DEPTHAI 未定义，无真实 OAK-D 读取路径，"
                 "强制 dry_run=1。集成步骤：1) pip install depthai 或编译 depthai C++ 库；"
                 "2) CMakeLists 加 find_package(depthai) + 链接；"
                 "3) target_compile_definitions(... PRIVATE HAVE_DEPTHAI)；"
                 "4) 在 read_stereo_frame() 的 #ifdef HAVE_DEPTHAI 分支填真实读取代码");
    }
    g.dry_run = 1;
#endif

    /* 向 discovery 通告本节点发布 sensor/stereo
     * (StereoFrame, type_id=0x669200d2, STEREOFRAME_TYPE_ID) */
    discovery_advertise(discovery, "sensor/stereo", STEREOFRAME_TYPE_ID,
                        CAP_PUBLISHER, (double)g.fps);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "stereo_camera");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)g.fps;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &stereo_vtable, &cfg) != 0) {
        LOG_WARN("stereo_camera", "task_base_init failed");
        return -1;
    }

    LOG_INFO("stereo_camera", "initialized: %dx%d fps=%d baseline=%.3fm fov=%.1f "
             "jpeg_q=%d device='%s' %s",
             g.width, g.height, g.fps, g.baseline_m, g.fov_deg,
             g.jpeg_quality, g.device_id,
             g.dry_run ? "[DRY-RUN]" : "[LIVE-OAKD]");

    if (g.dry_run) {
        LOG_INFO("stereo_camera", "DRY-RUN 模式：发布模拟立体数据 (左图伪 JPEG + "
                 "中心近/边缘远模拟深度)。集成真实 OAK-D 见本文件头注释 "
                 "(pip install depthai + CMakeLists find_package(depthai) + HAVE_DEPTHAI)");
    }
    return 0;
}

static int stereo_camera_start(void) {
    if (!g.enabled) return 0;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("stereo_camera", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("stereo_camera", "started (managed) (%s, %dfps target)",
             g.dry_run ? "dry-run" : "live OAK-D", g.fps);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void stereo_camera_stop(void) {
    task_stop(&g.taskbase);
}

static void stereo_camera_cleanup(void) {
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    LOG_INFO("stereo_camera", "cleanup: captured=%lu failed=%lu published=%lu",
             (unsigned long)g.frames_captured,
             (unsigned long)g.frames_failed,
             (unsigned long)g.stereo_published);
}

static int stereo_camera_health(void) {
    /* 健康判定：采集失败数明显多于成功数且失败超过阈值，视为异常。
     * 双目相机低频（10-30Hz 级），失败阈值比 IMU(1000) 小很多（10），
     * 低频相机连续失败更值得告警。 */
    if (!g.enabled) return 0;
    if (g.frames_failed > g.frames_captured && g.frames_failed > 10) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "stereo_camera",
    .version       = "1.0.0",
    .description   = "Stereo camera driver (OAK-D left+depth → StereoFrame → sensor/stereo)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = stereo_camera_init,
    .start         = stereo_camera_start,
    .stop          = stereo_camera_stop,
    .cleanup       = stereo_camera_cleanup,
    .health        = stereo_camera_health,
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
