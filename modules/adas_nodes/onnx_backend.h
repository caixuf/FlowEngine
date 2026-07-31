/**
 * onnx_backend.h — ONNX Runtime 推理后端（可选）· 纯接口
 *
 * 与 tiny_mlp.h 并存的第二个「前向计算」后端。本头文件**不依赖**
 * onnxruntime，任何编译单元都可 include；真正的 ORT 调用集中在
 * onnx_backend.cpp 的 `#ifdef HAVE_ONNXRUNTIME` 块内。未编译 ORT 时
 * 所有函数走降级路径（load 返回 -1、forward 返回 0），行为等价「无模型」。
 *
 * 契约与 tiny-MLP 最小接口对齐（inference_node.cpp 的 run_inference
 * 据此切换后端，其余特征组装/输出映射逻辑完全复用，不因后端而变）：
 *   - onnx_backend_load(path)  成功返回 0 并填 in_dim/out_dim/loaded=1；失败返回 -1
 *   - onnx_backend_forward(x,y) 返回实际输出维度 out_dim（未加载返回 0），
 *                               语义等价 tiny_mlp_forward
 *   - onnx_backend_free()       释放底层 session/env，loaded 置 0
 *
 * 输入/输出维度语义（in_dim ∈ {4,16,80,115}、out_dim ∈ {1,2,4,5,9}）由
 * inference_node.cpp 统一负责校验，本后端只做「读 in_dim 个 float、写
 * out_dim 个 float」的张量前向，不理解特征含义。
 */
#ifndef FLOW_ONNX_BACKEND_H
#define FLOW_ONNX_BACKEND_H

#ifdef __cplusplus
extern "C" {
#endif

/* 不透明后端句柄。session/env 是 ORT 会话/环境对象的擦除指针，
 * 未加载或未编译 ORT 时为 NULL——本结构体不引入任何 ORT 头依赖。 */
typedef struct OnnxBackend {
    void* session;   /* Ort::Session 擦除指针，NULL=未加载 */
    void* env;       /* Ort::Env 擦除指针 */
    int   in_dim;    /* 模型输入维度（由 .onnx 元信息读出） */
    int   out_dim;   /* 模型输出维度 */
    int   loaded;    /* 1=已加载可推理 */
} OnnxBackend;

/* 加载 .onnx 模型。成功返回 0（填 in_dim/out_dim/loaded=1）；
 * 失败——文件不存在 / 未编译 HAVE_ONNXRUNTIME / 图非单输入单输出 /
 * 维度非静态——返回 -1 且 loaded=0。 */
int  onnx_backend_load(OnnxBackend* b, const char* path);

/* 前向：读 in_dim 个 x，写 out_dim 个 y，返回 out_dim；未加载返回 0。 */
int  onnx_backend_forward(const OnnxBackend* b, const float* x, float* y);

/* 释放底层 session/env，loaded 置 0（可重复调用，幂等）。 */
void onnx_backend_free(OnnxBackend* b);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_ONNX_BACKEND_H */
