/**
 * onnx_backend.cpp — ONNX Runtime 推理后端实现
 *
 * 两条编译路径：
 *   - HAVE_ONNXRUNTIME 定义时（cmake -DENABLE_ONNX=ON 且找到 onnxruntime）：
 *     包 ORT C++ API，建 1×in_dim 输入张量、Session::Run、拷回 out_dim 个 float。
 *   - 未定义时（默认）：所有函数走降级路径（load→-1、forward→0），
 *     等价「无 ONNX 后端」，inference_node 自动回退 tiny-MLP。行为与未引入本文件前一致。
 *
 * 与 tiny_mlp_forward 的数值对齐：训练侧（tools/train_e2e）把归一化/反归一化
 * 折进导出的计算图，故本后端只喂原始特征 x、取原始输出 y，无需在这里做 norm。
 */
#include "onnx_backend.h"

#include <string.h>
#include <stdio.h>

#ifdef HAVE_ONNXRUNTIME

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>

/* session/env 擦除为 void*，此处按类型还原。 */
namespace {
struct OrtHolder {
    Ort::Env     env;
    Ort::Session session;
    std::string  input_name;
    std::string  output_name;
    OrtHolder(const char* path)
        : env(ORT_LOGGING_LEVEL_WARNING, "flow-onnx"),
          session(nullptr) {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, path, opts);
    }
};
}  /* namespace */

extern "C" int onnx_backend_load(OnnxBackend* b, const char* path) {
    if (!b || !path) return -1;
    memset(b, 0, sizeof(*b));

    try {
        OrtHolder* h = new OrtHolder(path);

        if (h->session.GetInputCount() != 1 || h->session.GetOutputCount() != 1) {
            fprintf(stderr, "[onnx] %s: 需单输入单输出（实际 in=%zu out=%zu）\n",
                    path, h->session.GetInputCount(), h->session.GetOutputCount());
            delete h;
            return -1;
        }

        Ort::AllocatorWithDefaultOptions alloc;
        Ort::AllocatedStringPtr in_nm  = h->session.GetInputNameAllocated(0, alloc);
        Ort::AllocatedStringPtr out_nm = h->session.GetOutputNameAllocated(0, alloc);
        h->input_name  = in_nm.get();
        h->output_name = out_nm.get();

        /* 读输入/输出形状与元素类型。 */
        auto in_ti  = h->session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        auto out_ti = h->session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
        auto in_shape  = in_ti.GetShape();
        auto out_shape = out_ti.GetShape();

        /* 元素类型必须 float32：forward 用 GetTensorData<float> 直接取数。
         * fp16/int8 等模型拒绝，避免静默错推理。 */
        if (in_ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            out_ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            fprintf(stderr, "[onnx] %s: 仅支持 float32 输入/输出 "
                    "(in_type=%d out_type=%d)\n",
                    path, (int)in_ti.GetElementType(), (int)out_ti.GetElementType());
            delete h;
            return -1;
        }

        /* 形状：接受 [N]（rank-1）或 [1, N]（rank-2）——本工具导出恒为 [1, N]。
         * rank≥3（如 [1,2,5]）无法确定单样本向量语义，forward 会按末维错取
         * 元素数；rank-2 首维须 1（单样本批）。一律拒绝而非静默错推理。 */
        size_t in_rank = in_shape.size(), out_rank = out_shape.size();
        if (!(in_rank == 1 || in_rank == 2) || !(out_rank == 1 || out_rank == 2)) {
            fprintf(stderr, "[onnx] %s: 输入/输出须 rank 1/2（实际 "
                    "in_rank=%zu out_rank=%zu）\n", path, in_rank, out_rank);
            delete h;
            return -1;
        }
        if (in_rank == 2 && in_shape[0] != 1) {
            fprintf(stderr, "[onnx] %s: 输入首维须 1（batch），实际 %lld\n",
                    path, (long long)in_shape[0]);
            delete h;
            return -1;
        }

        /* 末维即特征数，须静态且 > 0。 */
        int64_t in_last  = in_shape.back();
        int64_t out_last = out_shape.back();
        if (in_last <= 0 || out_last <= 0) {
            fprintf(stderr, "[onnx] %s: 输入/输出维度非静态（in=%lld out=%lld）\n",
                    path, (long long)in_last, (long long)out_last);
            delete h;
            return -1;
        }

        b->session = h;
        b->env     = &h->env;   /* 仅作非空标记，生命周期归 OrtHolder */
        b->in_dim  = (int)in_last;
        b->out_dim = (int)out_last;
        b->loaded  = 1;
        return 0;
    } catch (const Ort::Exception& e) {
        fprintf(stderr, "[onnx] load %s failed: %s\n", path, e.what());
        return -1;
    } catch (...) {
        fprintf(stderr, "[onnx] load %s failed: unknown\n", path);
        return -1;
    }
}

extern "C" int onnx_backend_forward(const OnnxBackend* b, const float* x, float* y) {
    if (!b || !b->loaded || !b->session || !x || !y) return 0;
    OrtHolder* h = static_cast<OrtHolder*>(b->session);

    try {
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);
        int64_t shape[2] = {1, (int64_t)b->in_dim};
        Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
            mem, const_cast<float*>(x), (size_t)b->in_dim, shape, 2);

        const char* in_names[]  = { h->input_name.c_str() };
        const char* out_names[] = { h->output_name.c_str() };
        auto outs = h->session.Run(Ort::RunOptions{nullptr},
                                   in_names, &in_tensor, 1, out_names, 1);
        if (outs.empty() || !outs[0].IsTensor()) return 0;

        const float* out_data = outs[0].GetTensorData<float>();
        size_t n = outs[0].GetTensorTypeAndShapeInfo().GetElementCount();
        int out_dim = (int)n < b->out_dim ? (int)n : b->out_dim;
        memcpy(y, out_data, (size_t)out_dim * sizeof(float));
        return out_dim;
    } catch (const Ort::Exception& e) {
        fprintf(stderr, "[onnx] forward failed: %s\n", e.what());
        return 0;
    } catch (...) {
        return 0;
    }
}

extern "C" void onnx_backend_free(OnnxBackend* b) {
    if (!b) return;
    if (b->session) {
        delete static_cast<OrtHolder*>(b->session);
    }
    memset(b, 0, sizeof(*b));
}

#else  /* !HAVE_ONNXRUNTIME —— 降级：无 ONNX 后端，一律回退 tiny-MLP */

extern "C" int onnx_backend_load(OnnxBackend* b, const char* path) {
    (void)path;
    if (b) memset(b, 0, sizeof(*b));
    return -1;  /* 未编译 ORT：拒绝，调用方降级 tiny-MLP */
}

extern "C" int onnx_backend_forward(const OnnxBackend* b, const float* x, float* y) {
    (void)b; (void)x; (void)y;
    return 0;
}

extern "C" void onnx_backend_free(OnnxBackend* b) {
    if (b) memset(b, 0, sizeof(*b));
}

#endif /* HAVE_ONNXRUNTIME */
