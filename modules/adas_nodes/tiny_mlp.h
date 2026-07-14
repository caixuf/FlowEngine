/**
 * tiny_mlp.h — 依赖无关的极小多层感知机 (single hidden layer)
 *
 * 车端学习闭环 (Stage 2) 的推理内核。刻意保持零外部依赖：
 *   - 不引入 LibTorch / ONNX Runtime，CI 即可构建
 *   - 权重从纯文本文件加载，Python 侧 (tools/train/) 训练后导出同一格式
 *   - 前向计算：y = out_denorm( W2 * tanh(W1 * norm(x) + b1) + b2 )
 *
 * 这是"模型跑进 pipeline"这条链路的最小可验证实现。后续可以在 inference_node
 * 中把 tiny_mlp_forward() 替换为 ONNX Runtime / TensorRT 的推理调用，而数据契约
 * (输入/输出语义) 保持不变。
 *
 * 权重文件格式 (tools/train/model.txt)，`#` 开头为注释，token 以空白分隔：
 *   # flowengine-tinymlp v1
 *   in <IN>
 *   hidden <HID>
 *   out <OUT>
 *   norm_mean   <IN floats>     # 输入标准化: xn = (x - mean) / scale
 *   norm_scale  <IN floats>
 *   out_mean    <OUT floats>    # 输出反标准化: y = yn * scale + mean
 *   out_scale   <OUT floats>
 *   w1 <HID*IN floats>          # 行主序 [HID][IN]
 *   b1 <HID floats>
 *   w2 <OUT*HID floats>         # 行主序 [OUT][HID]
 *   b2 <OUT floats>
 */
#ifndef FLOWENGINE_TINY_MLP_H
#define FLOWENGINE_TINY_MLP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#define TINY_MLP_MAX_IN     128
#define TINY_MLP_MAX_HID    64
#define TINY_MLP_MAX_OUT    16

typedef struct {
    int   in_dim;
    int   hid_dim;
    int   out_dim;
    float norm_mean[TINY_MLP_MAX_IN];
    float norm_scale[TINY_MLP_MAX_IN];
    float out_mean[TINY_MLP_MAX_OUT];
    float out_scale[TINY_MLP_MAX_OUT];
    float w1[TINY_MLP_MAX_HID * TINY_MLP_MAX_IN];   /* [hid][in] */
    float b1[TINY_MLP_MAX_HID];
    float w2[TINY_MLP_MAX_OUT * TINY_MLP_MAX_HID];  /* [out][hid] */
    float b2[TINY_MLP_MAX_OUT];
    int   loaded;   /* 1 = 从文件加载, 0 = 使用调用方提供的 fallback */
} TinyMLP;

/* 前向推理: x[in_dim] -> y[out_dim]。返回 out_dim；模型无效时返回 0。 */
static inline int tiny_mlp_forward(const TinyMLP* m, const float* x, float* y) {
    if (!m || !x || !y) return 0;
    if (m->in_dim <= 0 || m->hid_dim <= 0 || m->out_dim <= 0) return 0;

    float h[TINY_MLP_MAX_HID];
    for (int j = 0; j < m->hid_dim; j++) {
        float acc = m->b1[j];
        for (int i = 0; i < m->in_dim; i++) {
            float scale = m->norm_scale[i] != 0.0f ? m->norm_scale[i] : 1.0f;
            float xn = (x[i] - m->norm_mean[i]) / scale;
            acc += m->w1[j * m->in_dim + i] * xn;
        }
        h[j] = tanhf(acc);
    }
    for (int k = 0; k < m->out_dim; k++) {
        float acc = m->b2[k];
        for (int j = 0; j < m->hid_dim; j++) {
            acc += m->w2[k * m->hid_dim + j] * h[j];
        }
        y[k] = acc * m->out_scale[k] + m->out_mean[k];
    }
    return m->out_dim;
}

/* 读取 n 个 float 到 dst（跳过注释/空白）。返回读取个数。 */
static inline int tiny_mlp__read_floats(FILE* f, float* dst, int n, int cap) {
    int got = 0;
    for (int i = 0; i < n && i < cap; i++) {
        if (fscanf(f, " %f", &dst[i]) != 1) break;
        got++;
    }
    return got;
}

/*
 * 从文本文件加载模型。成功返回 0 并置 m->loaded=1；失败返回 -1（不修改 loaded）。
 * 解析器容忍任意行顺序，按关键字驱动。
 */
static inline int tiny_mlp_load(TinyMLP* m, const char* path) {
    if (!m || !path) return -1;
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    memset(m, 0, sizeof(*m));
    /* 默认归一化为恒等，避免文件未提供时除零 */
    for (int i = 0; i < TINY_MLP_MAX_IN; i++)  m->norm_scale[i] = 1.0f;
    for (int i = 0; i < TINY_MLP_MAX_OUT; i++) m->out_scale[i]  = 1.0f;

    char tok[64];
    int ok = 1;
    while (fscanf(f, " %63s", tok) == 1) {
        if (tok[0] == '#') {                 /* 跳过整行注释 */
            int c;
            while ((c = fgetc(f)) != EOF && c != '\n') { }
            continue;
        }
        if      (strcmp(tok, "in") == 0)     { if (fscanf(f, " %d", &m->in_dim)  != 1) { ok = 0; break; } }
        else if (strcmp(tok, "hidden") == 0) { if (fscanf(f, " %d", &m->hid_dim) != 1) { ok = 0; break; } }
        else if (strcmp(tok, "out") == 0)    { if (fscanf(f, " %d", &m->out_dim) != 1) { ok = 0; break; } }
        else if (strcmp(tok, "norm_mean") == 0)  tiny_mlp__read_floats(f, m->norm_mean,  m->in_dim,  TINY_MLP_MAX_IN);
        else if (strcmp(tok, "norm_scale") == 0) tiny_mlp__read_floats(f, m->norm_scale, m->in_dim,  TINY_MLP_MAX_IN);
        else if (strcmp(tok, "out_mean") == 0)   tiny_mlp__read_floats(f, m->out_mean,   m->out_dim, TINY_MLP_MAX_OUT);
        else if (strcmp(tok, "out_scale") == 0)  tiny_mlp__read_floats(f, m->out_scale,  m->out_dim, TINY_MLP_MAX_OUT);
        else if (strcmp(tok, "w1") == 0)  tiny_mlp__read_floats(f, m->w1, m->hid_dim * m->in_dim,  TINY_MLP_MAX_HID * TINY_MLP_MAX_IN);
        else if (strcmp(tok, "b1") == 0)  tiny_mlp__read_floats(f, m->b1, m->hid_dim,              TINY_MLP_MAX_HID);
        else if (strcmp(tok, "w2") == 0)  tiny_mlp__read_floats(f, m->w2, m->out_dim * m->hid_dim, TINY_MLP_MAX_OUT * TINY_MLP_MAX_HID);
        else if (strcmp(tok, "b2") == 0)  tiny_mlp__read_floats(f, m->b2, m->out_dim,              TINY_MLP_MAX_OUT);
        /* 未知关键字：忽略，继续 */
    }
    fclose(f);

    if (!ok ||
        m->in_dim  <= 0 || m->in_dim  > TINY_MLP_MAX_IN ||
        m->hid_dim <= 0 || m->hid_dim > TINY_MLP_MAX_HID ||
        m->out_dim <= 0 || m->out_dim > TINY_MLP_MAX_OUT) {
        return -1;
    }
    m->loaded = 1;
    return 0;
}

/*
 * 保存模型权重到文本文件（与 tiny_mlp_load 兼容的格式）。
 * 成功返回 0，失败返回 -1。
 */
static inline int tiny_mlp_save(const TinyMLP* m, const char* path) {
    if (!m || !path || !m->loaded) return -1;
    FILE* f = fopen(path, "w");
    if (!f) return -1;

    fprintf(f, "# flowengine-tinymlp v1\n");
    fprintf(f, "in %d\n", m->in_dim);
    fprintf(f, "hidden %d\n", m->hid_dim);
    fprintf(f, "out %d\n", m->out_dim);

    fprintf(f, "norm_mean");
    for (int i = 0; i < m->in_dim; i++) fprintf(f, " %g", (double)m->norm_mean[i]);
    fprintf(f, "\n");

    fprintf(f, "norm_scale");
    for (int i = 0; i < m->in_dim; i++) fprintf(f, " %g", (double)m->norm_scale[i]);
    fprintf(f, "\n");

    fprintf(f, "out_mean");
    for (int i = 0; i < m->out_dim; i++) fprintf(f, " %g", (double)m->out_mean[i]);
    fprintf(f, "\n");

    fprintf(f, "out_scale");
    for (int i = 0; i < m->out_dim; i++) fprintf(f, " %g", (double)m->out_scale[i]);
    fprintf(f, "\n");

    fprintf(f, "w1");
    for (int j = 0; j < m->hid_dim; j++)
        for (int i = 0; i < m->in_dim; i++)
            fprintf(f, " %g", (double)m->w1[j * m->in_dim + i]);
    fprintf(f, "\n");

    fprintf(f, "b1");
    for (int j = 0; j < m->hid_dim; j++) fprintf(f, " %g", (double)m->b1[j]);
    fprintf(f, "\n");

    fprintf(f, "w2");
    for (int k = 0; k < m->out_dim; k++)
        for (int j = 0; j < m->hid_dim; j++)
            fprintf(f, " %g", (double)m->w2[k * m->hid_dim + j]);
    fprintf(f, "\n");

    fprintf(f, "b2");
    for (int k = 0; k < m->out_dim; k++) fprintf(f, " %g", (double)m->b2[k]);
    fprintf(f, "\n");

    fclose(f);
    return 0;
}

/*
 * 单样本 SGD 权重更新。
 *
 * 损失函数: L = 0.5 * ||y_pred - y_true||^2  (反归一化空间 MSE)
 *
 * full_finetune=0: 仅更新 W2/b2（顶层，保留 W1/b1 backbone，防止灾难性遗忘）
 * full_finetune=1: 更新全部参数
 *
 * 返回本步骤的 MSE loss（反归一化空间）；模型无效时返回 0。
 */
static inline float tiny_mlp_sgd_step(TinyMLP* m, const float* x,
                                       const float* y_true,
                                       float lr, int full_finetune) {
    if (!m || !x || !y_true || !m->loaded) return 0.0f;
    if (m->in_dim <= 0 || m->hid_dim <= 0 || m->out_dim <= 0) return 0.0f;

    /* ── Forward ─────────────────────────────────────── */
    float xn[TINY_MLP_MAX_IN];
    for (int i = 0; i < m->in_dim; i++) {
        float s = m->norm_scale[i] != 0.0f ? m->norm_scale[i] : 1.0f;
        xn[i] = (x[i] - m->norm_mean[i]) / s;
    }

    float h[TINY_MLP_MAX_HID];
    for (int j = 0; j < m->hid_dim; j++) {
        float acc = m->b1[j];
        for (int i = 0; i < m->in_dim; i++)
            acc += m->w1[j * m->in_dim + i] * xn[i];
        h[j] = tanhf(acc);
    }

    float y_pred[TINY_MLP_MAX_OUT];
    for (int k = 0; k < m->out_dim; k++) {
        float acc = m->b2[k];
        for (int j = 0; j < m->hid_dim; j++)
            acc += m->w2[k * m->hid_dim + j] * h[j];
        y_pred[k] = acc * m->out_scale[k] + m->out_mean[k];
    }

    /* ── MSE Loss ─────────────────────────────────────── */
    float loss = 0.0f;
    for (int k = 0; k < m->out_dim; k++) {
        float e = y_pred[k] - y_true[k];
        loss += e * e;
    }
    loss *= 0.5f;

    /* ── Backward ─────────────────────────────────────── */
    /* delta_out[k] = dL/dy_norm[k] = (y_pred[k] - y_true[k]) * out_scale[k] */
    float delta_out[TINY_MLP_MAX_OUT];
    for (int k = 0; k < m->out_dim; k++) {
        float s = m->out_scale[k] != 0.0f ? m->out_scale[k] : 1.0f;
        delta_out[k] = (y_pred[k] - y_true[k]) * s;
    }

    /* delta_h[j] = (sum_k delta_out[k]*W2[k,j]) * (1 - h[j]^2) */
    float delta_h[TINY_MLP_MAX_HID];
    for (int j = 0; j < m->hid_dim; j++) {
        float acc = 0.0f;
        for (int k = 0; k < m->out_dim; k++)
            acc += delta_out[k] * m->w2[k * m->hid_dim + j];
        delta_h[j] = acc * (1.0f - h[j] * h[j]);
    }

    /* ── Weight update: W2, b2 (always) ────────────────── */
    for (int k = 0; k < m->out_dim; k++) {
        for (int j = 0; j < m->hid_dim; j++)
            m->w2[k * m->hid_dim + j] -= lr * delta_out[k] * h[j];
        m->b2[k] -= lr * delta_out[k];
    }

    /* ── Weight update: W1, b1 (full_finetune only) ─────── */
    if (full_finetune) {
        for (int j = 0; j < m->hid_dim; j++) {
            for (int i = 0; i < m->in_dim; i++)
                m->w1[j * m->in_dim + i] -= lr * delta_h[j] * xn[i];
            m->b1[j] -= lr * delta_h[j];
        }
    }

    return loss;
}

#endif /* FLOWENGINE_TINY_MLP_H */
