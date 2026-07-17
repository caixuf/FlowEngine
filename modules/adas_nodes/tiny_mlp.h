/**
 * tiny_mlp.h — 依赖无关的极小多层感知机 (v3: 支持多隐层)
 *
 * 车端学习闭环 (Stage 2) 的推理内核。刻意保持零外部依赖。
 *
 * 支持架构（通过 hidden_count + hidden_dims[] 表达）：
 *   v1/v2: 单隐层 MLP  in → hidden → out
 *   v3:    多隐层 MLP  in → hidden[0] → hidden[1] → ... → hidden[N-1] → out
 *
 * 权重文件格式 (tools/train/model.txt)，`#` 开头为注释，token 以空白分隔：
 *
 *   # flowengine-tinymlp v3
 *   in <IN>
 *   hidden <HID0> [<HID1> ...]    # 多个值 = 多隐层
 *   out <OUT>
 *   norm_mean   <IN floats>
 *   norm_scale  <IN floats>
 *   out_mean    <OUT floats>
 *   out_scale   <OUT floats>
 *   w1 <HID0*IN floats>           # 隐层 0
 *   b1 <HID0 floats>
 *   w2 <HID1*HID0 floats>        # 隐层 1（仅当 hidden_count > 1）
 *   b2 <HID1 floats>
 *   w3 <HID2*HID1 floats>        # 隐层 2
 *   b3 <HID2 floats>
 *   w4 <HID3*HID2 floats>        # 隐层 3
 *   b4 <HID3 floats>
 *   w_out <OUT*HID{N-1} floats>  # 输出层
 *   b_out <OUT floats>
 *
 * 向后兼容（单隐层）：
 *   hidden 32     → hidden_count=1，等效 v1/v2
 *   w1/b1 = 隐层，w2/b2 = 输出层（v1/v2 格式仍可加载）
 *   hidden_count==1 时 w2 关键字解读为输出层
 */
#ifndef FLOWENGINE_TINY_MLP_H
#define FLOWENGINE_TINY_MLP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#define TINY_MLP_MAX_HID_LAYERS  4
#define TINY_MLP_MAX_IN     295   /* v3: 59 维/帧 × 5 帧 */
#define TINY_MLP_MAX_HID    256   /* 单层最大隐元数 */
#define TINY_MLP_MAX_OUT    16

typedef struct {
    int   in_dim;
    int   hid_dim;              /* backward compat: hidden_dims[0] */
    int   out_dim;
    float norm_mean[TINY_MLP_MAX_IN];
    float norm_scale[TINY_MLP_MAX_IN];
    float out_mean[TINY_MLP_MAX_OUT];
    float out_scale[TINY_MLP_MAX_OUT];
    float w1[TINY_MLP_MAX_HID * TINY_MLP_MAX_IN];    /* 隐层 0 [hid0][in] */
    float b1[TINY_MLP_MAX_HID];
    float w2[TINY_MLP_MAX_HID * TINY_MLP_MAX_HID];   /* 隐层 1 [hid1][hid0] */
    float b2[TINY_MLP_MAX_HID];
    float w_out[TINY_MLP_MAX_OUT * TINY_MLP_MAX_HID]; /* 输出层 [out][last_hid] */
    float b_out[TINY_MLP_MAX_OUT];
    int   loaded;
    /* v3 扩展 */
    int   hidden_count;          /* 隐层层数 (1..4)。1=单隐层(v1/v2) */
    int   hidden_dims[TINY_MLP_MAX_HID_LAYERS];
    float w3[TINY_MLP_MAX_HID * TINY_MLP_MAX_HID];   /* 隐层 2 */
    float b3[TINY_MLP_MAX_HID];
    float w4[TINY_MLP_MAX_HID * TINY_MLP_MAX_HID];   /* 隐层 3 */
    float b4[TINY_MLP_MAX_HID];
} TinyMLP;

/* ── 内部：读取 n 个 float 到 dst ──────────────────────────── */

static inline int tiny_mlp__read_floats(FILE* f, float* dst, int n, int cap) {
    int got = 0;
    for (int i = 0; i < n && i < cap; i++) {
        if (fscanf(f, " %f", &dst[i]) != 1) break;
        got++;
    }
    return got;
}

/* ── 前向推理 ──────────────────────────────────────────────── */

/**
 * 前向计算: y = out_denorm( ... tanh(W2 · tanh(W1 · norm(x) + b1) + b2) ... )
 * 支持 1..4 层隐层。x[in_dim] → y[out_dim]。
 * 返回 out_dim；模型无效时返回 0。
 */
static inline int tiny_mlp_forward(const TinyMLP* m, const float* x, float* y) {
    if (!m || !x || !y) return 0;
    if (m->in_dim <= 0 || m->hidden_count <= 0 || m->out_dim <= 0) return 0;

    /* 标准化并进入隐层 0 输入 */
    float buf[TINY_MLP_MAX_HID], *prev = buf;
    int prev_dim = m->in_dim;
    for (int i = 0; i < m->in_dim && i < TINY_MLP_MAX_IN; i++) {
        float s = m->norm_scale[i] != 0.0f ? m->norm_scale[i] : 1.0f;
        prev[i] = (x[i] - m->norm_mean[i]) / s;
    }

    /* 逐隐层计算（最多 TINY_MLP_MAX_HID_LAYERS 层） */
    float h[TINY_MLP_MAX_HID];
    const float* w_layers[] = { m->w1, m->w2, m->w3, m->w4 };
    const float* b_layers[] = { m->b1, m->b2, m->b3, m->b4 };
    int nl = m->hidden_count > TINY_MLP_MAX_HID_LAYERS ? TINY_MLP_MAX_HID_LAYERS : m->hidden_count;

    for (int layer = 0; layer < nl; layer++) {
        int hid = m->hidden_dims[layer];
        if (hid <= 0 || hid > TINY_MLP_MAX_HID) break;
        if (prev_dim <= 0) break;

        const float* w = w_layers[layer];
        const float* b = b_layers[layer];

        for (int j = 0; j < hid; j++) {
            float acc = b[j];
            for (int i = 0; i < prev_dim; i++) {
                acc += w[j * prev_dim + i] * prev[i];
            }
            h[j] = tanhf(acc);
        }

        /* 准备下一层输入 */
        memcpy(buf, h, hid * sizeof(float));
        prev = buf;
        prev_dim = hid;
    }

    /* 输出层 */
    for (int k = 0; k < m->out_dim; k++) {
        float acc = m->b_out[k];
        for (int j = 0; j < prev_dim; j++) {
            acc += m->w_out[k * prev_dim + j] * (prev ? prev[j] : 0.0f);
        }
        y[k] = acc * m->out_scale[k] + m->out_mean[k];
    }
    return m->out_dim;
}
/* backward compat: 旧 forward 接口，等价于新 forward */
#define tiny_mlp_forward_v1 tiny_mlp_forward

/* ── 从文件加载模型 ─────────────────────────────────────────── */

/**
 * 加载模型权重文件。成功返回 0 并置 m->loaded=1；失败返回 -1。
 *
 * 解析策略：
 *   - `hidden` 字段后跟 1 个值 → hidden_count=1，单隐层
 *   - `hidden` 字段后跟 N 个值 → hidden_count=N
 *   - `w2` 关键字：hidden_count==1 时读作输出层 (w_out)，
 *                  hidden_count>1 时读作隐层 1 (m->w2)
 *   - `w_out` 关键字：始终读作输出层
 */
static inline int tiny_mlp_load(TinyMLP* m, const char* path) {
    if (!m || !path) return -1;
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    memset(m, 0, sizeof(*m));
    /* 默认归一化为恒等 */
    for (int i = 0; i < TINY_MLP_MAX_IN; i++)  m->norm_scale[i] = 1.0f;
    for (int i = 0; i < TINY_MLP_MAX_OUT; i++) m->out_scale[i]  = 1.0f;
    /* v3: default to single hidden if not specified */
    m->hidden_count = 1;
    m->hidden_dims[0] = 0;

    char tok[64];
    int ok = 1;
    while (fscanf(f, " %63s", tok) == 1) {
        if (tok[0] == '#') {
            int c;
            while ((c = fgetc(f)) != EOF && c != '\n') { }
            continue;
        }
        if (strcmp(tok, "in") == 0) {
            if (fscanf(f, " %d", &m->in_dim) != 1) { ok = 0; break; }
        } else if (strcmp(tok, "hidden") == 0) {
            /* 读入一个或多个隐层维度 */
            m->hidden_count = 0;
            char line[256];
            if (!fgets(line, sizeof(line), f)) { ok = 0; break; }
            char* p = line;
            int val, nread;
            while (sscanf(p, " %d%n", &val, &nread) == 1) {
                if (m->hidden_count < TINY_MLP_MAX_HID_LAYERS) {
                    m->hidden_dims[m->hidden_count++] = val;
                }
                p += nread;
            }
            if (m->hidden_count > 0) {
                m->hid_dim = m->hidden_dims[0];
            }
        } else if (strcmp(tok, "out") == 0) {
            if (fscanf(f, " %d", &m->out_dim) != 1) { ok = 0; break; }
        } else if (strcmp(tok, "norm_mean") == 0) {
            tiny_mlp__read_floats(f, m->norm_mean, m->in_dim, TINY_MLP_MAX_IN);
        } else if (strcmp(tok, "norm_scale") == 0) {
            tiny_mlp__read_floats(f, m->norm_scale, m->in_dim, TINY_MLP_MAX_IN);
        } else if (strcmp(tok, "out_mean") == 0) {
            tiny_mlp__read_floats(f, m->out_mean, m->out_dim, TINY_MLP_MAX_OUT);
        } else if (strcmp(tok, "out_scale") == 0) {
            tiny_mlp__read_floats(f, m->out_scale, m->out_dim, TINY_MLP_MAX_OUT);
        } else if (strcmp(tok, "w1") == 0) {
            int n = m->hidden_dims[0] * m->in_dim;
            tiny_mlp__read_floats(f, m->w1, n, TINY_MLP_MAX_HID * TINY_MLP_MAX_IN);
        } else if (strcmp(tok, "b1") == 0) {
            tiny_mlp__read_floats(f, m->b1, m->hidden_dims[0], TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "w2") == 0) {
            if (m->hidden_count > 1) {
                /* v3 多隐层: w2 = 隐层 1 */
                int n = m->hidden_dims[1] * m->hidden_dims[0];
                tiny_mlp__read_floats(f, m->w2, n, TINY_MLP_MAX_HID * TINY_MLP_MAX_HID);
            } else {
                /* v1/v2 兼容: w2 = 输出层 */
                int n = m->out_dim * m->hid_dim;
                tiny_mlp__read_floats(f, m->w_out, n, TINY_MLP_MAX_OUT * TINY_MLP_MAX_HID);
            }
        } else if (strcmp(tok, "b2") == 0) {
            if (m->hidden_count > 1) {
                tiny_mlp__read_floats(f, m->b2, m->hidden_dims[1], TINY_MLP_MAX_HID);
            } else {
                tiny_mlp__read_floats(f, m->b_out, m->out_dim, TINY_MLP_MAX_OUT);
            }
        } else if (strcmp(tok, "w3") == 0) {
            int n = m->hidden_dims[2] * m->hidden_dims[1];
            tiny_mlp__read_floats(f, m->w3, n, TINY_MLP_MAX_HID * TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "b3") == 0) {
            tiny_mlp__read_floats(f, m->b3, m->hidden_dims[2], TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "w4") == 0) {
            int n = m->hidden_dims[3] * m->hidden_dims[2];
            tiny_mlp__read_floats(f, m->w4, n, TINY_MLP_MAX_HID * TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "b4") == 0) {
            tiny_mlp__read_floats(f, m->b4, m->hidden_dims[3], TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "w_out") == 0) {
            int last_hid = m->hidden_dims[m->hidden_count - 1];
            int n = m->out_dim * last_hid;
            tiny_mlp__read_floats(f, m->w_out, n, TINY_MLP_MAX_OUT * TINY_MLP_MAX_HID);
        } else if (strcmp(tok, "b_out") == 0) {
            tiny_mlp__read_floats(f, m->b_out, m->out_dim, TINY_MLP_MAX_OUT);
        }
        /* 未知关键字忽略 */
    }
    fclose(f);

    if (!ok ||
        m->in_dim <= 0 || m->in_dim > TINY_MLP_MAX_IN ||
        m->hidden_count <= 0 || m->hidden_count > TINY_MLP_MAX_HID_LAYERS ||
        m->out_dim <= 0 || m->out_dim > TINY_MLP_MAX_OUT) {
        return -1;
    }
    for (int li = 0; li < m->hidden_count; li++) {
        if (m->hidden_dims[li] <= 0 || m->hidden_dims[li] > TINY_MLP_MAX_HID) return -1;
    }

    m->loaded = 1;
    return 0;
}

/* ── 保存模型到文件 ─────────────────────────────────────────── */

/**
 * 保存模型权重到文本文件（与 tiny_mlp_load 兼容的格式）。
 * 单隐层模型写入 v1/v2 兼容格式（w2 作为输出层），
 * 多隐层模型写入 v3 格式（w_out）。
 * 成功返回 0，失败返回 -1。
 */
static inline int tiny_mlp_save(const TinyMLP* m, const char* path) {
    if (!m || !path || !m->loaded) return -1;
    FILE* f = fopen(path, "w");
    if (!f) return -1;

    fprintf(f, "# flowengine-tinymlp v%d\n", m->hidden_count > 1 ? 3 : 1);
    fprintf(f, "in %d\n", m->in_dim);
    fprintf(f, "hidden");
    for (int li = 0; li < m->hidden_count; li++)
        fprintf(f, " %d", m->hidden_dims[li]);
    fprintf(f, "\n");
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

    /* w1/b1 — always layer 0 */
    fprintf(f, "w1");
    for (int j = 0; j < m->hidden_dims[0]; j++)
        for (int i = 0; i < m->in_dim; i++)
            fprintf(f, " %g", (double)m->w1[j * m->in_dim + i]);
    fprintf(f, "\n");
    fprintf(f, "b1");
    for (int j = 0; j < m->hidden_dims[0]; j++) fprintf(f, " %g", (double)m->b1[j]);
    fprintf(f, "\n");

    /* 多隐层: w2/w3/w4 */
    const float* w_arr[] = { m->w2, m->w3, m->w4 };
    const float* b_arr[] = { m->b2, m->b3, m->b4 };
    for (int li = 1; li < m->hidden_count; li++) {
        int pdim = m->hidden_dims[li - 1];
        int cd = m->hidden_dims[li];
        fprintf(f, "w%d", li + 1);
        for (int j = 0; j < cd; j++)
            for (int i = 0; i < pdim; i++)
                fprintf(f, " %g", (double)w_arr[li - 1][j * pdim + i]);
        fprintf(f, "\n");
        fprintf(f, "b%d", li + 1);
        for (int j = 0; j < cd; j++) fprintf(f, " %g", (double)b_arr[li - 1][j]);
        fprintf(f, "\n");
    }

    /* 输出层: w_out/b_out（单隐层时用 w2/b2 标签保持兼容） */
    int last_hid = m->hidden_dims[m->hidden_count - 1];
    if (m->hidden_count <= 1) {
        fprintf(f, "w2");
    } else {
        fprintf(f, "w_out");
    }
    for (int k = 0; k < m->out_dim; k++)
        for (int j = 0; j < last_hid; j++)
            fprintf(f, " %g", (double)m->w_out[k * last_hid + j]);
    fprintf(f, "\n");

    if (m->hidden_count <= 1) {
        fprintf(f, "b2");
    } else {
        fprintf(f, "b_out");
    }
    for (int k = 0; k < m->out_dim; k++) fprintf(f, " %g", (double)m->b_out[k]);
    fprintf(f, "\n");

    fclose(f);
    return 0;
}

/* ── SGD 权重更新 ───────────────────────────────────────────── */

/**
 * 单样本 SGD 更新。
 * 损失: L = 0.5 * ||y_pred - y_true||^2  (反归一化空间 MSE)
 *
 * full_finetune=0: 仅更新输出层 W_out/b_out（保留隐层特征）
 * full_finetune=1: 更新全部参数（全量微调）
 *
 * 返回本步 MSE loss（反归一化）；模型无效时返回 0。
 */
static inline float tiny_mlp_sgd_step(TinyMLP* m, const float* x,
                                       const float* y_true,
                                       float lr, int full_finetune) {
    if (!m || !x || !y_true || !m->loaded) return 0.0f;
    int nl = m->hidden_count;
    if (nl <= 0 || nl > TINY_MLP_MAX_HID_LAYERS) return 0.0f;
    if (m->in_dim <= 0 || m->out_dim <= 0) return 0.0f;

    /* ── 隐层权重/偏置指针数组 ── */
    const float* w_layers[] = { m->w1, m->w2, m->w3, m->w4 };
    float* w_layers_rw[] = { m->w1, m->w2, m->w3, m->w4 };
    float* b_layers_rw[] = { m->b1, m->b2, m->b3, m->b4 };

    /* ── Forward ── */
    float xn[TINY_MLP_MAX_IN];
    for (int i = 0; i < m->in_dim; i++) {
        float s = m->norm_scale[i] != 0.0f ? m->norm_scale[i] : 1.0f;
        xn[i] = (x[i] - m->norm_mean[i]) / s;
    }

    /* 逐层计算并存储激活值用于反向传播 */
    float acts[TINY_MLP_MAX_HID_LAYERS + 1][TINY_MLP_MAX_HID];
    int   act_dims[TINY_MLP_MAX_HID_LAYERS + 1];
    /* acts[0] = 输入 (norm 后) */
    for (int i = 0; i < m->in_dim; i++) acts[0][i] = xn[i];
    act_dims[0] = m->in_dim;

    for (int layer = 0; layer < nl; layer++) {
        int cd = m->hidden_dims[layer];
        int pd = act_dims[layer];
        if (cd <= 0 || cd > TINY_MLP_MAX_HID) break;
        const float* w = w_layers[layer];
        const float* b = b_layers_rw[layer];
        for (int j = 0; j < cd; j++) {
            float acc = b[j];
            for (int i = 0; i < pd; i++)
                acc += w[j * pd + i] * acts[layer][i];
            acts[layer + 1][j] = tanhf(acc);
        }
        act_dims[layer + 1] = cd;
    }

    /* 输出层 */
    int last_hid = act_dims[nl];
    float y_pred[TINY_MLP_MAX_OUT];
    for (int k = 0; k < m->out_dim; k++) {
        float acc = m->b_out[k];
        for (int j = 0; j < last_hid; j++)
            acc += m->w_out[k * last_hid + j] * acts[nl][j];
        y_pred[k] = acc * m->out_scale[k] + m->out_mean[k];
    }

    /* ── MSE Loss ── */
    float loss = 0.0f;
    for (int k = 0; k < m->out_dim; k++) {
        float e = y_pred[k] - y_true[k];
        loss += e * e;
    }
    loss *= 0.5f;

    /* ── Backward ── */
    float delta_out[TINY_MLP_MAX_OUT];
    for (int k = 0; k < m->out_dim; k++) {
        float s = m->out_scale[k] != 0.0f ? m->out_scale[k] : 1.0f;
        delta_out[k] = (y_pred[k] - y_true[k]) * s;
    }

    /* 更新输出层 */
    for (int k = 0; k < m->out_dim; k++) {
        for (int j = 0; j < last_hid; j++)
            m->w_out[k * last_hid + j] -= lr * delta_out[k] * acts[nl][j];
        m->b_out[k] -= lr * delta_out[k];
    }

    if (!full_finetune) return loss;

    /* 反向传播到各隐层 */
    float delta[TINY_MLP_MAX_HID_LAYERS + 1][TINY_MLP_MAX_HID];
    /* delta[nl][j] = sum_k delta_out[k] * w_out[k,j] * (1 - h[j]²) */
    for (int layer = nl - 1; layer >= 0; layer--) {
        int cd = m->hidden_dims[layer];
        int nd = layer < nl - 1 ? m->hidden_dims[layer + 1] : m->out_dim;
        float* w_w = (layer < nl - 1) ? w_layers_rw[layer + 1] : m->w_out;
        int w_stride = (layer < nl - 1) ? act_dims[layer + 1] : last_hid;
        float* d_prev = (layer < nl - 1) ? delta[layer + 1] : delta_out;
        int d_prev_dim = nd;

        for (int j = 0; j < cd; j++) {
            float acc = 0.0f;
            for (int k = 0; k < d_prev_dim; k++)
                acc += d_prev[k] * w_w[k * w_stride + j];
            delta[layer][j] = acc * (1.0f - acts[layer + 1][j] * acts[layer + 1][j]);
        }
    }

    /* 更新各隐层权重 */
    for (int layer = 0; layer < nl; layer++) {
        int cd = m->hidden_dims[layer];
        int pd = act_dims[layer];
        float* w = w_layers_rw[layer];
        float* b = b_layers_rw[layer];
        for (int j = 0; j < cd; j++) {
            for (int i = 0; i < pd; i++)
                w[j * pd + i] -= lr * delta[layer][j] * acts[layer][i];
            b[j] -= lr * delta[layer][j];
        }
    }

    return loss;
}

/* backward compat: sgd_step v1/v2 */
#define tiny_mlp_sgd_step_v1 tiny_mlp_sgd_step

#endif /* FLOWENGINE_TINY_MLP_H */
