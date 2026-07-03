/**
 * ipc_demo.c — 跨进程 IPC 通道演示（Step 1）
 *
 * 用法：
 *   # 终端1（订阅进程）
 *   ./ipc_demo sub
 *
 *   # 终端2（发布进程）
 *   ./ipc_demo pub
 *
 * 发布进程每秒发送一条消息，订阅进程收到后打印。
 */

#include "ipc_channel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#define CHANNEL_NAME  "ipc_demo_ch"
#define QUEUE_DEPTH   16

typedef struct {
    uint32_t seq;
    float    value;
    char     text[64];
} DemoMsg;

static volatile int g_running = 1;

static void sighandler(int sig) { (void)sig; g_running = 0; }

/* ── Subscriber callback ─────────────────────────────── */

static void on_msg(const Message* msg, void* user_data) {
    (void)user_data;
    if (msg->data_size != sizeof(DemoMsg)) {
        printf("[SUB] 收到未知大小消息 (size=%u)\n", msg->data_size);
        return;
    }
    const DemoMsg* d = (const DemoMsg*)msg->data;
    printf("[SUB] topic=%-20s seq=%-4u value=%.2f text=%s\n",
           msg->topic, d->seq, d->value, d->text);
}

/* ── Publisher ───────────────────────────────────────── */

static int run_publisher(void) {
    printf("[PUB] 打开 IPC 通道: %s\n", CHANNEL_NAME);
    IpcChannel* ch = ipc_channel_open(CHANNEL_NAME, IPC_ROLE_PUBLISHER, QUEUE_DEPTH);
    if (!ch) { fprintf(stderr, "[PUB] 打开失败\n"); return 1; }

    uint32_t seq = 0;
    printf("[PUB] 开始发布，按 Ctrl+C 停止\n");
    while (g_running) {
        DemoMsg d = {
            .seq   = seq,
            .value = (float)seq * 0.1f,
        };
        snprintf(d.text, sizeof(d.text), "msg-%u", seq);

        int ret = ipc_channel_publish(ch, "ipc/demo", "ipc_pub",
                                      &d, sizeof(d));
        if (ret == 0)
            printf("[PUB] 发布 seq=%u\n", seq);
        else
            printf("[PUB] 队列满，跳过 seq=%u\n", seq);

        seq++;
        sleep(1);
    }

    ipc_channel_close(ch);
    printf("[PUB] 关闭\n");
    return 0;
}

/* ── Subscriber ──────────────────────────────────────── */

static int run_subscriber(void) {
    printf("[SUB] 打开 IPC 通道: %s\n", CHANNEL_NAME);

    /* 等待发布进程创建共享内存（最多 5 秒） */
    IpcChannel* ch = NULL;
    for (int i = 0; i < 10 && !ch; i++) {
        ch = ipc_channel_open(CHANNEL_NAME, IPC_ROLE_SUBSCRIBER, QUEUE_DEPTH);
        if (!ch) { printf("[SUB] 等待发布进程...\n"); usleep(500000); }
    }
    if (!ch) { fprintf(stderr, "[SUB] 打开失败，请先启动发布进程\n"); return 1; }

    ipc_channel_subscribe(ch, on_msg, NULL);

    printf("[SUB] 等待消息，按 Ctrl+C 停止\n");
    while (g_running)
        ipc_channel_recv_once(ch, 1000);   /* 1s 超时以检查 g_running */

    ipc_channel_close(ch);
    printf("[SUB] 关闭\n");
    return 0;
}

/* ── main ────────────────────────────────────────────── */

int main(int argc, char* argv[]) {
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    if (argc < 2) {
        fprintf(stderr, "用法: %s <pub|sub>\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "pub") == 0) return run_publisher();
    if (strcmp(argv[1], "sub") == 0) return run_subscriber();

    fprintf(stderr, "未知角色: %s (应为 pub 或 sub)\n", argv[1]);
    return 1;
}
