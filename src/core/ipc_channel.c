/**
 * ipc_channel.c — 跨进程共享内存通道实现（广播 / 多订阅者扇出）
 *
 * 使用 POSIX shm_open + mmap 实现进程间通信, 互斥/唤醒使用存放在共享内存中的
 * process-shared pthread_mutex_t + pthread_cond_t（而非命名信号量 + 轮询）。
 * 环形缓冲区存储在共享内存中。
 *
 * 共享内存布局：
 *   [ShmHeader{mutex,cond,head,...}][ShmSlot * queue_depth]
 *
 * 广播语义（重要）:
 *   同一个 topic 可能有多个订阅者进程，它们打开的是同一块共享内存。
 *   若采用「单消费者队列」(消费即出队) 语义, 每条消息只会被其中一个订阅者
 *   抢到, 导致多进程模式下各节点各自只收到 ~1/N 的消息, 出现丢帧 / 卡顿。
 *
 *   因此本实现采用「广播环形缓冲」:
 *     - 发布者只递增全局写游标 head, 覆盖最旧的槽 (drop_oldest), 从不阻塞;
 *     - 每个订阅者在自己的进程内维护独立读游标 read_cursor, 只读不出队;
 *     - 订阅者落后超过 queue_depth 时自动跳到最新窗口 (丢弃过期消息)。
 *   这样每个订阅者都能独立读到全部消息, 实现真正的 pub/sub 扇出。
 *
 * 延迟/唤醒（重要）:
 *   早期实现里订阅者后台线程用 usleep(1ms) 轮询检查 head 是否前进, 这会带来
 *   约 0.5~1ms 的固定附加延迟和抖动, 在多跳流水线上会累积放大, 明显比同进程
 *   下 pthread_cond 事件唤醒的多线程模式"卡"。现在发布者写入后在共享内存里
 *   对 process-shared 条件变量做 broadcast, 所有订阅者进程的等待线程被立即
 *   唤醒（而不是等下一次轮询窗口）; 仍保留一个较长的 timedwait 兜底超时,
 *   用于响应停止请求 / 容错漏唤醒。
 */

#include "ipc_channel.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

/* ── Shared memory header (lives inside the shm region) ─
 * mutex/cond are PTHREAD_PROCESS_SHARED so every process that mmaps this
 * region can lock/wait/broadcast on the *same* underlying primitive — this
 * replaces the old named-semaphore + fixed-interval polling scheme with
 * real event-driven wakeups across process boundaries. */
typedef struct {
    pthread_mutex_t mutex;      /* protects head + slot array */
    pthread_cond_t  cond;       /* broadcast by publisher after each write */
    uint32_t  queue_depth;  /* capacity (number of slots) */
    uint32_t  _pad;         /* alignment */
    uint64_t  head;         /* monotonic count of messages ever written */
} ShmHeader;

/* One ring slot: a sequence stamp + the payload message.
 * seq == (write_index + 1) of the message currently stored here; 0 = empty.
 * Storing seq lets subscribers detect how far the publisher has advanced. */
typedef struct {
    uint64_t seq;
    Message  msg;
} ShmSlot;

#define SHM_NAME_MAX 128

/* Fallback bound used by the subscriber receive thread when waiting on the
 * process-shared condvar. Under normal operation the publisher's broadcast
 * wakes the waiter immediately (sub-millisecond); this timeout only protects
 * against a missed wakeup and lets the thread recheck recv_running so
 * ipc_channel_stop() doesn't hang. */
#define IPC_RECV_WAIT_MS 50

/* Single source of truth for which clock the process-shared condvar uses.
 * Must be applied consistently at both pthread_condattr_setclock() (publisher
 * init) and clock_gettime() (every waiter's deadline computation) — a
 * mismatch between the two would make pthread_cond_timedwait() misbehave. */
#if defined(CLOCK_MONOTONIC) && !defined(__APPLE__)
#define IPC_COND_CLOCK CLOCK_MONOTONIC
#else
#define IPC_COND_CLOCK CLOCK_REALTIME
#endif

/* ── Subscriber callbacks ─────────────────────────────── */

#define IPC_MAX_CALLBACKS 8

typedef struct {
    MessageCallback cb;
    void*           user_data;
} CbEntry;

/* ── IpcChannel (opaque) ─────────────────────────────── */

struct IpcChannel {
    IpcRole   role;
    char      shm_name[SHM_NAME_MAX];

    int       shm_fd;
    void*     shm_ptr;        /* mmap pointer */
    size_t    shm_size;
    uint32_t  queue_depth;

    /* Subscriber-side independent read cursor (broadcast semantics).
     * cursor_init defers initialization until the first receive so the
     * subscriber starts from the publisher's current window. */
    uint64_t  read_cursor;
    bool      cursor_init;

    CbEntry   callbacks[IPC_MAX_CALLBACKS];
    int       cb_count;

    pthread_t recv_thread;
    volatile bool recv_running;   /* read by recv thread + writer; volatile for visibility */
};

/* ── Helpers ─────────────────────────────────────────── */

static ShmHeader* get_header(IpcChannel* ch) {
    return (ShmHeader*)ch->shm_ptr;
}

static ShmSlot* get_slot_array(IpcChannel* ch) {
    return (ShmSlot*)((uint8_t*)ch->shm_ptr + sizeof(ShmHeader));
}

static size_t shm_total_size(uint32_t depth) {
    return sizeof(ShmHeader) + (size_t)depth * sizeof(ShmSlot);
}

/* ── Open ─────────────────────────────────────────────── */

IpcChannel* ipc_channel_open(const char* channel_name, IpcRole role,
                              uint32_t queue_depth) {
    if (!channel_name || queue_depth == 0) return NULL;

    IpcChannel* ch = (IpcChannel*)calloc(1, sizeof(IpcChannel));
    if (!ch) return NULL;

    ch->role        = role;
    ch->queue_depth = queue_depth;

    /* Build resource names */
    snprintf(ch->shm_name, sizeof(ch->shm_name), "/%s_shm", channel_name);

    ch->shm_size = shm_total_size(queue_depth);

    if (role == IPC_ROLE_PUBLISHER) {
        /* Create shared memory */
        shm_unlink(ch->shm_name); /* clean up stale */
        ch->shm_fd = shm_open(ch->shm_name, O_CREAT | O_RDWR, 0600);
        if (ch->shm_fd < 0) { free(ch); return NULL; }

        if (ftruncate(ch->shm_fd, (off_t)ch->shm_size) != 0) {
            close(ch->shm_fd);
            shm_unlink(ch->shm_name);
            free(ch);
            return NULL;
        }

        ch->shm_ptr = mmap(NULL, ch->shm_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, ch->shm_fd, 0);
        if (ch->shm_ptr == MAP_FAILED) {
            close(ch->shm_fd);
            shm_unlink(ch->shm_name);
            free(ch);
            return NULL;
        }

        /* Initialize header + slots (zeroed by ftruncate, but be explicit) */
        ShmHeader* hdr = get_header(ch);
        hdr->queue_depth = queue_depth;
        hdr->_pad  = 0;
        hdr->head  = 0;
        memset(get_slot_array(ch), 0, (size_t)queue_depth * sizeof(ShmSlot));

        /* Initialize process-shared mutex + condvar in-place inside the shm
         * region so every process that maps this region can lock/wait/
         * broadcast on the same underlying kernel futex — this is what lets
         * the publisher wake subscriber processes immediately instead of
         * them polling on a fixed interval. */
        pthread_mutexattr_t mattr;
        pthread_mutexattr_init(&mattr);
        pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&hdr->mutex, &mattr);
        pthread_mutexattr_destroy(&mattr);

        pthread_condattr_t cattr;
        pthread_condattr_init(&cattr);
        pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_SHARED);
        pthread_condattr_setclock(&cattr, IPC_COND_CLOCK);
        pthread_cond_init(&hdr->cond, &cattr);
        pthread_condattr_destroy(&cattr);
    } else {
        /* Subscriber: open existing shm (mutex/cond already initialized by
         * the publisher inside the shared region). */
        ch->shm_fd = shm_open(ch->shm_name, O_RDWR, 0600);
        if (ch->shm_fd < 0) { free(ch); return NULL; }

        ch->shm_ptr = mmap(NULL, ch->shm_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, ch->shm_fd, 0);
        if (ch->shm_ptr == MAP_FAILED) {
            close(ch->shm_fd);
            free(ch);
            return NULL;
        }
    }

    return ch;
}

/* ── Close ────────────────────────────────────────────── */

void ipc_channel_close(IpcChannel* ch) {
    if (!ch) return;

    ipc_channel_stop(ch);

    if (ch->shm_ptr && ch->shm_ptr != MAP_FAILED)
        munmap(ch->shm_ptr, ch->shm_size);
    if (ch->shm_fd >= 0)
        close(ch->shm_fd);

    if (ch->role == IPC_ROLE_PUBLISHER) {
        /* Do NOT unlink the shared memory here, and do NOT destroy the
         * process-shared mutex/cond: subscribers may still have the region
         * mapped and may still be waiting on the condvar. POSIX guarantees
         * an existing mmap remains valid after shm_unlink (the name is
         * removed but the backing object lives until all processes close
         * their last mmap/fd reference). Cleanup of stale names is already
         * performed at the start of ipc_channel_open() (IPC_ROLE_PUBLISHER
         * branch) so there is no permanent name leak. */
    }

    free(ch);
}

/* ── Publish ──────────────────────────────────────────── */

int ipc_channel_publish(IpcChannel* ch, const char* topic, const char* sender,
                        const void* data, uint32_t size) {
    if (!ch || ch->role != IPC_ROLE_PUBLISHER) return ERR_IO;
    if (!topic || size > MSG_BUS_MAX_DATA_SIZE) return ERR_IO;

    /* Broadcast ring: never block. Overwrite the oldest slot (drop_oldest for
     * any subscriber that has fallen behind). This avoids publisher stalls that
     * previously occurred when a slow/absent subscriber left the ring full. */
    ShmHeader* hdr = get_header(ch);
    pthread_mutex_lock(&hdr->mutex);

    ShmSlot*   arr = get_slot_array(ch);
    uint64_t   idx = hdr->head;
    ShmSlot*   slot = &arr[idx % hdr->queue_depth];

    memset(&slot->msg, 0, sizeof(slot->msg));
    snprintf(slot->msg.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (sender) snprintf(slot->msg.sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    slot->msg.type      = MSG_TYPE_PUBLISH;
    slot->msg.data_size = size;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    slot->msg.timestamp_us = (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;

    if (data && size > 0) memcpy(slot->msg.data, data, size);

    slot->seq = idx + 1;      /* stamp before advancing head */
    hdr->head = idx + 1;

    /* Wake every subscriber process/thread waiting on this channel — this is
     * the event-driven replacement for the old fixed-interval polling loop. */
    pthread_cond_broadcast(&hdr->cond);
    pthread_mutex_unlock(&hdr->mutex);
    return 0;
}

/* ── Subscribe ────────────────────────────────────────── */

int ipc_channel_subscribe(IpcChannel* ch, MessageCallback callback, void* user_data) {
    if (!ch || !callback) return ERR_IO;
    if (ch->cb_count >= IPC_MAX_CALLBACKS) return ERR_IO;
    ch->callbacks[ch->cb_count].cb        = callback;
    ch->callbacks[ch->cb_count].user_data = user_data;
    ch->cb_count++;
    return 0;
}

/* ── Receive one message ──────────────────────────────── */

/* Try to read a single message at the subscriber's cursor (non-blocking).
 * Returns 0 and fills *out if a message was available, ERR_IO if caught up. */
static int try_read_one(IpcChannel* ch, Message* out) {
    ShmHeader* hdr = get_header(ch);
    pthread_mutex_lock(&hdr->mutex);
    ShmSlot*   arr = get_slot_array(ch);
    uint64_t   head  = hdr->head;
    uint32_t   depth = hdr->queue_depth;

    /* Lazily anchor the cursor to the publisher's current window so a late
     * subscriber starts from recent history (up to `depth` messages) instead
     * of replaying the entire ring. */
    if (!ch->cursor_init) {
        ch->read_cursor = (head > depth) ? head - depth : 0;
        ch->cursor_init = true;
    }

    /* Fell behind further than the ring can hold: skip to the oldest valid
     * message still present (drop the overwritten ones). */
    if (head - ch->read_cursor > depth) {
        ch->read_cursor = head - depth;
    }

    if (ch->read_cursor >= head) {
        pthread_mutex_unlock(&hdr->mutex);
        return ERR_IO; /* caught up, nothing new */
    }

    ShmSlot* slot = &arr[ch->read_cursor % depth];
    *out = slot->msg;           /* copy payload under the mutex (no torn read) */
    ch->read_cursor++;
    pthread_mutex_unlock(&hdr->mutex);
    return 0;
}

/* Compute an absolute deadline `IPC_RECV_WAIT_MS` in the future, using the
 * same clock the process-shared condvar was configured with. Shared by every
 * caller of pthread_cond_timedwait() on this channel to avoid duplicating
 * (and risking divergence in) the overflow-carry arithmetic. */
static void compute_wait_deadline(struct timespec* ts) {
    clock_gettime(IPC_COND_CLOCK, ts);
    ts->tv_nsec += (long)IPC_RECV_WAIT_MS * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += ts->tv_nsec / 1000000000L;
        ts->tv_nsec %= 1000000000L;
    }
}

/* Block (bounded by IPC_RECV_WAIT_MS) until the publisher signals new data,
 * or until it becomes available anyway. Returns immediately if data is
 * already there. This replaces the old fixed-interval usleep() spin. */
static void wait_for_data(IpcChannel* ch) {
    ShmHeader* hdr = get_header(ch);
    pthread_mutex_lock(&hdr->mutex);

    /* Apply the same sliding-window clamp as try_read_one() before deciding
     * whether data is already available: a read_cursor that fell behind by
     * more than queue_depth is stale and must be pulled back into the valid
     * window, otherwise this fast path could wrongly conclude "nothing new"
     * (or wait on a cursor that no longer points at a real message). */
    if (ch->cursor_init) {
        uint64_t head  = hdr->head;
        uint32_t depth = hdr->queue_depth;
        if (head - ch->read_cursor > depth) {
            ch->read_cursor = head - depth;
        }
        if (ch->read_cursor < head) {
            pthread_mutex_unlock(&hdr->mutex);
            return; /* already have data, no need to wait */
        }
    }

    struct timespec ts;
    compute_wait_deadline(&ts);
    int rc = pthread_cond_timedwait(&hdr->cond, &hdr->mutex, &ts);
    if (rc != 0 && rc != ETIMEDOUT) {
        fprintf(stderr, "[ipc_channel] pthread_cond_timedwait unexpected error: %d (%s)\n",
                rc, strerror(rc));
    }
    pthread_mutex_unlock(&hdr->mutex);
}

int ipc_channel_recv_once(IpcChannel* ch, uint32_t timeout_ms) {
    if (!ch || ch->role != IPC_ROLE_SUBSCRIBER) return ERR_IO;

    Message msg;
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    uint64_t deadline_us = (uint64_t)deadline.tv_sec * 1000000ULL +
                           (uint64_t)deadline.tv_nsec / 1000ULL +
                           (uint64_t)timeout_ms * 1000ULL;

    for (;;) {
        if (try_read_one(ch, &msg) == 0) {
            for (int i = 0; i < ch->cb_count; i++) {
                ch->callbacks[i].cb(&msg, ch->callbacks[i].user_data);
            }
            return 0;
        }
        if (timeout_ms == 0) {
            /* Block until a message arrives (event-driven wait). */
            wait_for_data(ch);
            continue;
        }
        struct timespec nowts;
        clock_gettime(CLOCK_MONOTONIC, &nowts);
        uint64_t now_us = (uint64_t)nowts.tv_sec * 1000000ULL +
                          (uint64_t)nowts.tv_nsec / 1000ULL;
        if (now_us >= deadline_us) return ERR_IO;
        wait_for_data(ch);
    }
}

/* ── Background receive thread ────────────────────────── */

static void* recv_thread_fn(void* arg) {
    IpcChannel* ch = (IpcChannel*)arg;
    Message msg;
    while (ch->recv_running) {
        /* Drain all messages currently available (batched delivery), then
         * sleep on the condvar until the publisher signals more (or the
         * bounded fallback timeout elapses so recv_running is rechecked). */
        int drained = 0;
        while (ch->recv_running && try_read_one(ch, &msg) == 0) {
            for (int i = 0; i < ch->cb_count; i++) {
                ch->callbacks[i].cb(&msg, ch->callbacks[i].user_data);
            }
            drained = 1;
        }
        if (!drained && ch->recv_running) wait_for_data(ch);
    }
    return NULL;
}

int ipc_channel_start(IpcChannel* ch) {
    if (!ch || ch->recv_running) return 0;
    ch->recv_running = true;
    int ret = pthread_create(&ch->recv_thread, NULL, recv_thread_fn, ch);
    if (ret != 0) { ch->recv_running = false; return ERR_IO; }
    return 0;
}

void ipc_channel_stop(IpcChannel* ch) {
    if (!ch || !ch->recv_running) return;
    ch->recv_running = false;
    pthread_join(ch->recv_thread, NULL);
}
