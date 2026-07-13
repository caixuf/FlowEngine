/**
 * ipc_channel.c — 跨进程共享内存通道实现（广播 / 多订阅者扇出）
 *
 * 使用 POSIX shm_open + mmap + 命名信号量实现进程间通信。
 * 环形缓冲区存储在共享内存中。
 *
 * 共享内存布局：
 *   [ShmHeader][ShmSlot * queue_depth]
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
 */

#include "ipc_channel.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

/* ── Shared memory header (lives inside the shm region) ─ */

typedef struct {
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
#define SEM_MUTEX_SUFFIX "_mutex"

/* Poll interval used by the subscriber receive thread when it is caught up
 * with the publisher. Small enough for smooth 50Hz+ streams, cheap enough to
 * run one thread per subscribed channel. */
#define IPC_RECV_POLL_US 1000

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
    char      sem_mutex_name[SHM_NAME_MAX];

    int       shm_fd;
    void*     shm_ptr;        /* mmap pointer */
    size_t    shm_size;
    uint32_t  queue_depth;

    sem_t*    sem_mutex;      /* mutual exclusion for header + slots */

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
    snprintf(ch->shm_name,        sizeof(ch->shm_name),        "/%s_shm", channel_name);
    snprintf(ch->sem_mutex_name,  sizeof(ch->sem_mutex_name),  "/%s%s", channel_name, SEM_MUTEX_SUFFIX);

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

        /* Create mutex semaphore */
        sem_unlink(ch->sem_mutex_name);
        ch->sem_mutex = sem_open(ch->sem_mutex_name, O_CREAT | O_EXCL, 0600, 1);

        if (ch->sem_mutex == SEM_FAILED) {
            ipc_channel_close(ch);
            return NULL;
        }
    } else {
        /* Subscriber: open existing shm and sems */
        ch->shm_fd = shm_open(ch->shm_name, O_RDWR, 0600);
        if (ch->shm_fd < 0) { free(ch); return NULL; }

        ch->shm_ptr = mmap(NULL, ch->shm_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, ch->shm_fd, 0);
        if (ch->shm_ptr == MAP_FAILED) {
            close(ch->shm_fd);
            free(ch);
            return NULL;
        }

        ch->sem_mutex = sem_open(ch->sem_mutex_name, 0);

        if (ch->sem_mutex == SEM_FAILED) {
            ipc_channel_close(ch);
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

    if (ch->sem_mutex && ch->sem_mutex != SEM_FAILED)
        sem_close(ch->sem_mutex);

    if (ch->role == IPC_ROLE_PUBLISHER) {
        /* Do NOT unlink the shared memory or semaphore names here.
         * Subscribers may still have the region mapped; POSIX guarantees an
         * existing mmap remains valid after shm_unlink (the name is removed but
         * the backing object lives until all processes close their last mmap/fd
         * reference). Cleanup of stale names is already performed at the start of
         * ipc_channel_open() (IPC_ROLE_PUBLISHER branch) so there is no permanent
         * name leak. */
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
    sem_wait(ch->sem_mutex);

    ShmHeader* hdr = get_header(ch);
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

    sem_post(ch->sem_mutex);
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
    sem_wait(ch->sem_mutex);
    ShmHeader* hdr = get_header(ch);
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
        sem_post(ch->sem_mutex);
        return ERR_IO; /* caught up, nothing new */
    }

    ShmSlot* slot = &arr[ch->read_cursor % depth];
    *out = slot->msg;           /* copy payload under the mutex (no torn read) */
    ch->read_cursor++;
    sem_post(ch->sem_mutex);
    return 0;
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
            /* Block until a message arrives (poll). */
            usleep(IPC_RECV_POLL_US);
            continue;
        }
        struct timespec nowts;
        clock_gettime(CLOCK_MONOTONIC, &nowts);
        uint64_t now_us = (uint64_t)nowts.tv_sec * 1000000ULL +
                          (uint64_t)nowts.tv_nsec / 1000ULL;
        if (now_us >= deadline_us) return ERR_IO;
        usleep(IPC_RECV_POLL_US);
    }
}

/* ── Background receive thread ────────────────────────── */

static void* recv_thread_fn(void* arg) {
    IpcChannel* ch = (IpcChannel*)arg;
    Message msg;
    while (ch->recv_running) {
        /* Drain all messages currently available, then sleep briefly. */
        int drained = 0;
        while (ch->recv_running && try_read_one(ch, &msg) == 0) {
            for (int i = 0; i < ch->cb_count; i++) {
                ch->callbacks[i].cb(&msg, ch->callbacks[i].user_data);
            }
            drained = 1;
        }
        if (!drained) usleep(IPC_RECV_POLL_US);
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
