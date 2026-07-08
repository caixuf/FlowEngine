/**
 * ipc_channel.c — 跨进程共享内存通道实现
 *
 * 使用 POSIX shm_open + mmap + 命名信号量实现进程间通信。
 * 环形缓冲区存储在共享内存中。
 *
 * 共享内存布局：
 *   [ShmHeader][Message * queue_depth]
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
    uint32_t  queue_depth;  /* capacity */
    uint32_t  head;         /* write index (publisher advances) */
    uint32_t  tail;         /* read  index (subscriber advances) */
    uint32_t  count;        /* current fill level */
    /* Messages follow immediately after this header */
} ShmHeader;

#define SHM_NAME_MAX 128
#define SEM_ITEMS_SUFFIX "_items"
#define SEM_SPACE_SUFFIX "_space"
#define SEM_MUTEX_SUFFIX "_mutex"

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
    char      sem_items_name[SHM_NAME_MAX];
    char      sem_space_name[SHM_NAME_MAX];
    char      sem_mutex_name[SHM_NAME_MAX];

    int       shm_fd;
    void*     shm_ptr;        /* mmap pointer */
    size_t    shm_size;
    uint32_t  queue_depth;

    sem_t*    sem_items;      /* counts available messages */
    sem_t*    sem_space;      /* counts free slots */
    sem_t*    sem_mutex;      /* mutual exclusion for header */

    CbEntry   callbacks[IPC_MAX_CALLBACKS];
    int       cb_count;

    pthread_t recv_thread;
    bool      recv_running;
};

/* ── Helpers ─────────────────────────────────────────── */

static ShmHeader* get_header(IpcChannel* ch) {
    return (ShmHeader*)ch->shm_ptr;
}

static Message* get_msg_array(IpcChannel* ch) {
    return (Message*)((uint8_t*)ch->shm_ptr + sizeof(ShmHeader));
}

static size_t shm_total_size(uint32_t depth) {
    return sizeof(ShmHeader) + (size_t)depth * sizeof(Message);
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
    snprintf(ch->sem_items_name,  sizeof(ch->sem_items_name),  "/%s%s", channel_name, SEM_ITEMS_SUFFIX);
    snprintf(ch->sem_space_name,  sizeof(ch->sem_space_name),  "/%s%s", channel_name, SEM_SPACE_SUFFIX);
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

        /* Initialize header */
        ShmHeader* hdr = get_header(ch);
        hdr->queue_depth = queue_depth;
        hdr->head  = 0;
        hdr->tail  = 0;
        hdr->count = 0;

        /* Create semaphores */
        sem_unlink(ch->sem_items_name);
        sem_unlink(ch->sem_space_name);
        sem_unlink(ch->sem_mutex_name);

        ch->sem_items = sem_open(ch->sem_items_name, O_CREAT | O_EXCL, 0600, 0);
        ch->sem_space = sem_open(ch->sem_space_name, O_CREAT | O_EXCL, 0600, queue_depth);
        ch->sem_mutex = sem_open(ch->sem_mutex_name, O_CREAT | O_EXCL, 0600, 1);

        if (ch->sem_items == SEM_FAILED || ch->sem_space == SEM_FAILED ||
            ch->sem_mutex == SEM_FAILED) {
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

        ch->sem_items = sem_open(ch->sem_items_name, 0);
        ch->sem_space = sem_open(ch->sem_space_name, 0);
        ch->sem_mutex = sem_open(ch->sem_mutex_name, 0);

        if (ch->sem_items == SEM_FAILED || ch->sem_space == SEM_FAILED ||
            ch->sem_mutex == SEM_FAILED) {
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

    if (ch->sem_items && ch->sem_items != SEM_FAILED)
        sem_close(ch->sem_items);
    if (ch->sem_space && ch->sem_space != SEM_FAILED)
        sem_close(ch->sem_space);
    if (ch->sem_mutex && ch->sem_mutex != SEM_FAILED)
        sem_close(ch->sem_mutex);

    if (ch->role == IPC_ROLE_PUBLISHER) {
        /* Do NOT unlink the shared memory or semaphore names here.
         * Subscribers may still have the region mapped; POSIX guarantees
         * an existing mmap remains valid after shm_unlink (the name is
         * removed but the backing object lives until all mappings close).
         * However, unlink + sem_unlink at close races with live subscribers
         * that are blocked in sem_timedwait on the same handles, and the
         * next publisher open's "clean stale" step already calls shm_unlink
         * before creating fresh resources, so there is no name leakage. */
    }

    free(ch);
}

/* ── Publish ──────────────────────────────────────────── */

int ipc_channel_publish(IpcChannel* ch, const char* topic, const char* sender,
                        const void* data, uint32_t size) {
    if (!ch || ch->role != IPC_ROLE_PUBLISHER) return ERR_IO;
    if (!topic || size > MSG_BUS_MAX_DATA_SIZE) return ERR_IO;

    /* Try to acquire a free slot (non-blocking) */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 10000000LL; /* 10ms */
    if (ts.tv_nsec >= 1000000000LL) { ts.tv_sec++; ts.tv_nsec -= 1000000000LL; }

    if (sem_timedwait(ch->sem_space, &ts) != 0) return ERR_IO; /* full */

    sem_wait(ch->sem_mutex);

    ShmHeader* hdr = get_header(ch);
    Message*   arr = get_msg_array(ch);
    Message*   slot = &arr[hdr->head];

    memset(slot, 0, sizeof(*slot));
    snprintf(slot->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (sender) snprintf(slot->sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    slot->type      = MSG_TYPE_PUBLISH;
    slot->data_size = size;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    slot->timestamp_us = (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;

    if (data && size > 0) memcpy(slot->data, data, size);

    hdr->head = (hdr->head + 1) % hdr->queue_depth;
    hdr->count++;

    sem_post(ch->sem_mutex);
    sem_post(ch->sem_items);
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

int ipc_channel_recv_once(IpcChannel* ch, uint32_t timeout_ms) {
    if (!ch || ch->role != IPC_ROLE_SUBSCRIBER) return ERR_IO;

    int ret;
    if (timeout_ms == 0) {
        ret = sem_wait(ch->sem_items);
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000LL;
        if (ts.tv_nsec >= 1000000000LL) { ts.tv_sec++; ts.tv_nsec -= 1000000000LL; }
        ret = sem_timedwait(ch->sem_items, &ts);
    }
    if (ret != 0) return ERR_IO;

    sem_wait(ch->sem_mutex);
    ShmHeader* hdr = get_header(ch);
    Message*   arr = get_msg_array(ch);
    Message    msg = arr[hdr->tail];
    hdr->tail  = (hdr->tail + 1) % hdr->queue_depth;
    hdr->count--;
    sem_post(ch->sem_mutex);
    sem_post(ch->sem_space);

    /* Invoke callbacks */
    for (int i = 0; i < ch->cb_count; i++) {
        ch->callbacks[i].cb(&msg, ch->callbacks[i].user_data);
    }
    return 0;
}

/* ── Background receive thread ────────────────────────── */

static void* recv_thread_fn(void* arg) {
    IpcChannel* ch = (IpcChannel*)arg;
    while (ch->recv_running) {
        ipc_channel_recv_once(ch, 200); /* 200ms timeout to check recv_running */
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
