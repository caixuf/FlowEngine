#ifndef FLOWENGINE_COMPAT_WIN_SYS_QUEUE_H
#define FLOWENGINE_COMPAT_WIN_SYS_QUEUE_H

#define TAILQ_HEAD(name, type) \
struct name {                  \
    struct type* tqh_first;    \
    struct type** tqh_last;    \
}

#define TAILQ_ENTRY(type)      \
struct {                       \
    struct type* tqe_next;     \
    struct type** tqe_prev;    \
}

#define TAILQ_INIT(head) do {              \
    (head)->tqh_first = 0;                 \
    (head)->tqh_last = &(head)->tqh_first; \
} while (0)

#define TAILQ_FIRST(head) ((head)->tqh_first)

#define TAILQ_FOREACH(var, head, field) \
    for ((var) = TAILQ_FIRST(head); (var); (var) = (var)->field.tqe_next)

#define TAILQ_INSERT_TAIL(head, elm, field) do { \
    (elm)->field.tqe_next = 0;                   \
    (elm)->field.tqe_prev = (head)->tqh_last;    \
    *(head)->tqh_last = (elm);                   \
    (head)->tqh_last = &(elm)->field.tqe_next;   \
} while (0)

#define TAILQ_REMOVE(head, elm, field) do {                    \
    if (((elm)->field.tqe_next) != 0)                           \
        (elm)->field.tqe_next->field.tqe_prev = (elm)->field.tqe_prev; \
    else                                                        \
        (head)->tqh_last = (elm)->field.tqe_prev;               \
    *(elm)->field.tqe_prev = (elm)->field.tqe_next;             \
} while (0)

#endif /* FLOWENGINE_COMPAT_WIN_SYS_QUEUE_H */
