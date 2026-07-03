#include "msg_schema.h"
#include <string.h>
#include <stdio.h>
#include <pthread.h>

#define SCHEMA_MAX_ENTRIES 128

typedef struct {
    char   topic[64];
    size_t struct_size;
    char   type_name[64];
} SchemaEntry;

static SchemaEntry    g_schema_table[SCHEMA_MAX_ENTRIES];
static int            g_schema_count = 0;
static pthread_mutex_t g_schema_mutex = PTHREAD_MUTEX_INITIALIZER;

int msg_schema_register(const char* topic, size_t struct_size, const char* type_name) {
    if (!topic || !type_name || struct_size == 0) return -1;

    pthread_mutex_lock(&g_schema_mutex);

    /* Check for duplicate topic */
    for (int i = 0; i < g_schema_count; i++) {
        if (strcmp(g_schema_table[i].topic, topic) == 0) {
            /* Update existing entry */
            g_schema_table[i].struct_size = struct_size;
            strncpy(g_schema_table[i].type_name, type_name, sizeof(g_schema_table[i].type_name) - 1);
            pthread_mutex_unlock(&g_schema_mutex);
            return 0;
        }
    }

    if (g_schema_count >= SCHEMA_MAX_ENTRIES) {
        pthread_mutex_unlock(&g_schema_mutex);
        return -1;
    }

    SchemaEntry* e = &g_schema_table[g_schema_count++];
    strncpy(e->topic, topic, sizeof(e->topic) - 1);
    e->struct_size = struct_size;
    strncpy(e->type_name, type_name, sizeof(e->type_name) - 1);

    pthread_mutex_unlock(&g_schema_mutex);
    return 0;
}

int msg_schema_check(const char* topic, size_t actual_size, const char* call_site) {
    if (!topic) return 0;

    pthread_mutex_lock(&g_schema_mutex);
    for (int i = 0; i < g_schema_count; i++) {
        if (strcmp(g_schema_table[i].topic, topic) == 0) {
            size_t expected = g_schema_table[i].struct_size;
            if (actual_size != expected) {
                fprintf(stderr,
                    "[MSG_SCHEMA] WARNING: topic=\"%s\" size mismatch at %s: "
                    "expected %zu (type %s), got %zu\n",
                    topic, call_site ? call_site : "?",
                    expected, g_schema_table[i].type_name, actual_size);
                pthread_mutex_unlock(&g_schema_mutex);
                return -1;
            }
            pthread_mutex_unlock(&g_schema_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_schema_mutex);
    return 0; /* Not registered — no check */
}
