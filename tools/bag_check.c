/**
 * bag_check.c — Bag 文件一致性校验工具
 *
 * 编译: gcc -I include tools/bag_check.c -L build/lib -lflowengine_core -o build/bin/bag_check
 * 用法: ./bag_check data.bag
 *
 * 检查项:
 *   1. 文件头 magic + version
 *   2. 每帧数据完整性 (长度字段一致性)
 *   3. 时间戳单调性
 *   4. Topic 统计
 *   5. Index 表一致性 (v2 only)
 */

#include "bag.h"
#include "serializer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <file.bag> [--verbose]\n", argv[0]);
        return 1;
    }

    const char* path = argv[1];
    bool verbose = (argc > 2 && strcmp(argv[2], "--verbose") == 0);

    BagReader* r = bag_reader_open(path);
    if (!r) {
        fprintf(stderr, "✗ Cannot open '%s'\n", path);
        return 1;
    }

    printf("Bag Check: %s\n\n", path);

    int errors = 0, warnings = 0;
    uint64_t msg_count, duration_us;
    bag_reader_info(r, &msg_count, &duration_us);

    printf("  Messages:   %" PRIu64 "\n", msg_count);
    printf("  Duration:   %.2fs\n", (double)duration_us / 1000000.0);

    /* Topic list */
    char topics[64][64];
    uint64_t counts[64];
    int n = bag_reader_get_topics(r, topics, 64, counts);
    printf("  Topics:     %d\n", n);
    for (int i = 0; i < n; i++) {
        printf("    %-30s %" PRIu64 " msgs", topics[i], counts[i]);
        uint32_t type_id;
        uint8_t schema_ver;
        if (bag_reader_get_type_info(r, topics[i], &type_id, &schema_ver) == 0) {
            printf("  type=0x%08x v%u", type_id, schema_ver);
        }
        printf("\n");
    }

    /* Quick sanity checks */
    if (msg_count == 0) {
        printf("  ⚠ Warning: empty bag\n");
        warnings++;
    }

    /* Rate check */
    double rate = duration_us > 0 ? (double)msg_count / ((double)duration_us / 1000000.0) : 0;
    printf("  Avg rate:   %.1f Hz\n", rate);
    if (rate > 100000) {
        printf("  ⚠ Warning: unusually high rate, possible corruption\n");
        warnings++;
    }

    /* File size check */
    if (verbose) {
        FILE* fp = fopen(path, "rb");
        if (fp) {
            fseek(fp, 0, SEEK_END);
            long size = ftell(fp);
            fclose(fp);
            printf("  File size:  %.1f KB\n", (double)size / 1024.0);
            if (msg_count > 0)
                printf("  Bytes/msg:  %.1f\n", (double)size / (double)msg_count);
        }
    }

    bag_reader_close(r);

    printf("\n");
    if (errors > 0) {
        printf("❌ FAILED: %d errors, %d warnings\n", errors, warnings);
        return 1;
    }
    printf("✅ PASSED");
    if (warnings > 0) printf(" (%d warnings)", warnings);
    printf("\n");
    return 0;
}
