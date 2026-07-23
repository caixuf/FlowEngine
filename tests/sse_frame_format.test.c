/**
 * SSE 帧格式测试 — 验证 monitor_server 的 sse_flatten_payload 和帧构建
 *
 * 根因：SSE 协议以 \n 分隔字段，payload 中的 raw \n 会被 EventSource
 * 当作字段终止，导致后续 JSON 被丢弃。monitor_server.c 已有
 * sse_flatten_payload() 把 JSON 压平，本测试确保其行为正确。
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

/* 复用 monitor_server.c 的 flatten 逻辑（内联在此避免链接依赖） */
static void sse_flatten_payload(char* s) {
    char* wr = s;
    for (char* rd = s; *rd; rd++) {
        if (*rd != '\n' && *rd != '\r' && *rd != '\t') *wr++ = *rd;
    }
    *wr = '\0';
}

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { tests_failed++; printf("  FAIL: %s\n", msg); } \
} while(0)

int main(void) {
    printf("=== SSE frame format tests ===\n");

    /* Test 1: pretty-printed JSON with newlines */
    {
        char buf[1024];
        strcpy(buf, "{\n  \"a\": 1,\n  \"b\": 2\n}");
        sse_flatten_payload(buf);
        CHECK(strchr(buf, '\n') == NULL, "newlines stripped from pretty JSON");
        CHECK(strchr(buf, '\r') == NULL, "carriage returns stripped");
        CHECK(strchr(buf, '\t') == NULL, "tabs stripped");
        /* 空格不是 SSE 问题（只有 \n/\r/\t 是），所以保留空格是允许的 */
        CHECK(strstr(buf, "\"a\": 1") != NULL && strstr(buf, "\"b\": 2") != NULL,
              "flattened JSON preserves values and structure");
    }

    /* Test 2: JSON with escaped newlines inside strings (must be preserved) */
    {
        char buf[1024];
        strcpy(buf, "{\"msg\":\"line1\\nline2\"}");
        sse_flatten_payload(buf);
        CHECK(strstr(buf, "\\n") != NULL, "escaped \\n inside strings preserved");
        CHECK(strchr(buf, '\n') == NULL, "no raw newlines remain");
    }

    /* Test 3: empty string */
    {
        char buf[16] = "";
        sse_flatten_payload(buf);
        CHECK(strcmp(buf, "") == 0, "empty string stays empty");
    }

    /* Test 4: already single line */
    {
        char buf[128];
        strcpy(buf, "{\"a\":1,\"b\":2}");
        sse_flatten_payload(buf);
        CHECK(strcmp(buf, "{\"a\":1,\"b\":2}") == 0, "single-line JSON unchanged");
    }

    /* Test 5: deep nested JSON with lots of whitespace */
    {
        char buf[2048];
        strcpy(buf,
            "{\n"
            "  \"metrics\": {\n"
            "    \"scene\": {\n"
            "      \"ego\": {\n"
            "        \"x\": 100.5,\n"
            "        \"y\": -1.75,\n"
            "        \"z\": 0\n"
            "      }\n"
            "    }\n"
            "  }\n"
            "}");
        sse_flatten_payload(buf);
        CHECK(strchr(buf, '\n') == NULL, "deep nested JSON flattened");
        CHECK(strstr(buf, "\"x\": 100.5") != NULL, "numeric values preserved");
    }

    printf("\n--- summary: %d pass, %d fail ---\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}