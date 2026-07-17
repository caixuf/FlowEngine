/**
 * test_json_schema.c — JSON Schema 严格校验单元测试
 *
 * 覆盖：
 *  - json_get_*_strict 正向用例
 *  - 字段缺失返回 false 而非默认 0
 *  - 类型不匹配返回 false
 *  - json_validate 批量校验
 *  - type_name 字符串正确
 *  - 边界：空字符串、嵌套、key 超长
 */

#include "json_schema.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

static int g_passed = 0;
static int g_failed = 0;

#define TEST(cond, name) do { \
    if (cond) { g_passed++; printf("  PASS: %s\n", name); } \
    else      { g_failed++; printf("  FAIL: %s (line %d)\n", name, __LINE__); } \
} while (0)

static void test_strict_double(void) {
    const char* json = "{\"x\":12.5,\"y\":-3.0,\"z\":1e3}";
    double v;
    TEST(json_get_double_strict(json, "x", &v) && v == 12.5, "double positive");
    TEST(json_get_double_strict(json, "y", &v) && v == -3.0, "double negative");
    TEST(json_get_double_strict(json, "z", &v) && v == 1000.0, "double exponent");
    TEST(!json_get_double_strict(json, "missing", &v), "double missing → false");
    TEST(!json_get_double_strict(json, "x", NULL), "double NULL out → false");
}

static void test_strict_int(void) {
    const char* json = "{\"n\":42,\"p\":-7,\"f\":3.14,\"s\":\"5\"}";
    int v = 0;
    TEST(json_get_int_strict(json, "n", &v) && v == 42, "int positive");
    TEST(json_get_int_strict(json, "p", &v) && v == -7, "int negative");
    TEST(!json_get_int_strict(json, "f", &v), "int rejects float (3.14)");
    TEST(!json_get_int_strict(json, "s", &v), "int rejects string \"5\"");
    TEST(!json_get_int_strict(json, "missing", &v), "int missing → false");
}

static void test_strict_string(void) {
    const char* json = "{\"name\":\"hello\",\"empty\":\"\",\"num\":42}";
    char buf[64];
    TEST(json_get_string_strict(json, "name", buf, sizeof(buf)) && strcmp(buf, "hello") == 0,
         "string normal");
    TEST(json_get_string_strict(json, "empty", buf, sizeof(buf)) && buf[0] == '\0',
         "string empty quotes");
    TEST(!json_get_string_strict(json, "num", buf, sizeof(buf)), "string rejects int");
}

static void test_type_probe(void) {
    const char* json = "{\"d\":1.0,\"i\":1,\"s\":\"x\",\"b\":true,\"n\":null,\"bad\":[1]}";
    TEST(json_field_ok(json, "d", JSON_TYPE_DOUBLE), "probe double ok");
    TEST(json_field_ok(json, "i", JSON_TYPE_INT),    "probe int ok");
    TEST(json_field_ok(json, "s", JSON_TYPE_STRING), "probe string ok");
    TEST(json_field_ok(json, "b", JSON_TYPE_BOOL),   "probe bool ok");
    TEST(!json_field_ok(json, "n", JSON_TYPE_INT),   "probe null rejected as int");
    TEST(!json_field_ok(json, "bad", JSON_TYPE_INT), "probe array rejected as int");
}

static void test_validate_schema(void) {
    const JsonFieldDef schema[] = {
        {"x", JSON_TYPE_DOUBLE, true},
        {"y", JSON_TYPE_DOUBLE, true},
        {"name", JSON_TYPE_STRING, false},  /* optional */
        {NULL, 0, 0}
    };
    char err[64];

    const char* good = "{\"x\":1.5,\"y\":2.5,\"name\":\"a\"}";
    TEST(json_validate(good, "test", schema, err, sizeof(err)),
         "validate all required present");

    const char* no_name = "{\"x\":1.5,\"y\":2.5}";
    TEST(json_validate(no_name, "test", schema, err, sizeof(err)),
         "validate optional missing ok");

    const char* no_x = "{\"y\":2.5}";
    TEST(!json_validate(no_x, "test", schema, err, sizeof(err)),
         "validate required missing → false");
    TEST(strcmp(err, "x") == 0, "validate err_field = 'x'");

    const char* wrong_type = "{\"x\":\"not-a-number\",\"y\":2.5}";
    TEST(!json_validate(wrong_type, "test", schema, err, sizeof(err)),
         "validate wrong type → false");
}

static void test_edge_cases(void) {
    double v;
    TEST(!json_get_double_strict("", "x", &v), "empty json → false");
    TEST(!json_get_double_strict(NULL, "x", &v), "null json → false");
    /* Whitespace tolerance */
    const char* padded = "{ \"x\" : 7.5 }";
    TEST(json_get_double_strict(padded, "x", &v) && v == 7.5, "whitespace tolerant");
    /* Multiple matches — first wins (existing strstr semantics) */
    const char* dup = "{\"a\":{\"x\":1.0},\"x\":99.0}";
    TEST(json_get_double_strict(dup, "x", &v) && v == 1.0,
         "first-occurrence semantics (documented behavior)");
}

/* ════════════════════════════════════════════════════════════════
 *  DSL 提取测试 (key=value 文本格式)
 * ════════════════════════════════════════════════════════════════ */

static void test_dsl_double(void) {
    /* DSL format sample (historical — fusion_plugin.c, now removed, used this format) */
    const char* dsl = "pos=(1.5,2.5) gps=(39.9,116.4) speed=33.0 dt=50us";
    double v = 0;
    TEST(dsl_get_double_strict(dsl, "speed", &v) && v == 33.0,
         "dsl double: speed=33.0");
    /* dsl_get_double_strict doesn't parse tuples; it just extracts first number after key */
    /* Optional whitespace: "speed = 5.0" */
    const char* padded = "speed = 5.0 brake=0.0";
    TEST(dsl_get_double_strict(padded, "speed", &v) && v == 5.0,
         "dsl double: spaces around =");
    /* Colon separator also works */
    const char* colon = "speed:7.5";
    TEST(dsl_get_double_strict(colon, "speed", &v) && v == 7.5,
         "dsl double: colon separator");
    /* Negative */
    TEST(dsl_get_double_strict("steer=-0.05", "steer", &v) && v < -0.04 && v > -0.06,
         "dsl double: negative");
    /* Missing */
    TEST(!dsl_get_double_strict("foo=1.0", "bar", &v),
         "dsl double: missing key");
    /* Wrong type */
    TEST(!dsl_get_double_strict("speed=true", "speed", &v),
         "dsl double: rejects bool");
    /* NULL out */
    TEST(!dsl_get_double_strict("speed=1.0", "speed", NULL),
         "dsl double: NULL out");
    /* Boundary: 'mspeed' must not match 'speed' */
    TEST(!dsl_get_double_strict("mspeed=5.0", "speed", &v),
         "dsl double: word boundary");
    /* Boundary: 'speedo' must not match 'speed' */
    TEST(!dsl_get_double_strict("speedo=5.0", "speed", &v),
         "dsl double: right boundary");
    /* Empty / null */
    TEST(!dsl_get_double_strict("", "x", &v),
         "dsl double: empty");
    TEST(!dsl_get_double_strict(NULL, "x", &v),
         "dsl double: null");
}

static void test_dsl_int(void) {
    int n = 0;
    TEST(dsl_get_int_strict("n=42", "n", &n) && n == 42, "dsl int: positive");
    TEST(dsl_get_int_strict("n=-7", "n", &n) && n == -7, "dsl int: negative");
    TEST(!dsl_get_int_strict("n=3.14", "n", &n), "dsl int: rejects float");
    TEST(!dsl_get_int_strict("n=abc", "n", &n), "dsl int: rejects non-number");
    TEST(!dsl_get_int_strict("foo=42", "n", &n), "dsl int: missing");
}

static void test_dsl_validate(void) {
    const DslFieldDef schema[] = {
        {"speed", JSON_TYPE_DOUBLE, true},
        {"brake", JSON_TYPE_DOUBLE, false},  /* optional */
        {NULL, 0, 0}
    };
    char err[64];
    const char* good = "throttle=0.50 brake=0.00 steer=0.00 speed=5.0 target=10.0 mode=⏺ HOLD";
    TEST(dsl_validate(good, "control/cmd", schema, err, sizeof(err)),
         "dsl validate: speed present");
    const char* no_speed = "throttle=0.50 brake=0.00";
    TEST(!dsl_validate(no_speed, "control/cmd", schema, err, sizeof(err)),
         "dsl validate: speed missing");
    TEST(strcmp(err, "speed") == 0, "dsl validate: err_field='speed'");
    const char* wrong = "speed=fast";
    TEST(!dsl_validate(wrong, "control/cmd", schema, err, sizeof(err)),
         "dsl validate: wrong type");
}

int main(void) {
    printf("Running json_schema tests...\n");
    test_strict_double();
    test_strict_int();
    test_strict_string();
    test_type_probe();
    test_validate_schema();
    test_edge_cases();
    test_dsl_double();
    test_dsl_int();
    test_dsl_validate();
    printf("Results: %d passed, %d failed\n", g_passed, g_failed);
    return g_failed > 0 ? 1 : 0;
}
