#ifndef JSON_BUILDER_H
#define JSON_BUILDER_H

#include <stdint.h>

// ==================== FastBee JSON 构建器 ====================
// 用于构建 [{"id":"x","value":"y","ts":"z","remark":""},...]  格式的 JSON
// 所有操作基于一个外部 char buf[] (调用者负责分配)

typedef struct {
    char *buf;
    uint16_t size;
    uint16_t pos;
} JsonBuilder;

// 初始化构建器 (写入 '[')
void jb_begin(JsonBuilder *jb, char *buf, uint16_t buf_size);

// 添加字符串字段: {"id":"<id>","value":"<value>","ts":"<ts>","remark":""}
void jb_add_str(JsonBuilder *jb, const char *id, const char *value, const char *ts);

// 添加整数字段
void jb_add_int(JsonBuilder *jb, const char *id, int value, const char *ts);

// 添加浮点字段 (%.2f)
void jb_add_float(JsonBuilder *jb, const char *id, float value, const char *ts);

// 添加浮点字段 (%.1f)
void jb_add_float1(JsonBuilder *jb, const char *id, float value, const char *ts);

// 结束构建 (写入 ']'), 返回 JSON 字符串指针
const char *jb_end(JsonBuilder *jb);

#endif /* JSON_BUILDER_H */
