#include "json_builder.h"
#include <stdio.h>
#include <string.h>

void jb_begin(JsonBuilder *jb, char *buf, uint16_t buf_size)
{
    jb->buf = buf;
    jb->size = buf_size;
    jb->pos = 0;
    if(buf_size > 0) {
        buf[0] = '[';
        jb->pos = 1;
    }
}

// 内部: 追加逗号分隔符 (非首项时)
static void jb_comma(JsonBuilder *jb)
{
    if(jb->pos > 1 && jb->pos < jb->size - 1) {
        jb->buf[jb->pos++] = ',';
    }
}

void jb_add_str(JsonBuilder *jb, const char *id, const char *value, const char *ts)
{
    int n;
    jb_comma(jb);
    n = snprintf(jb->buf + jb->pos, jb->size - jb->pos,
        "{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}", id, value, ts);
    if(n > 0 && (uint16_t)n < jb->size - jb->pos)
        jb->pos += (uint16_t)n;
}

void jb_add_int(JsonBuilder *jb, const char *id, int value, const char *ts)
{
    int n;
    jb_comma(jb);
    n = snprintf(jb->buf + jb->pos, jb->size - jb->pos,
        "{\"id\":\"%s\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}", id, value, ts);
    if(n > 0 && (uint16_t)n < jb->size - jb->pos)
        jb->pos += (uint16_t)n;
}

void jb_add_float(JsonBuilder *jb, const char *id, float value, const char *ts)
{
    int n;
    jb_comma(jb);
    n = snprintf(jb->buf + jb->pos, jb->size - jb->pos,
        "{\"id\":\"%s\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}", id, value, ts);
    if(n > 0 && (uint16_t)n < jb->size - jb->pos)
        jb->pos += (uint16_t)n;
}

void jb_add_float1(JsonBuilder *jb, const char *id, float value, const char *ts)
{
    int n;
    jb_comma(jb);
    n = snprintf(jb->buf + jb->pos, jb->size - jb->pos,
        "{\"id\":\"%s\",\"value\":\"%.1f\",\"ts\":\"%s\",\"remark\":\"\"}", id, value, ts);
    if(n > 0 && (uint16_t)n < jb->size - jb->pos)
        jb->pos += (uint16_t)n;
}

const char *jb_end(JsonBuilder *jb)
{
    if(jb->pos < jb->size - 1) {
        jb->buf[jb->pos++] = ']';
    }
    if(jb->pos < jb->size) {
        jb->buf[jb->pos] = '\0';
    } else {
        jb->buf[jb->size - 1] = '\0';
    }
    return jb->buf;
}
