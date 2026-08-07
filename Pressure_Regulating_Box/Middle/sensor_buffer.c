/**
 * @file sensor_buffer.c
 * @brief RAM环形缓冲区实现
 *
 * 深休眠(WFI)后MCU从中断返回继续执行,SRAM内容完整保留。
 * 仅在上电冷启动时BSS被清零,此时magic=0≠MAGIC,触发初始化。
 */

#include "sensor_buffer.h"
#include <string.h>

// Magic值: BSS冷启动清零后 != 此值, 深休眠唤醒后保持
#define SENSOR_BUF_MAGIC   0xBEEF1234u

// 缓冲区管理结构
static struct {
    uint32_t magic;
    uint16_t write_idx;         // 下一个写入位置
    uint16_t read_idx;          // 最早有效记录位置 (首位位置)
    uint16_t count;             // 当前有效记录数 (≤ MAX_RECORDS)
    SensorRecord_t records[SENSOR_BUF_MAX_RECORDS];
} s_buf;

// ==================== 初始化 ====================
void sensor_buf_init(void)
{
    if(s_buf.magic != SENSOR_BUF_MAGIC)
    {
        // 冷启动: 清空
        memset(&s_buf, 0, sizeof(s_buf));
        s_buf.magic = SENSOR_BUF_MAGIC;
    }
    // 深休眠唤醒: magic有效, 数据保留不动
}

// ==================== 写入一条记录 ====================
void sensor_buf_push(const SensorRecord_t *record)
{
    if(!record) return;

    s_buf.records[s_buf.write_idx] = *record;

    s_buf.write_idx = (s_buf.write_idx + 1) % SENSOR_BUF_MAX_RECORDS;

    if(s_buf.count < SENSOR_BUF_MAX_RECORDS)
    {
        s_buf.count++;
    }
    else
    {
        // 满的时候被覆盖了最老的记录，所以读指针也必须前进1
        s_buf.read_idx = (s_buf.read_idx + 1) % SENSOR_BUF_MAX_RECORDS;
    }
}

// ==================== 读取第i条记录 ====================
const SensorRecord_t* sensor_buf_get(uint16_t index)
{
    if(index >= s_buf.count) return (const SensorRecord_t*)0;

    uint16_t actual = (s_buf.read_idx + index) % SENSOR_BUF_MAX_RECORDS;
    return &s_buf.records[actual];
}

// ==================== 弹出（删除）最老的一条记录 ====================
void sensor_buf_pop(void)
{
    if(s_buf.count == 0) return;

    s_buf.read_idx = (s_buf.read_idx + 1) % SENSOR_BUF_MAX_RECORDS;
    s_buf.count--;
}

// ==================== 当前记录数 ====================
uint16_t sensor_buf_count(void)
{
    return s_buf.count;
}

// ==================== 清空缓冲区 ====================
void sensor_buf_clear(void)
{
    s_buf.write_idx = 0;
    s_buf.read_idx = 0;
    s_buf.count = 0;
    // 不清除magic,保持有效状态
}
