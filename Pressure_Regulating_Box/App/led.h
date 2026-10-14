#ifndef LED_H
#define LED_H

#include <stdint.h>

/* ==================== 三路状态灯服务 (低电平点亮, 长亮) ====================
 * LED1 检修  (PA12): 阀门故障 或 阀门位置未确认时亮
 * LED2 阀关  (PA11): 阀门位置可信且处于关闭位时亮 (故障/未确认时以检修灯优先)
 * LED3 换电池(PA10): 电压低于 spupower.low 时亮
 *
 * 状态灯为"长亮": 状态成立即点亮, 不成立则熄灭; 深睡期间由引脚电平维持。
 * ==================================================================== */

/* 初始化三路 LED (由 gpio_init 统一调用, 默认熄灭) */
void    led_init(void);

/* 按当前系统状态刷新三路 LED (长亮真源) */
void    led_status_refresh(void);

/* 三路全灭 / 全亮 */
void    led_all_off(void);
void    led_all_on(void);

/* 校准失败提示: 三灯全亮 LED_FAIL_HOLD_MS 后恢复"按状态长亮" (内部喂狗) */
void    led_fail_hold(void);

/* ==================== 三灯同步提示 ("按键有效"反馈) ====================
 * 用途: 本产品无蜂鸣器, 按键是否被识别只能靠灯看。短按后三灯全亮 LED_NOTICE_MS,
 *       表示"按键已收到", 到期自动回到"按状态长亮"。
 * 范围: 任何状态都生效 —— 包含阀开(原本 LED2 灭)、阀关(原本 LED2 亮)、
 *       位置未确认(原本 LED1 检修亮); 提示是"按键反馈", 与阀门状态无关。
 * 实现: 非阻塞 — 点亮后照常回睡, 由引脚电平维持亮度(mypgio 休眠配置不动
 *       PA10~PA12 的 ODR), RTC 秒级闹钟到期唤醒熄灭; do_sleep 已把该截止
 *       时刻并入最近闹钟(led_notice_alarm_hhmmss)。
 * 粒度: 截止时刻按秒记账, 故实际时长 = 设定值截断到整秒(5s 即 4~5s); 时长宏
 *       LED_NOTICE_MS 见 Middle/app_config.h (与 LED_FAIL_HOLD_MS 同处)。 */

void    led_notice_show(void);            /* 点亮三灯(或重新计时) */
uint8_t led_notice_active(void);
int32_t led_notice_left_sec(void);        /* <=0 表示已到期 */
void    led_notice_end(void);             /* 立即结束并恢复按状态长亮 */
void    led_notice_expired_check(void);
/* 取截止时刻(HHMMSS), 供 do_sleep 设 RTC 闹钟; 返回 0=无提示 */
uint8_t led_notice_alarm_hhmmss(uint32_t *out_hhmmss);

#endif
