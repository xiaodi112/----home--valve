#include "delay.h"

#include "systick.h"
#include "gd32l23x.h"
#include "gd32l23x_rtc.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "tim.h"
#include "usart.h"

rtc_parameter_struct   rtc_initpara;
rtc_alarm_struct  rtc_alarm;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;
uint32_t RTCSRC_FLAG = 0;

// RTC闹钟唤醒标志
volatile uint8_t rtc_alarm_flag = 0;

static void rtc_pre_config(void);

// BCD ↔ DEC 工具函数
uint8_t dec2bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

uint8_t bcd2dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// ==============================
// RTC 总初始化（时钟+分频，不阻塞等待串口）
// 时间将由 rtc_set_from_cclk() 通过4G模块同步
// ==============================
void rtc_init_all(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKP);
    pmu_backup_write_enable();

    RTCSRC_FLAG = GET_BITS(RCU_BDCTL, 8, 9);

    // 检测时钟源是否与当前配置匹配, 不匹配则强制备份域复位
    {
#if defined (RTC_CLOCK_SOURCE_IRC32K)
        uint32_t desired_src = 0x02;  // IRC32K = RCU_RTCSRC_IRC32K
#elif defined (RTC_CLOCK_SOURCE_LXTAL)
        uint32_t desired_src = 0x01;  // LXTAL = RCU_RTCSRC_LXTAL
#endif
        if(RTCSRC_FLAG != 0x00 && RTCSRC_FLAG != desired_src)
        {
            // 时钟源不匹配(如从LXTAL切换到IRC32K), 必须复位备份域
            printf_debug("[RTC] Clock source mismatch (cur=0x%02X, want=0x%02X), reset BKP domain\r\n",
                         RTCSRC_FLAG, desired_src);
            rcu_bkp_reset_enable();
            rcu_bkp_reset_disable();
            RTCSRC_FLAG = 0;
        }
    }

    rtc_pre_config();

    if(BKP_VALUE == RTC_BKP0 && RTCSRC_FLAG != 0x00) {
        printf_debug("[RTC] Already configured\r\n");
        rtc_show_time();
        printf_debug("\r\n");
    } else {
        printf_debug("[RTC] Not configured, will sync from 4G\r\n");
    }
    rcu_all_reset_flag_clear();
}

// ==============================
// 仅做时钟预配置（供外部调用）
// ==============================
void rtc_pre_config_only(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKP);
    pmu_backup_write_enable();
    rtc_pre_config();
}

// ==============================
// RTC 预配置（时钟/分频）
// ==============================
static void rtc_pre_config(void)
{
#if defined (RTC_CLOCK_SOURCE_IRC32K)
    rcu_osci_on(RCU_IRC32K);
    rcu_osci_stab_wait(RCU_IRC32K);
    rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
    prescaler_s = 0x13F;
    prescaler_a = 0x63;

#elif defined (RTC_CLOCK_SOURCE_LXTAL)
    rcu_osci_on(RCU_LXTAL);

    /* 外部 32.768kHz 晶振起振时间典型 0.5-2 秒, GD32 库内置 timeout 不足
     * 用 polling + 10ms 间隔, 最多等 3 秒, 直到 LXTAL 真正稳定 */
    {
        uint16_t lxtal_wait;
        ErrStatus lxtal_ok = ERROR;
        for(lxtal_wait = 0; lxtal_wait < 300; lxtal_wait++)   // 300 × 10ms = 3s 上限
        {
            if(SUCCESS == rcu_osci_stab_wait(RCU_LXTAL))
            {
                lxtal_ok = SUCCESS;
                break;
            }
            trea_delay_ms(10);
        }
        if(SUCCESS == lxtal_ok)
            printf_debug("[RTC] LXTAL stable (waited ~%dms)\r\n", lxtal_wait * 10);
        else
            printf_debug("[RTC] LXTAL startup TIMEOUT after 3s (check crystal/load caps)\r\n");
    }

    rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
    prescaler_s = 0xFF;
    prescaler_a = 0x7F;
#endif

    rcu_periph_clock_enable(RCU_RTC);
    if(ERROR == rtc_register_sync_wait())
    {
        printf_debug("[RTC] register_sync_wait FAILED in pre_config\r\n");
    }
}

// ==============================
// 从AT+CCLK?应答解析时间并设置RTC
// 输入格式: "26/04/17,09:37:11+32"
// tz 单位为15分钟 (quarter-hours), 例: +32 = 8小时
// EC800E AT+CCLK? 返回 UTC 时间 + tz 偏移
// 若解析到 tz, 使用 tz 换算本地时间; 否则默认+8h (中国)
// ==============================

// 每月天数查找表 (非闰年)
static uint8_t days_in_month(int month, int year)
{
    static const uint8_t dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if(month < 1 || month > 12) return 31;
    if(month == 2)
    {
        int y = 2000 + year;
        if((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))
            return 29;
    }
    return dm[month - 1];
}

#define RTC_INIT_RETRY_COUNT  5

int rtc_set_from_cclk(const char *cclk_str)
{
    int year, month, day, hour, min, sec, tz = 0;
    int tz_negative = 0;

    // 找到引号内容
    const char *p = cclk_str;
    while(*p && *p != '"') p++;
    if(*p == '"') p++;  // 跳过开头引号

    // 解析: 26/04/17,09:37:11+32  (tz部分可选)
    int n = sscanf(p, "%d/%d/%d,%d:%d:%d+%d",
                   &year, &month, &day, &hour, &min, &sec, &tz);
    if(n < 6)
    {
        // 也尝试负时区: 26/04/17,09:37:11-xx
        n = sscanf(p, "%d/%d/%d,%d:%d:%d-%d",
                   &year, &month, &day, &hour, &min, &sec, &tz);
        if(n >= 7) tz_negative = 1;
    }
    if(n < 6)
    {
        printf_debug("[RTC] CCLK parse failed: %s\r\n", cclk_str);
        return 0;
    }

    // 计算本地时间偏移 (分钟)
    // tz 为 quarter-hours, 若未解析到 tz (n<7) 则默认 +8h = +480min
    int offset_min;
    if(n >= 7)
    {
        offset_min = tz * 15;
        if(tz_negative) offset_min = -offset_min;
        printf_debug("[RTC] Using tz=%d (offset %+d min)\r\n", tz_negative ? -tz : tz, offset_min);
    }
    else
    {
        offset_min = 8 * 60;
        printf_debug("[RTC] No tz info, default +8h\r\n");
    }

    // 应用偏移到 hour:min, 并处理日期进位/借位
    min += offset_min % 60;
    hour += offset_min / 60;

    // 分钟规范化
    while(min >= 60) { min -= 60; hour++; }
    while(min < 0)   { min += 60; hour--; }

    // 小时 → 日期进位/借位
    while(hour >= 24)
    {
        hour -= 24;
        day++;
    }
    while(hour < 0)
    {
        hour += 24;
        day--;
    }

    // 日期 → 月份进位
    while(day > days_in_month(month, year))
    {
        day -= days_in_month(month, year);
        month++;
        if(month > 12) { month = 1; year++; }
    }
    // 日期 → 月份借位
    while(day < 1)
    {
        month--;
        if(month < 1) { month = 12; year--; }
        day += days_in_month(month, year);
    }

    printf_debug("[RTC] CCLK parsed: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                 year, month, day, hour, min, sec);

    /* === 已配置且时间偏差小: 跳过 re-init ===
     * RTC 已运行时再次进入 init mode 在某些 GD32L23x 状态下会失败 (INITF 等待超时);
     * 而 LXTAL 精度 ±20ppm = 每天最多 ~1.7 秒, 完全无需每次 4G 重连都重新校准.
     * 仅在首次配置 (BKP0 未设) 或时间偏差 > 5 分钟时才走完整 init 路径 */
    if(BKP_VALUE == RTC_BKP0)
    {
        rtc_parameter_struct cur_para;
        long diff_sec, target_sec, cur_sec;
        int cur_year, cur_month, cur_date, cur_hour, cur_min, cur_second;

        rtc_current_time_get(&cur_para);
        cur_year   = bcd2dec(cur_para.year);
        cur_month  = bcd2dec(cur_para.month);
        cur_date   = bcd2dec(cur_para.date);
        cur_hour   = bcd2dec(cur_para.hour);
        cur_min    = bcd2dec(cur_para.minute);
        cur_second = bcd2dec(cur_para.second);

        target_sec = (long)hour * 3600 + (long)min * 60 + sec;
        cur_sec    = (long)cur_hour * 3600 + (long)cur_min * 60 + cur_second;
        diff_sec   = target_sec - cur_sec;
        if(diff_sec < 0) diff_sec = -diff_sec;

        // 同年同月同日 + 偏差 < 1 分钟: 跳过, 沿用现有 RTC 时间
        if(cur_year == (year % 100) && cur_month == month && cur_date == day && diff_sec < 60)
        {
            printf_debug("[RTC] Already configured, diff=%lds < 60s, skip re-init\r\n", diff_sec);
            rtc_show_time();
            printf_debug("\r\n");
            return 1;
        }
        printf_debug("[RTC] Drift too large (diff=%lds or date mismatch), force re-init\r\n", diff_sec);
    }

    // 配置RTC
    rtc_initpara.factor_asyn    = prescaler_a;
    rtc_initpara.factor_syn     = prescaler_s;
    rtc_initpara.display_format = RTC_24HOUR;
    rtc_initpara.am_pm          = RTC_AM;
    rtc_initpara.year           = dec2bcd((uint8_t)(year % 100));
    rtc_initpara.month          = dec2bcd((uint8_t)month);
    rtc_initpara.date           = dec2bcd((uint8_t)day);
    rtc_initpara.hour           = dec2bcd((uint8_t)hour);
    rtc_initpara.minute         = dec2bcd((uint8_t)min);
    rtc_initpara.second         = dec2bcd((uint8_t)sec);

    // 确保备份域可写 + RTC同步就绪
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();

    /* sync_wait 失败时用 polling + 50ms delay 多次重试 (最多 1s),
     * 应对 LXTAL 起振慢或影子寄存器同步延迟的边界情况 */
    {
        uint8_t sync_retry;
        ErrStatus sync_ok = ERROR;
        for(sync_retry = 0; sync_retry < 20; sync_retry++)   // 20 × 50ms = 1s 上限
        {
            if(SUCCESS == rtc_register_sync_wait())
            {
                sync_ok = SUCCESS;
                break;
            }
            trea_delay_ms(50);
        }
        if(ERROR == sync_ok)
            printf_debug("[RTC] Sync wait FAILED after 1s retry, RTC clock may not be running\r\n");
        else if(sync_retry > 0)
            printf_debug("[RTC] Sync wait OK after %d retries\r\n", sync_retry);
    }

    // 重试机制: rtc_init 可能因LXTAL未完全稳定而超时, 每次重试间 200ms 让 LXTAL 充分稳定
    {
        uint8_t retry;
        ErrStatus res = ERROR;
        for(retry = 0; retry < RTC_INIT_RETRY_COUNT; retry++)
        {
            res = rtc_init(&rtc_initpara);
            if(SUCCESS == res)
                break;
            printf_debug("[RTC] Init attempt %d failed, retrying...\r\n", retry + 1);
            trea_delay_ms(200);
        }
        if(ERROR == res)
        {
            printf_debug("[RTC] Init failed after %d retries!\r\n", RTC_INIT_RETRY_COUNT);
            return 0;
        }
    }

    RTC_BKP0 = BKP_VALUE;
    printf_debug("[RTC] Time set OK\r\n");
    rtc_show_time();
    printf_debug("\r\n");
    return 1;
}

// ==============================
// 显示当前时间
// ==============================
void rtc_show_time(void)
{
    rtc_current_time_get(&rtc_initpara);
    printf_debug("\rCurrent time: 20%0.2x-%0.2x-%0.2x %0.2x:%0.2x:%0.2x",
           rtc_initpara.year, rtc_initpara.month, rtc_initpara.date,
           rtc_initpara.hour, rtc_initpara.minute, rtc_initpara.second);
}

// ==============================
// 获取当前时间字符串 "20xx-xx-xx xx:xx:xx"
// ==============================
void rtc_get_time_str(char *buf, uint8_t buf_size)
{
    rtc_current_time_get(&rtc_initpara);
    snprintf(buf, buf_size, "20%02x-%02x-%02x %02x:%02x:%02x",
             rtc_initpara.year, rtc_initpara.month, rtc_initpara.date,
             rtc_initpara.hour, rtc_initpara.minute, rtc_initpara.second);
}

// ==============================
// 获取当前RTC时间的HHMMSS整数值
// 如 13:29:00 → 132900, 用于与窗口start/stop比较
// ==============================
uint32_t rtc_get_hhmmss(void)
{
    rtc_current_time_get(&rtc_initpara);
    uint8_t h = bcd2dec((uint8_t)(rtc_initpara.hour   & 0xFF));
    uint8_t m = bcd2dec((uint8_t)(rtc_initpara.minute  & 0xFF));
    uint8_t s = bcd2dec((uint8_t)(rtc_initpara.second  & 0xFF));
    return (uint32_t)h * 10000 + (uint32_t)m * 100 + s;
}

// ==============================
// RTC闹钟唤醒初始化
// 配置EXTI17 + NVIC使能RTC_Alarm中断
// ==============================
void rtc_alarm_wakeup_init(void)
{
    // EXTI17 = RTC Alarm事件线, 上升沿触发
    exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_flag_clear(EXTI_17);

    // 使能RTC Alarm中断 (IRQ #57)
    nvic_irq_enable(RTC_Alarm_IRQn, 1);

    rtc_alarm_flag = 0;
}

// ==============================
// 设置下一次RTC闹钟 (对齐到整数时间网格)
// interval_minutes: 采集间隔(分钟), 闹钟对齐到下一个整数倍时刻
// 例: interval=10, 当前10:25 → 下次闹钟10:30
//     interval=30, 当前10:25 → 下次闹钟10:30
//     interval=60, 当前10:25 → 下次闹钟11:00
// ==============================
void rtc_set_next_alarm(uint16_t interval_minutes)
{
    rtc_alarm_struct alarm_cfg;
    rtc_parameter_struct cur_time;
    uint16_t cur_total_min, next_total_min, wait_min;
    uint8_t next_hour, next_min;

    if(interval_minutes < 1) interval_minutes = 1;

    // 读取当前时间
    rtc_current_time_get(&cur_time);

    uint8_t cur_hour = bcd2dec(cur_time.hour);
    uint8_t cur_min  = bcd2dec(cur_time.minute);

    // 当前时刻转换为一天内的总分钟数
    cur_total_min = (uint16_t)cur_hour * 60 + cur_min;

    // 计算下一个对齐时间点: 当前分钟的下一个interval整数倍
    // 余数 = cur_total_min % interval_minutes
    // 下一对齐点 = cur_total_min + (interval - 余数), 如果余数为0则加一个完整interval
    {
        uint16_t remainder = cur_total_min % interval_minutes;
        if(remainder == 0)
            next_total_min = cur_total_min + interval_minutes;
        else
            next_total_min = cur_total_min + (interval_minutes - remainder);
    }

    // 处理跨天
    next_total_min = next_total_min % (24 * 60);

    wait_min = (next_total_min >= cur_total_min)
             ? (next_total_min - cur_total_min)
             : (24 * 60 - cur_total_min + next_total_min);

    next_hour = (uint8_t)(next_total_min / 60);
    next_min  = (uint8_t)(next_total_min % 60);

    // 先关闭闹钟0
    rtc_alarm_disable(RTC_ALARM0);

    // 配置闹钟: 匹配 时:分:秒=00, 屏蔽日期（每天都能触发）
    alarm_cfg.alarm_mask       = RTC_ALARM_DATE_MASK;     // 不比较日期
    alarm_cfg.weekday_or_date  = RTC_ALARM_DATE_SELECTED;
    alarm_cfg.alarm_day        = 0x01;
    alarm_cfg.am_pm            = RTC_AM;
    alarm_cfg.alarm_hour       = dec2bcd(next_hour);
    alarm_cfg.alarm_minute     = dec2bcd(next_min);
    alarm_cfg.alarm_second     = 0x00;

    rtc_alarm_config(RTC_ALARM0, &alarm_cfg);

    // 清除闹钟标志
    rtc_flag_clear(RTC_FLAG_ALARM0);
    exti_flag_clear(EXTI_17);

    // 使能闹钟中断
    rtc_interrupt_enable(RTC_INT_ALARM0);
    rtc_alarm_enable(RTC_ALARM0);

    // 清零唤醒标志
    rtc_alarm_flag = 0;

    printf_debug("[RTC] Next alarm: %02d:%02d:00 (in %d min)\r\n",
                 next_hour, next_min, wait_min);
}

// ==============================
// 设置秒级RTC闹钟 (用于实时采集模式)
// seconds: 1-10, 从当前时刻偏移N秒后触发
// ==============================
void rtc_set_next_alarm_sec(uint8_t seconds)
{
    rtc_alarm_struct alarm_cfg;
    rtc_parameter_struct cur_time;

    if(seconds < 1) seconds = 1;
    if(seconds > 60) seconds = 60;

    rtc_current_time_get(&cur_time);

    uint8_t cur_hour = bcd2dec(cur_time.hour);
    uint8_t cur_min  = bcd2dec(cur_time.minute);
    uint8_t cur_sec  = bcd2dec(cur_time.second);

    // 当前时刻转总秒数, 加偏移
    uint32_t total_sec = (uint32_t)cur_hour * 3600 + (uint32_t)cur_min * 60 + cur_sec + seconds;
    total_sec = total_sec % (24 * 3600);

    uint8_t next_hour = (uint8_t)(total_sec / 3600);
    uint8_t next_min  = (uint8_t)((total_sec % 3600) / 60);
    uint8_t next_sec  = (uint8_t)(total_sec % 60);

    rtc_alarm_disable(RTC_ALARM0);

    alarm_cfg.alarm_mask       = RTC_ALARM_DATE_MASK;
    alarm_cfg.weekday_or_date  = RTC_ALARM_DATE_SELECTED;
    alarm_cfg.alarm_day        = 0x01;
    alarm_cfg.am_pm            = RTC_AM;
    alarm_cfg.alarm_hour       = dec2bcd(next_hour);
    alarm_cfg.alarm_minute     = dec2bcd(next_min);
    alarm_cfg.alarm_second     = dec2bcd(next_sec);

    rtc_alarm_config(RTC_ALARM0, &alarm_cfg);

    rtc_flag_clear(RTC_FLAG_ALARM0);
    exti_flag_clear(EXTI_17);

    rtc_interrupt_enable(RTC_INT_ALARM0);
    rtc_alarm_enable(RTC_ALARM0);

    rtc_alarm_flag = 0;

    printf_debug("[RTC] Next frequency: %02d:%02d:%02d (in %d sec)\r\n",
                 next_hour, next_min, next_sec, seconds);
}

// ==============================
// 设置RTC闹钟到绝对HHMMSS时间 (定时阀门用)
// 如果目标时间已过, 设为明天同一时间
// 返回距离目标的秒数
// ==============================
uint32_t rtc_set_alarm_at_hhmmss(uint32_t target_hhmmss)
{
    rtc_alarm_struct alarm_cfg;
    rtc_parameter_struct cur_time;

    uint8_t t_hour = (uint8_t)(target_hhmmss / 10000);
    uint8_t t_min  = (uint8_t)((target_hhmmss / 100) % 100);
    uint8_t t_sec  = (uint8_t)(target_hhmmss % 100);

    rtc_current_time_get(&cur_time);
    uint8_t cur_hour = bcd2dec(cur_time.hour);
    uint8_t cur_min  = bcd2dec(cur_time.minute);
    uint8_t cur_sec  = bcd2dec(cur_time.second);

    uint32_t now_total = (uint32_t)cur_hour * 3600 + (uint32_t)cur_min * 60 + cur_sec;
    uint32_t tgt_total = (uint32_t)t_hour * 3600 + (uint32_t)t_min * 60 + t_sec;

    uint32_t wait_sec;
    if(tgt_total > now_total)
        wait_sec = tgt_total - now_total;
    else
        wait_sec = 86400 - now_total + tgt_total;  // 明天

    rtc_alarm_disable(RTC_ALARM0);

    alarm_cfg.alarm_mask       = RTC_ALARM_DATE_MASK;
    alarm_cfg.weekday_or_date  = RTC_ALARM_DATE_SELECTED;
    alarm_cfg.alarm_day        = 0x01;
    alarm_cfg.am_pm            = RTC_AM;
    alarm_cfg.alarm_hour       = dec2bcd(t_hour);
    alarm_cfg.alarm_minute     = dec2bcd(t_min);
    alarm_cfg.alarm_second     = dec2bcd(t_sec);

    rtc_alarm_config(RTC_ALARM0, &alarm_cfg);

    rtc_flag_clear(RTC_FLAG_ALARM0);
    exti_flag_clear(EXTI_17);

    rtc_interrupt_enable(RTC_INT_ALARM0);
    rtc_alarm_enable(RTC_ALARM0);

    rtc_alarm_flag = 0;

    printf_debug("[RTC] Valve alarm: %02d:%02d:%02d (in %lu sec)\r\n",
                 t_hour, t_min, t_sec, wait_sec);
    return wait_sec;
}