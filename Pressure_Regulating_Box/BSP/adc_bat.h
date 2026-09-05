#ifndef ADC_BAT_H
#define ADC_BAT_H

#include "gd32l23x.h"

// 电池电压检测: AD_BAT = PC2 (ADC_CHANNEL_12)
// 采集基准: PC3 必须配置为推挽输出并拉低后才能采样 (深睡转Analog后同样要先拉低)
// 电路: VBAT -- R3(2M) -- AD_BAT(PC2) -- R5(2M) -- PC3(基准, 输出低)
// V_BAT = ADC_value * (3.0/4095) * 2   (PC2 分压 2:1)
void adc_bat_init(void);

// 读取电池电压 (单位: V), 内部保证 PC3 基准已拉低
float adc_bat_read_voltage(void);

#endif /* ADC_BAT_H */
