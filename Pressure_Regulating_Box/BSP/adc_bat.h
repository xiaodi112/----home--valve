#ifndef ADC_BAT_H
#define ADC_BAT_H

#include "gd32l23x.h"

// 初始化PA5 ADC (电池电压检测)
void adc_bat_init(void);

// 读取电池电压 (单位: V)
// 电路: VIN3V6 -- R3(2M) -- AD_BAT(PA5) -- R5(2M) -- GND
// V_BAT = ADC_value * (3.3/4095) * 2
float adc_bat_read_voltage(void);

#endif /* ADC_BAT_H */
