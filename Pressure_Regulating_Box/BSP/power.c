#include "power.h"

void power_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* PA0 (4G_DTR): 默认 HIGH (保持 EC800E 唤醒) */
    gpio_mode_set(PWR_4G_DTR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_4G_DTR_PIN);
    gpio_output_options_set(PWR_4G_DTR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_4G_DTR_PIN);
    gpio_bit_set(PWR_4G_DTR_PORT, PWR_4G_DTR_PIN);

    /* PB5 (V4G_EN): 默认 LOW (4G 默认断电) */
    gpio_mode_set(PWR_4G_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_4G_PIN);
    gpio_output_options_set(PWR_4G_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_4G_PIN);

    /* PC12 (SW_BT): 默认 LOW (蓝牙默认断电) */
    gpio_mode_set(PWR_BLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_BLE_PIN);
    gpio_output_options_set(PWR_BLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_BLE_PIN);

    /* PC3 (SENSOR_PWR): 默认 LOW */
    gpio_mode_set(PWR_SENSOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_SENSOR_PIN);
    gpio_output_options_set(PWR_SENSOR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_SENSOR_PIN);

    /* PC13 (SW_Sensor/CH4_PWR): 默认 LOW */
    gpio_mode_set(PWR_CH4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_CH4_PIN);
    gpio_output_options_set(PWR_CH4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_CH4_PIN);
}

void power_4g_set(uint8_t on)
{
    if(on) gpio_bit_set(PWR_4G_PORT, PWR_4G_PIN);
    else   gpio_bit_reset(PWR_4G_PORT, PWR_4G_PIN);
}

uint8_t power_4g_is_on(void)
{
    return (gpio_input_bit_get(PWR_4G_PORT, PWR_4G_PIN) == SET) ? 1 : 0;
}

void power_ble_set(uint8_t on)
{
    if(on) gpio_bit_set(PWR_BLE_PORT, PWR_BLE_PIN);
    else   gpio_bit_reset(PWR_BLE_PORT, PWR_BLE_PIN);
}

void power_sensor_set(uint8_t on)
{
    if(on) gpio_bit_set(PWR_SENSOR_PORT, PWR_SENSOR_PIN);
    else   gpio_bit_reset(PWR_SENSOR_PORT, PWR_SENSOR_PIN);
}

void power_4g_dtr_set(uint8_t level)
{
    if(level) gpio_bit_set(PWR_4G_DTR_PORT, PWR_4G_DTR_PIN);
    else      gpio_bit_reset(PWR_4G_DTR_PORT, PWR_4G_DTR_PIN);
}

void power_ch4_set(uint8_t on)
{
    if(on) gpio_bit_set(PWR_CH4_PORT, PWR_CH4_PIN);
    else  gpio_bit_reset(PWR_CH4_PORT, PWR_CH4_PIN);
}

uint8_t power_ch4_is_on(void)
{
    return (gpio_input_bit_get(PWR_CH4_PORT, PWR_CH4_PIN) == SET) ? 1 : 0;
}
