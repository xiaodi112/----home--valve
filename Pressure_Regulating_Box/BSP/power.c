#include "power.h"

/* 蓝牙电源状态: 与引脚电平同步的模块内状态 (定义从 App/lcd_ui.c 迁移到此,
 * 使无 LCD 产品仍保有 BLE 电源状态) */
uint8_t g_bt_on = 0;

/* 4G 供电状态 (软件真源): 见 power_4g_set/apply_state
 * 初始 0 = 断电: 硬件复位后 V4G_EN 为输入(高阻)→外部上拉→默认供电,
 * 固件上电即拉低断电, 之后完全由业务策略控制。 */
static uint8_t s_4g_pwr_on = 0;

void power_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* PA0 (4G_DTR): 默认 HIGH (允许模块进入轻休眠) */
    gpio_mode_set(PWR_4G_DTR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_4G_DTR_PIN);
    gpio_output_options_set(PWR_4G_DTR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_4G_DTR_PIN);
    gpio_bit_set(PWR_4G_DTR_PORT, PWR_4G_DTR_PIN);

    /* PB13 (V4G_EN): 按软件供电状态施加引脚电气状态。
     * 冷启动(s_4g_pwr_on=0)拉低断电; 运行期 sys_init() 重入时保持 4G 现有供电。 */
    power_4g_apply_state();

    /* PB14 (4G_PWRKEY): 高阻释放态(输入). NPN 低侧开关, 脉冲期间由 EC 驱动
     * 临时切换为推挽输出并拉高, 脉冲结束后回到此状态 */
    gpio_mode_set(PWR_4G_KEY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PWR_4G_KEY_PIN);

    /* PC12 (SW_BT): 只配置为输出, 不改变电平 (重入 sys_init 时保持蓝牙现有供电状态) */
    gpio_mode_set(PWR_BLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_BLE_PIN);
    gpio_output_options_set(PWR_BLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_BLE_PIN);

    /* PC13 (SW_Sensor/CH4_PWR): 只配置为输出 (本产品无 CH4, 保持复位态 LOW) */
    gpio_mode_set(PWR_CH4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_CH4_PIN);
    gpio_output_options_set(PWR_CH4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_CH4_PIN);

    /* PC3 (SW_IIC/传感器电源): 默认 LOW (断电). 采集前由 PT304D 驱动上电 */
    gpio_mode_set(PWR_SENSOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_SENSOR_PIN);
    gpio_output_options_set(PWR_SENSOR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_SENSOR_PIN);
    gpio_bit_reset(PWR_SENSOR_PORT, PWR_SENSOR_PIN);
}

/* 按当前供电状态施加引脚电气状态: 供电=推挽输出高, 断电=推挽输出低。
 * 深睡眠期间本引脚保持推挽输出(电平不变), 由 gpio_enter_low_power() 调用本函数重新施加。 */
void power_4g_apply_state(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(PWR_4G_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_4G_PIN);
    gpio_output_options_set(PWR_4G_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_4G_PIN);

    if(s_4g_pwr_on)
        gpio_bit_set(PWR_4G_PORT, PWR_4G_PIN);      /* 高电平 = 电源使能 */
    else
        gpio_bit_reset(PWR_4G_PORT, PWR_4G_PIN);    /* 低电平 = 关闭电源 */
}

void power_4g_set(uint8_t on)
{
    s_4g_pwr_on = on ? 1u : 0u;
    power_4g_apply_state();
}

uint8_t power_4g_is_on(void)
{
    /* 软件真源: 高阻态下引脚回读的是外部网络电平, 不参与判定 */
    return s_4g_pwr_on;
}

void power_ble_set(uint8_t on)
{
    g_bt_on = on ? 1u : 0u;
    if(on) gpio_bit_set(PWR_BLE_PORT, PWR_BLE_PIN);
    else   gpio_bit_reset(PWR_BLE_PORT, PWR_BLE_PIN);
}

uint8_t power_ble_is_on(void)
{
    return g_bt_on;
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

void power_sensor_set(uint8_t on)
{
    /* 深休眠会把 PC3 转 Analog, 唤醒后首次使用需先恢复输出模式 */
    gpio_mode_set(PWR_SENSOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_SENSOR_PIN);
    gpio_output_options_set(PWR_SENSOR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_SENSOR_PIN);
    if(on) gpio_bit_set(PWR_SENSOR_PORT, PWR_SENSOR_PIN);
    else   gpio_bit_reset(PWR_SENSOR_PORT, PWR_SENSOR_PIN);
}

uint8_t power_sensor_is_on(void)
{
    return (gpio_input_bit_get(PWR_SENSOR_PORT, PWR_SENSOR_PIN) == SET) ? 1 : 0;
}
