/**
 * @file  boot_main.c
 * @brief Bootloader 主入口
 *
 * Flash 分区 (见 ota_info.h):
 *   0x08000000   16 KB   Bootloader  (本工程)
 *   0x08004000    4 KB   OTA InfoBlock
 *   0x08005000  116 KB   App 运行区
 *   0x08022000  120 KB   下载缓存区
 *
 * 流程:
 *   1. 读 InfoBlock
 *   2. magic 无效 → 直接跳 App
 *   3. status=PENDING:  retry++; if>3 → 标记 FAILED 跳 App
 *                       否则 status=FLASHING; 擦 App 区; 拷贝 Cache→App;
 *                       status=SUCCESS, cur_version=new_fw_version; 跳 App
 *   4. status=FLASHING (异常断电恢复): 重做擦+拷贝 (幂等)
 *   5. 其他 status (SUCCESS/FAILED/IDLE) → 跳 App (App 启动后自行处理回执)
 *
 * 设计要点:
 *   - 不写 0xFFFFFFFF (自然态), 写其他值需先擦页 (整页拷贝 InfoBlock)
 *   - SCB->VTOR 由 App sys_init 自行设置, BL 跳转前不强制
 *   - 跳转前关 IRQ + __set_MSP, 由 App startup 接管
 */

#include "gd32l23x.h"
#include "ota_info.h"
#include <string.h>

/* ==================== 简易延时 (Bootloader 不依赖 SysTick) ==================== */
static void boot_delay_ms(uint32_t ms)
{
    /* 系统时钟通常 ~64MHz, 粗略空循环延时 */
    volatile uint32_t i;
    while(ms--)
    {
        for(i = 0; i < 8000; i++) __NOP();
    }
}

/* ==================== 最简 UART 调试输出 (USART0 PC4-TX 115200) ==================== */
static void boot_uart_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_USART0);

    gpio_af_set(GPIOC, GPIO_AF_7, GPIO_PIN_4);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

static void boot_print(const char *s)
{
    while(*s)
    {
        usart_data_transmit(USART0, (uint16_t)*s++);
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    }
}

static void boot_print_hex(uint32_t val)
{
    char buf[11];
    const char hex[] = "0123456789ABCDEF";
    buf[0] = '0'; buf[1] = 'x';
    buf[2] = hex[(val >> 28) & 0xF];
    buf[3] = hex[(val >> 24) & 0xF];
    buf[4] = hex[(val >> 20) & 0xF];
    buf[5] = hex[(val >> 16) & 0xF];
    buf[6] = hex[(val >> 12) & 0xF];
    buf[7] = hex[(val >>  8) & 0xF];
    buf[8] = hex[(val >>  4) & 0xF];
    buf[9] = hex[(val >>  0) & 0xF];
    buf[10] = '\0';
    boot_print(buf);
}

/* ==================== InfoBlock 读写 ==================== */
static void boot_info_load(OtaInfo_t *info)
{
    memcpy(info, (const void *)OTA_INFO_ADDR, sizeof(OtaInfo_t));
}

static int boot_info_write(const OtaInfo_t *info)
{
    static uint32_t buf[OTA_FLASH_PAGE_SIZE / 4];
    uint32_t i;
    uint32_t addr;
    fmc_state_enum st;

    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, info, sizeof(OtaInfo_t));

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    st = fmc_page_erase(OTA_INFO_ADDR);
    if(st != FMC_READY) { fmc_lock(); return -1; }

    addr = OTA_INFO_ADDR;
    for(i = 0; i < (OTA_FLASH_PAGE_SIZE / 4); i++)
    {
        st = fmc_word_program(addr, buf[i]);
        if(st != FMC_READY) { fmc_lock(); return -1; }
        addr += 4;
    }
    fmc_lock();
    return 0;
}

/* ==================== 擦除 App 区 ==================== */
static int boot_app_erase(void)
{
    uint32_t addr;
    fmc_state_enum st;
    uint32_t pages = OTA_APP_SIZE / OTA_FLASH_PAGE_SIZE;
    uint32_t i;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    addr = OTA_APP_ADDR;
    for(i = 0; i < pages; i++)
    {
        st = fmc_page_erase(addr);
        if(st != FMC_READY) { fmc_lock(); return -1; }
        addr += OTA_FLASH_PAGE_SIZE;
    }
    fmc_lock();
    return 0;
}

/* ==================== Cache → App 拷贝 ==================== */
/* 按字 (4B) 拷贝, size 不足 4 字节末尾补 0xFF (与 App 端写入一致) */
static int boot_copy_cache_to_app(uint32_t size)
{
    uint32_t src = OTA_CACHE_ADDR;
    uint32_t dst = OTA_APP_ADDR;
    uint32_t words;
    uint32_t tail;
    uint32_t i;
    uint32_t w;
    fmc_state_enum st;

    if(size == 0 || size > OTA_APP_SIZE) return -1;

    words = size / 4;
    tail = size & 0x03;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    for(i = 0; i < words; i++)
    {
        w = *(volatile uint32_t *)src;
        st = fmc_word_program(dst, w);
        if(st != FMC_READY) { fmc_lock(); return -1; }
        src += 4;
        dst += 4;
    }
    if(tail)
    {
        /* 末尾不足 4B: 从缓存读, 高位补 0xFF */
        w = 0xFFFFFFFFU;
        memcpy(&w, (const void *)src, tail);
        st = fmc_word_program(dst, w);
        if(st != FMC_READY) { fmc_lock(); return -1; }
    }
    fmc_lock();
    return 0;
}

/* ==================== 缓存固件合法性校验 (擦 App 前调用) ==================== */
/* 校验缓存区头部向量表: MSP 在 SRAM, ResetHandler 在 App Flash 范围
 * 通过 → 返回 0;  失败 → 返回 -1, 此时不应擦除 App 区 (旧程序保留) */
static int boot_validate_cache(uint32_t fw_size)
{
    uint32_t cache_msp;
    uint32_t cache_reset;

    if(fw_size < 8 || fw_size > OTA_APP_SIZE)
        return -1;

    cache_msp   = *(volatile uint32_t *)(OTA_CACHE_ADDR);
    cache_reset = *(volatile uint32_t *)(OTA_CACHE_ADDR + 4);

    /* MSP 必须在 SRAM (0x20000000 - 0x20007FFF) */
    if((cache_msp & 0x2FFE0000U) != 0x20000000U)
        return -1;

    /* ResetHandler 必须在 App Flash 区域 (OTA_APP_ADDR ~ OTA_APP_END), 且 Thumb 位=1 */
    if((cache_reset & 0x01U) == 0)
        return -1;
    if((cache_reset & ~1U) < OTA_APP_ADDR || (cache_reset & ~1U) > OTA_APP_END)
        return -1;

    return 0;
}

/* ==================== 拷贝后回读验证 ==================== */
static int boot_verify_copy(uint32_t fw_size)
{
    uint32_t i;
    uint32_t words = fw_size / 4;
    uint32_t src = OTA_CACHE_ADDR;
    uint32_t dst = OTA_APP_ADDR;

    for(i = 0; i < words; i++)
    {
        if(*(volatile uint32_t *)dst != *(volatile uint32_t *)src)
            return -1;
        src += 4;
        dst += 4;
    }
    return 0;
}

/* ==================== 跳转 App ==================== */
typedef void (*app_entry_t)(void);

static void boot_jump_to_app(void)
{
    uint32_t app_msp;
    uint32_t app_reset;
    app_entry_t entry;

    /* App 向量表起始字 = 初始 MSP, 第二个字 = ResetHandler */
    app_msp   = *(volatile uint32_t *)(OTA_APP_ADDR);
    app_reset = *(volatile uint32_t *)(OTA_APP_ADDR + 4);

    boot_print("[BL] Jump: MSP=");
    boot_print_hex(app_msp);
    boot_print(" Reset=");
    boot_print_hex(app_reset);
    boot_print("\r\n");

    /* 简单合法性检查: MSP 必须落在 SRAM 范围内 (0x20000000-0x20007FFF) */
    if((app_msp & 0x2FFE0000U) != 0x20000000U)
    {
        /* App 区无效, 死循环 (避免乱跳) */
        boot_print("[BL] App MSP invalid! HALT\r\n");
        while(1) __NOP();
    }

    __disable_irq();

    /* 复位关键外设, 防止跳到 App 后中断意外触发 */
    rcu_periph_reset_enable(RCU_GPIOARST);
    rcu_periph_reset_disable(RCU_GPIOARST);

    /* 重定位向量表 (App 的 sys_init 也会再设一次, 此处先到位) */
    SCB->VTOR = OTA_APP_ADDR;

    __set_MSP(app_msp);
    __enable_irq();

    entry = (app_entry_t)app_reset;
    entry();

    /* 不应到达 */
    while(1) __NOP();
}

/* ==================== Bootloader 主流程 ==================== */
int main(void)
{
    OtaInfo_t info;

    /* SystemInit 已在 startup 中调用, 时钟已就绪 */
    boot_delay_ms(10);

    /* 显式拉低 PD4(SW_IIC), 确保 I2C 电子开关断电
     * 防止 App→Reset→BL 跳转路径中 PD4 浮空导致 M24M01 供电异常 */
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_4);
    gpio_bit_reset(GPIOD, GPIO_PIN_4);

    /* 初始化调试串口 */
    boot_uart_init();
    boot_print("[BL] Bootloader start\r\n");

    /* 1. 加载 InfoBlock */
    boot_info_load(&info);

    /* 2. magic 无效 → 直接跳 App */
    if(info.magic != OTA_MAGIC_VALID)
    {
        boot_print("[BL] No valid magic, jump App\r\n");
        boot_jump_to_app();
    }

    /* 3. 状态分发 */
    switch(info.status)
    {
        case OTA_STATUS_PENDING:
        case OTA_STATUS_FLASHING:
            boot_print("[BL] OTA status=PENDING/FLASHING, size=");
            boot_print_hex(info.new_fw_size);
            boot_print("\r\n");

            /* 升级流程 (FLASHING 是上次断电恢复, 与 PENDING 同样处理) */
            if(info.retry_count >= OTA_MAX_RETRY)
            {
                /* 超过重试上限, 标记 FAILED 让 App 上报后清除 */
                boot_print("[BL] Max retry, FAILED\r\n");
                info.status = OTA_STATUS_FAILED;
                boot_info_write(&info);
                boot_jump_to_app();
            }

            /* ★ 擦写前校验缓存固件合法性 → 不合法则保留旧 App (回滚) */
            {
                uint32_t c_msp = *(volatile uint32_t *)(OTA_CACHE_ADDR);
                uint32_t c_rst = *(volatile uint32_t *)(OTA_CACHE_ADDR + 4);
                boot_print("[BL] Cache[0]=MSP=");
                boot_print_hex(c_msp);
                boot_print(" [4]=Reset=");
                boot_print_hex(c_rst);
                boot_print("\r\n");
            }
            if(boot_validate_cache(info.new_fw_size) != 0)
            {
                boot_print("[BL] Validate FAIL\r\n");
                info.status = OTA_STATUS_FAILED;
                boot_info_write(&info);
                /* 旧 App 区未擦除, jump 后运行原程序 */
                boot_jump_to_app();
            }
            boot_print("[BL] Validate OK\r\n");

            info.retry_count++;
            info.status = OTA_STATUS_FLASHING;
            boot_info_write(&info);

            /* 擦 App 区 */
            boot_print("[BL] Erasing App...\r\n");
            if(boot_app_erase() != 0)
            {
                boot_print("[BL] Erase FAIL\r\n");
                info.status = OTA_STATUS_FAILED;
                boot_info_write(&info);
                boot_jump_to_app();
            }

            /* 拷贝 Cache → App */
            boot_print("[BL] Copying Cache->App...\r\n");
            if(boot_copy_cache_to_app(info.new_fw_size) != 0)
            {
                boot_print("[BL] Copy FAIL\r\n");
                /* 拷贝失败但缓存仍完好, 不标记 FAILED 以便下次重试 */
                boot_jump_to_app();
            }

            /* ★ 拷贝后回读验证: 确保 App 区数据与缓存一致 */
            boot_print("[BL] Verifying...\r\n");
            if(boot_verify_copy(info.new_fw_size) != 0)
            {
                boot_print("[BL] Verify FAIL\r\n");
                /* 校验不一致, 不标记 FAILED, 保留重试机会 */
                boot_jump_to_app();
            }

            /* 升级成功: 写 SUCCESS 状态 + 当前版本 */
            boot_print("[BL] SUCCESS, jump App\r\n");
            info.status = OTA_STATUS_SUCCESS;
            info.cur_version = info.new_fw_version;
            info.retry_count = 0;
            boot_info_write(&info);
            boot_jump_to_app();
            break;

        case OTA_STATUS_SUCCESS:
        case OTA_STATUS_FAILED:
        case OTA_STATUS_IDLE:
        default:
            /* 已就绪状态, 直接跳转 App; App 端自行上报回执并清除 */
            boot_jump_to_app();
            break;
    }

    /* 不应到达 */
    while(1) __NOP();
}
