/**
 * @file ec800e.c
 * @brief EC801E LTE Cat.1 模块驱动
 * @details AT指令驱动：电源、网络、MQTT、数据发布、时间同步
 *          MQTT参数从 g_mqtt 运行时配置读取（EEPROM保存）
 */

#include "ec800e.h"
#include "eeprom_config.h"
#include "app_config.h"
#include "usart.h"
#include "delay.h"
#include "tim.h"
#include "mygpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "ota_app.h"
#include "threshold.h"

// ==================== 内部: LPUART接收缓冲 ====================
char ec_rx_buf[1024];
static uint16_t ec_rx_len = 0;

// ==================== 模块共享 AT/MQTT 工作缓冲 ====================
// 这些函数全部主循环串行调用, 不嵌套不并发, 共用一组缓冲节省 ~4.5KB RAM
// 严禁 publish/subscribe 内部回调或重入场景中并发使用
static char s_cmd_buf[256];          // AT 命令格式化
static char s_payload[1024];         // MQTT publish payload (LY格式9条约870字节)
static char s_timestamp[32];         // 时间戳格式化
static char s_topic[MQTT_TOPIC_MAX]; // 临时 topic (订阅/OTA reply)


static EC_Status_t EC_SendCmd(const char *cmd, const char *expect, uint32_t timeout_ms);

// ==================== QMTPUBEX数据模式恢复 ====================
// QMTPUBEX要求精确payload_len字节, Ctrl+Z无法退出数据模式
// 发送填充数据满足字节计数, 让模块正常退出数据模式
static void ec_drain_data_mode(const char *data, uint16_t len)
{
    serial_write(LPUART0, (const uint8_t*)data, len);
    lpuart_rx_recover();
    trea_delay_ms(1000);   // 等模块处理(+QMTPUBEX: 0,0,x)
    lpuart_rx_flush();     // 清除模块错误/成功响应
}

// ==================== PUBEX发送+重试(1次) ====================
// 发送PUBEX命令等待'>', 超时则drain后重试一次
static EC_Status_t ec_pubex_send(const char *cmd_buf, const char *payload, uint16_t payload_len)
{
    uint8_t attempt;
    for(attempt = 0; attempt < 2; attempt++)
    {
        if(attempt > 0)
        {
            printf_debug("[EC] Publish retry\r\n");
            trea_delay_ms(500);
        }
        if(EC_SendCmd(cmd_buf, ">", APP_AT_TIMEOUT_PUBEX) == EC_OK)
        {
            trea_delay_ms(100);
            serial_write(LPUART0, (const uint8_t*)payload, payload_len);
            lpuart_rx_recover();
            return EC_OK;
        }
        printf_debug("[EC] no '>' (attempt %d/2)\r\n", attempt + 1);
        ec_drain_data_mode(payload, payload_len);
    }
    return EC_TIMEOUT;
}

// ==================== 安全AT指令发送（防卡死） ====================
// 核心改进:
//   1) 用字节计数(最多256)限制，防RBNE不清除导致死循环
//   2) 每字节超时30ms，无数据+已收到部分内容即退出
//   3) 缓冲区满强制退出
// 应答保存到ec_rx_buf供外部解析
static EC_Status_t EC_SendCmd(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    uint8_t ch;
    uint16_t cnt;
    uint32_t start;
    uint32_t byte_timeout;

    ec_rx_len = 0;
    ec_rx_buf[0] = '\0';
    lpuart_rx_flush();

    printf_cat1("%s", cmd);
    lpuart_rx_recover();  // TX后防御性恢复LPUART RX中断

    // 每字节最长等30ms，但总时间不超过timeout_ms
    // cnt只在实际收到字节时递增(防止无数据时cnt耗尽导致提前退出)
    start = trea_millis();
    cnt = 0;
    while(cnt < sizeof(ec_rx_buf) - 1)
    {
        ota_wdt_feed();  /* 喂狗保护：长AT（如QHTTPGET等数十秒大阻塞）下防止复位 */
        // 总超时检查
        if((trea_millis() - start) >= timeout_ms)
            break;

        // 剩余时间不足30ms时，用剩余时间作为单字节超时
        byte_timeout = timeout_ms - (trea_millis() - start);
        if(byte_timeout > 30) byte_timeout = 30;

        if(trea_uart_recv_byte_timeout(LPUART0, &ch, byte_timeout))
        {
            ec_rx_buf[cnt++] = (char)ch;
            ec_rx_len = cnt;
            ec_rx_buf[cnt] = '\0';

            // 检查是否收到期望应答
            if(expect && strstr(ec_rx_buf, expect) != NULL)
                return EC_OK;
            // ERROR响应快速退出(不白等满超时)
            if(strstr(ec_rx_buf, "\r\nERROR") != NULL)
                return EC_TIMEOUT;
        }
    }

    // 超时或缓冲区满 — 打印诊断信息
    printf_debug("[EC] TIMEOUT expect='%s' rx(%d):'%s' STAT=0x%08X\r\n",
                 expect ? expect : "?", cnt, ec_rx_buf,
                 (unsigned int)LPUART_STAT(LPUART0));
    return EC_TIMEOUT;
}

// 发送AT指令，失败重试（带总超时保护+智能提前退出）
static EC_Status_t EC_SendCmdRetry(const char *cmd, const char *expect,
                                   uint32_t timeout_ms, uint8_t max_retry)
{
    uint8_t i;
    uint8_t zero_rx_count = 0;  // 连续0字节应答计数

    for(i = 0; i < max_retry; i++)
    {
        if(EC_SendCmd(cmd, expect, timeout_ms) == EC_OK)
            return EC_OK;

        printf_debug("[EC] Retry %d/%d (got %d bytes): %s",
                     i + 1, max_retry, ec_rx_len, cmd);
        printf_debug("[EC] Response: %s\r\n", ec_rx_buf);

        // 如果连续2次完全没收到数据，说明模块可能掉了，提前退出
        if(ec_rx_len == 0)
        {
            zero_rx_count++;
            if(zero_rx_count >= 2)
            {
                printf_debug("[EC] Module not responding, abort\r\n");
                return EC_ERROR;
            }
        }
        else
        {
            zero_rx_count = 0;
        }

        trea_delay_ms(1000);
    }
    printf_debug("[EC] FAIL after %d retries: %s", max_retry, cmd);
    return EC_ERROR;
}

// ==================== 电源控制 ====================
void EC801E_PowerOn(void)
{
    gpio_bit_set(EC_PWRKEY_PORT, EC_PWRKEY_PIN);
    printf_debug("[EC] Power ON\r\n");
}

void EC801E_PowerOff(void)
{
    gpio_bit_reset(EC_PWRKEY_PORT, EC_PWRKEY_PIN);
    printf_debug("[EC] Power OFF\r\n");
}

// ==================== 网络初始化 ====================
EC_Status_t EC801E_InitNetwork(void)
{
    printf_debug("[EC] InitNetwork...\r\n");
	trea_delay_ms(1000);

    if(EC_SendCmdRetry("AT\r\n", "OK", APP_AT_TIMEOUT_SHORT, APP_EC_MAX_RETRY) != EC_OK)
        return EC_ERROR;

    // 关闭回显: 防止TX时模块同步回显导致LPUART RX溢出(ORE)死锁
    EC_SendCmd("ATE0\r\n", "OK", APP_AT_TIMEOUT_SHORT);

    if(EC_SendCmdRetry("AT+CPIN?\r\n", "READY", APP_AT_TIMEOUT_SHORT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] SIM not ready!\r\n");
        return EC_ERROR;
    }

    // SIM卡就绪，查询ICCID
    if(EC_SendCmd("AT+QCCID\r\n", "OK", APP_AT_TIMEOUT_SHORT) == EC_OK)
        printf_debug("[EC] %s\r\n", ec_rx_buf);
    else
        printf_debug("[EC] ICCID query failed\r\n");

    // 等待网络注册 (CFUN=1,1复位后需重新搜网, 通常5~20s)
    {
        uint8_t cereg_ok = 0;
        uint8_t i;
        for(i = 0; i < 10; i++)   // 最多等15s (10次×1.5s)
        {
            trea_delay_ms(1500);
            if(EC_SendCmd("AT+CEREG?\r\n", "OK", APP_AT_TIMEOUT_SHORT) == EC_OK)
            {
                // +CEREG: 0,1 (已注册本地) 或 +CEREG: 0,5 (已注册漫游)
                if(strstr(ec_rx_buf, ",1") || strstr(ec_rx_buf, ",5"))
                {
                    cereg_ok = 1;
                    printf_debug("[EC] Network registered (attempt %d)\r\n", i + 1);
                    break;
                }
            }
            printf_debug("[EC] CEREG waiting... (%d/10)\r\n", i + 1);
            swdt_feed();  // 喂狗: CEREG轮询可能持续~30s, 防止看门狗超时
        }
        if(!cereg_ok)
        {
            printf_debug("[EC] Network registration timeout!\r\n");
            return EC_ERROR;
        }
    }

    EC_SendCmd("AT+CSQ\r\n", "OK", APP_AT_TIMEOUT_SHORT);

    if(EC_SendCmdRetry("AT+CGACT=1,1\r\n", "OK", APP_AT_TIMEOUT_LONG, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] PDP activate failed!\r\n");
        return EC_ERROR;
    }

    printf_debug("[EC] Network ready\r\n");
    return EC_OK;
}

// ==================== 从4G模块同步网络时间到RTC ====================
EC_Status_t EC801E_SyncTime(void)
{
    uint8_t ch;
    uint16_t cnt;


    printf_debug("[EC] SyncTime start\r\n");

    // 1. 清空接收缓冲 + 环形缓冲(避免前序命令残留OK污染)
    ec_rx_len = 0;
    ec_rx_buf[0] = '\0';
    lpuart_rx_flush();

    // 2. 发送AT+CCLK?
    printf_cat1("AT+CCLK?\r\n");

    // 3. 读取应答（最多读200字节，每字节最长等30ms）
    //    RBNE在LPUART上可能不会被usart_data_receive清除，
    //    因此不能用"等到超时退出"的方式，必须用字节计数限制
    for(cnt = 0; cnt < 200; cnt++)
    {
        if(trea_uart_recv_byte_timeout(LPUART0, &ch, 30))
        {
            if(ec_rx_len < sizeof(ec_rx_buf) - 1)
            {
                ec_rx_buf[ec_rx_len++] = (char)ch;
                ec_rx_buf[ec_rx_len] = '\0';
            }
            // 找到完整的 OK 结尾就可以停了
            if(strstr(ec_rx_buf, "OK") != NULL)
                break;
        }
        else
        {
            // 30ms内无数据，如果已经有+CCLK:就够了
            if(strstr(ec_rx_buf, "+CCLK:") != NULL)
                break;
        }
    }

    printf_debug("[EC] CCLK raw(%d): %s\r\n", ec_rx_len, ec_rx_buf);

    // 4. 解析
    {
        char *p = strstr(ec_rx_buf, "+CCLK:");
        if(!p)
        {
            printf_debug("[EC] SyncTime: no +CCLK in response\r\n");
            return EC_ERROR;
        }

        if(rtc_set_from_cclk(p))
            return EC_OK;
    }

    return EC_ERROR;
}

// ==================== MQTT连接到指定服务器 ====================
static EC_Status_t mqtt_connect_to(const char *server, uint16_t port)
{
    // 共享 s_cmd_buf (模块级)

    // 断开旧连接
    printf_cat1("AT+QMTDISC=0\r\n");
    trea_delay_ms(500);
    printf_cat1("AT+QMTCLOSE=0\r\n");
    trea_delay_ms(700);

    // 打开MQTT连接
    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTOPEN=0,\"%s\",%d\r\n", server, port);
    if(EC_SendCmdRetry(s_cmd_buf, "+QMTOPEN: 0,0", APP_AT_TIMEOUT_MQTT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] MQTT open %s:%d failed!\r\n", server, port);
        return EC_ERROR;
    }

    trea_delay_ms(500);

    // 连接MQTT服务器
    // 注: QMTCONN的URC(+QMTCONN:)在LPUART上容易丢字节，
    //     改为先等OK确认命令已接受，再用QMTCONN?查询连接状态
    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n",
             g_mqtt.client_id, g_mqtt.username, g_mqtt.password);
    if(EC_SendCmd(s_cmd_buf, "OK", APP_AT_TIMEOUT_MQTT) != EC_OK)
    {
        printf_debug("[EC] MQTT CONN cmd rejected\r\n");
        return EC_ERROR;
    }

    // 等待服务器握手完成
    trea_delay_ms(4000);

    // 查询连接状态 (state=3 表示已连接)
    if(EC_SendCmd("AT+QMTCONN?\r\n", "+QMTCONN: 0,3", APP_AT_TIMEOUT_SHORT) == EC_OK)
    {
        printf_debug("[EC] MQTT connected to %s:%d\r\n", server, port);
        return EC_OK;
    }

    printf_debug("[EC] MQTT connect %s:%d failed (state != 3)\r\n", server, port);
    return EC_ERROR;
}

// ==================== MQTT连接（主→副自动切换） ====================
EC_Status_t EC801E_ConnectMQTT(void)
{
    printf_debug("[EC] ConnectMQTT (primary=%s:%d)\r\n", g_mqtt.server, g_mqtt.port);

    // 先尝试主IP
    if(mqtt_connect_to(g_mqtt.server, g_mqtt.port) == EC_OK)
        return EC_OK;

    // 主IP失败, 尝试副IP (如果已配置)
    if(g_mqtt.server2[0] != '\0')
    {
        printf_debug("[EC] Primary failed, trying secondary=%s:%d\r\n",
                     g_mqtt.server2, g_mqtt.port2);
        if(mqtt_connect_to(g_mqtt.server2, g_mqtt.port2) == EC_OK)
            return EC_OK;
    }

    printf_debug("[EC] ConnectMQTT: all servers failed\r\n");
    return EC_ERROR;
}

// ==================== MQTT断开 ====================
EC_Status_t EC801E_DisconnectMQTT(void)
{

    trea_delay_ms(300);
    EC_SendCmd("AT+QMTDISC=0\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    return EC_OK;
}

// ==================== 发布传感器数据 ====================
EC_Status_t EC801E_PublishSensorData(PT304D_Data *data, float voltage)
{

    if(!data) return EC_ERROR;

    rtc_get_time_str(s_timestamp, sizeof(s_timestamp));
    snprintf(s_payload, sizeof(s_payload),
        "[{\"id\":\"temp\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"temp2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"pressure\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"pressure2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
        data->temperature, s_timestamp,
        g_sensor.raw2.temperature, s_timestamp,
        data->pressure, s_timestamp,
        g_sensor.raw2.pressure, s_timestamp);

    printf_debug("[EC] Publish OK: P=%.2f T=%.2f P2=%.2f T2=%.2f\r\n",
                 data->pressure, data->temperature,
                 g_sensor.raw2.pressure, g_sensor.raw2.temperature);
    return EC801E_PublishRawJson(s_payload);
}

// ==================== 完整上报流程 ====================
EC_Status_t EC801E_FullUpload(PT304D_Data *data, float voltage)
{
    uint8_t retry;
    EC_Status_t ret;


    EC801E_SetPSM(0);
    trea_delay_ms(1000);

    printf_debug("[EC] === FullUpload start ===\r\n");

    for(retry = 0; retry < APP_EC_MAX_RETRY; retry++)
    {
        ret = EC801E_InitNetwork();
        if(ret != EC_OK)
        {
            printf_debug("[EC] InitNetwork fail, retry %d/%d\r\n", retry + 1, APP_EC_MAX_RETRY);
            trea_delay_ms(1000);
            continue;
        }

        ret = EC801E_ConnectMQTT();
        if(ret != EC_OK)
        {
            printf_debug("[EC] ConnectMQTT fail, retry %d/%d\r\n", retry + 1, APP_EC_MAX_RETRY);
            trea_delay_ms(1000);
            continue;
        }

        ret = EC801E_PublishSensorData(data, voltage);
        if(ret != EC_OK)
        {
            printf_debug("[EC] Publish fail, retry %d/%d\r\n", retry + 1, APP_EC_MAX_RETRY);
            EC801E_DisconnectMQTT();
            trea_delay_ms(1000);
            continue;
        }

        trea_delay_ms(1000);
        EC801E_DisconnectMQTT();
        trea_delay_ms(500);
        EC801E_SetPSM(1);

        printf_debug("[EC] === FullUpload success ===\r\n");
        return EC_OK;
    }

    printf_debug("[EC] === FullUpload FAILED after %d retries ===\r\n", APP_EC_MAX_RETRY);
    EC801E_DisconnectMQTT();
    return EC_ERROR;
}

// ==================== 模块休眠模式设置 ====================
// mode=0: 关闭休眠
// mode=1: QSCLK轻休眠(保持网络+MQTT, UART可唤醒)
// mode=3: PSM深休眠(脱网, T3412周期唤醒, UART可唤醒)
void EC801E_SetPSM(uint8_t mode)
{
    switch(mode)
    {
        case 3:
            // PSM: T3412=1分钟(10100001), T3324=20秒(00001010)
            EC_SendCmd("AT+CPSMS=1,,,\"10100001\",\"00001010\"\r\n", "OK", APP_AT_TIMEOUT_SHORT);
            EC_SendCmd("AT+QSCLK=3\r\n", "OK", APP_AT_TIMEOUT_SHORT);
            gpio_set_4g_dtr(1);  // 硬件拉高 DTR (PA0) 允许休眠
            printf_debug("[EC] PSM enabled (TAU=1min, Active=20s)\r\n");
            break;
        case 1:
			EC_SendCmd("AT+QSCLKEX=1,1,1\r\n", "OK", APP_AT_TIMEOUT_SHORT);
            gpio_set_4g_dtr(1);  // 硬件拉高 DTR (PA0) 允许 4G 模组在空闲 1s 后进入休眠
            printf_debug("[EC] QSCLKEX=1,1,1 enabled (DTR HIGH)\r\n");
            break;
        default:
            gpio_set_4g_dtr(0);  // 硬件拉低 DTR (PA0) 强制保持唤醒
            EC_SendCmd("AT+QSCLK=0\r\n", "OK", APP_AT_TIMEOUT_SHORT);
            EC_SendCmd("AT+CPSMS=0\r\n", "OK", APP_AT_TIMEOUT_SHORT);
            printf_debug("[EC] Sleep disabled (DTR LOW)\r\n");
            break;
    }
}

// ==================== 唤醒模块（纯引脚拉低，无指令无残留） ====================
void EC801E_Wakeup(void)
{
    gpio_set_4g_dtr(0);  // 硬件拉低 DTR (PA0) 强制唤醒模组
    trea_delay_ms(160);  // 硬件拉低后，等待300ms以便模组及串口硬件完全就绪稳定
	
    printf_debug("[EC] Module wakeup via DTR\r\n");
}

// ==================== MQTT订阅下发topic ====================
EC_Status_t EC801E_SubscribeTopic(void)
{

    // 共享 s_cmd_buf / s_topic (模块级, 此处 s_topic 复用为 topic_down)

    // 从上报topic推导订阅topic: /property/post → /function/get
    strncpy(s_topic, g_mqtt.topic_up, sizeof(s_topic) - 1);
    s_topic[sizeof(s_topic) - 1] = '\0';
    {
        char *p = strstr(s_topic, "/property/post");
        if(p) {
            memcpy(p, "/function/get", 13);
            *(p + 13) = '\0';
        }
    }

    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTSUB=0,1,\"%s\",0\r\n", s_topic);

    if(EC_SendCmdRetry(s_cmd_buf, "OK", APP_AT_TIMEOUT_MQTT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] Subscribe failed!\r\n");
        return EC_ERROR;
    }

    printf_debug("[EC] Subscribed: %s\r\n", s_topic);
    return EC_OK;
}

// ==================== 读取MQTT下发消息(主动查询) ====================
// LPUART唤醒时URC已丢失(system_wakeup_reinit禁用了LPUART)
// 因此使用AT+QMTRECV主动查询模块缓存的消息(需配合recv/mode=1)
EC_Status_t EC801E_ReadDownlink(char *payload_buf, uint16_t buf_size)
{
    char *json;
    char *json_end;
    char end_ch;
    uint16_t len;

    payload_buf[0] = '\0';

    // 唤醒模块(QSCLK=2下首字节可能丢失)
    EC801E_Wakeup();

    // 清空环形缓冲中的残留字节(Wakeup AT响应等)
    lpuart_rx_flush();

    // 主动查询缓存的下发消息
    // (LPUART已改为中断+环形缓冲, 不再有轮询丢字节/ORE问题)
    if(EC_SendCmd("AT+QMTRECV=0\r\n", "\nOK", APP_AT_TIMEOUT_MQTT) != EC_OK)
    {
        printf_debug("[EC] No buffered downlink\r\n");
        return EC_TIMEOUT;
    }

    printf_debug("[EC] QMTRECV(%d): %s\r\n", ec_rx_len, ec_rx_buf);

    // 检查是否有+QMTRECV:消息头(无则表示无缓存消息)
    if(!strstr(ec_rx_buf, "+QMTRECV:"))
    {
        printf_debug("[EC] No buffered downlink\r\n");
        return EC_TIMEOUT;
    }

    // 提取JSON payload: 定位+QMTRECV响应中payload的起始引号
    // 格式: +QMTRECV: 0,0,"<topic>","<payload>"
    // payload可能是 [...] (格式A) 或 {...} (格式B含commandStr)
    // 必须跳过topic字段, 从payload起始位置提取, 否则commandParms中的[会被误取
    {
        char *recv = strstr(ec_rx_buf, "+QMTRECV:");
        char *p;
        if(!recv) { printf_debug("[EC] No +QMTRECV header\r\n"); return EC_ERROR; }
        // 跳到第1个"(topic开始), 第2个"(topic结束), 第3个"(payload开始)
        p = strchr(recv, '"');                       // 1st " : topic start
        if(p) p = strchr(p + 1, '"');                // 2nd " : topic end
        if(p) p = strchr(p + 1, '"');                // 3rd " : payload start
        if(!p) { printf_debug("[EC] No JSON in downlink\r\n"); return EC_ERROR; }
        json = p + 1;  // payload首字符 ( [ 或 { )
    }

    end_ch = (*json == '[') ? ']' : '}';
    json_end = strrchr(json, end_ch);
    if(!json_end) { printf_debug("[EC] Incomplete JSON\r\n"); return EC_ERROR; }

    len = (uint16_t)(json_end - json + 1);
    if(len >= buf_size) len = buf_size - 1;
    memcpy(payload_buf, json, len);
    payload_buf[len] = '\0';

    printf_debug("[EC] Downlink: %s\r\n", payload_buf);
    return EC_OK;
}

// ==================== 确保MQTT已连接（QSCLK=1模式，断线自动重连） ====================
EC_Status_t EC801E_EnsureConnected(void)
{
    EC801E_Wakeup();

	
    // 查询MQTT连接状态 (state=3表示已连接)
    if(EC_SendCmd("AT+QMTCONN?\r\n", "+QMTCONN: 0,3", 5000) == EC_OK)
    {
        printf_debug("[EC] MQTT still connected\r\n");
        return EC_OK;
    }

    // 重连前先检查网络注册状态, 未入网则无需尝试MQTT
    {
        uint8_t cereg_stat = 0;
        if(EC801E_QueryCEREG(&cereg_stat) == EC_OK)
        {
            if(cereg_stat != 1 && cereg_stat != 5)
            {
                printf_debug("[EC] MQTT reconnect aborted: not registered (CEREG=%d)\r\n", cereg_stat);
                return EC_ERROR;
            }
        }
    }

    printf_debug("[EC] MQTT disconnected, reconnecting...\r\n");
    return EC801E_InitAndConnect();
}

// ==================== 首次初始化+连接+订阅+QSCLK ====================
EC_Status_t EC801E_InitAndConnect(void)
{
    uint8_t retry;
    EC_Status_t ret;

    EC801E_SetPSM(0);
    printf_debug("[EC] === InitAndConnect ===\r\n");

    // 软件复位模块射频+协议栈, 清除残留TCP/MQTT/PDP状态
    EC_SendCmd("AT+CFUN=1,1\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    trea_delay_ms(5000);  // 等待模块复位完成

    // 确认模块已就绪 (复位后AT可能需数秒才响应)
    if(EC_SendCmdRetry("AT\r\n", "OK", APP_AT_TIMEOUT_SHORT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] Module not responding after CFUN reset\r\n");
        return EC_ERROR;
    }

    for(retry = 0; retry < 1; retry++)
    {
        ret = EC801E_InitNetwork();
        if(ret != EC_OK) { trea_delay_ms(1000); continue; }

        // 设置keepalive (从eeprom_config或默认值)
        {
            extern uint16_t config_get_keepalive(void);
            uint16_t ka = config_get_keepalive();
            snprintf(s_cmd_buf, sizeof(s_cmd_buf), "AT+QMTCFG=\"keepalive\",0,%d\r\n", ka);
            EC_SendCmd(s_cmd_buf, "OK", APP_AT_TIMEOUT_SHORT);
        }

        // 消息缓存模式: 模块收到下发后缓存, MCU用AT+QMTRECV主动查询
        EC_SendCmd("AT+QMTCFG=\"recv/mode\",0,1,0\r\n", "OK", APP_AT_TIMEOUT_SHORT);

        ret = EC801E_ConnectMQTT();
        if(ret != EC_OK) { trea_delay_ms(1000); continue; }

        trea_delay_ms(500);

        // 订阅下发topic
        EC801E_SubscribeTopic();

        // 订阅升级 topic (FastBee /upgrade/set)
        trea_delay_ms(300);
        EC801E_SubscribeUpgradeTopic();

        // 同步时间
        trea_delay_ms(300);
        EC801E_SyncTime();

        // 启用QSCLK (模块自动休眠, 保持网络+MQTT)
        trea_delay_ms(300);
        EC801E_SetPSM(1);

        printf_debug("[EC] === Connected & QSCLK=1 ===\r\n");
        return EC_OK;
    }

    printf_debug("[EC] === InitAndConnect FAILED ===\r\n");
    return EC_ERROR;
}

// ==================== 发布单条历史记录 ====================
EC_Status_t EC801E_PublishSingleRecord(const SensorRecord_t *record)
{

    if(!record) return EC_ERROR;

    snprintf(s_payload, sizeof(s_payload),
        "[{\"id\":\"temp\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"temp2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"pressure\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"pressure2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
        record->temperature, record->timestamp,
        record->temperature2, record->timestamp,
        record->pressure, record->timestamp,
        record->pressure2, record->timestamp);

    printf_debug("[EC] Record: P=%.2f T=%.2f P2=%.2f T2=%.2f @%s\r\n",
                 record->pressure, record->temperature,
                 record->pressure2, record->temperature2,
                 record->timestamp);
    return EC801E_PublishRawJson(s_payload);
}

// ==================== 发布报警状态 (统一格式) ====================
EC_Status_t EC801E_PublishAlarmStatus(ThresholdResult_t result, float voltage)
{


    const char *id;
    const char *value;
    uint8_t is_voltage_alarm = 0;

    switch(result)
    {
        case THRESHOLD_VPROTECT:
        case THRESHOLD_VOLT_HIGH:
        case THRESHOLD_VOLT_LOW:
            id = "LY06"; value = "1"; is_voltage_alarm = 1; break;

        case THRESHOLD_OVER_LIMIT:
            id = "pressure_status"; value = "pressure_high"; break;
        case THRESHOLD_UNDER_LIMIT:
            id = "pressure_status"; value = "pressure_low"; break;
        case THRESHOLD_VERY_HIGH:
            id = "pressure_status"; value = "pressure_veryhigh"; break;
        case THRESHOLD_VERY_LOW:
            id = "pressure_status"; value = "pressure_verylow"; break;
        case THRESHOLD_MUTATION:
            id = "pressure_status"; value = "pressure_mutation"; break;

        case THRESHOLD_TEMP_HIGH:
            id = "temp_status"; value = "temp_high"; break;
        case THRESHOLD_TEMP_LOW:
            id = "temp_status"; value = "temp_low"; break;
        case THRESHOLD_TEMP_VERY_HIGH:
            id = "temp_status"; value = "temp_very_high"; break;
        case THRESHOLD_TEMP_VERY_LOW:
            id = "temp_status"; value = "temp_very_low"; break;

        case THRESHOLD_OK:
        default:
            id = "LY06"; value = "0"; is_voltage_alarm = 1; break;
    }

    rtc_get_time_str(s_timestamp, sizeof(s_timestamp));

    if(is_voltage_alarm)
    {
        snprintf(s_payload, sizeof(s_payload),
            "[{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
             "{\"id\":\"voltage\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
            id, value, s_timestamp, voltage, s_timestamp);
    }
    else
    {
        snprintf(s_payload, sizeof(s_payload),
            "[{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            id, value, s_timestamp);
    }

    printf_debug("[EC] ALARM status: id=%s value=%s V=%.2f\r\n", id, value, voltage);
    return EC801E_PublishRawJson(s_payload);
}

// ==================== 发布下发响应 ====================
EC_Status_t EC801E_PublishResponse(const char *id, const char *value)
{

    rtc_get_time_str(s_timestamp, sizeof(s_timestamp));
    snprintf(s_payload, sizeof(s_payload),
        "[{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", id, value, s_timestamp);
    printf_debug("[EC] Response: %s=%s\r\n", id, value);
    return EC801E_PublishRawJson(s_payload);
}

// ==================== 发布阀门状态 (合并 valve_control + valve_last_order) ====================
EC_Status_t EC801E_PublishValveStatus(const char *valve_result, int last_order)
{

    char v_order[4];
    snprintf(v_order, sizeof(v_order), "%d", last_order);
    rtc_get_time_str(s_timestamp, sizeof(s_timestamp));
    snprintf(s_payload, sizeof(s_payload),
        "[{\"id\":\"valve_control\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"valve_last_order\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
        valve_result, s_timestamp, v_order, s_timestamp);
    printf_debug("[EC] ValveStatus: %s order=%s\r\n", valve_result, v_order);
    return EC801E_PublishRawJson(s_payload);
}

// ==================== 发布任意JSON字符串 ====================
EC_Status_t EC801E_PublishRawJson(const char *json_str)
{
    // 共享 s_cmd_buf (模块级); json_str 可指向 s_payload (本函数仅写 s_cmd_buf)
    uint16_t payload_len;


    if(!json_str || json_str[0] == '\0') return EC_ERROR;

    payload_len = (uint16_t)strlen(json_str);

    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", g_mqtt.topic_up, payload_len);

    if(ec_pubex_send(s_cmd_buf, json_str, payload_len) != EC_OK)
        return EC_TIMEOUT;

    printf_debug("[EC] RawJson published (%d bytes)\r\n", payload_len);
    return EC_OK;
}

// ==================== 查询信号强度 ====================
EC_Status_t EC801E_QueryCSQ(char *out, uint16_t size)
{

	trea_delay_ms(130);
    // 响应: +CSQ: 22,99\r\nOK  (等OK确保完整响应)
    if(EC_SendCmd("AT+CSQ\r\n", "OK", APP_AT_TIMEOUT_SHORT) != EC_OK)
        return EC_ERROR;

    char *p = strstr(ec_rx_buf, "+CSQ: ");
    if(p)
    {
        p += 6;
        uint16_t i = 0;
        while(*p && *p != ',' && *p != '\r' && i < size - 1)
            out[i++] = *p++;
        out[i] = '\0';
        return EC_OK;
    }
    return EC_ERROR;
}

// ==================== 查询LTE信号质量 (RSRP/SINR) ====================
// 响应: +QCSQ: "LTE",-65,-93,15,-10  (rssi,rsrp,sinr,rsrq)
// out_rsrp: dBm (覆盖, 典型 -44~-140, <-110弱)
// out_sinr: dB  (信噪比, 典型 -20~30, <0差)
// 返回: EC_OK=解析成功
EC_Status_t EC801E_QueryQCSQ(int *out_rsrp, int *out_sinr)
{
    if(EC_SendCmd("AT+QCSQ\r\n", "OK", APP_AT_TIMEOUT_SHORT) != EC_OK)
        return EC_ERROR;

    // +QCSQ: "LTE",rssi,rsrp,sinr,rsrq
    char *p = strstr(ec_rx_buf, "+QCSQ:");
    if(!p) return EC_ERROR;

    // 跳到 "LTE" 后面的逗号分隔数值
    p = strstr(p, "\"LTE\"");
    if(!p) return EC_ERROR;
    p += 5;  // skip "LTE"

    // 解析: ,rssi,rsrp,sinr,rsrq
    int rssi = 0, rsrp = 0, sinr = 0, rsrq = 0;
    if(sscanf(p, ",%d,%d,%d,%d", &rssi, &rsrp, &sinr, &rsrq) >= 3)
    {
        if(out_rsrp) *out_rsrp = rsrp;
        if(out_sinr) *out_sinr = sinr;
        printf_debug("[EC] QCSQ: RSSI=%d RSRP=%d SINR=%d RSRQ=%d\r\n", rssi, rsrp, sinr, rsrq);
        return EC_OK;
    }
    return EC_ERROR;
}

// ==================== 查询网络注册状态 ====================
// 返回: 注册状态 (0=未注册, 1=已注册本地, 2=搜网中, 3=被拒, 5=漫游)
// out_stat: 输出注册状态值
// 返回: EC_OK=查询成功
EC_Status_t EC801E_QueryCEREG(uint8_t *out_stat)
{
    if(EC_SendCmd("AT+CEREG?\r\n", "OK", APP_AT_TIMEOUT_SHORT) != EC_OK)
        return EC_ERROR;

    // +CEREG: <n>,<stat>
    char *p = strstr(ec_rx_buf, "+CEREG:");
    if(!p) return EC_ERROR;

    // 找第一个逗号后的数字
    p = strchr(p, ',');
    if(!p) return EC_ERROR;
    p++;
    uint8_t stat = (uint8_t)atoi(p);
    if(out_stat) *out_stat = stat;
    printf_debug("[EC] CEREG stat=%d (%s)\r\n", stat,
        (stat == 1) ? "home" : (stat == 5) ? "roaming" :
        (stat == 2) ? "searching" : (stat == 3) ? "denied" : "not reg");
    return EC_OK;
}

// ==================== 查询SIM卡ICCID ====================
EC_Status_t EC801E_QueryICCID(char *out, uint16_t size)
{

    // 响应: +QCCID: 898604B6222280037080\r\nOK  (等OK确保完整响应)
    if(EC_SendCmd("AT+QCCID\r\n", "OK", APP_AT_TIMEOUT_SHORT) != EC_OK)
        return EC_ERROR;

    char *p = strstr(ec_rx_buf, "+QCCID: ");
    if(p)
    {
        p += 8;
        uint16_t i = 0;
        while(*p && *p != '\r' && *p != '\n' && i < size - 1)
            out[i++] = *p++;
        out[i] = '\0';
        return EC_OK;
    }
    return EC_ERROR;
}

// ==================== OTA: 推导 upgrade/set + upgrade/reply topic ====================
// 上报 topic: /<productId>/<deviceMac>/property/post
// 升级下发: /<productId>/<deviceMac>/upgrade/set
// 状态回执: /<productId>/<deviceMac>/upgrade/reply
static void ec_derive_upgrade_topic(const char *suffix, char *out, uint16_t out_size)
{
    char *p;

    out[0] = '\0';
    strncpy(out, g_mqtt.topic_up, out_size - 1);
    out[out_size - 1] = '\0';

    /* 把末尾的 /property/post 替换为 /upgrade/<suffix> */
    p = strstr(out, "/property/post");
    if(p)
    {
        /* 截断到 /property/post 之前 */
        *p = '\0';
        strncat(out, "/upgrade/", out_size - strlen(out) - 1);
        strncat(out, suffix, out_size - strlen(out) - 1);
    }
    else
    {
        /* 兜底: 直接追加 (不太可能) */
        strncat(out, "/upgrade/", out_size - strlen(out) - 1);
        strncat(out, suffix, out_size - strlen(out) - 1);
    }
}

// ==================== OTA: 订阅升级下发 topic ====================
EC_Status_t EC801E_SubscribeUpgradeTopic(void)
{

    // 共享 s_topic / s_cmd_buf (模块级)

    ec_derive_upgrade_topic("set", s_topic, sizeof(s_topic));

    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTSUB=0,2,\"%s\",1\r\n", s_topic);

    if(EC_SendCmdRetry(s_cmd_buf, "OK", APP_AT_TIMEOUT_MQTT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] Subscribe upgrade FAILED\r\n");
        return EC_ERROR;
    }
    printf_debug("[EC] Subscribed upgrade: %s\r\n", s_topic);
    return EC_OK;
}

// ==================== OTA: 升级状态回执 ====================
EC_Status_t EC801E_PublishUpgradeReply(int reply_code)
{

    // 共享 s_topic / s_payload / s_cmd_buf (模块级)
    uint16_t payload_len;

    ec_derive_upgrade_topic("reply", s_topic, sizeof(s_topic));

    snprintf(s_payload, sizeof(s_payload),
             "{\"mqttUpgradeReply\":%d}", reply_code);
    payload_len = (uint16_t)strlen(s_payload);

    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", s_topic, payload_len);

    if(ec_pubex_send(s_cmd_buf, s_payload, payload_len) != EC_OK)
    {
        printf_debug("[EC] UpgradeReply (%d) timeout\r\n", reply_code);
        return EC_TIMEOUT;
    }
    printf_debug("[EC] UpgradeReply: %d\r\n", reply_code);
    return EC_OK;
}

// ==================== OTA: HTTP 下载到 Flash ====================
// 流程:
//   AT+QHTTPCFG="contextid",1
//   AT+QHTTPCFG="responseheader",0
//   AT+QIACT=1                         (尝试激活, 已激活则忽略 ERROR)
//   AT+QHTTPURL=<len>,60               → CONNECT, 写入 url, 等 OK
//   AT+QHTTPGET=60                     → +QHTTPGET: 0,200,<content_length>
//   AT+QHTTPREAD=60                    → CONNECT + body + OK/+QHTTPREAD: 0
// 期间从 LPUART 直接读字节, 4 字节凑齐就 fmc_word_program 写入
EC_Status_t EC801E_HttpDownloadToFlash(const char *url,
                                       uint32_t flash_addr,
                                       uint32_t max_size,
                                       uint32_t *actual_size)
{

    // 共享 s_cmd_buf (模块级)
    uint32_t url_len;
    uint32_t content_length = 0;
    uint32_t received = 0;
    uint32_t cur_addr = flash_addr;
    uint32_t word_buf;
    uint8_t  word_pos;
    uint8_t  ch;
    uint32_t tail_idx;
    char *p;
    fmc_state_enum st;

    if(actual_size) *actual_size = 0;
    if(!url || flash_addr == 0 || max_size == 0)
        return EC_ERROR;

    url_len = (uint32_t)strlen(url);
    printf_debug("[EC] HTTP download: %s (max %u B)\r\n",
                 url, (unsigned int)max_size);

    /* === 配置 === */
    EC_SendCmd("AT+QHTTPCFG=\"contextid\",1\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    EC_SendCmd("AT+QHTTPCFG=\"responseheader\",0\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    /* 激活 PDP context (CGACT=1,1 已激活时此处可能 ERROR, 忽略) */
    EC_SendCmd("AT+QIACT=1\r\n", "OK", APP_AT_TIMEOUT_LONG);

    /* === 写 URL === */
    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QHTTPURL=%u,60\r\n", (unsigned int)url_len);
    if(EC_SendCmd(s_cmd_buf, "CONNECT", APP_AT_TIMEOUT_LONG) != EC_OK)
    {
        printf_debug("[EC] QHTTPURL no CONNECT\r\n");
        return EC_ERROR;
    }
    /* 模块进入数据模式, 直接发 url 字节 */
    serial_write(LPUART0, (const uint8_t*)url, (uint16_t)url_len);
    lpuart_rx_recover();
    /* 等模块吃完返回 OK */
    {
        uint32_t t0 = trea_millis();
        ec_rx_len = 0;
        ec_rx_buf[0] = '\0';
        while((trea_millis() - t0) < 5000)
        {
            if(trea_uart_recv_byte_timeout(LPUART0, &ch, 30))
            {
                if(ec_rx_len < sizeof(ec_rx_buf) - 1)
                {
                    ec_rx_buf[ec_rx_len++] = (char)ch;
                    ec_rx_buf[ec_rx_len] = '\0';
                }
                if(strstr(ec_rx_buf, "OK")) break;
            }
        }
    }

    /* === 发起 GET, 等待 +QHTTPGET: 0,<httprsp>,<length> URC === */
    if(EC_SendCmd("AT+QHTTPGET=60\r\n", "+QHTTPGET:", 65000) != EC_OK)
    {
        printf_debug("[EC] QHTTPGET no URC\r\n");
        return EC_ERROR;
    }
    /* EC_SendCmd 匹配到 "+QHTTPGET:" 瞬间返回, 行尾数字可能尚未收完
     * 继续读取直到换行, 确保 sscanf 能解析完整参数 */
    {
        uint8_t tail_ch;
        uint32_t drain_t = trea_millis();
        while(ec_rx_len < sizeof(ec_rx_buf) - 1 && (trea_millis() - drain_t) < 2000)
        {
            if(trea_uart_recv_byte_timeout(LPUART0, &tail_ch, 100))
            {
                ec_rx_buf[ec_rx_len++] = (char)tail_ch;
                ec_rx_buf[ec_rx_len] = '\0';
                if(tail_ch == '\n') break;
            }
            else break;
        }
    }
    /* 解析 +QHTTPGET: <err>,<httprsp>,<content_length> */
    p = strstr(ec_rx_buf, "+QHTTPGET:");
    if(p)
    {
        int err_code = -1, http_code = -1;
        unsigned int clen = 0;
        if(sscanf(p, "+QHTTPGET: %d,%d,%u",
                  &err_code, &http_code, &clen) >= 2)
        {
            printf_debug("[EC] HTTP err=%d code=%d len=%u\r\n",
                         err_code, http_code, clen);
            if(err_code != 0 || (http_code != 200 && http_code != -1))
                return EC_ERROR;
            content_length = clen;
        }
    }
    if(content_length == 0 || content_length > max_size)
    {
        printf_debug("[EC] bad content_length: %u\r\n", (unsigned int)content_length);
        return EC_ERROR;
    }

    /* === 准备 Flash 写入 === */
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    /* === 启动 READ === */
    if(EC_SendCmd("AT+QHTTPREAD=60\r\n", "CONNECT", APP_AT_TIMEOUT_LONG) != EC_OK)
    {
        printf_debug("[EC] QHTTPREAD no CONNECT\r\n");
        fmc_lock();
        return EC_ERROR;
    }
    /* EC_SendCmd 匹配 "CONNECT" 后立即返回, 但模块实际发送的是 "CONNECT\r\n<data>"
     * 必须消费掉 \r\n, 否则这2字节会被当作固件头部写入 Flash 导致数据偏移 */
    {
        uint8_t skip_ch;
        uint32_t skip_t = trea_millis();
        while((trea_millis() - skip_t) < 500)
        {
            if(trea_uart_recv_byte_timeout(LPUART0, &skip_ch, 200))
            {
                if(skip_ch == '\n') break;  /* \r\n 已消费完毕 */
            }
            else break;
        }
    }

    /* === 流式接收 + 写 Flash === */
    word_buf = 0xFFFFFFFFU;
    word_pos = 0;
    received = 0;
    {
        uint32_t last_byte_time = trea_millis();
        while(received < content_length)
        {
            ota_wdt_feed();  /* 喂狗保护：长连接流式分包接收中持续喂狗 */
            if(trea_uart_recv_byte_timeout(LPUART0, &ch, 100))
            {
                last_byte_time = trea_millis();
                /* 累积到 32-bit word */
                ((uint8_t *)&word_buf)[word_pos++] = ch;
                received++;

                if(word_pos == 4)
                {
                    st = fmc_word_program(cur_addr, word_buf);
                    if(st != FMC_READY)
                    {
                        fmc_lock();
                        printf_debug("[EC] flash prog fail @0x%08X (%d)\r\n",
                                     (unsigned int)cur_addr, (int)st);
                        if(actual_size) *actual_size = received;
                        return EC_ERROR;
                    }
                    cur_addr += 4;
                    word_buf = 0xFFFFFFFFU;
                    word_pos = 0;

                    /* 进度日志: 每 4KB 一次 */
                    if((received & 0x0FFF) == 0)
                    {
                        printf_debug("[EC] dl %u/%u\r\n",
                                     (unsigned int)received,
                                     (unsigned int)content_length);
                    }
                }
            }
            else
            {
                /* 30s 无数据则视为下载中断 */
                if((trea_millis() - last_byte_time) > 30000)
                {
                    fmc_lock();
                    printf_debug("[EC] dl timeout @%u/%u\r\n",
                                 (unsigned int)received,
                                 (unsigned int)content_length);
                    if(actual_size) *actual_size = received;
                    return EC_TIMEOUT;
                }
            }
        }
    }

    /* 末尾不足 4 字节, 补 0xFF 写入 */
    if(word_pos != 0)
    {
        while(word_pos < 4)
            ((uint8_t *)&word_buf)[word_pos++] = 0xFF;
        st = fmc_word_program(cur_addr, word_buf);
        if(st != FMC_READY)
        {
            fmc_lock();
            printf_debug("[EC] flash tail prog fail (%d)\r\n", (int)st);
            return EC_ERROR;
        }
        cur_addr += 4;
    }
    fmc_lock();

    /* === 读模块尾部响应: +QHTTPREAD: 0 / OK === */
    {
        uint32_t t0 = trea_millis();
        tail_idx = 0;
        ec_rx_buf[0] = '\0';
        while((trea_millis() - t0) < 5000 && tail_idx < sizeof(ec_rx_buf) - 1)
        {
            if(trea_uart_recv_byte_timeout(LPUART0, &ch, 50))
            {
                ec_rx_buf[tail_idx++] = (char)ch;
                ec_rx_buf[tail_idx] = '\0';
                if(strstr(ec_rx_buf, "+QHTTPREAD: 0") ||
                   strstr(ec_rx_buf, "\r\nOK\r\n"))
                    break;
            }
        }
        ec_rx_len = (uint16_t)tail_idx;
    }

    if(actual_size) *actual_size = received;
    printf_debug("[EC] HTTP download done: %u bytes\r\n", (unsigned int)received);
    return (received == content_length) ? EC_OK : EC_ERROR;
}

// ==================== 查询模块IMEI ====================
EC_Status_t EC801E_QueryIMEI(char *out, uint16_t size)
{

    // AT+GSN 响应: 直接返回IMEI号
    if(EC_SendCmd("AT+GSN\r\n", "OK", APP_AT_TIMEOUT_SHORT) != EC_OK)
        return EC_ERROR;

    // 跳过AT echo(已关闭)，找第一行数字
    char *p = ec_rx_buf;
    while(*p && (*p == '\r' || *p == '\n')) p++;
    uint16_t i = 0;
    while(*p && *p != '\r' && *p != '\n' && i < size - 1)
        out[i++] = *p++;
    out[i] = '\0';
    return (i > 0) ? EC_OK : EC_ERROR;
}

// ==================== 4G 开关状态上报 ====================
void report_4g_power_state(uint8_t on)
{
    char val[2];
    val[0] = on ? '1' : '0';
    val[1] = '\0';
    if(EC801E_EnsureConnected() == EC_OK)
        EC801E_PublishResponse("4Gpower_control", val);
}

// ==================== 用户端MQTT (id=1, 双实例) ====================

// 判断用户端MQTT是否已配置 (server + client_id 不为空)
uint8_t EC801E_UserMqtt_IsConfigured(void)
{
    return (g_user_mqtt.server[0] != '\0' && g_user_mqtt.client_id[0] != '\0') ? 1 : 0;
}

// 连接用户端MQTT服务器 (id=1)
EC_Status_t EC801E_UserMqtt_Connect(void)
{

    if(!EC801E_UserMqtt_IsConfigured())
    {
        printf_debug("[EC] UserMqtt: not configured, skip\r\n");
        return EC_ERROR;
    }

    EC801E_Wakeup();

    // 前置检查: 主平台MQTT(id=0)是否已连接, 未连接说明网络不可用, 跳过无谓重试
    if(EC_SendCmd("AT+QMTCONN?\r\n", "+QMTCONN: 0,3", 3000) != EC_OK)
    {
        printf_debug("[EC] UserMqtt: main MQTT not connected, skip\r\n");
        return EC_ERROR;
    }
	trea_delay_ms(300);
    // keepalive 配置
    snprintf(s_cmd_buf, sizeof(s_cmd_buf), "AT+QMTCFG=\"keepalive\",1,600\r\n");
    EC_SendCmd(s_cmd_buf, "OK", APP_AT_TIMEOUT_SHORT);
	trea_delay_ms(300);
    // recv/mode: URC模式(用户端只发布不订阅, 无需主动查询)
    EC_SendCmd("AT+QMTCFG=\"recv/mode\",1,0,1\r\n", "OK", APP_AT_TIMEOUT_SHORT);

	trea_delay_ms(300);
    // 打开MQTT连接 id=1
    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTOPEN=1,\"%s\",%d\r\n", g_user_mqtt.server, g_user_mqtt.port);
    if(EC_SendCmdRetry(s_cmd_buf, "+QMTOPEN: 1,0", APP_AT_TIMEOUT_MQTT, APP_EC_MAX_RETRY) != EC_OK)
    {
        printf_debug("[EC] UserMqtt: open %s:%d failed\r\n", g_user_mqtt.server, g_user_mqtt.port);
        return EC_ERROR;
    }

    trea_delay_ms(300);

    // 连接MQTT
    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTCONN=1,\"%s\",\"%s\",\"%s\"\r\n",
             g_user_mqtt.client_id, g_user_mqtt.username, g_user_mqtt.password);
    if(EC_SendCmd(s_cmd_buf, "OK", APP_AT_TIMEOUT_MQTT) != EC_OK)
    {
        printf_debug("[EC] UserMqtt: CONN cmd rejected\r\n");
        return EC_ERROR;
    }

    // 轮询等待握手完成 (最多5s, 每1s查一次)
    {
        uint8_t poll;
        for(poll = 0; poll < 5; poll++)
        {
            trea_delay_ms(1000);
            if(EC_SendCmd("AT+QMTCONN?\r\n", "+QMTCONN: 1,3", APP_AT_TIMEOUT_SHORT) == EC_OK)
            {
                printf_debug("[EC] UserMqtt: connected to %s:%d (poll %d)\r\n",
                             g_user_mqtt.server, g_user_mqtt.port, poll + 1);
                return EC_OK;
            }
        }
    }

    printf_debug("[EC] UserMqtt: connect failed after 5s polling\r\n");
    return EC_ERROR;
}

// 断开用户端MQTT (id=1)
EC_Status_t EC801E_UserMqtt_Disconnect(void)
{

    EC_SendCmd("AT+QMTDISC=1\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    trea_delay_ms(300);
    EC_SendCmd("AT+QMTCLOSE=1\r\n", "OK", APP_AT_TIMEOUT_SHORT);
    printf_debug("[EC] UserMqtt: disconnected\r\n");
    return EC_OK;
}

// ==================== 内部: LY格式 payload 构建 ====================
// 将9个字段格式化为 {"param":[...]} JSON, 写入 s_payload
// LY01=进口压力(kPa) LY02=进口温度 LY03=电池电压 LY04=探头故障
// LY05=信号强度 LY06=电压报警 LY07=上报周期(min) LY08=出口压力(kPa) LY11=出口温度
static void ly_build_payload(float p1_kpa, float t1, float voltage,
                             int fault, const char *csq, int volt_alarm,
                             uint32_t report_min, float p2_kpa, float t2,
                             const char *ts)
{
//    snprintf(s_payload, sizeof(s_payload),
//        "{\"param\":["
//        "{\"deviceKey\":\"%s\",\"key\":\"LY01\",\"value\":\"%.2f\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY02\",\"value\":\"%.2f\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY03\",\"value\":\"%.2f\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY04\",\"value\":\"%d\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY05\",\"value\":\"%s\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY06\",\"value\":\"%d\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY07\",\"value\":\"%u\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY08\",\"value\":\"%.2f\",\"ts\":\"%s\"},"
//        "{\"deviceKey\":\"%s\",\"key\":\"LY11\",\"value\":\"%.2f\",\"ts\":\"%s\"}"
//        "]}",
//        g_user_mqtt.client_id, p1_kpa, ts,
//        g_user_mqtt.client_id, t1, ts,
//        g_user_mqtt.client_id, voltage, ts,
//        g_user_mqtt.client_id, fault, ts,
//        g_user_mqtt.client_id, csq, ts,
//        g_user_mqtt.client_id, volt_alarm, ts,
//        g_user_mqtt.client_id, (unsigned int)report_min, ts,
//        g_user_mqtt.client_id, p2_kpa, ts,
//        g_user_mqtt.client_id, t2, ts);

snprintf(s_payload, sizeof(s_payload),
         "[{\"id\":\"temp\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"temp2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"pressure\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"LY04\",\"value\":\"%d\",\"ts\":\"%s\"},"
         "{\"id\":\"csq\",\"value\":\"%s\",\"ts\":\"%s\"},"
         "{\"id\":\"LY06\",\"value\":\"%d\",\"ts\":\"%s\"},"
         "{\"id\":\"LY07\",\"value\":\"%u\",\"ts\":\"%s\"},"
         "{\"id\":\"pressure2\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"voltage\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
        t1, ts,
        t2, ts,
        p1_kpa, ts,
		fault,ts,
		csq,ts,
		volt_alarm,ts,
		(unsigned int)report_min,ts,
        p2_kpa, ts,
        voltage, ts);
}

// ==================== 内部: 用户端MQTT发布通用发送 ====================
static EC_Status_t umqtt_publish_payload(void)
{
    uint16_t payload_len = (uint16_t)strlen(s_payload);

    snprintf(s_cmd_buf, sizeof(s_cmd_buf),
             "AT+QMTPUBEX=1,0,0,0,\"%s\",%d\r\n", g_user_mqtt.topic_up, payload_len);

    if(ec_pubex_send(s_cmd_buf, s_payload, payload_len) != EC_OK)
    {
        printf_debug("[EC] UserMqtt: publish failed\r\n");
        return EC_TIMEOUT;
    }
    return EC_OK;
}

// 发布用户端传感器数据 (LY格式, 实时)
// JSON格式: {"param":[{"deviceKey":"xxx","key":"LYxx","value":"xxx","ts":"xxx"},…]}
EC_Status_t EC801E_UserMqtt_PublishSensorData(PT304D_Data *inlet, PT304D_Data *outlet, float voltage)
{
    char csq_buf[8] = {0};
    uint32_t report_min;

    if(!EC801E_UserMqtt_IsConfigured())
        return EC_ERROR;

    // 获取信号强度
    EC801E_QueryCSQ(csq_buf, sizeof(csq_buf));

    // 上报周期(分钟)
    report_min = cycle_to_seconds(&g_report_cycle) / 60;
    if(report_min == 0) report_min = 1;

    // 获取时间戳
    rtc_get_time_str(s_timestamp, sizeof(s_timestamp));

    {
        float p1_kpa = (inlet && inlet->err == PT304D_OK) ? (inlet->pressure / 1000.0f) : 0.0f;
        float t1 = (inlet && inlet->err == PT304D_OK) ? inlet->temperature : 0.0f;
        float p2_kpa = (outlet && outlet->err == PT304D_OK) ? (outlet->pressure / 1000.0f) : 0.0f;
        float t2 = (outlet && outlet->err == PT304D_OK) ? outlet->temperature : 0.0f;
        int fault = 0;
        int volt_alarm = threshold_voltage_alarm(voltage);

        if(inlet && inlet->err != PT304D_OK) fault |= 1;
        if(outlet && outlet->err != PT304D_OK) fault |= 2;

        ly_build_payload(p1_kpa, t1, voltage, fault, csq_buf,
                         volt_alarm, report_min, p2_kpa, t2, s_timestamp);
    }

    printf_debug("[EC] UserMqtt: published %d bytes\r\n", (int)strlen(s_payload));
    return umqtt_publish_payload();
}

// ==================== 用户端MQTT: 发布单条历史记录 (LY格式) ====================
// 将 SensorRecord_t 转为 LY 格式 JSON 并通过 id=1 发布, 使用记录自身的 timestamp
// csq/report_min 由调用者在批量上报前查好, 通过参数传入避免重复AT查询
EC_Status_t EC801E_UserMqtt_PublishRecord(const SensorRecord_t *record)
{
    char csq_buf[8] = {0};
    uint32_t report_min;

    if(!record) return EC_ERROR;
    if(!EC801E_UserMqtt_IsConfigured()) return EC_ERROR;

    // CSQ: 使用上报时刻值 (历史记录无CSQ字段)
    EC801E_QueryCSQ(csq_buf, sizeof(csq_buf));

    // 上报周期(分钟)
    report_min = cycle_to_seconds(&g_report_cycle) / 60;
    if(report_min == 0) report_min = 1;

    {
        float p1_kpa = record->pressure / 1000.0f;
        float p2_kpa = record->pressure2 / 1000.0f;
        int volt_alarm = threshold_voltage_alarm(record->voltage);

        // fault=0: 采集失败时不入缓冲, 历史记录均为有效数据
        ly_build_payload(p1_kpa, record->temperature, record->voltage,
                         0, csq_buf, volt_alarm, report_min,
                         p2_kpa, record->temperature2, record->timestamp);
    }

    return umqtt_publish_payload();
}

