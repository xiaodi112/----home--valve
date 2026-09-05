#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>
#include "app_config.h"

/* EEPROM chip driver selection (macro in app_config.h): M24M01 / AT24C64 */
#if defined(EEPROM_DRV_AT24C64)
#include "at24c64.h"
#define EE_READ(addr, buf, len)       AT24C64_Read((addr), (buf), (len))
#define EE_WRITE(addr, buf, len)      AT24C64_Write((addr), (buf), (len))
#define EE_WRITE_BYTE(addr, data)     AT24C64_WriteByte((addr), (data))
#else
#include "m24m01.h"
#define EE_READ(addr, buf, len)       M24M01_Read((addr), (buf), (len))
#define EE_WRITE(addr, buf, len)      M24M01_Write((addr), (buf), (len))
#define EE_WRITE_BYTE(addr, data)     M24M01_WriteByte((addr), (data))
#endif

/* ==================== 配置持久化 (EEPROM 读写/加载/保存) ==================== */
void config_recalc_interval(void);
void config_load_all(void);
void config_save_interval(void);
void config_save_mqtt(void);
void config_save_ext(void);
void config_save_vpthreshold(void);
void config_save_tthreshold(void);
void config_save_spupower(void);
void config_save_vdelay(void);
void config_save_vpthreshold2(void);
void config_save_tthreshold2(void);
void config_save_user_mqtt(void);
void config_save_valve_timing(void);
void config_save_abn23(void);
uint16_t config_get_keepalive(void);
void derive_topic_from_client_id(void);

#endif /* CONFIG_STORE_H */
