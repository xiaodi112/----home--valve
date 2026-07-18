# Bootloader (GD32L233RC) — 4G OTA 升级引导

配合 `Projects/07_LPUART_Deepsleep_Wakeup/` 的 App 工程，实现通过 EC801E 4G 模块 + FastBee MQTT 平台远程升级。

## 一、Flash 分区

```
0x08000000  16 KB   Bootloader        (本工程)           (pages 0-3)
0x08004000   4 KB   OTA InfoBlock     (升级元数据)         (page 4)
0x08005000 108 KB   App 运行区                          (pages 5-31)
0x08020000 128 KB   下载缓存区                        (pages 32-63)
```

详见 `../Projects/07_LPUART_Deepsleep_Wakeup/MDK-ARM/my_Library/ota_info.h`。

## 二、构建步骤

1. **构建 Bootloader**: 用 Keil 打开 `MDK-ARM/Bootloader.uvprojx` → Rebuild
   - 输出: `MDK-ARM/Objects/Bootloader.hex` (烧入 0x08000000)
2. **构建 App**: 用 Keil 打开 `../Projects/07_LPUART_Deepsleep_Wakeup/MDK-ARM/GD32L233R_EVAL.uvprojx` → Rebuild
   - 输出: `Project.hex` 链接基址已改为 0x08005000
   - 同时导出原始 bin (FromELF: `--bin --output=Project.bin`) 用于上传到 HTTP 服务器
3. **首次出厂烧录**: 同时烧入 Bootloader.hex 和 Project.hex (Keil 支持单独 Flash Download)

## 三、远程升级流程

### 3.1 平台下发 (FastBee `/upgrade/set` topic)

```json
{
  "version": 2,
  "downloadUrl": "http://your-server/path/firmware.bin"
}
```

### 3.2 设备上报 (FastBee `/upgrade/reply` topic)

```json
{ "mqttUpgradeReply": <code> }
```

| code | 含义 |
|------|------|
| 2 | REPLY (升级中) — 收到指令立即上报 |
| 3 | SUCCESS — 新 App 启动后上报 |
| 4 | FAILED — 下载失败/校验失败/重试超限 |
| 5 | STOP — 设备主动放弃 (低电压等) |

### 3.3 设备端流程

```
App 收到 upgrade/set
  → 上报 reply=2
  → 擦缓存区 (128 KB / 32 页)
  → AT+QHTTPGET 下载 bin → 边收边写缓存区 (字节级)
  → 写 InfoBlock: status=PENDING, size, version
  → NVIC_SystemReset()

Bootloader (复位后)
  → 读 InfoBlock
  → status=PENDING/FLASHING:
      retry_count++
      if retry > 3: 标记 FAILED 跳 App
      status=FLASHING
      擦 App 区 → 拷贝 Cache→App
      status=SUCCESS, cur_version=new_fw_version
  → 跳 App

新 App 启动
  → ota_app_report_boot_status() 检测 InfoBlock
      status=SUCCESS → 上报 reply=3, 清 InfoBlock
      status=FAILED  → 上报 reply=4, 清 InfoBlock
```

## 四、可重复升级保证

- Bootloader 物理位于 0x08000000-0x08003FFF, App 链接区从 0x08005000 起，**App 永远无法擦除 Bootloader**。
- 每次升级在升级前 retry_count 自增 (>3 放弃)，升级成功后清零。
- 缓存区在每次新升级开始时整体擦除，避免脏数据。
- 升级中断电恢复：BL 检测 `FLASHING` 状态会幂等重做拷贝。
- InfoBlock 在升级完成后被 App 清除 (整页擦除)，下次重启不会重复处理。

## 五、注意事项

1. **App 工程的 IROM 起址必须为 0x08005000, 长度 0x1B000 (108 KB)** — 已配置在 `GD32L233R_EVAL.uvprojx`。
2. **App 必须在 sys_init 顶部设置 `SCB->VTOR = OTA_APP_ADDR`** — 已修改 `main.c`。
3. **MQTT 订阅** `EC801E_InitAndConnect()` 末尾会自动订阅 `upgrade/set`，无需额外操作。
4. **HTTP 文件大小上限** = `OTA_APP_SIZE` = 108 KB；如 App 增长超出，需调整分区。
5. **当前未启用 CRC 校验**，仅按 HTTP `Content-Length` 与累计字节数对比。
