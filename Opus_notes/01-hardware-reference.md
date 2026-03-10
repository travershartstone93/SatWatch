# Hardware Reference — Waveshare ESP32-S3-Touch-AMOLED-2.06

## Board Identity
- FQBN: `esp32:esp32:waveshare_esp32_s3_touch_amoled_206`
- Required options: `PSRAM=enabled,PartitionScheme=max_app_32MB`
- Flash: 32 MB (NOT 4 MB — wrong FQBN = silent bootloader hang)
- PSRAM: 8 MB OPI
- Compile-time guard: `BOARD_IS_AMOLED_206` (derived from `AMOLED_PWR_EN`, `AMOLED_CS`, `AMOLED_WIDTH`, `AMOLED_HEIGHT`)

## Display — CO5300 QSPI AMOLED
- Resolution: 408×360 (physical panel), firmware uses 410×360 for scaled output
- Interface: QSPI (4-data-line SPI)
- Driver: LovyanGFX `Panel_CO5300`
- QSPI pins: CS=`AMOLED_CS`, SCK=`AMOLED_SCK`, D0-D3=`AMOLED_D0`..`AMOLED_D3`
- Reset: `AMOLED_RESET`
- Power enable: `AMOLED_PWR_EN` = **GPIO21** (OUTPUT HIGH = display on)
- SPI write freq: 40 MHz
- Pixel format: RGB565 (16-bit), byte-swap detection via probe pixel
- Panel config: offset_x=22, offset_y=0, no inversion, no rgb_order
- Also has Arduino_GFX `Arduino_CO5300` for direct QSPI buffer writes (`s_amoledOut`)

## I2C Bus (Wire)
- SDA: GPIO15
- SCL: GPIO14
- Shared by: touch IC, QMI8658 IMU, PCF85063A RTC, AXP2101 PMIC

## Peripherals on I2C

| Device | Address | INT Pin | Purpose |
|--------|---------|---------|---------|
| AXP2101 PMIC | 0x34 | — | Battery, charge, power key |
| PCF85063A RTC | 0x51 | GPIO39 | Hardware clock (persists across power-off) |
| QMI8658 IMU | 0x6B | GPIO21 (INT1 only) | Shake-to-wake (WoM) |
| Touch IC | — | GPIO38 (active-low) | Touch events (RST=GPIO9) |
| ES8311 Codec | 0x18 | — | Audio output (I2S) |

## CRITICAL: GPIO21 Shared Pin
GPIO21 = `AMOLED_PWR_EN` (display power) = `QMI8658_INT1` (IMU interrupt)
- Two push-pull drivers on same pin
- Display init drives HIGH constantly — IMU cannot toggle
- Fix: `gpio_reset_pin()` + `INPUT_PULLUP` before sleep, restore `OUTPUT HIGH` after wake

## SD Card — SDMMC (NOT SPI)
- Interface: SD_MMC 1-bit mode
- Pins: CLK=2, CMD=1, D0=3, CS=17
- Mount: `SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_*)`
- VFS auto-prepends "/sdcard" — paths must NOT include it
- `#define SD SD_MMC` in firmware

## Audio
- Codec: ES8311 (I2C 0x18, I2S for data)
- I2S class: `ESP_I2S` (`s_audioI2s`)
- Cue playback via FreeRTOS queue task (`s_audioCueTaskHandle`)
- Cues preloaded to PSRAM (`s_audioCueBuf`, max 1 MB)
- DMA buffers must be internal DRAM (not PSRAM)

## Buttons & Wake Sources
- Top button: GPIO defined by board variant, polled via FreeRTOS task
  - Short press: sleep/wake
  - Long press (>1.5s): toggle sleep mode enabled
- AXP2101 PKEY: short press → `ESP.restart()`; long hold ≥4s → hardware power-off
- Touch INT: GPIO38, active-low, usable for portal tap-to-skip
- Boot button: GPIO0 (used on non-AMOLED boards only)

## Memory Layout
- Internal DRAM: ~320 KB total, ~200 KB usable after BSS
  - WiFiClientSecure/mbedTLS needs ~35-40 KB contiguous
  - Keep global BSS under ~200 KB
- PSRAM (8 MB): used for frame buffers, audio cue, sprites
  - `s_frameDisplayBuf`: 295,200 bytes (410×360×2)
  - `s_terrainDisplayBuf`: 295,200 bytes
  - `s_topBarBuf`, `s_botBarBuf`: scaled bar height × 410 × 2 bytes each
  - Audio cue buffer: up to 1 MB
  - Sprites: 320×176×2 each

## Partition Table (`max_app_32MB.csv`)
| Name | Offset | Size |
|------|--------|------|
| nvs | 0x9000 | 0x5000 (20 KB) |
| otadata | 0xE000 | 0x2000 (8 KB) |
| app0 | 0x10000 | 0x1FE0000 (~32 MB) |
| coredump | 0x1FF0000 | 0x10000 (64 KB) |
