# Build & Flash Reference

## Prerequisites
- `arduino-cli` installed
- ESP32 board package >= 3.0.0 (`esp32 by Espressif Systems`)
- Libraries: LovyanGFX, JPEGDEC, SensorLib (Lewis He)
- Board connected via USB at `/dev/ttyACM0`

## FQBN (CRITICAL — wrong FQBN = silent bootloader hang)
```
esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB
```

## Compile
```bash
arduino-cli compile \
  --fqbn "esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB" \
  --build-path /tmp/s3-build \
  /home/whisper/Desktop/LiveSat/esp32-LiveSat-s3
```

## Upload
```bash
arduino-cli upload \
  --fqbn "esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB" \
  --build-path /tmp/s3-build \
  -p /dev/ttyACM0 \
  /home/whisper/Desktop/LiveSat/esp32-LiveSat-s3
```

## One-Liner (Compile + Upload)
```bash
arduino-cli compile \
  --fqbn "esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB" \
  --build-path /tmp/s3-build \
  /home/whisper/Desktop/LiveSat/esp32-LiveSat-s3 \
&& arduino-cli upload \
  --fqbn "esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB" \
  --build-path /tmp/s3-build \
  -p /dev/ttyACM0 \
  /home/whisper/Desktop/LiveSat/esp32-LiveSat-s3
```

## Serial Monitor
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```
Note: Serial output is only active when `ENABLE_SERIAL_DIAG 1` is set in `config-s3.h`.
When set to 0, all Serial calls are compiled to no-ops via `NullSerialSink`.

## Port Discovery
```bash
arduino-cli board list
```

## Partition Table
- Uses `max_app_32MB.csv` — single 32 MB app partition
- NVS: 20 KB at 0x9000
- App: ~32 MB at 0x10000
- Coredump: 64 KB at 0x1FF0000

## After Full Flash Erase
`erase_flash` destroys bootloader (0x0) and partition table (0x8000).
`arduino-cli upload` alone does NOT restore them.
After full erase: flash entire `merged.bin` via `esptool`.

## Key Compile-Time Switches
| Define | Location | Purpose |
|--------|----------|---------|
| `ENABLE_SERIAL_DIAG` | config-s3.h | 0=no serial output, 1=enable |
| `BOARD_IS_AMOLED_206` | derived | Enables AMOLED-specific code paths |
| `BOARD_HAS_SDMMC` | derived | SD_MMC vs SPI SD |
| `BOARD_HAS_PSRAM_SPRITES` | derived | PSRAM sprite allocation |
| `INDEX_MAGIC` | main .ino | 0x4C534658 — index.bin format marker |
| `WEATHER_VIEW_VERSION` | main .ino | Bump to force weather redownload |

## Source Files
| File | Purpose |
|------|---------|
| `esp32-LiveSat-s3.ino` | Main firmware (~15,400 lines) |
| `config-s3.h` | User-editable config (WiFi, timing, timezone) |
| `es8311.h` / `es8311.c` | ES8311 audio codec driver |
| `es8311_reg.h` | ES8311 register definitions |
| `partitions.csv` | Partition table |
| `max_app_32MB.csv` | 32 MB partition layout |
