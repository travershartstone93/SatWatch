# Opus Notes — LiveSat ESP32-S3 Firmware Reference

Personal reference documentation for the LiveSat firmware codebase.
Main file: `esp32-LiveSat-s3/esp32-LiveSat-s3.ino` (~15,400 lines)

## File Index

| File | Contents |
|------|----------|
| [01-hardware-reference.md](01-hardware-reference.md) | Board pinout, peripherals, I2C map, display, SD card |
| [02-firmware-architecture.md](02-firmware-architecture.md) | High-level architecture, setup/loop flow, state machine |
| [03-frame-pipeline.md](03-frame-pipeline.md) | Download → validate → cache → raw build → playback |
| [04-corruption-detectors.md](04-corruption-detectors.md) | All 10+ corruption detectors: purpose, thresholds, where wired |
| [05-sync-decision-tree.md](05-sync-decision-tree.md) | syncFramesRolling() decision paths, rolling/tail-fill/full-refresh |
| [06-critical-pitfalls.md](06-critical-pitfalls.md) | Distilled "don't do this" list from The-fuckup-sofar.txt |
| [07-playback-and-overlay.md](07-playback-and-overlay.md) | Animation loop, zoom stages, terrain crossfade, clock sweep |
| [08-peripherals-and-io.md](08-peripherals-and-io.md) | IMU WoM, AXP2101 PMIC, PCF85063A RTC, ES8311 audio, WiFi portal |
| [09-constants-and-paths.md](09-constants-and-paths.md) | Key #defines, file paths, meta versions, buffer sizes |
| [10-build-and-flash.md](10-build-and-flash.md) | Compile/upload commands, partition layout, FQBN |

## Quick Links to Source

- Firmware: `/home/whisper/Desktop/LiveSat/esp32-LiveSat-s3/esp32-LiveSat-s3.ino`
- Config: `/home/whisper/Desktop/LiveSat/esp32-LiveSat-s3/config-s3.h`
- Board examples: `/home/whisper/Desktop/LiveSat/ESP32-S3-Touch-AMOLED-2.06/examples/`
- SensorLib: `/home/whisper/Arduino/libraries/SensorLib/`
- Pitfalls doc: `/home/whisper/Desktop/LiveSat/Legacy docs/The-fuckup-sofar03-08-26.txt`
- Fixes log: `/home/whisper/Desktop/LiveSat/Legacy docs/S3-Fixes.txt`
- WoM debug: `/home/whisper/Desktop/LiveSat/Legacy docs/Shake-Wake-Issue.txt`
- System outline: `/home/whisper/Desktop/LiveSat/Legacy docs/LiveSat-System_outline.txt`
