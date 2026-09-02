# SatWatch: LiveSat firmware

Firmware for **LiveSat**, an ESP32-S3 smartwatch that shows a live 24-hour loop
of GOES-East satellite weather imagery on your wrist, with hurricane/storm
alerts for your location.

It's a single-sketch Arduino build (`esp32-LiveSat-s3.ino`), deliberately one
file for easy flashing. Section markers inside the sketch divide the major
subsystems.

## What it does

- On startup: syncs/refreshes 24 h of GOES-East GeoColor frames from NASA GIBS
  into an SD cache, then plays the loop and sleeps
- On wake: reuses the SD cache, refreshing only when the data is stale
- Location-aware: overlays your position and pulls storm/hurricane alerts
  (NHC/GDACS) for it
- WiFi setup via a captive portal (no hardcoded credentials); OTA updates are
  delivered through the companion [LiveSat-OTA](https://github.com/travershartstone93/LiveSat-OTA)
  releases repo

## Hardware

- Waveshare **ESP32-S3-Touch-AMOLED-2.06** (primary target)
- Waveshare ESP32-S3-LCD-1.47 (also supported)
- MicroSD card for the frame cache

## Build & flash

Arduino IDE or arduino-cli with the "esp32 by Espressif Systems" board
package (>= 3.0.0), matching ESP32-S3 board profile, and the partition table
from `partitions.csv` / `max_app_32MB.csv`.

Required libraries (Library Manager):
- [LovyanGFX](https://github.com/lovyan03/LovyanGFX)
- [JPEGDEC](https://github.com/bitbank2/JPEGDEC)

Timing, loop counts, and sleep cadence are configured in `config-s3.h`.

## Files

- `esp32-LiveSat-s3.ino`: the application (single-sketch build)
- `config-s3.h`: build-time configuration
- `flags_rgb565.h`: generated pixel data, do not edit
- `es8311.*`: vendored audio codec driver
- `partitions.csv`, `max_app_32MB.csv`: flash partition tables
