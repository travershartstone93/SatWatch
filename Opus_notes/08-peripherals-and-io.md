# Peripherals & I/O

## QMI8658 IMU — Shake-to-Wake (WoM)

### Configuration
- Library: SensorLib by Lewis He (`/home/whisper/Arduino/libraries/SensorLib/`)
- I2C address: 0x6B
- INT1: GPIO21 (shared with AMOLED_PWR_EN!)
- Use `configWakeOnMotion()` — NOT `configMotion()+enableMotionDetect()`
- Threshold: 100mg (better than 250mg for wrist-flick)
- `configWakeOnMotion()` resets sensor internally — always call `begin()` first

### WoM Interrupt Behavior
- Toggle, NOT stable level — pin flips on each event
- ESP32 light sleep: level-triggered only (no edge trigger)
- Strategy: read current INT1 level → arm OPPOSITE level → next toggle wakes
- `defaultPinValue=1` (idle HIGH, motion→LOW) + `GPIO_INTR_LOW_LEVEL` + `gpio_pullup_en`

### GPIO21 Pin Sharing Protocol
Before sleep (arming WoM):
```cpp
gpio_reset_pin((gpio_num_t)QMI8658_INT1);
pinMode(QMI8658_INT1, INPUT_PULLUP);  // pullup keeps display power ~HIGH
```
After wake (restore display power):
```cpp
pinMode(LCD_PWR, OUTPUT);
digitalWrite(LCD_PWR, HIGH);
```

### Wake Confirmation
- `s_shakeEntryIgnoreMs = 2000` — ignore WoM for 2s after sleep entry (settle window)
- `s_shakeConfirmMs = 600` — after settle: require 2nd WoM within 600ms (0 = any single event)
- After wake: `s_qmi.getStatusRegister()` to acknowledge interrupt

## AXP2101 PMIC

### I2C: 0x34
- Register access: `readAxp2101Register(reg)`, `writeAxp2101Register(reg, val)`
- SOC (State of Charge): reg 0xA4 → `s_batPct` (0-100%)
- Charge state: reg 0x01 (STATUS2) → `s_batChargeState`
  - Near 99% SOC: legitimately oscillates between top-off/done phases → icon flips

### Power Key
- `configureAxp2101PowerKey()`:
  - Short press: sets INTSTS2 bit → firmware polls → `ESP.restart()`
  - Long hold ≥4s: hardware power-off threshold

### Battery Icon
- `drawBatteryIcon()`: reads AXP2101 every render pass
- Known issue: glyph flip near 99% SOC (hardware behavior, not bug)

## PCF85063A RTC

### I2C: 0x51
- Confirmed in `pins_arduino.h`: `RTC_ADDRESS 0x51`
- Shares I2C bus with touch, IMU, PMIC
- INT: GPIO39

### Write: `writePcf85063(time_t)`
1. Halt oscillator (reg 0x00 bit 5)
2. Write BCD time to regs 0x02–0x08
3. Resume oscillator

### Read: `readPcf85063(time_t*)`
1. Read 7 bytes from reg 0x02
2. Check OS (oscillator-stop) flag — if set, data is invalid
3. Reconstruct Unix epoch → `settimeofday()`

### Boot Priority
1. RTC (PCF85063A) — applied before portal if OS flag clear
2. NTP (during sync) — overwrites RTC, writes back to RTC
3. NVS UTC offset ("utcoff") — restores timezone without IP geolocation
4. IP geolocation (ip-api.com) — refined offset, saved to NVS

### Key Function
- `tryApplyPcf85063Time()`: called before `runWifiConfigPortal()` so local time is correct even without internet

## ES8311 Audio Codec (AMOLED board only)

### Configuration
- I2C address: 0x18 (or 0x19)
- Driver: `es8311.h` / `es8311.c` (Espressif)
- Data bus: I2S (`s_audioI2s`)
- Init: `es8311_init()`, volume: `es8311_codec_set_voice_volume()`, mute: `es8311_set_voice_mute()`

### Startup Cue System
- Cue modes: Off / Chime / Vibe Pulse / Chime2 / Chime3
- Default cue: `/power_up.raw` on SD card
- Preload: `preloadSelectedCueToPsram()` → `s_audioCueBuf` (PSRAM, max 1 MB)
- Worker: `ensureAudioCueWorker()` → FreeRTOS task (`s_audioCueTaskHandle`)
- Queue: `s_audioCueQueue` — `AudioCueRequest{volume}`
- Trigger: `s_startCuePending = true` → `playStartCueIfEnabled()` on first valid frame
- If configured cue file missing → fallback to `/power_up.raw`
- DMA requirement: I2S DMA buffers must be internal DRAM, not PSRAM

## WiFi & Captive Portal

### AP Mode
- SSID: "Sat Watch", Password: "123456789"
- IP: 192.168.4.1
- DNS: all queries → portal IP (captive redirect)
- mDNS: `satwatch.local`

### STA-Connected Mode
- Portal HTTP server still runs on LAN
- mDNS still active (`satwatch.local`)
- NO AP broadcast, NO DNS captive redirect
- `disconnectWifiAfterSync()` → `startWifiPortalServer(false)` (STA-only)

### Portal Features
- 5-slot WiFi credential management
- Clock format: 12h / 24h
- Update mode: Manual / Auto (with schedule presets: 15min, 30min, 1h)
- Start cue: Off / Chime / Vibe pulse + volume
- Save action: persist to NVS → reboot

### Auto-Retry
- While portal guide displayed: `connectWifiForSync()` every 60 seconds
- On success: `ESP.restart()` for normal boot/sync
- On failure: restore AP mode, continue showing guide

### Tap-to-Skip (cached frames exist)
- GPIO38 (TP_INT, active-low) or AXP2101 PKEY short press
- Dismisses portal → plays cached animation without network
- Guide shows amber "Tap to play saved frames" prompt

### Config Storage (NVS)
- Namespace: `satwatch`
- Keys: WiFi slots, clock format, update mode, schedule minutes, start cue mode/path/volume, sleep mode, UTC offset
- `loadWifiPortalConfig()`, `saveWifiPortalConfig()`

## Button Handling

### Top Button (FreeRTOS Task)
- Task: `topButtonPollTask()`, core 1, 2048 stack, priority 3
- Poll interval: 1ms
- Debounce: 4ms
- Short press (>5ms): `goToSleep(true)` (button-only sleep)
- Long press (>1500ms): toggle `s_sleepModeEnabled`, save to NVS
- Suppress window: 250ms after action
- ISR-safe state: `portMUX_TYPE s_topBtnStateMux`

### AXP2101 Power Key (AMOLED)
- Short press: INTSTS2 interrupt → `ESP.restart()`
- Long hold ≥4s: hardware power-off

## IP Geolocation
- Source: `ip-api.com/json` (free, no API key)
- Returns: UTC offset, lat/lon, city, region, country
- Manual JSON parsing: `jsonExtractIntField()`, `jsonExtractFloatField()`, `jsonExtractStringField()`
- Location hysteresis: <0.15° change → keep existing center (no cache invalidation)
- Updates `s_weatherCenterLat/Lon`, `s_displayUtcOffsetSec`, `s_displayLocationLabel/Full`
