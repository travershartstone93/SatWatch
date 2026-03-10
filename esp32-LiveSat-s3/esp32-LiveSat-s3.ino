/*
 * esp32-caribbean-s3.ino
 *
 * Location-aware weather loop, ported to ESP32-S3 targets.
 * Current S3 hardware path supports:
 *   - Waveshare ESP32-S3-LCD-1.47
 *   - Waveshare ESP32-S3-Touch-AMOLED-2.06
 *
 * The app keeps the original 320x172 logical framebuffer on S3 so the weather
 * pipeline and cache format stay unchanged during hardware bring-up.
 *
 * On startup    : syncs/refreshes 24 h of GOES-East GeoColor frames from NASA GIBS
 *                 into SD cache as needed, then plays and sleeps.
 * On wake       : reuses SD cache and may refresh if timer/cadence says data is stale.
 * Loop count and sleep timing are controlled by config-s3.h.
 *
 * Required libraries (Arduino Library Manager):
 *   LovyanGFX   https://github.com/lovyan03/LovyanGFX
 *   JPEGDEC     https://github.com/bitbank2/JPEGDEC
 *
 * Board package: "esp32 by Espressif Systems" >= 3.0.0
 * Board target : use the matching ESP32-S3 board profile
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SD.h>
#if defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
#include <SD_MMC.h>
#define SD SD_MMC
#endif
#include <SPI.h>
#include <stdarg.h>
#include <time.h>
#include <Wire.h>
#include <ESP_I2S.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <driver/gpio.h>   // gpio_wakeup_enable()
#include <esp_wifi.h>      // esp_wifi_set_protocol()
#include "SensorQMI8658.hpp"
#if defined(AMOLED_PWR_EN) && defined(AMOLED_CS) && defined(AMOLED_WIDTH) && defined(AMOLED_HEIGHT)
#include <Arduino_GFX_Library.h>
#endif

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <JPEGDEC.h>

#include "config-s3.h"

#if defined(AMOLED_PWR_EN) && defined(AMOLED_CS) && defined(AMOLED_WIDTH) && defined(AMOLED_HEIGHT)
#define BOARD_IS_AMOLED_206 1
#else
#define BOARD_IS_AMOLED_206 0
#endif

#if defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147) || BOARD_IS_AMOLED_206
#define BOARD_HAS_SDMMC 1
#define BOARD_HAS_PSRAM_SPRITES 1
#else
#define BOARD_HAS_SDMMC 0
#define BOARD_HAS_PSRAM_SPRITES 0
#endif

#if BOARD_IS_AMOLED_206
#include <SD_MMC.h>
#define SD SD_MMC
#define BOARD_HAS_PHYSICAL_BOOT_WAKE 0
#include "es8311.h"
#else
#define BOARD_HAS_PHYSICAL_BOOT_WAKE 1
#endif

#ifndef ENABLE_SERIAL_DIAG
#define ENABLE_SERIAL_DIAG 0
#endif

#if !ENABLE_SERIAL_DIAG
class NullSerialSink {
public:
  inline void begin(unsigned long) {}
  template <typename... Args> inline size_t print(Args...) { return 0; }
  template <typename... Args> inline size_t println(Args...) { return 0; }
  template <typename... Args> inline size_t printf(const char*, Args...) { return 0; }
};
static NullSerialSink s_nullSerial;
#define Serial s_nullSerial
#endif

// ─────────────────────────────────────────────────────────────
//  Pin assignments
// ─────────────────────────────────────────────────────────────
#if BOARD_IS_AMOLED_206
#define LCD_CS   AMOLED_CS
#define LCD_SCLK AMOLED_SCK
#define LCD_D0   AMOLED_D0
#define LCD_D1   AMOLED_D1
#define LCD_D2   AMOLED_D2
#define LCD_D3   AMOLED_D3
#define LCD_RST  AMOLED_RESET
#define LCD_PWR  AMOLED_PWR_EN
#elif defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
#define LCD_MOSI 45
#define LCD_SCLK 40
#define LCD_CS   42
#define LCD_DC   41
#define LCD_RST  39
#define LCD_BL   48
#define SD_MISO  -1
#define SD_CLK_PIN 14
#define SD_CMD_PIN 15
#define SD_D0_PIN  16
#define SD_D1_PIN  18
#define SD_D2_PIN  17
#define SD_D3_PIN  21
#else
#define LCD_MOSI  6
#define LCD_SCLK  7
#define LCD_CS   14
#define LCD_DC   15
#define LCD_RST  21
#define LCD_BL   22
#define SD_MISO   5
#define SD_CS     4
#endif

// Fetch resolution for raw cache. Both dimensions must be MCU-aligned (multiples of 16)
// to avoid partial-MCU stride issues in jpegDraw. 320×176 gives 112 KB per frame
// vs 180 KB at 410×220, cutting SD read time from 200 ms to ~125 ms (~8 fps).
// scaleSpriteTo410x360() pre-scales 320×176 sprite to 410×360 canonical RGB565 in PSRAM.
#define DISP_W  320
#define DISP_H  176

// BOOT button wake remains enabled on the original C6/1.47 paths.
// The 2.06 AMOLED board does not expose a safe dedicated wake button in this
// sketch yet, so it uses timer wake only.
#define BOOT_BTN_GPIO  0

// SD frame directory and files
#define SD_ROOT ""          // VFSImpl internally prepends the mountpoint ("/sdcard"); paths must NOT include it
#define FRAMES_DIR  SD_ROOT "/frames"
#define META_FILE   SD_ROOT "/frames/meta.txt"

// Max JPEG bytes we'll buffer for one frame. Keep this aligned with s_dlBuf.
// Increased for supersampled zoom snapshot fetches.
#define MAX_JPEG_BYTES  (128 * 1024)
#define RAW_FRAME_BYTES ((size_t)DISP_W * (size_t)DISP_H * 2U)

// Pre-scaled display frame dimensions (post-scale output stored in stream.raw and terrain.raw)
#define SCALED_W    410
#define SCALED_H    360
#define SCALED_FRAME_BYTES ((size_t)SCALED_W * (size_t)SCALED_H * 2U)  // 295,200 bytes
#define RAW_CACHE_META_FILE SD_ROOT "/frames/raw.meta"
#define RAW_STREAM_FILE     SD_ROOT "/frames/stream.raw"
#define FRAME_DIM_FILE      SD_ROOT "/frames/dim.cfg"
#define WEATHER_VIEW_META_FILE SD_ROOT "/frames/view.meta"
#define CACHE_VALIDATE_META_FILE SD_ROOT "/frames/validate.meta"
#define WIFI_PORTAL_AP_SSID  "Sat Watch"
#define WIFI_PORTAL_AP_PASS  "123456789"
#define WIFI_PORTAL_HOSTNAME "satwatch"
#define ZOOM_SNAPSHOT_META_FILE SD_ROOT "/frames/zoom.meta"

// ── Pre-allocated frame store (replaces per-frame .jpg files) ──
#define JPEG_SLOT_BYTES     (64 * 1024)
#define FRAMES_BIN_FILE     SD_ROOT "/frames/frames.bin"
#define INDEX_BIN_FILE      SD_ROOT "/frames/index.bin"
#define INDEX_TMP_FILE      SD_ROOT "/frames/index.tmp"
#define INDEX_MAGIC         0x4C534658    // "LSFX"
#define ZOOM1_FILE SD_ROOT "/frames/vz1.jpg"
#define ZOOM2_FILE SD_ROOT "/frames/vz2.jpg"
#define ZOOM3_FILE SD_ROOT "/frames/vz3.jpg"
#define ZOOM1_RAW_FILE SD_ROOT "/frames/vz1.raw"
#define ZOOM2_RAW_FILE SD_ROOT "/frames/vz2.raw"
#define ZOOM3_RAW_FILE SD_ROOT "/frames/vz3.raw"
// Supersample zoom snapshots at 2x source resolution, then decode back down to
// the logical framebuffer size. Use a 172px content height (344px at 2x) to
// match the proven C6 zoom path and avoid over-requesting a 352px-tall source.
#define ZOOM_FETCH_W (DISP_W * 2)
#define ZOOM_FETCH_H ((DISP_H - 4) * 2)
// Terrain needs a true full-height 2x raster so the terrain stage stays centered
// on the location pin instead of using a 172-row decode plus padded bottom rows.
#define TERRAIN_FETCH_W (DISP_W * 2)
#define TERRAIN_FETCH_H (DISP_H * 2)
// Final zoom floor used by ZOOM3 and the pre-zoom locator cue on the freeze frame.
#define ZOOM3_FINAL_W_KM 250.0f
#define ZOOM3_FINAL_H_KM 135.0f
#define ZOOM_TERRAIN_DAY_FILE   SD_ROOT "/frames/terrain_day.jpg"   // BlueMarble
#define ZOOM_TERRAIN_NIGHT_FILE SD_ROOT "/frames/terrain_night.jpg" // VIIRS Black Marble
#define ZOOM_TERRAIN_DAY_RAW    SD_ROOT "/frames/terrain_day.raw"
#define ZOOM_TERRAIN_NIGHT_RAW  SD_ROOT "/frames/terrain_night.raw"
#define ZOOM_TERRAIN_RADAR_FILE SD_ROOT "/frames/terrain_radar_z3.jpg"
#define ZOOM_TERRAIN_RADAR_RAW_TMP_FILE SD_ROOT "/frames/.terrain_radar.raw"
#define CACHE_VALIDATE_VERSION 2
#define ZOOM_META_VERSION 6
#define WIFI_CONFIG_SLOTS 5
#define WEATHER_VIEW_VERSION 5  // bump: force one-time fresh main animation redownload

// GIBS WMS endpoint (no API key required)
#define GIBS_WMS_BASE \
  "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi" \
  "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap" \
  "&STYLES=&SRS=EPSG:4326"

#define WEATHER_LAYER_GOES_EAST   "GOES-East_ABI_GeoColor"
#define WEATHER_LAYER_GOES_WEST   "GOES-West_ABI_GeoColor"
#define WEATHER_LAYER_HIMAWARI_IR "Himawari_AHI_Band13_Clean_Infrared"

// ─────────────────────────────────────────────────────────────
//  LovyanGFX display configuration
// ─────────────────────────────────────────────────────────────
class LGFX : public lgfx::LGFX_Device {
  #if BOARD_IS_AMOLED_206
  lgfx::Panel_CO5300 _panel;
  #else
  lgfx::Panel_ST7789 _panel;
  lgfx::Light_PWM    _light;
  #endif
  lgfx::Bus_SPI       _bus;

public:
  LGFX() {
    // SPI bus
    {
      auto cfg        = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
#if BOARD_IS_AMOLED_206
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.pin_sclk    = LCD_SCLK;
      cfg.pin_miso    = -1;
      cfg.pin_mosi    = -1;
      cfg.pin_dc      = -1;
      cfg.pin_io0     = LCD_D0;
      cfg.pin_io1     = LCD_D1;
      cfg.pin_io2     = LCD_D2;
      cfg.pin_io3     = LCD_D3;
#elif defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
      cfg.freq_write  = 20000000;
      cfg.freq_read   = 0;
      cfg.pin_sclk    = LCD_SCLK;
      cfg.pin_mosi    = LCD_MOSI;
      cfg.pin_miso    = SD_MISO;
      cfg.pin_dc      = LCD_DC;
#else
      // Stable display write clock on this board.
      cfg.freq_write  = 30000000;
      cfg.freq_read   = 16000000;
      cfg.pin_sclk    = LCD_SCLK;
      cfg.pin_mosi    = LCD_MOSI;
      cfg.pin_miso    = SD_MISO;
      cfg.pin_dc      = LCD_DC;
#endif
      cfg.dma_channel = SPI_DMA_CH_AUTO;  // baseline: DMA enabled
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    // Panel
    {
      auto cfg           = _panel.config();
      cfg.pin_cs         = LCD_CS;
      cfg.pin_rst        = LCD_RST;
#if BOARD_IS_AMOLED_206
      cfg.pin_busy       = -1;
      cfg.panel_width    = AMOLED_WIDTH;
      cfg.panel_height   = AMOLED_HEIGHT;
      cfg.memory_width   = AMOLED_WIDTH;
      cfg.memory_height  = AMOLED_HEIGHT;
      cfg.offset_x       = 22;
      cfg.offset_y       = 0;
      cfg.offset_rotation = 0;
      cfg.readable       = false;
      cfg.invert         = false;
      cfg.rgb_order      = false;
      cfg.dlen_16bit     = false;
      cfg.bus_shared     = false;
#else
      cfg.pin_busy       = -1;
      // Current Waveshare board does not expose the ST7789 TE pin/readback on
      // the LCD path. When the replacement TE-pin board arrives, this is where
      // TE-synchronised frame updates should be wired/configured.
      cfg.panel_width    = 172;      // physical panel width  (portrait native)
      cfg.panel_height   = 320;      // physical panel height (portrait native)
      cfg.memory_width   = 240;      // ST7789 internal GRAM width
      cfg.memory_height  = 320;      // ST7789 internal GRAM height
      cfg.offset_x       = 34;       // (240-172)/2 — centres 172px panel in 240px GRAM
      cfg.offset_y       = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;   // ST7789 needs inversion
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       =
#if defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
        false;
#else
        true;    // SD card shares the same SPI bus
#endif
#endif
      _panel.config(cfg);
    }
#if !BOARD_IS_AMOLED_206
    // Backlight
    {
      auto cfg       = _light.config();
      cfg.pin_bl     = LCD_BL;
      cfg.invert     = false;
      cfg.freq       = 44100;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
#endif
    setPanel(&_panel);
  }

  bool init(void) {
#if BOARD_IS_AMOLED_206
    pinMode(LCD_PWR, OUTPUT);
    digitalWrite(LCD_PWR, HIGH);
    delay(50);
#endif
    return lgfx::LGFX_Device::init();
  }
};

static LGFX   tft;
static JPEGDEC jpeg;
#if BOARD_IS_AMOLED_206
static LGFX_Sprite sprite;
static LGFX_Sprite s_clockFxSprite;
static LGFX_Sprite s_barSprite;
static Arduino_DataBus* s_amoledBus =
  new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
static Arduino_CO5300* s_amoledOut =
  new Arduino_CO5300(s_amoledBus, LCD_RST, 0, AMOLED_WIDTH, AMOLED_HEIGHT, 22, 0, 0, 0);
#else
static LGFX_Sprite sprite(&tft);
static LGFX_Sprite s_clockFxSprite(&tft);
static LGFX_Sprite s_barSprite(&tft);
#endif
#if BOARD_IS_AMOLED_206
static bool s_amoledClearBeforeNextPresent = true;
#endif
// Base scaled height of a single metadata row.
static const int SCALED_BAR_H = (14 * SCALED_H + DISP_H - 1) / DISP_H + 2;  // ~31
static const int SCALED_TOP_ROW_H = SCALED_BAR_H;
static const int SCALED_TOP_BAR_H = SCALED_BAR_H * 2;
static const int SCALED_BAR_SPRITE_H = SCALED_BAR_H;

// Pre-scaled display frame (410×360×2 = 295,200 bytes) — allocated from PSRAM in setup()
static uint16_t* s_frameDisplayBuf   = nullptr;
// Terrain wipe pre-load (295,200 bytes) — loaded from SD once before wipe starts
static uint16_t* s_terrainDisplayBuf = nullptr;
// Persistent top/bottom timestamp bars at 410×SCALED_BAR_H — overlaid every present.
static uint16_t* s_topBarBuf = nullptr;
static uint16_t* s_botBarBuf = nullptr;

// Dirty-rect state for clock overlay partial scale.
// When s_dirtyRectDstW > 0, presentSpriteToDisplay() scales only the clock
// sub-rect instead of the full 410×360 frame.
static int s_dirtyRectSrcX = 0, s_dirtyRectSrcY = 0, s_dirtyRectSrcW = 0, s_dirtyRectSrcH = 0;
static int s_dirtyRectDstX = 0, s_dirtyRectDstY = 0, s_dirtyRectDstW = 0, s_dirtyRectDstH = 0;

static bool spriteReady = false;
static bool s_clockFxSpriteReady = false;
static bool s_barSpriteReady = false;
static bool s_mainSpritePixelsByteSwapped = false;
static bool s_barSpritePixelsByteSwapped = false;
static int  s_clockFxSpriteW = 0;
static int  s_clockFxSpriteH = 0;
static bool s_clockFxSpritePixelsByteSwapped = false;
#if BOARD_IS_AMOLED_206
static LovyanGFX* g_drawTarget = nullptr;
#else
static LovyanGFX* g_drawTarget = &tft;
#endif
static int s_jpegMinX = 0;
static int s_jpegMinY = 0;
static int s_jpegMaxX = 0;
static int s_jpegMaxY = 0;
static uint16_t s_jpegDrawCalls = 0;
static bool s_jpegDrawOutOfBounds = false;
struct WeatherSemanticSignature {
  uint8_t meanR = 0;
  uint8_t meanG = 0;
  uint8_t meanB = 0;
  uint8_t cyanTiles = 0;
  uint8_t texture = 0;
  uint8_t tileMean[11][20][3] = {};
};
static uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t alpha);
static bool decodeJpegPathToSprite(const char* path, bool relaxedHeight = false);
static bool spriteLooksHoldFrameBlockCorrupted();
static bool spriteLooksBlackSlabCorrupted();
static bool spriteLooksCyanWhiteBlockCorrupted();
static bool spriteLooksBottomBandJunkCorrupted();
static bool terrainUsesNightLayerForUtc(time_t weatherFrameUtc);
static int activeCadenceMin();
static int activeLagHours();
static void selectSatelliteForLon(float lonDeg, bool force = false);

// ─────────────────────────────────────────────────────────────
//  RTC memory — survives deep sleep, cleared on power cycle
// ─────────────────────────────────────────────────────────────
RTC_DATA_ATTR static bool   framesReady = false;
RTC_DATA_ATTR static int    loopsDone   = 0;
RTC_DATA_ATTR static int    frameCount  = 0;
RTC_DATA_ATTR static int32_t s_displayUtcOffsetSec = -4 * 3600;  // initial default, overwritten by user timezone
RTC_DATA_ATTR static bool    s_displayUtcOffsetValid = false;
RTC_DATA_ATTR static bool    s_weatherGeoValid = false;
RTC_DATA_ATTR static float   s_weatherCenterLat = 0.0f;
RTC_DATA_ATTR static float   s_weatherCenterLon = 0.0f;
RTC_DATA_ATTR static int     s_activeCadenceMin = CADENCE_MIN;
RTC_DATA_ATTR static int     s_activeLagHours = GIBS_LAG_HOURS;
RTC_DATA_ATTR static char    s_activeGibsLayer[48] = WEATHER_LAYER_GOES_EAST;
RTC_DATA_ATTR static char    s_activeWeatherSource[20] = "GOES-East";

// Battery state — updated once per bar render from AXP2101 PMIC
static int8_t s_batPct = -1;  // -1 = not yet read / unavailable
static int    s_batChargeState = -1;  // Raw AXP2101 STATUS2 (0x01) byte

// Per-frame UTC timestamps — loaded from SD at playback start
#define MAX_FRAMES 144
static time_t s_frameTimes[MAX_FRAMES];
static bool   s_timesLoaded = false;
static uint8_t s_sourceBlackLogged[MAX_FRAMES];

struct FrameStoreIndex {
  uint32_t magic;
  uint16_t head;
  uint16_t count;
  time_t   times[MAX_FRAMES];
  uint32_t jpegLen[MAX_FRAMES];
  uint8_t  jpegValid[MAX_FRAMES];
  uint8_t  rawValid[MAX_FRAMES];
};
static FrameStoreIndex s_idx;
RTC_DATA_ATTR static char s_displayLocationLabel[16] = DISPLAY_TZ_LABEL;
RTC_DATA_ATTR static char s_displayLocationFull[64]  = DISPLAY_TZ_LABEL;

// Stream playback: one contiguous file, kept open during playback.
// s_streamValid[i] == 1 means frame i was successfully decoded and written.
static uint8_t s_streamValid[MAX_FRAMES];
static File    s_streamFile;
static bool    s_streamReady = false;

// Cached valid-frame index list — rebuilt lazily when s_validCount < 0
static int  s_validIdx[MAX_FRAMES];
static int  s_validCount = -1;
static int  s_newestCachedIdx = -1;

#define TIMES_FILE  SD_ROOT "/frames/times.bin"
#define RADAR_META_FILE SD_ROOT "/frames/radar.meta"

RTC_DATA_ATTR static time_t s_lastRadarUtc = 0;
RTC_DATA_ATTR static bool   s_lastRadarUtcValid = false;
static bool s_radarMetaLoaded = false;
static bool s_radarNoSignatures = false;  // download succeeded but no precipitation echoes ("Clear")
RTC_DATA_ATTR static time_t s_lastRadarCheckUtc = 0;  // when radar was last polled (for "Clear" age)
static bool s_radarDownloadFailed = false; // download itself failed — outside coverage or service error ("no sig")
static bool s_topBarUseRadarScanTime = false; // Terrain stage shows radar scan time/min age in top bar.
static bool s_zoomSnapshotsRefreshPending = false;

// ── Moon phase complication ──────────────────────────────────────────────
#define MOON_FRAME_COUNT  30
#define MOON_DECODED_PX   54   // 216 / 4 = 54 (JPEG_SCALE_QUARTER)
static uint16_t* s_moonBuf = nullptr;      // PSRAM, MOON_DECODED_PX² pixels
static bool      s_moonDrawn = false;       // one-shot: AMOLED retains pixels
// Weather zoom stages only refresh when new weather frames were downloaded
// in this sync cycle (or when zoom assets are missing/stale).
static bool s_zoomWeatherRefreshNeeded = false;
struct WifiConfigEntry {
  char ssid[33];
  char pass[65];
};
static WifiConfigEntry s_wifiConfig[WIFI_CONFIG_SLOTS];
static bool s_wifiConfigLoaded = false;
static bool s_clockUse12Hour = true;
enum UpdateMode : uint8_t {
  UPDATE_MODE_MANUAL    = 0,
  UPDATE_MODE_AUTO      = 1,
  UPDATE_MODE_SCHEDULED = 2,
};
enum StartCueMode : uint8_t {
  START_CUE_OFF        = 0,
  START_CUE_CHIME      = 1,  // Chime 1 -> /power_up.raw
  START_CUE_VIBE_PULSE = 2,
  START_CUE_CHIME2     = 3,  // Chime 2 -> /chime2.raw (reserved for future use)
  START_CUE_CHIME3     = 4,  // Chime 3 -> /chime3.raw (reserved for future use)
};
static constexpr size_t START_CUE_PATH_MAX = 96;
static constexpr size_t START_CUE_MAX_BYTES = 1024 * 1024;
static constexpr int MAX_SCHED_UPDATES = 8;
static uint8_t  s_scheduledUpdateCount = 0;
static uint16_t s_scheduledUpdateMinutes[MAX_SCHED_UPDATES] = {};
static UpdateMode s_updateMode = UPDATE_MODE_MANUAL;
static uint16_t s_autoUpdateIntervalMin = 60;
static bool s_autoUpdateTopOfHour = false;
static StartCueMode s_startCueMode = START_CUE_OFF;
static uint8_t s_chimeVolume = 80;
static char s_startCuePath[START_CUE_PATH_MAX] = "/power_up.raw";
static time_t s_lastSuccessfulSyncUtc = 0;
static uint32_t s_lastSuccessfulScanMs = 0;
static bool s_lastSuccessfulScanMsValid = false;
static char s_wifiDisplayName[33] = WIFI_SSID;
static int16_t s_wifiRssi = -127;
static bool s_wifiSyncInProgress = false;
static IPAddress s_wifiPortalApIp(192, 168, 4, 1);
static DNSServer s_wifiPortalDns;
static WebServer s_wifiPortalServer(80);
static bool s_wifiPortalHandlersReady = false;
static bool s_wifiPortalHttpRunning = false;
static bool s_wifiPortalDnsRunning = false;
static bool s_wifiPortalApActive = false;
static bool s_wifiPortalMdnsRunning = false;
static bool s_startCuePending = false;

// Unified sync progress tracker (single progress bar across full sync pipeline).
static bool     s_syncProgActive = false;
static uint32_t s_syncProgTotalUnits = 0;
static uint32_t s_syncProgDoneUnits = 0;
static uint32_t s_syncProgPhaseBase = 0;
static uint32_t s_syncProgPhaseUnits = 0;
static uint32_t s_syncProgCursorUnits = 0;
static char     s_syncProgPhaseLabel[24] = "sync";

RTC_DATA_ATTR static bool s_sleepModeEnabled = true;
RTC_DATA_ATTR static bool s_autoUpdateInSleep = true;

// ── Hurricane Watch ─────────────────────────────────────────────────────────
struct HurricaneInfo {
  char     id[12];         // ATCF ID: "AL092025"
  char     name[20];       // "KATRINA"
  float    lat;            // Current center latitude
  float    lon;            // Current center longitude
  uint8_t  category;       // Saffir-Simpson 0(TD/TS) or 1-5
  uint16_t windKt;         // Max sustained wind (knots)
  uint16_t pressureMb;     // MSLP
  time_t   advisoryUtc;    // Advisory timestamp
  char     stormType[4];   // "HU", "TS", "TD"
};

RTC_DATA_ATTR static bool          s_hurricaneMode = false;
RTC_DATA_ATTR static HurricaneInfo s_activeStorm = {};
RTC_DATA_ATTR static float         s_savedWeatherCenterLat = 0.0f;
RTC_DATA_ATTR static float         s_savedWeatherCenterLon = 0.0f;
RTC_DATA_ATTR static bool          s_savedWeatherGeoValid = false;
RTC_DATA_ATTR static char          s_lastHurricaneAlertId[12] = {};

static bool    s_hurricaneWatchEnabled = false;
static bool    s_hurricaneIncludeTS = false;
static bool    s_hurricaneIncludeTD = false;
static uint8_t s_hurricaneAlertVolume = 200;
static char    s_hurricaneAlertSound[32] = "/hurricane_alert.raw";
static int     s_hurricaneLoopsSinceCheck = 0;
// ─────────────────────────────────────────────────────────────────────────────

static int s_loopsBeforeSleep = LOOPS_BEFORE_SLEEP;
static bool s_shakeToWakeEnabled = false;
static uint16_t s_shakeEntryIgnoreMs = 2000;  // ignore WoM for this long after entering sleep (settle window)
static uint16_t s_shakeConfirmMs     = 600;   // after settle: require 2nd WoM event within this window (0=any single event wakes)
static SensorQMI8658 s_qmi;
static bool s_qmiInitialized = false;
#if BOARD_IS_AMOLED_206
static I2SClass s_audioI2s;
static es8311_handle_t s_audioCodec = nullptr;
static bool s_audioReady = false;
static bool s_audioPathPrimed = false;
struct AudioCueRequest {
  uint8_t volume = 80;
};
static TaskHandle_t s_audioCueTaskHandle = nullptr;
static QueueHandle_t s_audioCueQueue = nullptr;
static volatile bool s_audioCueBusy = false;
static uint8_t* s_audioCueBuf = nullptr;
static size_t s_audioCueLen = 0;
static bool s_audioCueReady = false;
static char s_audioCueLoadedPath[START_CUE_PATH_MAX] = "";
#endif
static bool s_buttonSleepTransition = false;
static void serviceUserButtons();

static constexpr uint32_t TOP_BTN_SHORT_PRESS_MS = 5U;
static constexpr uint32_t TOP_BTN_DEBOUNCE_MS = 4U;
static constexpr uint32_t TOP_BTN_LONG_PRESS_MS = 1500U;
static constexpr uint32_t TOP_BTN_SUPPRESS_MS = 250U;
static constexpr uint32_t TOP_BTN_WAKE_RELEASE_MAX_MS = 0U;
static constexpr uint32_t TOP_BTN_POLL_MS = 1U;
static TaskHandle_t s_topBtnPollTaskHandle = nullptr;
static portMUX_TYPE s_topBtnStateMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_topBtnRawPressed = false;
static volatile bool s_topBtnStablePressed = false;
static volatile bool s_topBtnReleasePending = false;
static volatile uint32_t s_topBtnRawChangedMs = 0;
static volatile uint32_t s_topBtnPressStartMs = 0;
static volatile uint32_t s_topBtnReleaseMs = 0;
static bool s_topBtnIgnoreUntilRelease = false;

struct ClockOverlayLayout {
  int textX, textY;
  int textW, textH;
  float textSize;
  int bgX, bgY, bgW, bgH;  // saved/restore region in s_frameDisplayBuf
};
static void showMessage(const char* line1, const char* line2);
static void serviceWifiPortalServer();
static bool connectWifiForSync(bool required, const char* statusLine);
static bool playStartCueIfEnabled();
#if BOARD_IS_AMOLED_206
static void ensureAudioCueWorker();
static bool preloadSelectedCueToPsram(bool forceReload);
#endif
static void noteSuccessfulScanNow();
static bool autoUpdateDueNow();
static bool syncProgressIsActive();
static void syncProgressBegin(uint32_t totalUnits, const char* line1, const char* line2);
static void syncProgressBeginPhase(const char* label, uint32_t phaseUnits);
static void syncProgressCompletePhase();
static void syncProgressSetPhaseProgress(int current, int total);
static void syncProgressTick(uint32_t units);
static void syncProgressEnd();
static void initFrameStore();
static bool loadIndex();
static void writeIndex();
static bool readJpegFromSlot(int logicalIdx, uint8_t* buf, size_t* outLen);
static bool writeJpegToSlot(int logicalIdx, const uint8_t* buf, size_t len);
static bool writeRawToSlot(int logicalIdx, const uint8_t* buf);
static void syncWeatherFrames();
static void rebuildRawFromStored();

static void portalFriendlyDelay(uint32_t ms) {
  uint32_t deadline = millis() + ms;
  while (true) {
    serviceWifiPortalServer();
    serviceUserButtons();
    int32_t remaining = (int32_t)(deadline - millis());
    if (remaining <= 0) break;
    uint32_t slice = (remaining > 5) ? 5U : (uint32_t)remaining;
    vTaskDelay(pdMS_TO_TICKS(slice));
  }
}

#define delay(ms) portalFriendlyDelay((uint32_t)(ms))

// Read battery SOC from AXP2101 PMIC (I2C 0x34, register 0xA4, bits[6:0] = 0..100 %).
// Returns -1 if the PMIC doesn't respond (no battery / device not present).
static int8_t readAxp2101BatPct() {
  Wire.beginTransmission(0x34);
  Wire.write(0xA4);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)0x34, (uint8_t)1) != 1) return -1;
  int raw = Wire.read() & 0x7F;
  return (int8_t)((raw > 100) ? 100 : raw);
}

// Read raw AXP2101 STATUS2 (0x01) register. Upper bits expose charge/discharge state,
// low 3 bits expose charger phase/done/stop. Returns -1 on I2C failure.
static int readAxp2101ChargeState() {
  Wire.beginTransmission(0x34);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)0x34, (uint8_t)1) != 1) return -1;
  return (int)(Wire.read() & 0xFF);
}

static void refreshCachedWifiDisplayState() {
  if (WiFi.status() != WL_CONNECTED) return;
  String ssid = WiFi.SSID();
  if (ssid.length() > 0) {
    snprintf(s_wifiDisplayName, sizeof(s_wifiDisplayName), "%s", ssid.c_str());
  } else {
    snprintf(s_wifiDisplayName, sizeof(s_wifiDisplayName), "%s", WIFI_SSID);
  }
  int rssi = WiFi.RSSI();
  if (rssi <= 0 && rssi >= -127) s_wifiRssi = (int16_t)rssi;
}

static void refreshWifiDisplayNameFromConfig() {
  for (int i = 0; i < WIFI_CONFIG_SLOTS; ++i) {
    if (s_wifiConfig[i].ssid[0] != '\0') {
      snprintf(s_wifiDisplayName, sizeof(s_wifiDisplayName), "%s", s_wifiConfig[i].ssid);
      return;
    }
  }
  snprintf(s_wifiDisplayName, sizeof(s_wifiDisplayName), "%s", WIFI_SSID);
}

static void loadWifiPortalConfig() {
  if (s_wifiConfigLoaded) return;
  memset(s_wifiConfig, 0, sizeof(s_wifiConfig));
  snprintf(s_wifiConfig[0].ssid, sizeof(s_wifiConfig[0].ssid), "%s", WIFI_SSID);
  snprintf(s_wifiConfig[0].pass, sizeof(s_wifiConfig[0].pass), "%s", WIFI_PASS);
  s_clockUse12Hour = true;
  s_updateMode = UPDATE_MODE_MANUAL;
  s_autoUpdateIntervalMin = 60;
  s_autoUpdateTopOfHour = false;
  s_startCueMode = START_CUE_CHIME;
  s_chimeVolume = 80;
  snprintf(s_startCuePath, sizeof(s_startCuePath), "%s", "/power_up.raw");
  s_sleepModeEnabled = true;
  s_autoUpdateInSleep = true;
  s_loopsBeforeSleep = LOOPS_BEFORE_SLEEP;
  s_shakeToWakeEnabled = false;
  s_lastSuccessfulSyncUtc = 0;
  s_lastSuccessfulScanMsValid = false;
  s_lastSuccessfulScanMs = 0;

  Preferences prefs;
  if (prefs.begin("satwatch", true)) {
    bool inited = prefs.getBool("init", false);
    if (inited) {
      for (int i = 0; i < WIFI_CONFIG_SLOTS; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "s%d", i);
        prefs.getString(key, s_wifiConfig[i].ssid, sizeof(s_wifiConfig[i].ssid));
        snprintf(key, sizeof(key), "p%d", i);
        prefs.getString(key, s_wifiConfig[i].pass, sizeof(s_wifiConfig[i].pass));
      }
      s_clockUse12Hour = prefs.getBool("t12", true);
      int updateMode = prefs.getUChar("updm", (uint8_t)UPDATE_MODE_MANUAL);
      if (updateMode < UPDATE_MODE_MANUAL || updateMode > UPDATE_MODE_SCHEDULED) updateMode = UPDATE_MODE_MANUAL;
      s_updateMode = (UpdateMode)updateMode;
      int updateMins = prefs.getUShort("updi", 60);
      s_autoUpdateTopOfHour = prefs.getBool("upth", false);
      if (s_autoUpdateTopOfHour) {
        updateMins = 60;
      } else if (updateMins == 15 || updateMins == 30 || updateMins == 60) {
        // keep as-is
      } else {
        // Legacy "rescan minutes" values are no longer used; normalize to hourly.
        updateMins = 60;
      }
      s_autoUpdateIntervalMin = (uint16_t)updateMins;
      // load scheduled times
      char schedStr[64] = {};
      prefs.getString("scht", schedStr, sizeof(schedStr));
      s_scheduledUpdateCount = 0;
      char* tok = strtok(schedStr, ",");
      while (tok && s_scheduledUpdateCount < MAX_SCHED_UPDATES) {
        int v = atoi(tok);
        if (v >= 0 && v < 1440) s_scheduledUpdateMinutes[s_scheduledUpdateCount++] = (uint16_t)v;
        tok = strtok(nullptr, ",");
      }
      if (prefs.isKey("chmd")) {
        int mode = prefs.getUChar("chmd", (uint8_t)START_CUE_OFF);
        if (mode < START_CUE_OFF || mode > START_CUE_CHIME3) mode = START_CUE_OFF;
        s_startCueMode = (StartCueMode)mode;
      } else {
        s_startCueMode = prefs.getBool("chm", true) ? START_CUE_CHIME : START_CUE_OFF;
      }
      // Path is always derived from mode; manual cuep is no longer used.
      const char* modePath = cuePathForMode(s_startCueMode);
      snprintf(s_startCuePath, sizeof(s_startCuePath), "%s",
               (modePath && modePath[0]) ? modePath : "/power_up.raw");
      int vol = prefs.getUChar("chmv", 80);
      if (vol < 0) vol = 0;
      if (vol > 100) vol = 100;
      s_chimeVolume = (uint8_t)vol;
      s_sleepModeEnabled = prefs.getBool("slp", true);
      s_autoUpdateInSleep = prefs.getBool("ausl", true);
      int lbsl = prefs.getInt("lbsl", LOOPS_BEFORE_SLEEP);
      if (lbsl < 1) lbsl = 1;
      if (lbsl > 99) lbsl = 99;
      s_loopsBeforeSleep = lbsl;
      s_shakeToWakeEnabled = prefs.getBool("stwk", false);
      {
        int setms = prefs.getInt("stwksetms", 2000);
        if (setms < 0) setms = 0;
        if (setms > 5000) setms = 5000;
        s_shakeEntryIgnoreMs = (uint16_t)setms;
        int dbms = prefs.getInt("stwkdbms", 600);
        if (dbms < 0) dbms = 0;
        if (dbms > 2000) dbms = 2000;
        s_shakeConfirmMs = (uint16_t)dbms;
      }
      s_lastSuccessfulSyncUtc = (time_t)prefs.getULong64("lsyn", 0ULL);
      // Hurricane watch config
      s_hurricaneWatchEnabled = prefs.getBool("hwen", false);
#ifdef HURRICANE_TEST_MODE
      s_hurricaneWatchEnabled = true;  // force-enable for testing
#endif
      s_hurricaneIncludeTS = prefs.getBool("hwts", false);
      s_hurricaneIncludeTD = prefs.getBool("hwtd", false);
      int hwvol = prefs.getUChar("hwvol", 200);
      if (hwvol < 0) hwvol = 0;
      if (hwvol > 255) hwvol = 255;
      s_hurricaneAlertVolume = (uint8_t)hwvol;
      prefs.getString("hwsnd", s_hurricaneAlertSound, sizeof(s_hurricaneAlertSound));
      if (s_hurricaneAlertSound[0] == '\0')
        strlcpy(s_hurricaneAlertSound, "/hurricane_alert.raw", sizeof(s_hurricaneAlertSound));
    }
    prefs.end();
  }

  refreshWifiDisplayNameFromConfig();
  s_wifiConfigLoaded = true;
}

static void saveHurricaneConfig() {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putBool("hwen", s_hurricaneWatchEnabled);
  prefs.putBool("hwts", s_hurricaneIncludeTS);
  prefs.putBool("hwtd", s_hurricaneIncludeTD);
  prefs.putUChar("hwvol", s_hurricaneAlertVolume);
  prefs.putString("hwsnd", s_hurricaneAlertSound);
  prefs.end();
}

static void saveSleepModePreference(bool enabled) {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putBool("slp", enabled);
  prefs.end();
}

static const char* cuePathForMode(StartCueMode mode) {
  switch (mode) {
    case START_CUE_CHIME:      return "/power_up.raw";
    case START_CUE_VIBE_PULSE: return "/230.raw";
    case START_CUE_CHIME2:     return "/chime2.raw";
    case START_CUE_CHIME3:     return "/chime3.raw";
    default:                   return "";
  }
}

static void initQmiShakeToWake() {
  if (!s_qmi.begin(Wire, QMI8658_ADDRESS, SDA, SCL)) {
    appendDiagLog("qmi: init fail\n");
    return;
  }
  // WoM hardware path: resets sensor, arms dedicated threshold comparator.
  // defaultPinValue=1: INT1 idles HIGH (push-pull), pulses LOW on motion.
  // Pair with GPIO_INTR_LOW_LEVEL and gpio_pullup_en in goToSleep().
  // 200mg threshold — sensitive enough for wrist flick, filters table vibration.
  int ret = s_qmi.configWakeOnMotion(255,
                                     SensorQMI8658::ACC_ODR_LOWPOWER_128Hz,
                                     SensorQMI8658::INTERRUPT_PIN_1,
                                     1,     // idle HIGH, goes LOW on motion
                                     0x20); // 32-sample blanking (~250ms)
  if (ret != 0) {
    appendDiagLog("qmi: WoM init fail %d\n", ret);
    return;
  }
  s_qmiInitialized = true;
  appendDiagLog("qmi: shake-to-wake WoM init ok INT1=GPIO%d\n", QMI8658_INT1);
}

static void saveWifiPortalConfig(const WifiConfigEntry* entries, bool use12Hour,
                                 UpdateMode updateMode, uint16_t autoUpdateIntervalMin,
                                 bool autoUpdateTopOfHour,
                                 StartCueMode startCueMode, uint8_t chimeVolume,
                                 bool autoUpdateInSleep,
                                 uint8_t schedCount, const uint16_t* schedMinutes,
                                 int loopsBeforeSleep, bool shakeToWake,
                                 int shakeEntryIgnoreMs, int shakeConfirmMs) {
  if (!entries) return;
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putBool("init", true);
  for (int i = 0; i < WIFI_CONFIG_SLOTS; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "s%d", i);
    prefs.putString(key, entries[i].ssid);
    snprintf(key, sizeof(key), "p%d", i);
    prefs.putString(key, entries[i].pass);
    snprintf(s_wifiConfig[i].ssid, sizeof(s_wifiConfig[i].ssid), "%s", entries[i].ssid);
    snprintf(s_wifiConfig[i].pass, sizeof(s_wifiConfig[i].pass), "%s", entries[i].pass);
  }
  prefs.putBool("t12", use12Hour);
  prefs.putUChar("updm", (uint8_t)updateMode);
  prefs.putUShort("updi", autoUpdateIntervalMin);
  prefs.putBool("upth", autoUpdateTopOfHour);
  prefs.putBool("chm", startCueMode == START_CUE_CHIME);
  prefs.putUChar("chmd", (uint8_t)startCueMode);
  prefs.putUChar("chmv", chimeVolume);
  const char* derivedPath = cuePathForMode(startCueMode);
  prefs.putString("cuep", (derivedPath && derivedPath[0]) ? derivedPath : "/power_up.raw");
  prefs.putBool("ausl", autoUpdateInSleep);
  prefs.putInt("lbsl", loopsBeforeSleep);
  prefs.putBool("stwk", shakeToWake);
  prefs.putInt("stwksetms", shakeEntryIgnoreMs);
  prefs.putInt("stwkdbms", shakeConfirmMs);
  // build and save scheduled times string
  {
    char schedStr[64] = {};
    int pos = 0;
    for (int i = 0; i < schedCount && i < MAX_SCHED_UPDATES; i++) {
      if (pos > 0 && pos < (int)sizeof(schedStr) - 1) schedStr[pos++] = ',';
      pos += snprintf(schedStr + pos, sizeof(schedStr) - pos, "%u", schedMinutes[i]);
    }
    prefs.putString("scht", schedStr);
  }
  prefs.end();
  s_clockUse12Hour = use12Hour;
  s_updateMode = updateMode;
  s_autoUpdateIntervalMin = autoUpdateIntervalMin;
  s_autoUpdateTopOfHour = autoUpdateTopOfHour;
  s_startCueMode = startCueMode;
  s_chimeVolume = chimeVolume;
  snprintf(s_startCuePath, sizeof(s_startCuePath), "%s",
           (derivedPath && derivedPath[0]) ? derivedPath : "/power_up.raw");
  s_autoUpdateInSleep = autoUpdateInSleep;
  s_loopsBeforeSleep = loopsBeforeSleep;
  s_shakeToWakeEnabled = shakeToWake;
  s_shakeEntryIgnoreMs = (uint16_t)shakeEntryIgnoreMs;
  s_shakeConfirmMs = (uint16_t)shakeConfirmMs;
  if (shakeToWake && !s_qmiInitialized && s_sleepModeEnabled) initQmiShakeToWake();
  s_scheduledUpdateCount = schedCount;
  for (int i = 0; i < schedCount; i++) s_scheduledUpdateMinutes[i] = schedMinutes[i];
#if BOARD_IS_AMOLED_206
  ensureAudioCueWorker();
  preloadSelectedCueToPsram(true);
#endif
  s_wifiConfigLoaded = true;
  refreshWifiDisplayNameFromConfig();
}

static void setSkipNextSyncOnce() {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putBool("skp", true);
  prefs.end();
}

static bool consumeSkipNextSyncOnce() {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return false;
  bool skip = prefs.getBool("skp", false);
  if (skip) prefs.putBool("skp", false);
  prefs.end();
  return skip;
}

static void noteSuccessfulScanNow() {
  s_lastSuccessfulScanMs = millis();
  s_lastSuccessfulScanMsValid = true;

  time_t nowUtc = time(nullptr);
  if (nowUtc <= 0) return;
  s_lastSuccessfulSyncUtc = nowUtc;

  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putULong64("lsyn", (uint64_t)nowUtc);
  prefs.end();
}

static bool autoUpdateDueNow() {
  if (s_updateMode == UPDATE_MODE_SCHEDULED) {
    if (s_scheduledUpdateCount == 0) return false;
    time_t nowUtc = time(nullptr);
    if (nowUtc <= 0) return false;
    time_t localNow = nowUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
    int minuteOfDay = (int)((localNow % 86400) / 60);
    if (minuteOfDay < 0) minuteOfDay += 1440;
    bool matched = false;
    for (int i = 0; i < s_scheduledUpdateCount; i++) {
      if (minuteOfDay == (int)s_scheduledUpdateMinutes[i]) { matched = true; break; }
    }
    if (!matched) return false;
    // Only fire once per minute bucket
    int64_t currentBucket = (int64_t)localNow / 60LL;
    if (s_lastSuccessfulSyncUtc <= 0) return true;
    time_t lastLocal = s_lastSuccessfulSyncUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
    return currentBucket != (int64_t)lastLocal / 60LL;
  }
  if (s_updateMode != UPDATE_MODE_AUTO) return false;
  time_t nowUtc = time(nullptr);
  if (nowUtc <= 0) return false;

  int intervalMin = 60;
  if (s_autoUpdateTopOfHour) {
    intervalMin = 60;
  } else if (s_autoUpdateIntervalMin == 15 || s_autoUpdateIntervalMin == 30 || s_autoUpdateIntervalMin == 60) {
    intervalMin = (int)s_autoUpdateIntervalMin;
  }

  time_t localNow = nowUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
  int minute = (int)((localNow % 3600) / 60);
  if (minute < 0) minute += 60;
  if (minute % intervalMin != 0) return false;

  int64_t currentBucket = (int64_t)localNow / ((int64_t)intervalMin * 60LL);
  if (s_lastSuccessfulSyncUtc <= 0) return true;
  time_t lastLocal = s_lastSuccessfulSyncUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
  int64_t lastBucket = (int64_t)lastLocal / ((int64_t)intervalMin * 60LL);
  return currentBucket != lastBucket;
}

static bool wifiHasInternetConnectivity() {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient client;
  HTTPClient http;
  if (!http.begin(client, "http://connectivitycheck.gstatic.com/generate_204")) {
    return false;
  }
  http.setTimeout(3000);
  int code = http.GET();
  http.end();
  return (code == 204 || code == 200);
}

static String htmlEscape(const char* src) {
  String out;
  if (!src) return out;
  while (*src) {
    switch (*src) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += *src; break;
    }
    ++src;
  }
  return out;
}

static String wifiPortalRootUrl() {
  if (WiFi.status() == WL_CONNECTED) {
    return String("http://") + WiFi.localIP().toString() + "/";
  }
  if (s_wifiPortalApActive) {
    return String("http://") + s_wifiPortalApIp.toString() + "/";
  }
  return String("http://") + s_wifiPortalApIp.toString() + "/";
}

static void sendWifiPortalPage() {
  String html;
  html.reserve(5800);
  html += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Sat Watch Setup</title><style>"
            "body{font-family:Arial,sans-serif;background:#111827;color:#f9fafb;margin:0;padding:16px;}"
            "h1{font-size:24px;margin:0 0 8px;} h2{font-size:18px;margin:18px 0 8px;}"
            ".card{background:#1f2937;border-radius:12px;padding:16px;margin-bottom:14px;}"
            "label{display:block;font-size:14px;margin:8px 0 4px;}"
            "input[type=text],input[type=password],select{width:100%;padding:10px;border-radius:8px;border:1px solid #4b5563;background:#111827;color:#f9fafb;box-sizing:border-box;}"
            ".hint{font-size:12px;color:#9ca3af;margin-top:6px;}"
            "button{width:100%;padding:12px;border:0;border-radius:10px;background:#2563eb;color:white;font-size:16px;font-weight:700;margin-top:12px;}"
            "</style></head><body>");
  html += F("<h1>Sat Watch</h1><div class='hint'>Connect this device to your home WiFi and set basic display options.</div>");
  html += F("<form method='POST' action='/save'>");
  html += F("<div class='card'><h2>WiFi Networks</h2>");
  for (int i = 0; i < WIFI_CONFIG_SLOTS; ++i) {
    html += F("<label>");
    if (i == 0) html += F("Primary Network");
    else {
      html += F("Fallback ");
      html += String(i);
    }
    html += F(" SSID</label><input name='ssid");
    html += String(i);
    html += F("' value='");
    html += htmlEscape(s_wifiConfig[i].ssid);
    html += F("'>");
    html += F("<label>Password</label><input type='password' name='pass");
    html += String(i);
    html += F("' value='");
    html += htmlEscape(s_wifiConfig[i].pass);
    html += F("'>");
  }
  html += F("</div>");
  html += F("<div class='card'><h2>Device Settings</h2>");
  html += F("<label>Time Format</label><select name='timefmt'>");
  html += F("<option value='12'");
  if (s_clockUse12Hour) html += F(" selected");
  html += F(">12-hour</option><option value='24'");
  if (!s_clockUse12Hour) html += F(" selected");
  html += F(">24-hour</option></select>");
  html += F("<label style='margin-top:12px;'>Update Mode</label><select name='upmode'>");
  html += F("<option value='0'");
  if (s_updateMode == UPDATE_MODE_MANUAL) html += F(" selected");
  html += F(">Manual update</option><option value='1'");
  if (s_updateMode == UPDATE_MODE_AUTO) html += F(" selected");
  html += F(">Auto update</option></select>");
  html += F("<label style='margin-top:10px;display:flex;align-items:center;gap:8px;'><input type='checkbox' name='uphalf' value='1' style='width:auto;'");
  if (!s_autoUpdateTopOfHour && s_autoUpdateIntervalMin == 30) html += F(" checked");
  html += F(">");
  html += F("Every half hour (30 min)</label>");
  html += F("<label style='margin-top:6px;display:flex;align-items:center;gap:8px;'><input type='checkbox' name='upqtr' value='1' style='width:auto;'");
  if (!s_autoUpdateTopOfHour && s_autoUpdateIntervalMin == 15) html += F(" checked");
  html += F(">");
  html += F("Every quarter hour (15 min)</label>");
  html += F("<label style='margin-top:10px;display:flex;align-items:center;gap:8px;'><input type='checkbox' name='upth' value='1' style='width:auto;'");
  if (s_autoUpdateTopOfHour) html += F(" checked");
  html += F(">");
  html += F("Top of the hour</label>");
  html += F("<div class='hint'>Auto update uses real local clock boundaries only (:00, :15, :30), not minutes-since-boot.</div>");
  html += F("<div class='hint'>Any schedule checkbox forces Auto mode for this save.</div>");
  html += F("<hr style='border-color:#374151;margin:14px 0;'>");
  html += F("<div style='font-size:14px;margin-bottom:6px;font-weight:600;'>Update at local time</div>");
  html += F("<div id='sched-times-container'></div>");
  html += F("<button type='button' id='add-sched-btn' onclick='addSchedTime()' style='width:auto;padding:6px 14px;font-size:13px;margin-top:6px;background:#374151;'>+ Add update time</button>");
  html += F("<div class='hint'>Times use local clock. Scheduling specific times disables the interval options above (and vice-versa).</div>");
  // Emit hidden inputs with saved scheduled times so JS can read them on load
  for (int i = 0; i < s_scheduledUpdateCount; i++) {
    html += F("<input type='hidden' class='sched-preload' value='");
    html += String(s_scheduledUpdateMinutes[i]);
    html += F("'>");
  }
  html += F("<label style='margin-top:12px;'>Start Cue</label><select name='cuemode'>");
  html += F("<option value='0'");
  if (s_startCueMode == START_CUE_OFF) html += F(" selected");
  html += F(">Off</option><option value='1'");
  if (s_startCueMode == START_CUE_CHIME) html += F(" selected");
  html += F(">Chime 1</option><option value='3'");
  if (s_startCueMode == START_CUE_CHIME2) html += F(" selected");
  html += F(">Chime 2</option><option value='4'");
  if (s_startCueMode == START_CUE_CHIME3) html += F(" selected");
  html += F(">Chime 3</option><option value='2'");
  if (s_startCueMode == START_CUE_VIBE_PULSE) html += F(" selected");
  html += F(">Vibe pulse</option></select>");
  html += F("<label>Chime Volume (0-100)</label><input type='text' name='chimevol' value='");
  html += String((unsigned)s_chimeVolume);
  html += F("'>");
  html += F("<div class='hint'>Chime 1 = power_up.raw &nbsp;|&nbsp; Chime 2 = chime2.raw &nbsp;|&nbsp; Chime 3 = chime3.raw &nbsp;|&nbsp; Vibe pulse uses fixed volume 80.</div>");
  html += F("<div class='mb-4'><label class='block text-sm font-medium text-gray-300 mb-1'>Sleep behaviour</label>");
  html += F("<label class='flex items-center gap-2 text-sm text-gray-200'>");
  html += F("<input type='checkbox' name='ausl' value='1'");
  if (s_autoUpdateInSleep) html += F(" checked");
  html += F(">Wake device from sleep on auto update</label>");
  html += F("<div style='margin-top:8px'><label class='text-sm text-gray-200'>Animation loops before sleep: ");
  html += F("<input type='number' name='lbsl' min='1' max='99' style='width:4em' value='");
  html += String(s_loopsBeforeSleep);
  html += F("'></label></div>");
  html += F("<div style='margin-top:8px'><label class='flex items-center gap-2 text-sm text-gray-200'>");
  html += F("<input type='checkbox' name='stwk' value='1'");
  if (s_shakeToWakeEnabled) html += F(" checked");
  html += F(">Shake to wake (wrist flick wakes from sleep)</label></div>");
  html += F("<div style='margin-top:8px'><label class='text-sm text-gray-200'>Shake settle delay (ms): <input type='number' name='stwksetms' min='0' max='5000' style='width:5em' value='");
  html += String((unsigned)s_shakeEntryIgnoreMs);
  html += F("'></label><div class='hint'>Ignore motion for this long after pressing sleep (filters arm-settling). Default 2000 ms.</div></div>");
  html += F("<div style='margin-top:8px'><label class='text-sm text-gray-200'>Shake confirm window (ms): <input type='number' name='stwkdbms' min='0' max='2000' style='width:5em' value='");
  html += String((unsigned)s_shakeConfirmMs);
  html += F("'></label><div class='hint'>After the settle delay: require a 2nd motion event within this window. Filters single bumps/knocks. Default 600 ms. 0 = any single motion wakes.</div></div>");
  html += F("</div>");
  html += F("<div class='hint'>Portal AP: Sat Watch / 123456789</div>");
  html += F("<div class='hint'>Portal URL: http://satwatch.local/</div>");
  if (WiFi.status() == WL_CONNECTED) {
    html += F("<div class='hint'>Local portal: ");
    html += htmlEscape(WiFi.localIP().toString().c_str());
    html += F("</div>");
  }
  html += F("</div><button type='submit'>Save and Reboot</button></form>");
  html += F("<script>"
            "var s_rowCount=0;"
            "function minToHMAmpm(m){"
              "var h=Math.floor(m/60),mn=m%60,ap='AM';"
              "if(h===0){h=12;}else if(h===12){ap='PM';}else if(h>12){h-=12;ap='PM';}"
              "return {h:h,m:mn,ap:ap};"
            "}"
            "function addSchedTime(minsVal){"
              "var idx=s_rowCount++;"
              "var container=document.getElementById('sched-times-container');"
              "if(!container)return;"
              "var hv=12,mv=0,pv='AM';"
              "if(minsVal!==undefined&&minsVal!==null&&minsVal!==''){"
                "var hma=minToHMAmpm(parseInt(minsVal));"
                "hv=hma.h;mv=hma.m;pv=hma.ap;"
              "}"
              "var row=document.createElement('div');"
              "row.className='sched-row';"
              "row.style.cssText='display:flex;align-items:center;gap:8px;margin-bottom:6px;';"
              "row.innerHTML="
                "'<input class=\"sh\" name=\"uth_'+idx+'\" type=\"number\" min=\"1\" max=\"12\" value=\"'+hv+'\" style=\"width:60px;\">'"
                "+'<span>:</span>'"
                "+'<input class=\"sm\" name=\"utm_'+idx+'\" type=\"number\" min=\"0\" max=\"59\" value=\"'+(mv<10?'0':'')+mv+'\" style=\"width:60px;\">'"
                "+'<label style=\"display:flex;align-items:center;gap:4px;font-weight:normal;\"><input class=\"sp\" type=\"checkbox\" name=\"utp_'+idx+'\" value=\"1\" style=\"width:auto;\"'+(pv==='PM'?' checked':'')+'>PM</label>'"
                "+'<button type=\"button\" onclick=\"this.closest(\\'.sched-row\\').remove();syncSchedLock();\" style=\"width:auto;padding:4px 10px;font-size:13px;background:#7f1d1d;\">&#10005;</button>';"
              "var inputs=row.querySelectorAll('input');"
              "inputs.forEach(function(el){el.addEventListener('change',syncSchedLock);});"
              "container.appendChild(row);"
              "syncSchedLock();"
            "}"
            "function syncSchedLock(){"
              "var mode=document.querySelector(\"select[name='upmode']\");"
              "var half=document.querySelector(\"input[name='uphalf']\");"
              "var qtr=document.querySelector(\"input[name='upqtr']\");"
              "var top=document.querySelector(\"input[name='upth']\");"
              "if(!mode||!half||!qtr||!top)return;"
              "var rows=document.querySelectorAll('.sched-row');"
              "var hasSched=false;"
              "rows.forEach(function(r){"
                "var h=r.querySelector('.sh').value,m=r.querySelector('.sm').value;"
                "if(h!==''||m!=='')hasSched=true;"
              "});"
              "var hasInterval=(half.checked||qtr.checked||top.checked);"
              "[half,qtr,top].forEach(function(el){el.disabled=hasSched;});"
              "if(hasSched){mode.value='1';mode.disabled=true;}"
              "else if(!hasInterval){mode.disabled=false;}"
              "var addBtn=document.getElementById('add-sched-btn');"
              "var schedArea=document.getElementById('sched-times-container');"
              "var lock=hasInterval;"
              "if(schedArea){schedArea.style.opacity=lock?'0.4':'';schedArea.style.pointerEvents=lock?'none':'';}"
              "if(addBtn)addBtn.disabled=lock;"
            "}"
            "function bindUpdatePresetHooks(){"
              "var half=document.querySelector(\"input[name='uphalf']\");"
              "var qtr=document.querySelector(\"input[name='upqtr']\");"
              "var top=document.querySelector(\"input[name='upth']\");"
              "if(!half||!qtr||!top)return;"
              "half.addEventListener('change',function(){if(half.checked)qtr.checked=false;syncSchedLock();});"
              "qtr.addEventListener('change',function(){if(qtr.checked)half.checked=false;syncSchedLock();});"
              "top.addEventListener('change',syncSchedLock);"
              "syncSchedLock();"
            "}"
            "document.addEventListener('DOMContentLoaded',function(){"
              "bindUpdatePresetHooks();"
              "var preloads=document.querySelectorAll('.sched-preload');"
              "preloads.forEach(function(el){addSchedTime(el.value);});"
              "loadDiag();"
            "});"
            "function loadDiag(){"
              "var b=document.getElementById('diagbox');"
              "if(b)b.textContent='(loading...)';"
              "fetch('/diag').then(function(r){return r.text();})"
              ".then(function(t){if(b)b.textContent=t||'(empty)';b.scrollTop=b.scrollHeight;})"
              ".catch(function(){if(b)b.textContent='(fetch failed)';});"
            "}"
            "</script>"
            "<div class='card' style='margin-top:14px;border:1px solid #f59e0b;'>"
              "<h2 style='color:#fbbf24;margin-bottom:8px;'>Hurricane Watch</h2>"
              "<label style='display:flex;align-items:center;gap:6px;margin-bottom:8px;'>"
                "<input type='checkbox' name='hwen' value='1' style='width:auto;'"
  );
  if (s_hurricaneWatchEnabled) html += " checked";
  html += F(
                "> Watch for Atlantic hurricanes (Cat 1+)"
              "</label>"
              "<label style='display:flex;align-items:center;gap:6px;margin-bottom:8px;'>"
                "<input type='checkbox' name='hwts' value='1' style='width:auto;'"
  );
  if (s_hurricaneIncludeTS) html += " checked";
  html += F(
                "> Also notify for tropical storms"
              "</label>"
              "<label style='display:flex;align-items:center;gap:6px;margin-bottom:8px;'>"
                "<input type='checkbox' name='hwtd' value='1' style='width:auto;'"
  );
  if (s_hurricaneIncludeTD) html += " checked";
  html += F(
                "> Also notify for tropical depressions"
              "</label>"
              "<div class='hint' style='margin-top:4px;'>Alert volume (0-255): </div>"
              "<input type='number' name='hwvol' min='0' max='255' value='"
  );
  html += String((int)s_hurricaneAlertVolume);
  html += F(
              "' style='width:80px;'>"
              "<div class='hint' style='margin-top:4px;'>Alert sound SD path: </div>"
              "<input type='text' name='hwsnd' value='"
  );
  html += String(s_hurricaneAlertSound);
  html += F(
              "' style='width:100%;'>"
  );
  // Show suppression list (read-only)
  {
    Preferences prefs;
    char supList[128] = {};
    if (prefs.begin("satwatch", true)) {
      prefs.getString("hwsup", supList, sizeof(supList));
      prefs.end();
    }
    if (supList[0] != '\0') {
      html += F("<div class='hint' style='margin-top:8px;'>Suppressed storms: ");
      html += String(supList);
      html += F("</div>");
    }
  }
  html += F(
              "<form method='POST' action='/hwclearsup' style='margin-top:8px;'>"
                "<button type='submit' style='background:#374151;width:auto;padding:4px 14px;font-size:13px;'>Clear suppression list</button>"
              "</form>"
            "</div>"
            "<div class='card' style='margin-top:16px;'>"
              "<h2 style='display:flex;align-items:center;gap:10px;margin-bottom:10px;'>Boot Log"
              "<button type='button' onclick='loadDiag()' style='width:auto;padding:4px 14px;font-size:13px;margin:0;background:#374151;'>Refresh</button>"
              "</h2>"
              "<pre id='diagbox' style='background:#0a0f1a;color:#86efac;font-size:11px;line-height:1.5;padding:10px;border-radius:8px;overflow:auto;max-height:340px;white-space:pre-wrap;word-break:break-all;margin:0;'>(loading...)</pre>"
            "</div>"
            "<div class='card' style='margin-top:14px;border:1px solid #dc2626;'>"
              "<h2 style='color:#f87171;margin-bottom:8px;'>Maintenance</h2>"
              "<div class='hint' style='margin-bottom:10px;'>Deletes all cached satellite frames and forces a full re-download on reboot. Use after moving locations or if frames look corrupted.</div>"
              "<form method='POST' action='/clearframes' onsubmit=\"return confirm('Delete all cached frames and reboot?');\">"
                "<button type='submit' style='background:#dc2626;'>Clear Frames &amp; Reboot</button>"
              "</form>"
            "</div>"
            "</body></html>");
  s_wifiPortalServer.send(200, "text/html", html);
}

static void redirectWifiPortalRoot() {
  s_wifiPortalServer.sendHeader("Location", "/", true);
  s_wifiPortalServer.send(302, "text/plain", "");
}

static void handleWifiPortalSave() {
  WifiConfigEntry entries[WIFI_CONFIG_SLOTS] = {};
  for (int i = 0; i < WIFI_CONFIG_SLOTS; ++i) {
    String ssid = s_wifiPortalServer.arg(String("ssid") + String(i));
    String pass = s_wifiPortalServer.arg(String("pass") + String(i));
    ssid.trim();
    pass.trim();
    ssid.toCharArray(entries[i].ssid, sizeof(entries[i].ssid));
    pass.toCharArray(entries[i].pass, sizeof(entries[i].pass));
  }
  bool use12 = s_wifiPortalServer.arg("timefmt") != "24";
  int upMode = s_wifiPortalServer.arg("upmode").toInt();
  int upMins = 60;
  bool upTopHour = s_wifiPortalServer.hasArg("upth");
  bool upHalfHour = s_wifiPortalServer.hasArg("uphalf");
  bool upQuarterHour = s_wifiPortalServer.hasArg("upqtr");
  // Parse scheduled times from form (rows named uth_N, utm_N, utp_N)
  uint16_t schedMins[MAX_SCHED_UPDATES] = {};
  uint8_t  schedCount = 0;
  for (int i = 0; i < MAX_SCHED_UPDATES; i++) {
    String hStr = s_wifiPortalServer.arg(String("uth_") + i);
    String mStr = s_wifiPortalServer.arg(String("utm_") + i);
    if (!hStr.length()) break;  // rows are dense from 0
    int h = hStr.toInt(), m = mStr.toInt();
    bool pm = s_wifiPortalServer.hasArg(String("utp_") + i);
    if (h < 1 || h > 12 || m < 0 || m > 59) continue;
    if (h == 12) h = 0;
    if (pm) h += 12;
    schedMins[schedCount++] = (uint16_t)(h * 60 + m);
  }
  // Conflict resolution: scheduled times take priority over interval
  if (schedCount > 0) {
    upMode = UPDATE_MODE_SCHEDULED;
    upMins = 60;
    upTopHour = false;
  } else {
    if (upHalfHour || upQuarterHour || upTopHour) upMode = UPDATE_MODE_AUTO;
    if (upMode < UPDATE_MODE_MANUAL || upMode > UPDATE_MODE_SCHEDULED) upMode = UPDATE_MODE_MANUAL;
    if (upQuarterHour) {
      upMins = 15;
      upTopHour = false;
    } else if (upHalfHour) {
      upMins = 30;
      upTopHour = false;
    } else if (upTopHour) {
      upMins = 60;
    } else if (upMode == UPDATE_MODE_AUTO) {
      // Default auto schedule with no preset selected.
      upMins = 60;
      upTopHour = true;
    }
  }
  int mode = s_wifiPortalServer.arg("cuemode").toInt();
  if (mode < START_CUE_OFF || mode > START_CUE_CHIME3) mode = START_CUE_OFF;
  int chimeVol = s_wifiPortalServer.arg("chimevol").toInt();
  if (chimeVol < 0) chimeVol = 0;
  if (chimeVol > 100) chimeVol = 100;
  bool autoUpdateInSleep = s_wifiPortalServer.hasArg("ausl");
  int loopsBeforeSleep = s_wifiPortalServer.arg("lbsl").toInt();
  if (loopsBeforeSleep < 1) loopsBeforeSleep = 1;
  if (loopsBeforeSleep > 99) loopsBeforeSleep = 99;
  bool shakeToWake = s_wifiPortalServer.hasArg("stwk");
  int shakeEntryIgnoreMs = s_wifiPortalServer.arg("stwksetms").toInt();
  if (shakeEntryIgnoreMs < 0) shakeEntryIgnoreMs = 0;
  if (shakeEntryIgnoreMs > 5000) shakeEntryIgnoreMs = 5000;
  int shakeConfirmMs = s_wifiPortalServer.arg("stwkdbms").toInt();
  if (shakeConfirmMs < 0) shakeConfirmMs = 0;
  if (shakeConfirmMs > 2000) shakeConfirmMs = 2000;
  // Hurricane watch config
  s_hurricaneWatchEnabled = s_wifiPortalServer.hasArg("hwen");
  s_hurricaneIncludeTS = s_wifiPortalServer.hasArg("hwts");
  s_hurricaneIncludeTD = s_wifiPortalServer.hasArg("hwtd");
  {
    int hvol = s_wifiPortalServer.arg("hwvol").toInt();
    if (hvol < 0) hvol = 0;
    if (hvol > 255) hvol = 255;
    s_hurricaneAlertVolume = (uint8_t)hvol;
  }
  {
    String hwsnd = s_wifiPortalServer.arg("hwsnd");
    hwsnd.trim();
    if (hwsnd.length() > 0)
      hwsnd.toCharArray(s_hurricaneAlertSound, sizeof(s_hurricaneAlertSound));
  }
  saveHurricaneConfig();

  saveWifiPortalConfig(entries, use12, (UpdateMode)upMode, (uint16_t)upMins, upTopHour,
                       (StartCueMode)mode, (uint8_t)chimeVol, autoUpdateInSleep,
                       schedCount, schedMins, loopsBeforeSleep, shakeToWake,
                       shakeEntryIgnoreMs, shakeConfirmMs);
  s_wifiPortalServer.send(200, "text/html",
                          "<!doctype html><html><body style='font-family:Arial;background:#111827;color:#f9fafb;padding:24px;'>"
                          "<h2>Saved</h2><p>Settings stored. Rebooting now.</p></body></html>");
  delay(600);
  SD_MMC.end();
  ESP.restart();
}

static void sendDiagHandler() {
  s_wifiPortalServer.sendHeader("Cache-Control", "no-cache");
  File f = SD.open(SD_ROOT "/diag.txt", FILE_READ);
  if (!f) {
    s_wifiPortalServer.send(200, "text/plain", "(no diag.txt on SD)");
    return;
  }
  s_wifiPortalServer.streamFile(f, "text/plain");
  f.close();
}

static void handleClearHurricaneSuppression() {
  Preferences prefs;
  if (prefs.begin("satwatch", false)) {
    prefs.putString("hwsup", "");
    prefs.end();
  }
  s_wifiPortalServer.sendHeader("Location", "/", true);
  s_wifiPortalServer.send(302, "text/plain", "");
}

static void handleClearFrames() {
  // Invalidate the frame cache so the next boot triggers a full re-download.
  SD.remove(INDEX_BIN_FILE);
  SD.remove(INDEX_TMP_FILE);
  SD.remove(META_FILE);           // legacy cleanup
  SD.remove(CACHE_VALIDATE_META_FILE);  // legacy cleanup
  // Clear the skip-sync NVS flag so the reboot doesn't bypass the sync.
  {
    Preferences prefs;
    if (prefs.begin("satwatch", false)) {
      prefs.putBool("skp", false);
      prefs.end();
    }
  }
  appendDiagLog("portal: clear-frames requested\n");
  s_wifiPortalServer.send(200, "text/html",
    "<!doctype html><html><body style='font-family:Arial;background:#111827;"
    "color:#f9fafb;padding:24px;'>"
    "<h2>Cache Cleared</h2>"
    "<p>Frame cache invalidated. Rebooting now to re-download all frames.</p>"
    "</body></html>");
  delay(600);
  SD_MMC.end();
  ESP.restart();
}

static void handleServeFrame() {
  if (!s_wifiPortalServer.hasArg("idx")) {
    s_wifiPortalServer.send(400, "text/plain", "missing idx param");
    return;
  }
  int idx = s_wifiPortalServer.arg("idx").toInt();
  if (idx < 0 || idx >= (int)s_idx.count || !s_idx.jpegValid[idx]) {
    s_wifiPortalServer.send(404, "text/plain", "frame not available");
    return;
  }
  int phys = ((int)s_idx.head + idx) % MAX_FRAMES;
  size_t len = s_idx.jpegLen[idx];
  if (len == 0 || len > JPEG_SLOT_BYTES) {
    s_wifiPortalServer.send(404, "text/plain", "invalid frame length");
    return;
  }
  File f = SD.open(FRAMES_BIN_FILE, FILE_READ);
  if (!f) {
    s_wifiPortalServer.send(500, "text/plain", "frames.bin open error");
    return;
  }
  f.seek((uint32_t)phys * JPEG_SLOT_BYTES);
  s_wifiPortalServer.sendHeader("Cache-Control", "no-cache");
  s_wifiPortalServer.setContentLength(len);
  s_wifiPortalServer.send(200, "image/jpeg", "");
  WiFiClient& cl = s_wifiPortalServer.client();
  uint8_t chunk[1024];
  size_t remaining = len;
  while (remaining > 0) {
    size_t n = f.read(chunk, min(remaining, sizeof(chunk)));
    if (n == 0) break;
    cl.write(chunk, n);
    remaining -= n;
  }
  f.close();
}

static void handleGravity() {
  // Temporarily configure QMI8658 for normal accel reads (WoM mode doesn't allow it)
  SensorQMI8658 grav;
  if (!grav.begin(Wire, QMI8658_ADDRESS, SDA, SCL)) {
    s_wifiPortalServer.send(500, "application/json", "{\"error\":\"qmi init fail\"}");
    return;
  }
  grav.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                           SensorQMI8658::ACC_ODR_1000Hz,
                           SensorQMI8658::LPF_MODE_3);
  grav.enableAccelerometer();
  delay(50);  // let filter settle

  const int N = 4000;
  double sx = 0, sy = 0, sz = 0;
  int good = 0;
  for (int i = 0; i < N; i++) {
    float x, y, z;
    if (grav.getAccelerometer(x, y, z)) {
      sx += x; sy += y; sz += z;
      good++;
    }
    delayMicroseconds(500);  // ~2 kHz polling for 1 kHz ODR
  }
  // Restore WoM if it was active
  if (s_qmiInitialized) initQmiShakeToWake();

  if (good == 0) {
    s_wifiPortalServer.send(500, "application/json", "{\"error\":\"no readings\"}");
    return;
  }
  double ax = sx / good;
  double ay = sy / good;
  double az = sz / good;
  double mag = sqrt(ax * ax + ay * ay + az * az);

  char json[256];
  snprintf(json, sizeof(json),
    "{\"samples\":%d,\"ax\":%.6f,\"ay\":%.6f,\"az\":%.6f,\"magnitude_g\":%.6f,\"magnitude_ms2\":%.6f}",
    good, ax, ay, az, mag, mag * 9.80665);
  s_wifiPortalServer.send(200, "application/json", json);
}

static void sendLsHandler() {
  s_wifiPortalServer.sendHeader("Cache-Control", "no-cache");
  String path = s_wifiPortalServer.hasArg("path") ? s_wifiPortalServer.arg("path") : String("/");
  String out;
  out.reserve(2048);
  out += "ls: ";
  out += path;
  out += "\n\n";
  File dir = SD.open(path);
  if (!dir) {
    out += "(not found)\n";
    s_wifiPortalServer.send(200, "text/plain", out);
    return;
  }
  if (!dir.isDirectory()) {
    out += "(not a directory, size=";
    out += String((unsigned long)dir.size());
    out += ")\n";
    dir.close();
    s_wifiPortalServer.send(200, "text/plain", out);
    return;
  }
  int count = 0;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (entry.isDirectory()) {
      out += "  [DIR]  ";
    } else {
      char sz[16];
      snprintf(sz, sizeof(sz), "%7lu  ", (unsigned long)entry.size());
      out += "  ";
      out += sz;
    }
    out += entry.name();
    out += "\n";
    entry.close();
    count++;
  }
  dir.close();
  out += "\n";
  out += String(count);
  out += " entries\n";
  s_wifiPortalServer.send(200, "text/plain", out);
}

static void ensureWifiPortalHandlers() {
  if (s_wifiPortalHandlersReady) return;
  s_wifiPortalServer.on("/", HTTP_GET, sendWifiPortalPage);
  s_wifiPortalServer.on("/generate_204", HTTP_ANY, sendWifiPortalPage);
  s_wifiPortalServer.on("/gen_204", HTTP_ANY, sendWifiPortalPage);
  s_wifiPortalServer.on("/hotspot-detect.html", HTTP_ANY, sendWifiPortalPage);
  s_wifiPortalServer.on("/connecttest.txt", HTTP_ANY, sendWifiPortalPage);
  s_wifiPortalServer.on("/fwlink", HTTP_ANY, sendWifiPortalPage);
  s_wifiPortalServer.on("/save", HTTP_POST, handleWifiPortalSave);
  s_wifiPortalServer.on("/diag", HTTP_GET, sendDiagHandler);
  s_wifiPortalServer.on("/ls", HTTP_GET, sendLsHandler);
  s_wifiPortalServer.on("/clearframes", HTTP_POST, handleClearFrames);
  s_wifiPortalServer.on("/hwclearsup", HTTP_POST, handleClearHurricaneSuppression);
  s_wifiPortalServer.on("/frame", HTTP_GET, handleServeFrame);
  s_wifiPortalServer.on("/gravity", HTTP_GET, handleGravity);
  s_wifiPortalServer.onNotFound(redirectWifiPortalRoot);
  s_wifiPortalHandlersReady = true;
}

#if BOARD_IS_AMOLED_206
static bool initAudioStartCue() {
  if (s_audioReady) return true;

  pinMode(46, OUTPUT);  // PA_CTRL
  digitalWrite(46, LOW);  // Keep amp path off until codec is stable and muted.

  s_audioI2s.setPins(41, 45, 40, 42, 16);  // bclk, lrck, dout, din, mclk
  if (!s_audioI2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT,
                        I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("audio: i2s init failed");
    return false;
  }

  s_audioCodec = es8311_create(0, ES8311_ADDRRES_0);
  if (!s_audioCodec) {
    Serial.println("audio: codec create failed");
    return false;
  }

  const es8311_clock_config_t clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = 16000 * 256,
    .sample_frequency = 16000,
  };
  if (es8311_init(s_audioCodec, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK ||
      es8311_sample_frequency_config(s_audioCodec, clk.mclk_frequency, clk.sample_frequency) != ESP_OK ||
      es8311_microphone_config(s_audioCodec, false) != ESP_OK) {
    Serial.println("audio: codec init failed");
    return false;
  }

  int unused = 0;
  es8311_voice_fade(s_audioCodec, ES8311_FADE_4096LRCK);
  es8311_voice_mute(s_audioCodec, true);
  es8311_voice_volume_set(s_audioCodec, (int)s_chimeVolume, &unused);
  s_audioPathPrimed = false;
  s_audioReady = true;
  return true;
}

static void writeAudioSilenceMs(uint32_t ms) {
  if (!s_audioReady || ms == 0) return;
  static uint8_t silence[512] = {};
  const size_t bytesPerMs = (size_t)(16000U * 2U * 2U) / 1000U;  // 64 bytes/ms @ 16kHz stereo 16-bit
  size_t remaining = (size_t)ms * bytesPerMs;
  while (remaining > 0) {
    size_t chunk = (remaining > sizeof(silence)) ? sizeof(silence) : remaining;
    size_t wrote = s_audioI2s.write(silence, chunk);
    if (wrote == 0) break;
    remaining -= wrote;
    taskYIELD();
  }
}

static void primeAudioPathForCue() {
  if (!s_audioReady || s_audioPathPrimed) return;
  es8311_voice_mute(s_audioCodec, true);
  writeAudioSilenceMs(30);   // settle I2S stream at zero before enabling PA
  digitalWrite(46, HIGH);    // enable amp path after digital side is quiet
  vTaskDelay(pdMS_TO_TICKS(15));
  writeAudioSilenceMs(20);   // let amp/output bias settle
  s_audioPathPrimed = true;
}

static bool playRawCueFromBuffer(const uint8_t* data, size_t len, uint8_t volume) {
  if (!data || len == 0) return false;
  if (!initAudioStartCue()) return false;
  primeAudioPathForCue();

  int unused = 0;
  es8311_voice_fade(s_audioCodec, ES8311_FADE_4096LRCK);
  es8311_voice_mute(s_audioCodec, true);
  es8311_voice_volume_set(s_audioCodec, (int)volume, &unused);
  writeAudioSilenceMs(8);
  es8311_voice_mute(s_audioCodec, false);
  vTaskDelay(pdMS_TO_TICKS(3));

  size_t off = 0;
  while (off < len) {
    size_t chunk = len - off;
    if (chunk > 4096U) chunk = 4096U;
    size_t wrote = s_audioI2s.write(data + off, chunk);
    if (wrote == 0) break;
    off += wrote;
    taskYIELD();
  }
  writeAudioSilenceMs(6);
  es8311_voice_mute(s_audioCodec, true);
  return off == len;
}

static bool resolveSelectedCuePathAndVolume(char* outPath, size_t outPathLen, uint8_t* outVolume) {
  if (s_startCueMode == START_CUE_OFF) return false;
  const char* selectedPath = cuePathForMode(s_startCueMode);
  if (!selectedPath || !selectedPath[0]) selectedPath = "/power_up.raw";
  if (outPath && outPathLen > 0) {
    snprintf(outPath, outPathLen, "%s", selectedPath);
  }
  if (outVolume) {
    *outVolume = (s_startCueMode == START_CUE_VIBE_PULSE) ? 80 : s_chimeVolume;
  }
  return true;
}

static void unloadActiveCueBuffer() {
  if (s_audioCueBuf) {
    heap_caps_free(s_audioCueBuf);
    s_audioCueBuf = nullptr;
  }
  s_audioCueLen = 0;
  s_audioCueReady = false;
  s_audioCueLoadedPath[0] = '\0';
}

static bool preloadSelectedCueToPsram(bool forceReload) {
  char cuePath[START_CUE_PATH_MAX] = {};
  uint8_t cueVol = 0;
  if (!resolveSelectedCuePathAndVolume(cuePath, sizeof(cuePath), &cueVol)) {
    unloadActiveCueBuffer();
    return false;
  }
  (void)cueVol;
  if (!forceReload && s_audioCueReady && s_audioCueBuf &&
      strcmp(s_audioCueLoadedPath, cuePath) == 0) {
    return true;
  }

  File cueFile = SD.open(cuePath, FILE_READ);
  if (!cueFile) {
    Serial.printf("audio: preload open fail %s\n", cuePath);
    // Fallback to default startup cue if custom/selected file is missing.
    if (strcmp(cuePath, "/power_up.raw") != 0) {
      snprintf(cuePath, sizeof(cuePath), "%s", "/power_up.raw");
      cueFile = SD.open(cuePath, FILE_READ);
    }
    if (!cueFile) return false;
  }

  size_t cueLen = (size_t)cueFile.size();
  if (cueLen == 0 || cueLen > START_CUE_MAX_BYTES) {
    cueFile.close();
    Serial.printf("audio: preload size bad %s len=%u\n", cuePath, (unsigned)cueLen);
    return false;
  }

  uint8_t* newBuf = (uint8_t*)heap_caps_malloc(cueLen, MALLOC_CAP_SPIRAM);
  if (!newBuf) {
    cueFile.close();
    Serial.printf("audio: preload alloc fail len=%u\n", (unsigned)cueLen);
    return false;
  }

  size_t got = cueFile.read(newBuf, cueLen);
  cueFile.close();
  if (got != cueLen) {
    heap_caps_free(newBuf);
    Serial.printf("audio: preload short read %u/%u\n", (unsigned)got, (unsigned)cueLen);
    return false;
  }

  unloadActiveCueBuffer();
  s_audioCueBuf = newBuf;
  s_audioCueLen = cueLen;
  s_audioCueReady = true;
  snprintf(s_audioCueLoadedPath, sizeof(s_audioCueLoadedPath), "%s", cuePath);
  Serial.printf("audio: preloaded %s len=%u\n", s_audioCueLoadedPath, (unsigned)s_audioCueLen);
  return true;
}

static void audioCueWorkerTask(void* arg) {
  (void)arg;
  AudioCueRequest req{};
  for (;;) {
    if (xQueueReceive(s_audioCueQueue, &req, portMAX_DELAY) != pdTRUE) continue;
    s_audioCueBusy = true;
    if (s_audioCueReady && s_audioCueBuf && s_audioCueLen > 0) {
      playRawCueFromBuffer(s_audioCueBuf, s_audioCueLen, req.volume);
    }
    s_audioCueBusy = false;
  }
}

static void ensureAudioCueWorker() {
  if (!s_audioCueQueue) {
    s_audioCueQueue = xQueueCreate(1, sizeof(AudioCueRequest));
  }
  if (!s_audioCueTaskHandle && s_audioCueQueue) {
    BaseType_t ok = xTaskCreatePinnedToCore(
      audioCueWorkerTask,
      "audio_cue",
      4096,
      nullptr,
      2,
      &s_audioCueTaskHandle,
      0
    );
    if (ok != pdPASS) {
      s_audioCueTaskHandle = nullptr;
    }
  }
}
#endif

static bool playStartCueIfEnabled() {
#if BOARD_IS_AMOLED_206
  char cuePath[START_CUE_PATH_MAX] = {};
  uint8_t cueVol = 0;
  if (!resolveSelectedCuePathAndVolume(cuePath, sizeof(cuePath), &cueVol)) return true;
  if (!s_audioCueReady || strcmp(s_audioCueLoadedPath, cuePath) != 0) {
    if (!preloadSelectedCueToPsram(false)) {
      Serial.println("audio: cue not ready");
      return false;
    }
  }
  ensureAudioCueWorker();
  if (!s_audioCueQueue || s_audioCueBusy) return false;
  AudioCueRequest req{};
  req.volume = cueVol;
  if (uxQueueSpacesAvailable(s_audioCueQueue) == 0) return false;
  return (xQueueSend(s_audioCueQueue, &req, 0) == pdTRUE);
#else
  (void)0;
  return false;
#endif
}

static int readAxp2101Register(uint8_t reg) {
#if BOARD_IS_AMOLED_206
  Wire.beginTransmission(0x34);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)0x34, (uint8_t)1) != 1) return -1;
  return (int)(Wire.read() & 0xFF);
#else
  (void)reg;
  return -1;
#endif
}

static bool writeAxp2101Register(uint8_t reg, uint8_t value) {
#if BOARD_IS_AMOLED_206
  Wire.beginTransmission(0x34);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
#else
  (void)reg; (void)value;
  return false;
#endif
}

static void configureAxp2101PowerKey() {
#if BOARD_IS_AMOLED_206
  int cfg = readAxp2101Register(0x27);  // IRQ_OFF_ON_LEVEL_CTRL
  if (cfg >= 0) {
    cfg &= ~0x0C;  // bits[3:2] = 00 => 4s power-off hold
    writeAxp2101Register(0x27, (uint8_t)cfg);
  }
  int inten2 = readAxp2101Register(0x41);  // INTEN2
  if (inten2 >= 0) {
    inten2 |= 0x0F;  // pkey positive/negative/long/short
    writeAxp2101Register(0x41, (uint8_t)inten2);
  }
  writeAxp2101Register(0x49, 0xFF);  // clear stale INTSTS2
#endif
}

static void restartWifiPortalMdns() {
  if (s_wifiPortalMdnsRunning) {
    MDNS.end();
    s_wifiPortalMdnsRunning = false;
  }
  if (!s_wifiPortalApActive && WiFi.status() != WL_CONNECTED) return;
  if (!MDNS.begin(WIFI_PORTAL_HOSTNAME)) {
    Serial.println("mDNS start failed");
    return;
  }
  MDNS.addService("http", "tcp", 80);
  s_wifiPortalMdnsRunning = true;
}

static void stopWifiPortalMdns() {
  if (!s_wifiPortalMdnsRunning) return;
  MDNS.end();
  s_wifiPortalMdnsRunning = false;
}

static void startWifiPortalServer(bool enableAp) {
  loadWifiPortalConfig();
  ensureWifiPortalHandlers();
  if (enableAp) {
    if (WiFi.status() == WL_CONNECTED) WiFi.mode(WIFI_AP_STA);
    else WiFi.mode(WIFI_AP);
    IPAddress mask(255, 255, 255, 0);
    WiFi.softAPConfig(s_wifiPortalApIp, s_wifiPortalApIp, mask);
    WiFi.softAP(WIFI_PORTAL_AP_SSID, WIFI_PORTAL_AP_PASS);
    s_wifiPortalDns.stop();
    s_wifiPortalDns.start(53, "*", s_wifiPortalApIp);
    s_wifiPortalDnsRunning = true;
    s_wifiPortalApActive = true;
  } else {
    if (s_wifiPortalDnsRunning) {
      s_wifiPortalDns.stop();
      s_wifiPortalDnsRunning = false;
    }
    if (s_wifiPortalApActive) {
      WiFi.softAPdisconnect(true);
      s_wifiPortalApActive = false;
    }
    if (WiFi.status() != WL_CONNECTED) {
      stopWifiPortalMdns();
      return;
    }
  }
  s_wifiPortalServer.begin();
  s_wifiPortalHttpRunning = true;
  restartWifiPortalMdns();
}

static void serviceWifiPortalServer() {
  if (s_wifiPortalDnsRunning) s_wifiPortalDns.processNextRequest();
  if (s_wifiPortalHttpRunning) s_wifiPortalServer.handleClient();
}

static void showWifiPortalJoinInfo(bool canSkip = false) {
#if BOARD_IS_AMOLED_206
  int screenH = s_amoledOut ? s_amoledOut->height() : AMOLED_HEIGHT;
  int screenW = s_amoledOut ? s_amoledOut->width()  : AMOLED_WIDTH;
  if (s_amoledOut) {
    s_amoledOut->fillScreen(0x0000);
    s_amoledOut->setTextColor(0xFFFF, 0x0000);

    int y = 24;  // move everything down one line
    s_amoledOut->setTextSize(4);
    const char* title = "WiFi Setup";
    int titleW = (int)strlen(title) * 6 * 4;  // default font width model
    int titleX = (screenW - titleW) / 2;
    if (titleX < 0) titleX = 0;
    s_amoledOut->setCursor(titleX, y);
    s_amoledOut->print(title);
    y += 44;

    s_amoledOut->setTextSize(4);  // SSID label 2x larger
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print("Network Name");
    y += 34;  // extra spacing before value
    s_amoledOut->setTextSize(6);  // 6x requested
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print(WIFI_PORTAL_AP_SSID);
    y += 64;  // extra section spacing

    s_amoledOut->setTextSize(4);  // PASSWORD label 2x larger
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print("PASSWORD");
    y += 34;  // extra spacing before value
    s_amoledOut->setTextSize(6);  // 6x requested
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print(WIFI_PORTAL_AP_PASS);
    y += 66;  // extra section spacing

    s_amoledOut->setTextSize(4);  // match SSID/PASSWORD label size
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print("Enter in browser");
    y += 34;  // keep spacing consistent with other sections
    s_amoledOut->setTextSize(6);  // 6x requested
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print("satwatch");
    y += 46;
    s_amoledOut->setCursor(8, y);
    s_amoledOut->print(".local");
    if (canSkip) {
      y += 54;
      s_amoledOut->setTextSize(2);
      s_amoledOut->setTextColor(0xFD20, 0x0000);  // amber
      s_amoledOut->setCursor(8, y);
      s_amoledOut->print("Tap to play saved frames");
      s_amoledOut->setTextColor(0xFFFF, 0x0000);
    }
  }
#else
  int screenH = tft.height();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  int y = screenH / 2 - 64;
  if (y < 4) y = 4;
  tft.setCursor(4, y);
  tft.print("WiFi Setup");
  tft.setTextSize(1);
  y += 28;
  tft.setCursor(4, y);
  tft.print("SSID: " WIFI_PORTAL_AP_SSID);
  y += 16;
  tft.setCursor(4, y);
  tft.print("Password: " WIFI_PORTAL_AP_PASS);
  y += 16;
  tft.setCursor(4, y);
  tft.print("URL: http://satwatch.local/");
  y += 16;
  tft.setCursor(4, y);
  tft.print("Join AP then open URL");
  if (canSkip) {
    y += 16;
    tft.setCursor(4, y);
    tft.print("Press button to play saved frames");
  }
#endif
  serviceWifiPortalServer();
}

static void runWifiConfigPortal(bool canSkip = false) {
  loadWifiPortalConfig();
  if (canSkip) pinMode(38, INPUT_PULLUP);  // TP_INT — active-low touch interrupt

  const uint32_t kRetryIntervalMs = 60000UL;  // re-scan for known networks every 60 s

  for (;;) {
    // (Re)start AP + portal — done on first entry and after every failed retry
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    vTaskDelay(pdMS_TO_TICKS(100));
    startWifiPortalServer(true);
    showWifiPortalJoinInfo(canSkip);

    uint32_t retryAt = millis() + kRetryIntervalMs;
    for (;;) {
      serviceWifiPortalServer();
      if (canSkip) {
        if (pollPortalSkip()) return;  // user dismissed → play cached frames
      } else {
        serviceUserButtons();  // keeps PKEY → restart working in must-configure case
      }
      if ((int32_t)(millis() - retryAt) >= 0) break;
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Retry all known SSIDs (tears down AP temporarily)
    appendDiagLog("portal: wifi-retry\n");
    if (connectWifiForSync(false, "Retrying WiFi...")) {
      // Connected — reboot so setup() runs the full sync path cleanly
      appendDiagLog("portal: wifi-retry ok -> restart\n");
      SD_MMC.end();
      ESP.restart();
    }
    // Still no WiFi — outer loop restarts AP + portal and waits again
  }
}

static int wifiBarsForRssi(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -67) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

static void drawWifiIndicator(LGFX_Sprite& spr, int x, int y, int w, int h, int rssi, bool connectedNow) {
  // Fixed 24×20 monochrome icon asset following a standard mobile Wi‑Fi
  // silhouette: three arcs plus a separate bottom dot.
  static const uint32_t outerArc[] = {
    0b000000000111111000000000,
    0b000000011111111110000000,
    0b000000111111111111000000,
    0b000011111000001111110000,
    0b000111100000000001111000,
    0b001111000000000000111100,
  };
  static const uint32_t midArc[] = {
    0b000000000000000000000000,
    0b000000000111111000000000,
    0b000000011111111110000000,
    0b000000111100001111000000,
    0b000001111000000111100000,
  };
  static const uint32_t innerArc[] = {
    0b000000000000000000000000,
    0b000000000000000000000000,
    0b000000000011110000000000,
    0b000000001111111100000000,
    0b000000011000001100000000,
  };
  static const uint32_t dotMask[] = {
    0b000000000000000000000000,
    0b000000000000000000000000,
    0b000000000000000000000000,
    0b000000000011110000000000,
    0b000000000011110000000000,
  };

  constexpr int baseW = 24;
  constexpr int baseH = 20;
  int strength = wifiBarsForRssi(rssi);
  int activeColor = TFT_GREEN;
  int inactiveColor = gray565(110);

  int maxW = w;
  int maxH = h;
  if (maxW < baseW) maxW = baseW;
  if (maxH < baseH) maxH = baseH;
  int scale = min(maxW / baseW, maxH / baseH);
  if (scale < 1) scale = 1;
  int scaleX = (scale * 5 + 2) / 4;  // ~1.25x wider in X to avoid the squished look
  if (scaleX < 1) scaleX = 1;
  int drawW = baseW * scaleX;
  int drawH = baseH * scale;
  if (drawW > w) {
    scaleX = max(1, w / baseW);
    drawW = baseW * scaleX;
  }
  int ox = x + (w - drawW) / 2;
  int oy = y + (h - drawH) / 2;

  auto drawMask = [&](const uint32_t* rows, int rowsCount, int rowOffset, uint16_t color) {
    for (int row = 0; row < rowsCount; ++row) {
      int dstRow = row + rowOffset;
      if (dstRow < 0 || dstRow >= baseH) continue;
      uint32_t bits = rows[row];
      for (int col = 0; col < baseW; ++col) {
        if ((bits >> (baseW - 1 - col)) & 0x1) {
          spr.fillRect(ox + col * scaleX, oy + dstRow * scale, scaleX, scale, color);
        }
      }
    }
  };

  uint16_t outerColor = (connectedNow && strength >= 4) ? activeColor : inactiveColor;
  uint16_t midColor   = (connectedNow && strength >= 3) ? activeColor : inactiveColor;
  uint16_t innerColor = (connectedNow && strength >= 2) ? activeColor : inactiveColor;
  uint16_t dotColor   = (connectedNow && strength >= 1) ? activeColor : inactiveColor;

  drawMask(outerArc, (int)(sizeof(outerArc) / sizeof(outerArc[0])), 0, outerColor);
  drawMask(midArc,   (int)(sizeof(midArc)   / sizeof(midArc[0])),   6, midColor);
  drawMask(innerArc, (int)(sizeof(innerArc) / sizeof(innerArc[0])), 11, innerColor);
  drawMask(dotMask,  (int)(sizeof(dotMask)  / sizeof(dotMask[0])), 15, dotColor);
}

static void drawSleepModeGlyph(LGFX_Sprite& spr, int x, int y, int w, int h, bool sleepEnabled) {
  auto drawMask = [&](const uint32_t* rows, int rowsCount, int baseW, int baseH,
                      int ox, int oy, int scale, uint16_t color) {
    for (int row = 0; row < rowsCount; ++row) {
      uint32_t bits = rows[row];
      for (int col = 0; col < baseW; ++col) {
        if ((bits >> (baseW - 1 - col)) & 0x1U) {
          spr.fillRect(ox + col * scale, oy + row * scale, scale, scale, color);
        }
      }
    }
  };

  if (sleepEnabled) {
    static const uint32_t kSleepGlyph[] = {
      0b111111000000000000,
      0b000111000000000000,
      0b001110000000000000,
      0b011100000000000000,
      0b111111000111000000,
      0b000000000011000000,
      0b000000000110000000,
      0b000000001111000000,
      0b000000000000001110,
      0b000000000000000110,
      0b000000000000001100,
      0b000000000000011110,
      0b000000000000000000,
      0b000000000000000000,
      0b000000000000000000,
      0b000000000000000000,
    };
    constexpr int baseW = 18;
    constexpr int baseH = 16;
    int scale = min(max(1, w / baseW), max(1, h / baseH));
    if (scale < 1) scale = 1;
    int drawW = baseW * scale;
    int drawH = baseH * scale;
    int ox = x + (w - drawW) / 2;
    int oy = y + (h - drawH) / 2;
    drawMask(kSleepGlyph, baseH, baseW, baseH, ox, oy, scale, TFT_CYAN);
    return;
  }

  static const uint32_t kWakeFace[] = {
    0b00000011111111000000,
    0b00001111111111110000,
    0b00011111111111111000,
    0b00111111111111111100,
    0b01111111111111111110,
    0b01111111111111111110,
    0b11111111111111111111,
    0b11111111111111111111,
    0b11111111111111111111,
    0b11111111111111111111,
    0b01111111111111111110,
    0b01111111111111111110,
    0b00111111111111111100,
    0b00011111111111111000,
    0b00001111111111110000,
    0b00000011111111000000,
  };
  // Sunglasses + U-shaped smile.
  static const uint32_t kWakeDetail[] = {
    0b00000000000000000000,
    0b00000000000000000000,
    0b00000000000000000000,
    0b00000000000000000000,
    0b00011111100111111000,  // lenses: cols 3-8, gap cols 9-10, cols 11-16
    0b00011111100111111000,
    0b00011111100111111000,
    0b00000000000000000000,
    0b00000000000000000000,
    0b00100000000000000100,  // smile corners (cols 2, 17)
    0b00011000000000011000,  // smile sides (cols 3-4, 15-16)
    0b00000111111111100000,  // smile center (cols 5-14)
    0b00000000000000000000,
    0b00000000000000000000,
    0b00000000000000000000,
    0b00000000000000000000,
  };
  constexpr int baseW = 20;
  constexpr int baseH = 16;
  int scale = min(max(1, w / baseW), max(1, h / baseH));
  if (scale < 1) scale = 1;
  int drawW = baseW * scale;
  int drawH = baseH * scale;
  int ox = x + (w - drawW) / 2;
  int oy = y + (h - drawH) / 2;
  drawMask(kWakeFace, baseH, baseW, baseH, ox, oy, scale, TFT_YELLOW);
  drawMask(kWakeDetail, baseH, baseW, baseH, ox, oy, scale, TFT_BLACK);
}

// ─────────────────────────────────────────────────────────────
//  JPEGDEC draw callback — renders into current target
// ─────────────────────────────────────────────────────────────
static int jpegDraw(JPEGDRAW* pDraw) {
  if (!g_drawTarget) return 0;
  int x0 = pDraw->x;
  int y0 = pDraw->y;
  int blockW = pDraw->iWidth;  // full MCU-strip width = source buffer row stride
  int drawW  = (pDraw->iWidthUsed > 0) ? pDraw->iWidthUsed : blockW; // valid pixels
  int x1 = x0 + drawW;
  int y1 = y0 + pDraw->iHeight;
  if (s_jpegDrawCalls == 0) {
    s_jpegMinX = x0;
    s_jpegMinY = y0;
    s_jpegMaxX = x1;
    s_jpegMaxY = y1;
  } else {
    if (x0 < s_jpegMinX) s_jpegMinX = x0;
    if (y0 < s_jpegMinY) s_jpegMinY = y0;
    if (x1 > s_jpegMaxX) s_jpegMaxX = x1;
    if (y1 > s_jpegMaxY) s_jpegMaxY = y1;
  }
  if (x0 < 0 || y0 < 0 || x1 > DISP_W || y1 > DISP_H) {
    s_jpegDrawOutOfBounds = true;
  }
  s_jpegDrawCalls++;

  if (drawW == blockW) {
    // Full MCU strip — buffer stride matches draw width, single call is correct
    g_drawTarget->pushImage(x0, y0, blockW, pDraw->iHeight,
                            (uint16_t*)pDraw->pPixels);
  } else {
    // Partial last column: source buffer stride is blockW but we only draw drawW
    // pixels. Write row-by-row to preserve the correct source stride and avoid
    // LovyanGFX treating the clipped width as the row pitch (which smears content).
    uint16_t* src = (uint16_t*)pDraw->pPixels;
    for (int row = 0; row < pDraw->iHeight; row++) {
      g_drawTarget->pushImage(x0, y0 + row, drawW, 1, src + row * blockW);
    }
  }
  return 1;
}

// Prepare an off-screen sprite so JPEGs can be decoded off-display to avoid tearing.
static bool ensureSprite() {
  if (spriteReady) return true;
  sprite.setColorDepth(16);
  sprite.setSwapBytes(false);         // keep native order; we already request big-endian from JPEGDEC
#if BOARD_HAS_PSRAM_SPRITES
  sprite.setPsram(psramFound());
#endif
  spriteReady = sprite.createSprite(DISP_W, DISP_H);
  if (spriteReady) {
    uint16_t* probe = (uint16_t*)sprite.getBuffer();
    if (probe) {
      sprite.fillScreen(0);
      sprite.drawPixel(0, 0, 0xF800);  // pure red probe
      s_mainSpritePixelsByteSwapped = (probe[0] == 0x00F8);
      sprite.fillScreen(0);
    } else {
      s_mainSpritePixelsByteSwapped = false;
    }
  }
  return spriteReady;
}

// Guard against occasional all-black decode results/corrupted frames.
// Sample the decoded image area (skip the timestamp bar rows) and only reject
// frames that appear completely black.
static bool spriteLooksCompletelyBlack() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return true;

  const int startY = 14;
  const int stepY = 8;
  const int stepX = 8;
  for (int y = startY; y < DISP_H; y += stepY) {
    int row = y * DISP_W;
    for (int x = 0; x < DISP_W; x += stepX) {
      if (px[row + x] != 0) return false;
    }
  }
  return true;
}

// Detect "visually black" frames that are not exact-zero due to JPEG noise.
// This is stricter than exact-zero blank detection and is only used for
// source-black frame rejection in the stream cache path.
static bool spriteLooksNearlyBlack() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return true;

  const int startY = 14;
  const int stepY = 4;
  const int stepX = 4;
  int dark = 0;
  int nonDark = 0;
  int exactZero = 0;
  int bright = 0;
  uint32_t sumScore = 0;

  for (int y = startY; y < DISP_H; y += stepY) {
    int row = y * DISP_W;
    for (int x = 0; x < DISP_W; x += stepX) {
      uint16_t c = px[row + x];
      int r = (c >> 11) & 0x1F;
      int g = (c >> 5)  & 0x3F;
      int b = c & 0x1F;
      int score = (r * 2) + (g * 3) + (b * 1);
      sumScore += (uint32_t)score;
      if (c == 0) exactZero++;
      if (score >= 70) bright++;
      if (score < 28) dark++;
      else nonDark++;
    }
  }

  int total = dark + nonDark;
  if (total <= 0) return true;

  // Extremely dark overall with almost no non-dark samples.
  if (nonDark <= 8) return true;

  int darkPermille = (dark * 1000) / total;
  if (darkPermille >= 997 && nonDark <= 20) return true;
  if (darkPermille >= 990 && nonDark <= 40) return true;
  if (darkPermille >= 980 && nonDark <= 80 && bright <= 12) return true;

  // If almost all samples are exact zero, this is also effectively black.
  int zeroPermille = (exactZero * 1000) / total;
  if (zeroPermille >= 990) return true;

  // Low overall luminance with very few bright samples is still visually black.
  uint32_t avgScore = sumScore / (uint32_t)total;
  if (avgScore <= 10 && bright <= 16) return true;
  if (avgScore <= 14 && bright <= 8 && nonDark <= 80) return true;

  return false;
}

// Detect partial JPEG decode artifacts.  A fully-decoded JPEG has almost no
// pixels that are exactly 0x0000 (JPEG compression adds rounding noise), while
// a partial decode leaves large unfilled areas at exactly 0x0000 (the sprite
// pre-fill).  If >40 % of sampled pixels are exactly zero AND the non-zero
// content is concentrated in one corner, reject the frame.
static bool spriteLooksPartialDecode() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return false;

  const int startY = 14;   // skip timestamp bar
  const int stepY = 4;
  const int stepX = 4;

  int exactZero = 0;
  int nonZero   = 0;

  for (int y = startY; y < DISP_H; y += stepY) {
    int row = y * DISP_W;
    for (int x = 0; x < DISP_W; x += stepX) {
      if (px[row + x] == 0) exactZero++;
      else                   nonZero++;
    }
  }

  int sampled = exactZero + nonZero;
  if (sampled == 0) return false;

  // A real JPEG frame has very few exactly-zero pixels.  Threshold at 40 %.
  int zeroPct = exactZero * 100 / sampled;
  if (zeroPct < 40) return false;   // mostly non-zero → valid frame

  // High exact-zero fraction — now check if the non-zero content is
  // concentrated in a single quadrant (hallmark of partial decode).
  if (nonZero < 20) return false;   // too few non-zero to judge

  const int midX = DISP_W / 2;
  const int midY = (startY + DISP_H) / 2;
  int qTL = 0, qTR = 0, qBL = 0, qBR = 0;

  for (int y = startY; y < DISP_H; y += stepY) {
    int row = y * DISP_W;
    for (int x = 0; x < DISP_W; x += stepX) {
      if (px[row + x] == 0) continue;
      if (x < midX) {
        if (y < midY) qTL++; else qBL++;
      } else {
        if (y < midY) qTR++; else qBR++;
      }
    }
  }

  int occupied = (qTL > 0) + (qTR > 0) + (qBL > 0) + (qBR > 0);
  if (occupied <= 1) {
    return true;
  }

  // Content in 2+ quadrants but one dominates >90 % — still partial
  int maxQ = qTL;
  if (qTR > maxQ) maxQ = qTR;
  if (qBL > maxQ) maxQ = qBL;
  if (qBR > maxQ) maxQ = qBR;
  if (maxQ * 100 / nonZero > 90) {
    return true;
  }

  return false;
}

// Detect partial SD read: one horizontal third of the frame is ~all zeros while
// another third has substantial content.  This catches the specific corruption
// pattern where s_streamFile.read() returns the correct byte count but the
// trailing portion of the buffer is zeroed due to SPI signal-integrity noise on
// the shared LCD+SD bus.  spriteLooksNearlyBlack() misses this (only ~50% dark)
// and spriteLooksPartialDecode() misses it (content spreads across 2 quadrants).
static bool spriteLooksHorizontallyCorrupted() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return false;

  const int startY  = 14;              // skip timestamp bar
  const int usableH = DISP_H - startY; // 158 usable rows
  const int bandH   = usableH / 3;     // ~52 rows per band
  const int stepX   = 4;

  int bandZeroPct[3] = {0, 0, 0};

  for (int b = 0; b < 3; b++) {
    int y0 = startY + b * bandH;
    int y1 = (b == 2) ? DISP_H : (y0 + bandH);
    int zero = 0, total = 0;
    for (int y = y0; y < y1; y += 4) {
      int row = y * DISP_W;
      for (int x = 0; x < DISP_W; x += stepX) {
        total++;
        if (px[row + x] == 0) zero++;
      }
    }
    bandZeroPct[b] = (total > 0) ? (zero * 100 / total) : 100;
  }

  // Any band >90% exact-zero while another band is <20% exact-zero
  // signals a corrupted partial read.
  for (int empty = 0; empty < 3; empty++) {
    if (bandZeroPct[empty] < 90) continue;
    for (int full = 0; full < 3; full++) {
      if (full == empty) continue;
      if (bandZeroPct[full] < 20) {
        return true;
      }
    }
  }
  return false;
}

// Detect vertical corruption: left or right column half is ~all zeros while
// the opposite half has substantial content.  This is the exact pattern of the
// "right half / left half black" display artifact caused by the ST7789 GRAM
// window being left in a sub-region state after many clock-overlay partial
// pushes — the next full-frame sprite.pushSprite() writes to the wrong window
// and half the GRAM retains the old (zero) content.
static bool spriteLooksVerticallyCorrupted() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return false;

  const int startY = 14;               // skip timestamp bar
  const int halfX  = DISP_W / 2;      // 160 — split point
  const int stepX  = 4;
  const int stepY  = 4;

  int leftZero = 0, leftTotal = 0;
  int rightZero = 0, rightTotal = 0;

  for (int y = startY; y < DISP_H; y += stepY) {
    int row = y * DISP_W;
    for (int x = 0; x < halfX; x += stepX) {
      leftTotal++;
      if (px[row + x] == 0) leftZero++;
    }
    for (int x = halfX; x < DISP_W; x += stepX) {
      rightTotal++;
      if (px[row + x] == 0) rightZero++;
    }
  }

  int leftPct  = (leftTotal  > 0) ? (leftZero  * 100 / leftTotal)  : 100;
  int rightPct = (rightTotal > 0) ? (rightZero * 100 / rightTotal) : 100;

  if ((leftPct > 90 && rightPct < 20) || (rightPct > 90 && leftPct < 20)) {
    return true;
  }
  return false;
}

// Detect the intermittent cyan/white blocky overlay corruption seen in some
// cached weather JPGs. The failure presents as a broad cyan-tinted band with
// white rectangular chunks, not as zeros. This detector is intentionally
// conservative: it looks for 16×16 tiles that are mostly cyan/white mixed
// content, then requires a wide horizontal run of those tiles.
static bool spriteLooksCyanWhiteBlockCorrupted() {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px) return false;

  constexpr int TILE_W = 16;
  constexpr int TILE_H = 16;
  constexpr int COLS = DISP_W / TILE_W;
  constexpr int ROWS = DISP_H / TILE_H;
  constexpr int START_Y = 14;  // skip the top metadata strip area

  bool suspect[ROWS][COLS] = {};

  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < COLS; ++bc) {
      int cyanish = 0;
      int whiteish = 0;
      int sampled = 0;
      for (int dy = 0; dy < TILE_H; ++dy) {
        int py = br * TILE_H + dy;
        if (py < START_Y || py >= DISP_H) continue;
        const uint16_t* row = px + py * DISP_W + bc * TILE_W;
        for (int dx = 0; dx < TILE_W; ++dx) {
          uint16_t p = row[dx];
          if (s_mainSpritePixelsByteSwapped) p = __builtin_bswap16(p);
          int r = (p >> 11) & 0x1F;
          int g6 = (p >> 5) & 0x3F;
          int b = p & 0x1F;
          int g = g6 >> 1;  // normalize to 5-bit scale
          bool isWhiteish = (r >= 22 && g >= 22 && b >= 22);
          bool isCyanish = (g >= 20 && b >= 15 && r <= 10 && (g + b - (r * 2) >= 20));
          if (isWhiteish) whiteish++;
          if (isCyanish) cyanish++;
          sampled++;
        }
      }
      if (sampled <= 0) continue;

      int mixPct = ((cyanish + whiteish) * 100) / sampled;
      int cyanPct = (cyanish * 100) / sampled;
      int whitePct = (whiteish * 100) / sampled;
      if (mixPct >= 70 && cyanPct >= 12 && whitePct >= 8) {
        suspect[br][bc] = true;
      }
    }
  }

  for (int br = 1; br < ROWS - 1; ++br) {
    int count = 0;
    int bestRun = 0;
    int run = 0;
    for (int bc = 0; bc < COLS; ++bc) {
      if (suspect[br][bc]) {
        count++;
        run++;
        if (run > bestRun) bestRun = run;
      } else {
        run = 0;
      }
    }
    if (count >= 8 && bestRun >= 4) {
      Serial.printf("cyanband row=%d cnt=%d run=%d\n", br, count, bestRun);
      return true;
    }
  }

  for (int br = 1; br < ROWS - 2; ++br) {
    int row0 = 0;
    int row1 = 0;
    for (int bc = 0; bc < COLS; ++bc) {
      if (suspect[br][bc]) row0++;
      if (suspect[br + 1][bc]) row1++;
    }
    if (row0 >= 6 && row1 >= 6) {
      Serial.printf("cyanband2 rows=%d-%d cnt=%d,%d\n", br, br + 1, row0, row1);
      return true;
    }
  }

  return false;
}

static void invalidateStreamSlot(int idx, const char* reason) {
  if (idx < 0 || idx >= MAX_FRAMES) return;
  if (!s_streamValid[idx]) return;
  s_streamValid[idx] = 0;
  if (idx < (int)s_idx.count) s_idx.rawValid[idx] = 0;
  invalidateValidIdxCache();
  Serial.printf("INV idx=%d %s\n", idx, reason ? reason : "?");
}

static void resetJpegDrawStats() {
  s_jpegMinX = 0;
  s_jpegMinY = 0;
  s_jpegMaxX = 0;
  s_jpegMaxY = 0;
  s_jpegDrawCalls = 0;
  s_jpegDrawOutOfBounds = false;
}

static bool jpegDrawLooksFullFrame() {
  if (s_jpegDrawCalls == 0) return false;
  if (s_jpegDrawOutOfBounds) return false;
  if (s_jpegMinX != 0 || s_jpegMinY != 0) return false;
  if (s_jpegMaxX != DISP_W || s_jpegMaxY != DISP_H) return false;
  return true;
}

// ─────────────────────────────────────────────────────────────
//  UI helpers
// ─────────────────────────────────────────────────────────────
static void showMessage(const char* line1, const char* line2 = nullptr) {
#if BOARD_IS_AMOLED_206
  int screenW = s_amoledOut ? s_amoledOut->width() : AMOLED_WIDTH;
  int screenH = s_amoledOut ? s_amoledOut->height() : AMOLED_HEIGHT;
  uint16_t bg = s_hurricaneMode ? 0xA800 : 0x0000;  // visible dark red or black
  if (s_amoledOut) s_amoledOut->fillScreen(bg);
  if (s_amoledOut) {
    s_amoledOut->setTextColor(0xFFFF);
    s_amoledOut->setTextSize(2);
    s_amoledOut->setCursor(4, screenH / 2 - (line2 ? 22 : 10));
    s_amoledOut->print(line1);
    if (line2) {
      s_amoledOut->setCursor(4, screenH / 2 + 8);
      s_amoledOut->setTextSize(1);
      s_amoledOut->print(line2);
    }
  }
  s_amoledClearBeforeNextPresent = true;
#else
  int screenW = tft.width();
  int screenH = tft.height();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, screenH / 2 - (line2 ? 22 : 10));
  tft.print(line1);
  if (line2) {
    tft.setCursor(4, screenH / 2 + 8);
    tft.setTextSize(1);
    tft.print(line2);
  }
#endif
  serviceWifiPortalServer();
}

static void drawProgressBarUi(uint32_t current, uint32_t total, const char* label) {
  if (total == 0) total = 1;
  if (current > total) current = total;
  uint32_t percent = (uint32_t)(((uint64_t)current * 100ULL) / (uint64_t)total);
  if (percent > 100U) percent = 100U;

#if BOARD_IS_AMOLED_206
  if (!s_amoledOut) return;
  int screenW = s_amoledOut->width();
  int screenH = s_amoledOut->height();
  int barH = 14;
  int progressTextY = screenH / 2 + 24;
  int barY = progressTextY + 12;
  int barW = (int)((long)screenW * (long)current / (long)total);

  uint16_t bg = s_hurricaneMode ? 0xA800 : 0x0000;        // visible dark red or black
  uint16_t trackBg = s_hurricaneMode ? 0xC000 : 0x2104;   // medium red or dark grey

  // Fill areas above and below bar+text with background color
  if (s_hurricaneMode) {
    s_amoledOut->fillRect(0, 0, screenW, progressTextY - 2, bg);
    int barBottom = barY + barH;
    s_amoledOut->fillRect(0, barBottom, screenW, screenH - barBottom, bg);
  }

  s_amoledOut->fillRect(0, barY, barW, barH, 0x07E0);
  s_amoledOut->fillRect(barW, barY, screenW - barW, barH, trackBg);

  char buf[64];
  snprintf(buf, sizeof(buf), "%s  %lu%%", label ? label : "sync", (unsigned long)percent);
  s_amoledOut->setTextColor(0xFFFF);
  s_amoledOut->setTextSize(1);
  s_amoledOut->fillRect(0, progressTextY - 2, screenW, 12, bg);
  s_amoledOut->setCursor(4, progressTextY);
  s_amoledOut->print(buf);
#else
  int screenW = tft.width();
  int screenH = tft.height();
  int barH = 14;
  int progressTextY = screenH / 2 + 24;
  int barY = progressTextY + 12;
  int barW = (int)((long)screenW * (long)current / (long)total);

  tft.fillRect(0, barY, barW, barH, TFT_GREEN);
  tft.fillRect(barW, barY, screenW - barW, barH, 0x2104 /*dark grey*/);

  char buf[64];
  snprintf(buf, sizeof(buf), "%s  %lu%%", label ? label : "sync", (unsigned long)percent);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.fillRect(0, progressTextY - 2, screenW, 12, TFT_BLACK);
  tft.setCursor(4, progressTextY);
  tft.print(buf);
#endif
  serviceWifiPortalServer();
}

static bool syncProgressIsActive() {
  return s_syncProgActive && s_syncProgTotalUnits > 0;
}

static void syncProgressBegin(uint32_t totalUnits, const char* line1, const char* line2) {
  if (totalUnits == 0) totalUnits = 1;
  s_syncProgActive = true;
  s_syncProgTotalUnits = totalUnits;
  s_syncProgDoneUnits = 0;
  s_syncProgPhaseBase = 0;
  s_syncProgPhaseUnits = 0;
  s_syncProgCursorUnits = 0;
  strlcpy(s_syncProgPhaseLabel, "sync", sizeof(s_syncProgPhaseLabel));
  showMessage(line1 ? line1 : "Updating cache...", line2 ? line2 : "Validating data");
  drawProgressBarUi(0, s_syncProgTotalUnits, s_syncProgPhaseLabel);
}

static void syncProgressBeginPhase(const char* label, uint32_t phaseUnits) {
  if (!syncProgressIsActive()) return;
  uint32_t prevEnd = s_syncProgPhaseBase + s_syncProgPhaseUnits;
  if (s_syncProgCursorUnits < prevEnd) s_syncProgCursorUnits = prevEnd;
  if (s_syncProgCursorUnits > s_syncProgTotalUnits) s_syncProgCursorUnits = s_syncProgTotalUnits;
  s_syncProgPhaseBase = s_syncProgCursorUnits;
  s_syncProgPhaseUnits = phaseUnits;
  strlcpy(s_syncProgPhaseLabel, (label && label[0]) ? label : "sync", sizeof(s_syncProgPhaseLabel));
  drawProgressBarUi(s_syncProgDoneUnits, s_syncProgTotalUnits, s_syncProgPhaseLabel);
}

static void syncProgressSetPhaseProgress(int current, int total) {
  if (!syncProgressIsActive()) return;
  if (s_syncProgPhaseUnits == 0) return;
  if (total <= 0) total = 1;
  if (current < 0) current = 0;
  if (current > total) current = total;

  uint32_t mapped = s_syncProgPhaseBase +
                    (uint32_t)(((uint64_t)(uint32_t)current * (uint64_t)s_syncProgPhaseUnits) /
                               (uint64_t)(uint32_t)total);
  if (mapped < s_syncProgDoneUnits) mapped = s_syncProgDoneUnits;
  if (mapped >= s_syncProgTotalUnits) mapped = s_syncProgTotalUnits - 1;
  s_syncProgDoneUnits = mapped;
  drawProgressBarUi(s_syncProgDoneUnits, s_syncProgTotalUnits, s_syncProgPhaseLabel);
}

static void syncProgressTick(uint32_t units) {
  if (!syncProgressIsActive()) return;
  uint32_t next = s_syncProgDoneUnits + units;
  if (next >= s_syncProgTotalUnits) next = s_syncProgTotalUnits - 1;
  if (next < s_syncProgDoneUnits) next = s_syncProgDoneUnits;
  s_syncProgDoneUnits = next;
  drawProgressBarUi(s_syncProgDoneUnits, s_syncProgTotalUnits, s_syncProgPhaseLabel);
}

static void syncProgressCompletePhase() {
  if (!syncProgressIsActive()) return;
  uint32_t endUnits = s_syncProgPhaseBase + s_syncProgPhaseUnits;
  if (endUnits > s_syncProgTotalUnits) endUnits = s_syncProgTotalUnits;
  s_syncProgCursorUnits = endUnits;
  s_syncProgDoneUnits = (endUnits >= s_syncProgTotalUnits) ? (s_syncProgTotalUnits - 1) : endUnits;
  drawProgressBarUi(s_syncProgDoneUnits, s_syncProgTotalUnits, s_syncProgPhaseLabel);
  s_syncProgPhaseBase = s_syncProgCursorUnits;
  s_syncProgPhaseUnits = 0;
}

static void syncProgressEnd() {
  if (!syncProgressIsActive()) return;
  s_syncProgDoneUnits = s_syncProgTotalUnits;
  drawProgressBarUi(s_syncProgDoneUnits, s_syncProgTotalUnits, "done");
  s_syncProgActive = false;
  s_syncProgTotalUnits = 0;
  s_syncProgDoneUnits = 0;
  s_syncProgPhaseBase = 0;
  s_syncProgPhaseUnits = 0;
  s_syncProgCursorUnits = 0;
}

static void showProgress(int current, int total, const char* label) {
  if (syncProgressIsActive() && s_syncProgPhaseUnits > 0) {
    syncProgressSetPhaseProgress(current, total);
    return;
  }
  if (total <= 0) total = 1;
  if (current < 0) current = 0;
  if (current > total) current = total;
  drawProgressBarUi((uint32_t)current, (uint32_t)total, label);
}

// Fast bilinear lerp for RGB565. t is 0–15 (0 = all a, 15 = ~all b, 16 means full b).
// Packs R,G,B into non-overlapping bit-fields of a single uint32 so one multiply
// blends all three channels simultaneously — no per-channel division.
// Layout: G in bits [26:21], R in bits [15:11], B in bits [4:0] (zero gaps between).
static inline uint16_t lerp565_16(uint16_t a, uint16_t b, uint8_t t) {
  uint32_t a32 = (uint32_t)(a | ((uint32_t)a << 16)) & 0x07E0F81FU;
  uint32_t b32 = (uint32_t)(b | ((uint32_t)b << 16)) & 0x07E0F81FU;
  uint32_t r32 = (a32 * (uint32_t)(16 - t) + b32 * (uint32_t)t) >> 4;
  r32 &= 0x07E0F81FU;
  return (uint16_t)(r32 | (r32 >> 16));
}

// Scale sprite (320×176) bilinear → 410×360 canonical little-endian RGB565 into dst.
// Byte-swap is normalised out in the bswap branch so dst is always canonical.
// Called during raw-slot decode and before each present
// for paths that draw into sprite then scale.
static void scaleSpriteTo410x360(uint16_t* dst) {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px || !dst) return;
  for (int oy = 0; oy < SCALED_H; oy++) {
    int64_t exactY = (int64_t)oy * DISP_H;
    int sy  = (int)(exactY / SCALED_H); if (sy >= DISP_H - 1) sy = DISP_H - 2;
    int sy1 = sy + 1;
    uint8_t fy = (uint8_t)((exactY % SCALED_H) * 16 / SCALED_H);
    for (int ox = 0; ox < SCALED_W; ox++) {
      int64_t exactX = (int64_t)ox * DISP_W;
      int sx  = (int)(exactX / SCALED_W); if (sx >= DISP_W - 1) sx = DISP_W - 2;
      int sx1 = sx + 1;
      uint8_t fx = (uint8_t)((exactX % SCALED_W) * 16 / SCALED_W);
      uint16_t p00, p10, p01, p11;
      if (s_mainSpritePixelsByteSwapped) {
        p00 = __builtin_bswap16(px[sy *DISP_W+sx ]);
        p10 = __builtin_bswap16(px[sy *DISP_W+sx1]);
        p01 = __builtin_bswap16(px[sy1*DISP_W+sx ]);
        p11 = __builtin_bswap16(px[sy1*DISP_W+sx1]);
      } else {
        p00 = px[sy *DISP_W+sx ]; p10 = px[sy *DISP_W+sx1];
        p01 = px[sy1*DISP_W+sx ]; p11 = px[sy1*DISP_W+sx1];
      }
      dst[oy*SCALED_W+ox] = lerp565_16(lerp565_16(p00,p10,fx), lerp565_16(p01,p11,fx), fy);
    }
  }
}

// Scale only the sprite sub-rectangle (srcX,srcY,srcW,srcH) into the
// corresponding region of dst (410×360). All other dst pixels are unchanged.
// Used by the clock overlay dirty-rect path to avoid rescaling the full frame.
static void scaleSubrectTo410x360(uint16_t* dst,
                                   int srcX, int srcY, int srcW, int srcH) {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px || !dst || srcW <= 0 || srcH <= 0) return;
  int dstX  = (int)((int64_t)srcX * SCALED_W / DISP_W);
  int dstY  = (int)((int64_t)srcY * SCALED_H / DISP_H);
  int dstX2 = (int)(((int64_t)(srcX + srcW) * SCALED_W + DISP_W - 1) / DISP_W) + 1;
  int dstY2 = (int)(((int64_t)(srcY + srcH) * SCALED_H + DISP_H - 1) / DISP_H) + 1;
  if (dstX  < 0)        dstX  = 0;
  if (dstY  < 0)        dstY  = 0;
  if (dstX2 > SCALED_W) dstX2 = SCALED_W;
  if (dstY2 > SCALED_H) dstY2 = SCALED_H;
  for (int oy = dstY; oy < dstY2; oy++) {
    int64_t exactY = (int64_t)oy * DISP_H;
    int sy  = (int)(exactY / SCALED_H); if (sy >= DISP_H - 1) sy = DISP_H - 2;
    int sy1 = sy + 1;
    uint8_t fy = (uint8_t)((exactY % SCALED_H) * 16 / SCALED_H);
    for (int ox = dstX; ox < dstX2; ox++) {
      int64_t exactX = (int64_t)ox * DISP_W;
      int sx  = (int)(exactX / SCALED_W); if (sx >= DISP_W - 1) sx = DISP_W - 2;
      int sx1 = sx + 1;
      uint8_t fx = (uint8_t)((exactX % SCALED_W) * 16 / SCALED_W);
      uint16_t p00, p10, p01, p11;
      if (s_mainSpritePixelsByteSwapped) {
        p00 = __builtin_bswap16(px[sy *DISP_W+sx ]);
        p10 = __builtin_bswap16(px[sy *DISP_W+sx1]);
        p01 = __builtin_bswap16(px[sy1*DISP_W+sx ]);
        p11 = __builtin_bswap16(px[sy1*DISP_W+sx1]);
      } else {
        p00 = px[sy *DISP_W+sx ]; p10 = px[sy *DISP_W+sx1];
        p01 = px[sy1*DISP_W+sx ]; p11 = px[sy1*DISP_W+sx1];
      }
      dst[oy*SCALED_W+ox] = lerp565_16(lerp565_16(p00,p10,fx), lerp565_16(p01,p11,fx), fy);
    }
  }
}

// Scale 14 sprite rows starting at srcY0 into dst[0..SCALED_BAR_H-1][0..SCALED_W-1].
// dst is a compact SCALED_W × SCALED_BAR_H buffer, NOT the full 410×360 frame.
static void scaleBarRowsToBuffer(uint16_t* dst, int srcY0) {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px || !dst) return;
  int srcY1 = srcY0 + 14;
  int dstY0 = (int)((int64_t)srcY0 * SCALED_H / DISP_H);
  int dstY1 = (int)(((int64_t)srcY1 * SCALED_H + DISP_H - 1) / DISP_H) + 1;
  if (dstY1 > SCALED_H) dstY1 = SCALED_H;
  int dstH = dstY1 - dstY0;
  if (dstH > SCALED_BAR_H) dstH = SCALED_BAR_H;
  for (int r = 0; r < dstH; r++) {
    int oy = dstY0 + r;
    int64_t exactY = (int64_t)oy * DISP_H;
    int sy  = (int)(exactY / SCALED_H); if (sy >= DISP_H - 1) sy = DISP_H - 2;
    int sy1 = sy + 1;
    uint8_t fy = (uint8_t)((exactY % SCALED_H) * 16 / SCALED_H);
    for (int ox = 0; ox < SCALED_W; ox++) {
      int64_t exactX = (int64_t)ox * DISP_W;
      int sx  = (int)(exactX / SCALED_W); if (sx >= DISP_W - 1) sx = DISP_W - 2;
      int sx1 = sx + 1;
      uint8_t fx = (uint8_t)((exactX % SCALED_W) * 16 / SCALED_W);
      uint16_t p00, p10, p01, p11;
      if (s_mainSpritePixelsByteSwapped) {
        p00 = __builtin_bswap16(px[sy *DISP_W+sx ]);
        p10 = __builtin_bswap16(px[sy *DISP_W+sx1]);
        p01 = __builtin_bswap16(px[sy1*DISP_W+sx ]);
        p11 = __builtin_bswap16(px[sy1*DISP_W+sx1]);
      } else {
        p00 = px[sy *DISP_W+sx ]; p10 = px[sy *DISP_W+sx1];
        p01 = px[sy1*DISP_W+sx ]; p11 = px[sy1*DISP_W+sx1];
      }
      dst[r * SCALED_W + ox] = lerp565_16(lerp565_16(p00,p10,fx), lerp565_16(p01,p11,fx), fy);
    }
  }
}

// Forward declaration — defined later in file.
static void drawTimestamp(int frameIdx, LovyanGFX* target);

// Use real UTC time when available; otherwise fall back to a monotonic
// timeline seeded from newest known frame/radar time so "x h ago" and
// "Radar: y min" don't collapse to 0 when NTP is temporarily unavailable.
static time_t currentUtcForAgeMetrics() {
  time_t nowUtc = time(nullptr);
  if (nowUtc > 1700000000) return nowUtc;

  static time_t s_ageFallbackBaseUtc = 0;
  static uint32_t s_ageFallbackBaseMs = 0;
  uint32_t nowMs = millis();

  time_t seedUtc = 0;
  if (s_lastRadarUtcValid && s_lastRadarUtc > seedUtc) seedUtc = s_lastRadarUtc;
  if (s_timesLoaded && frameCount > 0) {
    for (int i = frameCount - 1; i >= 0; --i) {
      if (s_frameTimes[i] > seedUtc) {
        seedUtc = s_frameTimes[i];
        break;
      }
    }
  }
  if (seedUtc <= 0) seedUtc = 1700000000;

  if (s_ageFallbackBaseUtc == 0 || seedUtc > s_ageFallbackBaseUtc) {
    s_ageFallbackBaseUtc = seedUtc;
    s_ageFallbackBaseMs = nowMs;
  }
  return s_ageFallbackBaseUtc + (time_t)((nowMs - s_ageFallbackBaseMs) / 1000U);
}

// Draw a battery icon at (x,y) of size (w×h) at battery level pct (0-100).
// Nub is on the right side. Fill color: green>=50, yellow>=20, red<20.
static void drawBatteryIcon(LGFX_Sprite& spr, int x, int y, int w, int h, int pct, int chargeState) {
    int nubW = max(2, w / 8);
    int bodyW = w - nubW;
    int nubH = max(2, h / 3);
    int nubY = y + (h - nubH) / 2;

    // Body outline
    spr.drawRect(x, y, bodyW, h, TFT_WHITE);

    // Fill area inside body
    int innerX = x + 1;
    int innerY = y + 1;
    int innerW = bodyW - 2;
    int innerH = h - 2;
    int clampedPct = pct;
    if (clampedPct < 0) clampedPct = 0;
    if (clampedPct > 100) clampedPct = 100;
    int fillW = (int)((clampedPct / 100.0f) * innerW);
    if (fillW < 1 && clampedPct > 0) fillW = 1;

    // Clear interior first
    spr.fillRect(innerX, innerY, innerW, innerH, TFT_BLACK);

    // Full interior fill so state is readable even with no glyph.
    uint16_t fillColor = (pct >= 50) ? TFT_GREEN : (pct >= 20) ? TFT_YELLOW : TFT_RED;
    if (fillW > 0) spr.fillRect(innerX, innerY, fillW, innerH, fillColor);

    uint8_t status2 = (chargeState >= 0) ? (uint8_t)chargeState : 0xFF;
    uint8_t powerState = (status2 == 0xFF) ? 0xFF : ((status2 >> 5) & 0x03);
    uint8_t chargerPhase = (status2 == 0xFF) ? 0xFF : (status2 & 0x07);
    bool isCharging = (powerState == 0x01) || (powerState != 0x02 && chargerPhase <= 0x03);
    bool showChargeGlyph = isCharging;
    bool showPlugGlyph = (!isCharging && chargerPhase == 0x04);

    auto drawBatMask = [&](const uint32_t* rows, int rowCount, int bW,
                            int ox, int oy, int sc, uint16_t color) {
      for (int row = 0; row < rowCount; ++row) {
        uint32_t bits = rows[row];
        for (int col = 0; col < bW; ++col) {
          if ((bits >> (bW - 1 - col)) & 0x1U)
            spr.fillRect(ox + col * sc, oy + row * sc, sc, sc, color);
        }
      }
    };

    if (showChargeGlyph) {
      // 8×8 Z-bolt: top arm tilts right, crossbar full-width, bottom arm tilts left.
      static const uint32_t kBoltGlyph[] = {
        0b00001111,  // ....####
        0b00011110,  // ...####.
        0b00111100,  // ..####..
        0b11111110,  // #######.
        0b00111100,  // ..####..
        0b01111000,  // .####...
        0b11110000,  // ####....
        0b00000000,
      };
      constexpr int bW = 8, bH = 8;
      int sc = min(max(1, innerW / bW), max(1, innerH / bH));
      int ox = innerX + (innerW - bW * sc) / 2;
      int oy = innerY + (innerH - bH * sc) / 2;
      spr.fillRect(ox, oy, bW * sc, bH * sc, TFT_BLACK);
      drawBatMask(kBoltGlyph, bH, bW, ox, oy, sc, TFT_YELLOW);
    } else if (showPlugGlyph) {
      // 8×8 plug: two 1px prongs (cols 2, 5), wide body (cols 1-6), cord (cols 3-4)
      static const uint32_t kPlugGlyph[] = {
        0b00100100,  // ..#..#..  prongs
        0b00100100,  // ..#..#..  prongs
        0b01111110,  // .######.  body
        0b01111110,  // .######.  body
        0b01111110,  // .######.  body
        0b00011000,  // ...##...  cord
        0b00011000,  // ...##...  cord
        0b00000000,
      };
      constexpr int bW = 8, bH = 8;
      int sc = min(max(1, innerW / bW), max(1, innerH / bH));
      int ox = innerX + (innerW - bW * sc) / 2;
      int oy = innerY + (innerH - bH * sc) / 2;
      spr.fillRect(ox, oy, bW * sc, bH * sc, TFT_BLACK);
      drawBatMask(kPlugGlyph, bH, bW, ox, oy, sc, TFT_GREEN);
    }

    // Positive terminal nub (right side)
    spr.fillRect(x + bodyW, nubY, nubW, nubH, TFT_WHITE);
}

#if BOARD_IS_AMOLED_206
// Copy s_barSprite pixel buffer to dst, byte-swapping if the sprite stores pixels
// in swapped format. Mirrors the same correction done in copyClockFxSpriteToMainSprite()
// so non-symmetric colors (TFT_GREEN, TFT_YELLOW, gray) appear correctly in the frame.
static void copyBarSpriteToBuffer(uint16_t* dst, size_t npixels) {
  const uint16_t* src = (const uint16_t*)s_barSprite.getBuffer();
  if (s_barSpritePixelsByteSwapped) {
    for (size_t i = 0; i < npixels; i++) dst[i] = __builtin_bswap16(src[i]);
  } else {
    memcpy(dst, src, npixels * 2U);
  }
}

// Lazy-create the shared bar sprite used by both normal and hurricane renderers.
static void ensureBarSpriteReady() {
  if (s_barSpriteReady) return;
  s_barSprite.setColorDepth(16);
  s_barSprite.setSwapBytes(false);
#if BOARD_HAS_PSRAM_SPRITES
  s_barSprite.setPsram(psramFound());
#endif
  s_barSpriteReady = (bool)s_barSprite.createSprite(SCALED_W, SCALED_BAR_SPRITE_H);
  if (s_barSpriteReady) {
    uint16_t* barProbe = (uint16_t*)s_barSprite.getBuffer();
    if (barProbe) {
      s_barSprite.fillScreen(0);
      s_barSprite.drawPixel(0, 0, 0xF800);  // pure red in RGB565
      s_barSpritePixelsByteSwapped = (barProbe[0] == 0x00F8);
      s_barSprite.fillScreen(0);
    }
  }
}

// Render timestamp bar text directly at display resolution.
// Top rows use SCALED_TOP_ROW_H; bottom row uses SCALED_BAR_H.
// into s_topBarBuf / s_botBarBuf. Avoids the 2× vertical upscale that makes bitmap
// fonts look chunky/blocky.
static void renderBarsAtScaledRes(int frameIdx) {
  if (!s_topBarBuf || !s_botBarBuf) return;
  if (frameIdx >= frameCount || !s_timesLoaded) return;

  ensureBarSpriteReady();
  if (!s_barSpriteReady) return;

  loadRadarMetaIfNeeded();
  time_t frameUtc = s_frameTimes[frameIdx];
  if (frameUtc == 0) return;
  time_t nowForAge = currentUtcForAgeMetrics();
  bool useRadarTopTime = s_topBarUseRadarScanTime && s_lastRadarUtcValid && s_lastRadarUtc > 0;
  time_t topUtc = useRadarTopTime ? s_lastRadarUtc : frameUtc;

  time_t localFrame = topUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
  struct tm tmLocal;
  memset(&tmLocal, 0, sizeof(tmLocal));
  gmtime_r(&localFrame, &tmLocal);

  double hoursAgo = difftime(nowForAge, frameUtc) / 3600.0;
  if (hoursAgo < 0) hoursAgo = 0;
  int minsAgo = (int)(difftime(nowForAge, topUtc) / 60.0 + 0.5);
  if (minsAgo < 0) minsAgo = 0;

  char dateBuf[16]; strftime(dateBuf, sizeof(dateBuf), "%b %d %Y", &tmLocal);
  char wdayBuf[8];  strftime(wdayBuf, sizeof(wdayBuf), "%a", &tmLocal);
  char timeBuf[16];
  {
    if (s_clockUse12Hour) {
      int hr12 = tmLocal.tm_hour % 12; if (!hr12) hr12 = 12;
      const char* ampm = (tmLocal.tm_hour < 12) ? "am" : "pm";
      snprintf(timeBuf, sizeof(timeBuf), "%d:%02d%s", hr12, tmLocal.tm_min, ampm);
    } else {
      snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", tmLocal.tm_hour, tmLocal.tm_min);
    }
  }

  const char* loc = (s_displayLocationFull[0] != '\0') ? s_displayLocationFull : DISPLAY_TZ_LABEL;

  char radarBuf[32];
  if (s_lastRadarUtcValid && s_lastRadarUtc > 0) {
    int radarMins = (int)(difftime(nowForAge, s_lastRadarUtc) / 60.0 + 0.5);
    if (radarMins < 0) radarMins = 0;
    snprintf(radarBuf, sizeof(radarBuf), "Radar: %d min", radarMins);
  } else if (s_radarNoSignatures) {
    if (s_lastRadarCheckUtc > 0) {
      int clearMins = (int)(difftime(nowForAge, s_lastRadarCheckUtc) / 60.0 + 0.5);
      if (clearMins < 0) clearMins = 0;
      snprintf(radarBuf, sizeof(radarBuf), "Radar: Clear %dm", clearMins);
    } else {
      snprintf(radarBuf, sizeof(radarBuf), "Radar: Clear");
    }
  } else if (s_radarDownloadFailed) {
    snprintf(radarBuf, sizeof(radarBuf), "Radar: no sig");
  } else {
    snprintf(radarBuf, sizeof(radarBuf), "Radar: n/a");
  }

  // Read battery once per bar render so the indicator stays fresh
  s_batPct = readAxp2101BatPct();
  s_batChargeState = readAxp2101ChargeState();
  // Playback normally runs with WiFi off, so this updates cached SSID/RSSI only
  // when the radio is actually connected and otherwise renders the last known state.
  refreshCachedWifiDisplayState();

  // Build strings for top (date/time) and bottom (location + radar split)
  char topBuf[64];
  if (useRadarTopTime) {
    snprintf(topBuf, sizeof(topBuf), "%s %s  %s  %d min ago", wdayBuf, dateBuf, timeBuf, minsAgo);
  } else {
    snprintf(topBuf, sizeof(topBuf), "%s %s  %s  %.1f h ago", wdayBuf, dateBuf, timeBuf, hoursAgo);
  }

  // Find the largest textSize (starting at 2.0) where the top string fits in one line.
  s_barSprite.setTextColor(TFT_WHITE);
  s_barSprite.setTextWrap(false);  // safety: clip at right edge, never wrap to row 2
  float ts = 2.0f;
  while (ts > 1.0f) {
    s_barSprite.setTextSize(ts);
    if (s_barSprite.textWidth(topBuf) <= SCALED_W) break;
    ts -= 0.1f;
  }

  // Vertically center top-row text within the larger top-row height.
  int fh = s_barSprite.fontHeight();
  int topTy = (SCALED_TOP_ROW_H - fh) / 2;
  if (topTy < 1) topTy = 1;

  // --- Battery icon row (row 0 of top bar) ---
  s_barSprite.fillScreen(0x0000);
  {
    int batPct = (int)s_batPct;
    int iconH = SCALED_TOP_ROW_H - 6;
    int iconW = iconH * 5 / 2;
    int iconX = SCALED_W - iconW - 6;
    int iconY = (SCALED_TOP_ROW_H - iconH) / 2;
    char pctBuf[6];
    if (batPct >= 0) snprintf(pctBuf, sizeof(pctBuf), "%d%%", (batPct > 100) ? 100 : batPct);
    else snprintf(pctBuf, sizeof(pctBuf), "--");
    s_barSprite.setTextWrap(false);
    s_barSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    s_barSprite.setTextSize(2);
    int pctW = s_barSprite.textWidth(pctBuf);
    if (pctW > 54) {
      s_barSprite.setTextSize(1);
      pctW = s_barSprite.textWidth(pctBuf);
    }
    int pctH = s_barSprite.fontHeight();
    int pctX = iconX - pctW - 6;
    if (pctX < 4) pctX = 4;
    int pctY = (SCALED_TOP_ROW_H - pctH) / 2;
    if (pctY < 1) pctY = 1;
    s_barSprite.setCursor(pctX, pctY);
    s_barSprite.print(pctBuf);
    drawBatteryIcon(s_barSprite, iconX, iconY, iconW, iconH, batPct, s_batChargeState);

    int wifiIconH = iconH - 2;
    if (wifiIconH < 8) wifiIconH = iconH;
    int wifiIconW = max(32, (wifiIconH * 8) / 5);
    int wifiIconX = 6;
    int wifiIconY = (SCALED_TOP_ROW_H - wifiIconH) / 2;
    bool wifiConnectedNow = (s_wifiRssi < 0);  // show bars whenever we have a valid RSSI from last sync
    drawWifiIndicator(s_barSprite, wifiIconX, wifiIconY, wifiIconW, wifiIconH, (int)s_wifiRssi, wifiConnectedNow);

    const int wifiTextX = wifiIconX + wifiIconW + 8;
    int modeGlyphH = max(16, wifiIconH);
    int modeGlyphW = max(18, (modeGlyphH * 9) / 8);
    int modeGlyphX = pctX - modeGlyphW - 10;
    if (modeGlyphX < wifiTextX + 12) modeGlyphX = wifiTextX + 12;
    int modeGlyphY = (SCALED_TOP_ROW_H - modeGlyphH) / 2;
    drawSleepModeGlyph(s_barSprite, modeGlyphX, modeGlyphY, modeGlyphW, modeGlyphH, s_sleepModeEnabled);

    int wifiAvailW = modeGlyphX - wifiTextX - 8;
    if (wifiAvailW > 12) {
      char wifiBuf[33];
      snprintf(wifiBuf, sizeof(wifiBuf), "%s", s_wifiDisplayName[0] ? s_wifiDisplayName : WIFI_SSID);
      float wifiTs = 2.0f;
      s_barSprite.setTextSize(wifiTs);
      int wifiTextW = s_barSprite.textWidth(wifiBuf);
      while (wifiTextW > wifiAvailW && wifiTs > 0.8f) {
        wifiTs -= 0.1f;
        s_barSprite.setTextSize(wifiTs);
        wifiTextW = s_barSprite.textWidth(wifiBuf);
      }
      size_t nameLen = strlen(wifiBuf);
      while (nameLen > 0 && wifiTextW > wifiAvailW) {
        wifiBuf[--nameLen] = '\0';
        wifiTextW = s_barSprite.textWidth(wifiBuf);
      }
      int wifiTextY = (SCALED_TOP_ROW_H - s_barSprite.fontHeight()) / 2;
      if (wifiTextY < 1) wifiTextY = 1;
      s_barSprite.setCursor(wifiTextX, wifiTextY);
      s_barSprite.print(wifiBuf);
    }
  }
  // Copy battery row to s_topBarBuf[0 .. SCALED_TOP_ROW_H-1]
  copyBarSpriteToBuffer(s_topBarBuf, (size_t)SCALED_W * SCALED_TOP_ROW_H);
  s_barSprite.setTextColor(TFT_WHITE);
  s_barSprite.setTextWrap(false);
  s_barSprite.setTextSize(ts);

  // --- Date/time row (row 1 of top bar) ---
  s_barSprite.fillScreen(0x0000);
  {
    int tw = s_barSprite.textWidth(topBuf);
    int tx = (SCALED_W - tw) / 2;
    if (tx < 0) tx = 0;
    s_barSprite.setCursor(tx, topTy);
    s_barSprite.print(topBuf);
  }
  // Copy date/time row to s_topBarBuf[SCALED_TOP_ROW_H .. 2*SCALED_TOP_ROW_H-1]
  copyBarSpriteToBuffer(s_topBarBuf + (size_t)SCALED_W * SCALED_TOP_ROW_H,
                        (size_t)SCALED_W * SCALED_TOP_ROW_H);

  // Bottom bar: location upper line, radar lower line.
  s_barSprite.fillScreen(0x0000);
  {
    // Keep the two bottom lines at one stable size; do not inherit the top
    // row's dynamic fit size, or the bottom text will visibly change when the
    // top date/time string width changes.
    const char* radarMeasure = "Radar: 999 min";
    const int minBottomGapPx = 3;
    float bottomTs = 2.0f;
    while (bottomTs > 1.0f) {
      s_barSprite.setTextSize(bottomTs);
      int botFhProbe = s_barSprite.fontHeight();
      int widestBottom = max(s_barSprite.textWidth(loc), s_barSprite.textWidth(radarMeasure));
      if (widestBottom <= SCALED_W && ((2 * botFhProbe) + minBottomGapPx) <= SCALED_BAR_H) break;
      bottomTs -= 0.1f;
    }
    s_barSprite.setTextSize(bottomTs);
    int botFh = s_barSprite.fontHeight();
    int usedH = (2 * botFh) + minBottomGapPx;
    int topPad = (SCALED_BAR_H - usedH) / 2;
    if (topPad < 0) topPad = 0;
    int locY = topPad;
    int radarY = locY + botFh + minBottomGapPx;
    if (radarY > SCALED_BAR_H - botFh) radarY = SCALED_BAR_H - botFh;
    if (radarY <= locY) radarY = min(SCALED_BAR_H - botFh, locY + botFh + 1);

    int locW = s_barSprite.textWidth(loc);
    int locX = (SCALED_W - locW) / 2;
    if (locX < 0) locX = 0;
    s_barSprite.setCursor(locX, locY);
    s_barSprite.print(loc);

    int radarW = s_barSprite.textWidth(radarBuf);
    int radarX = (SCALED_W - radarW) / 2;
    if (radarX < 0) radarX = 0;
    s_barSprite.setCursor(radarX, radarY);
    s_barSprite.print(radarBuf);
  }
  copyBarSpriteToBuffer(s_botBarBuf, (size_t)SCALED_W * SCALED_BAR_H);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hurricane Watch — storm bars, NOAA poll, suppression, mode enter/exit
// ═══════════════════════════════════════════════════════════════════════════

static uint16_t hurricaneCategoryColor(uint8_t cat) {
  switch (cat) {
    case 1:  return 0xFE60;  // yellow
    case 2:  return 0xFCC0;  // orange
    case 3:  return 0xF800;  // red
    case 4:
    case 5:  return 0xF81F;  // magenta
    default: return 0x07FF;  // cyan (TS/TD)
  }
}

static bool s_hurricaneBarsLoggedOnce = false;

static void renderHurricaneStormBars(int frameIdx) {
  if (!s_topBarBuf || !s_botBarBuf) return;
  ensureBarSpriteReady();
  if (!s_barSpriteReady) return;
  if (!s_hurricaneBarsLoggedOnce) {
    s_hurricaneBarsLoggedOnce = true;
  }

  // --- Top bar row 0: battery/wifi (reuse normal) ---
  s_batPct = readAxp2101BatPct();
  s_batChargeState = readAxp2101ChargeState();
  refreshCachedWifiDisplayState();
  s_barSprite.fillScreen(0x0000);
  {
    int iconH = SCALED_TOP_ROW_H - 6;
    int iconW = iconH * 5 / 2;
    int iconX = SCALED_W - iconW - 6;
    int iconY = (SCALED_TOP_ROW_H - iconH) / 2;
    char pctBuf[6];
    int batPct = (int)s_batPct;
    if (batPct >= 0) snprintf(pctBuf, sizeof(pctBuf), "%d%%", (batPct > 100) ? 100 : batPct);
    else snprintf(pctBuf, sizeof(pctBuf), "--");
    s_barSprite.setTextWrap(false);
    s_barSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    s_barSprite.setTextSize(2);
    int pctW = s_barSprite.textWidth(pctBuf);
    if (pctW > 54) { s_barSprite.setTextSize(1); pctW = s_barSprite.textWidth(pctBuf); }
    int pctH = s_barSprite.fontHeight();
    int pctX = iconX - pctW - 6; if (pctX < 4) pctX = 4;
    int pctY = (SCALED_TOP_ROW_H - pctH) / 2; if (pctY < 1) pctY = 1;
    s_barSprite.setCursor(pctX, pctY);
    s_barSprite.print(pctBuf);
    drawBatteryIcon(s_barSprite, iconX, iconY, iconW, iconH, batPct, s_batChargeState);
  }
  copyBarSpriteToBuffer(s_topBarBuf, (size_t)SCALED_W * SCALED_TOP_ROW_H);

  // --- Top bar row 1: storm name + category ---
  s_barSprite.fillScreen(0x0000);
  {
    char nameBuf[48];
    if (strcmp(s_activeStorm.stormType, "HU") == 0)
      snprintf(nameBuf, sizeof(nameBuf), "HURRICANE %s", s_activeStorm.name);
    else if (strcmp(s_activeStorm.stormType, "TS") == 0)
      snprintf(nameBuf, sizeof(nameBuf), "T.S. %s", s_activeStorm.name);
    else
      snprintf(nameBuf, sizeof(nameBuf), "T.D. %s", s_activeStorm.name);

    char catBuf[8];
    if (s_activeStorm.category >= 1)
      snprintf(catBuf, sizeof(catBuf), "CAT %d", s_activeStorm.category);
    else if (strcmp(s_activeStorm.stormType, "TS") == 0)
      strlcpy(catBuf, "TS", sizeof(catBuf));
    else
      strlcpy(catBuf, "TD", sizeof(catBuf));

    float ts = 2.0f;
    s_barSprite.setTextSize(ts);
    int totalW = s_barSprite.textWidth(nameBuf) + s_barSprite.textWidth(catBuf) + 20;
    while (totalW > SCALED_W && ts > 1.0f) {
      ts -= 0.1f;
      s_barSprite.setTextSize(ts);
      totalW = s_barSprite.textWidth(nameBuf) + s_barSprite.textWidth(catBuf) + 20;
    }
    int fh = s_barSprite.fontHeight();
    int ty = (SCALED_TOP_ROW_H - fh) / 2; if (ty < 1) ty = 1;

    s_barSprite.setTextColor(TFT_WHITE);
    s_barSprite.setCursor(4, ty);
    s_barSprite.print(nameBuf);

    uint16_t catColor = hurricaneCategoryColor(s_activeStorm.category);
    s_barSprite.setTextColor(catColor);
    int catW = s_barSprite.textWidth(catBuf);
    s_barSprite.setCursor(SCALED_W - catW - 4, ty);
    s_barSprite.print(catBuf);
  }
  copyBarSpriteToBuffer(s_topBarBuf + (size_t)SCALED_W * SCALED_TOP_ROW_H,
                        (size_t)SCALED_W * SCALED_TOP_ROW_H);

  // --- Bottom bar: wind/pressure + advisory age ---
  s_barSprite.fillScreen(0x0000);
  {
    char windBuf[32];
    snprintf(windBuf, sizeof(windBuf), "%u kt  %u mb",
             (unsigned)s_activeStorm.windKt, (unsigned)s_activeStorm.pressureMb);

    char ageBuf[32];
    time_t now = currentUtcForAgeMetrics();
    int ageMin = (int)(difftime(now, s_activeStorm.advisoryUtc) / 60.0 + 0.5);
    if (ageMin < 0) ageMin = 0;
    if (ageMin >= 120)
      snprintf(ageBuf, sizeof(ageBuf), "%dh ago", ageMin / 60);
    else
      snprintf(ageBuf, sizeof(ageBuf), "%dm ago", ageMin);

    float bottomTs = 2.0f;
    s_barSprite.setTextSize(bottomTs);
    int totalW = s_barSprite.textWidth(windBuf) + s_barSprite.textWidth(ageBuf) + 20;
    while (totalW > SCALED_W && bottomTs > 1.0f) {
      bottomTs -= 0.1f;
      s_barSprite.setTextSize(bottomTs);
      totalW = s_barSprite.textWidth(windBuf) + s_barSprite.textWidth(ageBuf) + 20;
    }
    int botFh = s_barSprite.fontHeight();
    int by = (SCALED_BAR_H - botFh) / 2; if (by < 0) by = 0;

    s_barSprite.setTextColor(TFT_WHITE);
    s_barSprite.setCursor(4, by);
    s_barSprite.print(windBuf);

    int ageW = s_barSprite.textWidth(ageBuf);
    s_barSprite.setCursor(SCALED_W - ageW - 4, by);
    s_barSprite.print(ageBuf);
  }
  copyBarSpriteToBuffer(s_botBarBuf, (size_t)SCALED_W * SCALED_BAR_H);
}

// Draw "Reboot for local weather" in the black border below the frame.
// Uses direct AMOLED text draw (same approach as showMessage).
static bool s_hurricaneHintDrawn = false;

static void drawHurricaneRebootHint(uint16_t* buf) {
  (void)buf;
  if (!s_amoledOut || s_hurricaneHintDrawn) return;

  int screenW = s_amoledOut->width();
  int frameBottom = (AMOLED_HEIGHT - SCALED_H) / 2 + SCALED_H;  // 71 + 360 = 431
  int borderH = AMOLED_HEIGHT - frameBottom;  // 71px

  // Clear the bottom border
  s_amoledOut->fillRect(0, frameBottom, screenW, borderH, 0x0000);

  // Draw text directly to AMOLED
  const char* hint = "Reboot for local weather";
  s_amoledOut->setTextColor(0xFFFF);
  s_amoledOut->setTextSize(2);
  int cx = screenW / 2 - (int)(strlen(hint) * 6 * 2) / 2;  // 6px per char at size 1, *2 for size 2
  int cy = frameBottom + (borderH - 16) / 2;  // 16px tall at size 2
  if (cx < 2) cx = 2;
  s_amoledOut->setCursor(cx, cy);
  s_amoledOut->print(hint);

  s_hurricaneHintDrawn = true;
}

// ── Storm Bbox Computation ────────────────────────────────────────────────
static void computeStormBbox(float lat, float lon, uint8_t category,
                             float* west, float* south, float* east, float* north) {
  float radiusKm;
  switch (category) {
    case 1:  radiusKm = 800.0f; break;
    case 2:  radiusKm = 700.0f; break;
    case 3:  radiusKm = 600.0f; break;
    case 4:  radiusKm = 500.0f; break;
    case 5:  radiusKm = 450.0f; break;
    default: radiusKm = 900.0f; break;  // TD/TS — wider view
  }
  float cosLat = cosf(lat * 0.01745329252f);
  if (cosLat < 0.2f) cosLat = 0.2f;
  float dLat = radiusKm / 111.32f;
  float dLon = radiusKm / (111.32f * cosLat);
  float aspectRatio = 41.0f / 36.0f;
  if (dLon / dLat < aspectRatio) dLon = dLat * aspectRatio;
  else dLat = dLon / aspectRatio;

  float w = lon - dLon, e2 = lon + dLon;
  float s = lat - dLat, n = lat + dLat;
  if (s < -89.5f) s = -89.5f;
  if (n >  89.5f) n =  89.5f;
  if (w < -180.0f) w = -180.0f;
  if (e2 > 180.0f) e2 = 180.0f;
  if (west)  *west  = w;
  if (south) *south = s;
  if (east)  *east  = e2;
  if (north) *north = n;
}

// ── NOAA Poll + JSON Parse ────────────────────────────────────────────────
static bool parseNoaaStormJson(const String& body, HurricaneInfo* storms,
                               int maxStorms, int* outCount) {
  *outCount = 0;
  const char* s = body.c_str();
  const char* cursor = s;

  while (*outCount < maxStorms) {
    const char* propStart = strstr(cursor, "\"properties\"");
    if (!propStart) break;
    // Find the closing brace for this properties block
    const char* braceOpen = strchr(propStart, '{');
    if (!braceOpen) break;
    // Find the next "properties" or end to bound the search
    const char* nextProp = strstr(braceOpen + 1, "\"properties\"");
    int blockLen = nextProp ? (int)(nextProp - braceOpen) : (int)(strlen(braceOpen));
    // Create a substring for this feature's properties
    String block = body.substring((int)(braceOpen - s), (int)(braceOpen - s) + blockLen);

    char stormType[8] = {};
    char stormName[20] = {};
    int32_t stormNum = 0, ssNum = 0, intensity = 0, mslp = 0;
    float lat = 0, lon = 0;

    jsonExtractStringField(block, "\"STORMTYPE\"", stormType, sizeof(stormType));
    jsonExtractStringField(block, "\"STORMNAME\"", stormName, sizeof(stormName));
    jsonExtractIntField(block, "\"STORMNUM\"", &stormNum);
    jsonExtractIntField(block, "\"SSNUM\"", &ssNum);
    jsonExtractIntField(block, "\"INTENSITY\"", &intensity);
    jsonExtractIntField(block, "\"MSLP\"", &mslp);
    jsonExtractFloatField(block, "\"LAT\"", &lat);
    jsonExtractFloatField(block, "\"LON\"", &lon);

    bool qualifies = false;
    if (stormName[0] != '\0') {
      if (strcmp(stormType, "HU") == 0 && ssNum >= 1) qualifies = true;
      if (s_hurricaneIncludeTS && strcmp(stormType, "TS") == 0) qualifies = true;
      if (s_hurricaneIncludeTD && strcmp(stormType, "TD") == 0) qualifies = true;
    }

    if (qualifies) {
      HurricaneInfo& st = storms[*outCount];
      memset(&st, 0, sizeof(st));
      struct tm tmNow; time_t now = time(nullptr); gmtime_r(&now, &tmNow);
      snprintf(st.id, sizeof(st.id), "AL%02d%04d", (int)stormNum, tmNow.tm_year + 1900);
      strlcpy(st.name, stormName, sizeof(st.name));
      strlcpy(st.stormType, stormType, sizeof(st.stormType));
      st.lat = lat;
      st.lon = lon;
      st.category = (uint8_t)ssNum;
      st.windKt = (uint16_t)intensity;
      st.pressureMb = (uint16_t)mslp;
      st.advisoryUtc = time(nullptr);
      (*outCount)++;
    }
    cursor = braceOpen + 1;
  }

  // Sort by STORMNUM descending (newest first) — bubble sort, max 4 items
  for (int i = 0; i < *outCount - 1; i++) {
    for (int j = 0; j < *outCount - 1 - i; j++) {
      // Extract storm number from ID (characters 2-3)
      int numA = atoi(storms[j].id + 2);
      int numB = atoi(storms[j + 1].id + 2);
      if (numB > numA) {
        HurricaneInfo tmp = storms[j];
        storms[j] = storms[j + 1];
        storms[j + 1] = tmp;
      }
    }
  }
  return (*outCount > 0);
}

static bool pollNoaaForHurricane(HurricaneInfo* storms, int maxStorms, int* outCount) {
  *outCount = 0;
#ifdef HURRICANE_TEST_MODE
  // Inject fake Cat 3 storm for off-season testing
  HurricaneInfo& fake = storms[0];
  memset(&fake, 0, sizeof(fake));
  strlcpy(fake.id, "AL052026", sizeof(fake.id));
  strlcpy(fake.name, "TESTRINA", sizeof(fake.name));
  strlcpy(fake.stormType, "HU", sizeof(fake.stormType));
  fake.lat = 25.0f;
  fake.lon = -75.0f;
  fake.category = 3;
  fake.windKt = 120;
  fake.pressureMb = 955;
  fake.advisoryUtc = time(nullptr);
  *outCount = 1;
  Serial.println("hurricane: TEST MODE — fake Cat 3");
  return true;
#endif

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, "https://www.nhc.noaa.gov/CurrentSurfaces/AT.json");
  http.setTimeout(8000);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("hurricane: NOAA HTTP %d\n", httpCode);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  if (body.length() < 10) {
    Serial.println("hurricane: NOAA response too short");
    return false;
  }
  bool ok = parseNoaaStormJson(body, storms, maxStorms, outCount);
  Serial.printf("hurricane: NOAA %d bytes, %d storms\n", (int)body.length(), *outCount);
  return ok;
}

// ── Suppression Logic ─────────────────────────────────────────────────────
static bool isStormSuppressed(const char* stormId) {
  Preferences prefs;
  if (!prefs.begin("satwatch", true)) return false;
  char supList[128] = {};
  prefs.getString("hwsup", supList, sizeof(supList));
  prefs.end();
  if (supList[0] == '\0') return false;
  // Search for stormId in comma-separated list
  const char* p = supList;
  size_t idLen = strlen(stormId);
  while (p) {
    if (strncmp(p, stormId, idLen) == 0 && (p[idLen] == ',' || p[idLen] == '\0')) return true;
    p = strchr(p, ',');
    if (p) p++;
  }
  return false;
}

static void suppressStorm(const char* stormId) {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  char supList[128] = {};
  prefs.getString("hwsup", supList, sizeof(supList));
  if (supList[0] != '\0') {
    // Check if already in list
    const char* p = supList;
    size_t idLen = strlen(stormId);
    while (p) {
      if (strncmp(p, stormId, idLen) == 0 && (p[idLen] == ',' || p[idLen] == '\0')) {
        prefs.end();
        return;
      }
      p = strchr(p, ',');
      if (p) p++;
    }
    strlcat(supList, ",", sizeof(supList));
  }
  strlcat(supList, stormId, sizeof(supList));
  prefs.putString("hwsup", supList);
  prefs.end();
  Serial.printf("hurricane: suppressed %s\n", stormId);
}

static void cleanupSuppressedStorms(const HurricaneInfo* activeStorms, int activeCount) {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  char supList[128] = {};
  prefs.getString("hwsup", supList, sizeof(supList));
  if (supList[0] == '\0') { prefs.end(); return; }

  // Seasonal expiry: clear if any ID is from a previous year and we're past June 1
  struct tm tmNow; time_t now = time(nullptr); gmtime_r(&now, &tmNow);
  int curYear = tmNow.tm_year + 1900;

  char cleaned[128] = {};
  char* tok = strtok(supList, ",");
  while (tok) {
    bool keep = false;
    // Check if storm is still active
    for (int i = 0; i < activeCount; i++) {
      if (strcmp(tok, activeStorms[i].id) == 0) { keep = true; break; }
    }
    // Seasonal check: extract year from last 4 chars
    size_t tokLen = strlen(tok);
    if (!keep && tokLen >= 4) {
      int idYear = atoi(tok + tokLen - 4);
      if (idYear < curYear && tmNow.tm_mon >= 5) keep = false;  // expired
      else if (idYear == curYear) keep = true;  // same year, storm just not in current list — keep for safety
    }
    if (keep) {
      if (cleaned[0] != '\0') strlcat(cleaned, ",", sizeof(cleaned));
      strlcat(cleaned, tok, sizeof(cleaned));
    }
    tok = strtok(nullptr, ",");
  }
  prefs.putString("hwsup", cleaned);
  prefs.end();
}

// ── Mode Enter/Exit ───────────────────────────────────────────────────────
static void triggerHurricaneAlert() {
#if BOARD_IS_AMOLED_206
  // Try to load hurricane alert sound directly
  File cueFile = SD.open(s_hurricaneAlertSound, FILE_READ);
  if (!cueFile) {
    Serial.printf("hurricane: alert sound %s not found, using default chime\n", s_hurricaneAlertSound);
    s_startCuePending = true;
    return;
  }
  size_t cueLen = (size_t)cueFile.size();
  if (cueLen == 0 || cueLen > 512000) { cueFile.close(); s_startCuePending = true; return; }
  uint8_t* newBuf = (uint8_t*)heap_caps_malloc(cueLen, MALLOC_CAP_SPIRAM);
  if (!newBuf) { cueFile.close(); s_startCuePending = true; return; }
  size_t got = cueFile.read(newBuf, cueLen);
  cueFile.close();
  if (got != cueLen) { heap_caps_free(newBuf); s_startCuePending = true; return; }
  // Replace active cue buffer
  if (s_audioCueBuf) heap_caps_free(s_audioCueBuf);
  s_audioCueBuf = newBuf;
  s_audioCueLen = cueLen;
  s_audioCueReady = true;
  strlcpy(s_audioCueLoadedPath, s_hurricaneAlertSound, sizeof(s_audioCueLoadedPath));
  Serial.printf("hurricane: loaded alert %s (%u bytes)\n", s_hurricaneAlertSound, (unsigned)cueLen);
  s_startCuePending = true;
#endif
}

static void restoreNormalAudioCue() {
#if BOARD_IS_AMOLED_206
  preloadSelectedCueToPsram(true);
#endif
}

static void enterHurricaneMode(const HurricaneInfo& storm) {
  // Save current weather center
  s_savedWeatherCenterLat = s_weatherCenterLat;
  s_savedWeatherCenterLon = s_weatherCenterLon;
  s_savedWeatherGeoValid = s_weatherGeoValid;

  // Override center to storm position
  s_weatherCenterLat = storm.lat;
  s_weatherCenterLon = storm.lon;
  s_weatherGeoValid = true;
  selectSatelliteForLon(storm.lon, true);

  s_hurricaneMode = true;
  memcpy(&s_activeStorm, &storm, sizeof(HurricaneInfo));

  // Write active storm ID to NVS for reboot suppression
  {
    Preferences prefs;
    if (prefs.begin("satwatch", false)) {
      prefs.putString("hwact", storm.id);
      prefs.end();
    }
  }

  // Trigger alert if this is a new storm
  if (strcmp(s_lastHurricaneAlertId, storm.id) != 0) {
    strlcpy(s_lastHurricaneAlertId, storm.id, sizeof(s_lastHurricaneAlertId));
    triggerHurricaneAlert();
  }

  framesReady = false;  // force re-download with storm bbox
  s_hurricaneLoopsSinceCheck = 0;
  s_hurricaneBarsLoggedOnce = false;
  s_hurricaneHintDrawn = false;
  Serial.printf("hurricane: enter %s %s cat=%d at %.1f,%.1f\n",
                storm.id, storm.name, storm.category, (double)storm.lat, (double)storm.lon);
  appendDiagLog("hurricane: enter %s %s cat=%d lat=%.1f lon=%.1f wind=%ukt %umb\n",
                storm.id, storm.name, storm.category, (double)storm.lat, (double)storm.lon,
                (unsigned)storm.windKt, (unsigned)storm.pressureMb);
}

static void exitHurricaneMode() {
  // Restore saved weather center
  s_weatherCenterLat = s_savedWeatherCenterLat;
  s_weatherCenterLon = s_savedWeatherCenterLon;
  s_weatherGeoValid = s_savedWeatherGeoValid;

  Serial.printf("hurricane: exit %s\n", s_activeStorm.id);
  appendDiagLog("hurricane: exit %s\n", s_activeStorm.id);

  s_hurricaneMode = false;
  memset(&s_activeStorm, 0, sizeof(s_activeStorm));
  restoreNormalAudioCue();
  framesReady = false;  // force view refresh with restored bbox
}

// ── Periodic Re-check ─────────────────────────────────────────────────────
static void hurricaneRecheckAndUpdate() {
  if (!connectWifiForSync(false, "Checking storms...")) return;
  HurricaneInfo storms[4];
  int stormCount = 0;
  bool polled = pollNoaaForHurricane(storms, 4, &stormCount);
  disconnectWifiAfterSync();

  if (!polled || stormCount == 0) {
    // Storm dissipated — exit mode
    exitHurricaneMode();
    return;
  }

  // Check if our active storm is still in the list
  bool found = false;
  for (int i = 0; i < stormCount; i++) {
    if (strcmp(storms[i].id, s_activeStorm.id) == 0) {
      // Update position/category
      float dLat = storms[i].lat - s_activeStorm.lat;
      float dLon = storms[i].lon - s_activeStorm.lon;
      float distKm = sqrtf(dLat * dLat + dLon * dLon) * 111.32f;

      s_activeStorm.lat = storms[i].lat;
      s_activeStorm.lon = storms[i].lon;
      s_activeStorm.category = storms[i].category;
      s_activeStorm.windKt = storms[i].windKt;
      s_activeStorm.pressureMb = storms[i].pressureMb;
      s_activeStorm.advisoryUtc = storms[i].advisoryUtc;
      strlcpy(s_activeStorm.stormType, storms[i].stormType, sizeof(s_activeStorm.stormType));

      s_weatherCenterLat = storms[i].lat;
      s_weatherCenterLon = storms[i].lon;

      if (distKm > 100.0f) {
        Serial.printf("hurricane: %s moved %.0fkm, re-sync\n", storms[i].id, (double)distKm);
        selectSatelliteForLon(storms[i].lon, true);
        framesReady = false;
      } else {
      }
      found = true;
      break;
    }
  }

  if (!found) {
    // Active storm not in list anymore — check for new unsuppressed storm
    cleanupSuppressedStorms(storms, stormCount);
    bool entered = false;
    for (int i = 0; i < stormCount; i++) {
      if (!isStormSuppressed(storms[i].id)) {
        exitHurricaneMode();
        enterHurricaneMode(storms[i]);
        entered = true;
        break;
      }
    }
    if (!entered) {
      exitHurricaneMode();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  End of Hurricane Watch section
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  Moon Phase Complication
// ═══════════════════════════════════════════════════════════════════════════

// Compute moon phase as 0.0 (new) → 0.5 (full) → 1.0 (new).
// Standard synodic month algorithm referenced to a known new moon epoch.
static float computeMoonPhase(time_t utc) {
  // Reference new moon: Jan 6 2000 18:14 UTC (J2000 lunation 0)
  const double refNewMoon = 947182440.0;  // Unix epoch of that new moon
  const double synodicMonth = 29.53058770576;  // days
  double daysSinceRef = difftime((double)utc, refNewMoon) / 86400.0;
  double phase = fmod(daysSinceRef / synodicMonth, 1.0);
  if (phase < 0.0) phase += 1.0;
  return (float)phase;
}

// JPEGDEC draw callback for moon — renders into s_moonBuf
static int moonJpegDraw(JPEGDRAW* pDraw) {
  if (!s_moonBuf) return 0;
  int drawW = (pDraw->iWidthUsed > 0) ? pDraw->iWidthUsed : pDraw->iWidth;
  uint16_t* src = (uint16_t*)pDraw->pPixels;
  for (int row = 0; row < pDraw->iHeight; row++) {
    int dstY = pDraw->y + row;
    if (dstY < 0 || dstY >= MOON_DECODED_PX) continue;
    for (int col = 0; col < drawW; col++) {
      int dstX = pDraw->x + col;
      if (dstX < 0 || dstX >= MOON_DECODED_PX) continue;
      s_moonBuf[dstY * MOON_DECODED_PX + dstX] = src[row * pDraw->iWidth + col];
    }
  }
  return 1;
}

// Download all 30 moon phase frames from NASA SVS on first boot.
// Frames are from the 2026 Moon Phase & Libration set, 216×216, evenly
// spaced across one lunation starting at new moon (Jan 18 2026).
static void downloadMoonFramesIfMissing() {
  // Check if already cached
  if (SD.exists("/moon/moon_00.jpg") && SD.exists("/moon/moon_14.jpg")) return;

  SD.mkdir("/moon");
  showMessage("Downloading moon...", "One-time setup");

  WiFiClientSecure client;
  client.setInsecure();

  // Frame numbers in the NASA SVS 8760-frame set (hourly, Jan 1 00:00 UTC).
  // New moon Jan 18 ~19:00 UTC = frame 427. Step = 709/30 ≈ 24.
  const int startFrame = 427;
  const int step = 24;
  int ok = 0;

  for (int i = 0; i < MOON_FRAME_COUNT; i++) {
    char sdPath[32];
    snprintf(sdPath, sizeof(sdPath), "/moon/moon_%02d.jpg", i);
    if (SD.exists(sdPath)) { ok++; continue; }  // resume partial download

    int nasaFrame = startFrame + i * step;
    char url[128];
    snprintf(url, sizeof(url),
      "https://svs.gsfc.nasa.gov/vis/a000000/a005500/a005587/frames/216x216_1x1_30p/moon.%04d.jpg",
      nasaFrame);

    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(10000);
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      int len = http.getSize();
      if (len > 0 && len < 20000) {
        WiFiClient* stream = http.getStreamPtr();
        File f = SD.open(sdPath, FILE_WRITE);
        if (f) {
          uint8_t buf[512];
          int total = 0;
          while (total < len) {
            int avail = stream->available();
            if (avail <= 0) { delay(5); continue; }
            int toRead = (avail < (int)sizeof(buf)) ? avail : (int)sizeof(buf);
            if (toRead > len - total) toRead = len - total;
            int got = stream->read(buf, toRead);
            if (got <= 0) break;
            f.write(buf, got);
            total += got;
          }
          f.flush();
          f.close();
          if (total == len) ok++;
          else SD.remove(sdPath);  // partial — delete
        }
      }
    }
    http.end();
    drawProgressBarUi(i + 1, MOON_FRAME_COUNT, "moon phases");
  }
  Serial.printf("moon: downloaded %d/%d frames\n", ok, MOON_FRAME_COUNT);
}

// Decode the appropriate moon phase JPEG from SD into s_moonBuf.
static bool decodeMoonPhase() {
  time_t now = time(nullptr);
  if (now < 1000000000) {
    appendDiagLog("moon: skip — NTP not set (now=%ld)\n", (long)now);
    return false;
  }

  float phase = computeMoonPhase(now);
  int frameIdx = (int)(phase * MOON_FRAME_COUNT + 0.5f) % MOON_FRAME_COUNT;

  char path[32];
  snprintf(path, sizeof(path), "/moon/moon_%02d.jpg", frameIdx);

  File f = SD.open(path, FILE_READ);
  if (!f) { appendDiagLog("moon: %s not found\n", path); return false; }
  size_t fSize = (size_t)f.size();
  if (fSize == 0 || fSize > 20000) { f.close(); return false; }

  uint8_t* jpegBuf = (uint8_t*)heap_caps_malloc(fSize, MALLOC_CAP_SPIRAM);
  if (!jpegBuf) { f.close(); return false; }
  size_t got = f.read(jpegBuf, fSize);
  f.close();
  if (got != fSize) { heap_caps_free(jpegBuf); return false; }

  if (!s_moonBuf) {
    s_moonBuf = (uint16_t*)heap_caps_malloc(MOON_DECODED_PX * MOON_DECODED_PX * 2, MALLOC_CAP_SPIRAM);
    if (!s_moonBuf) { heap_caps_free(jpegBuf); return false; }
  }
  memset(s_moonBuf, 0, MOON_DECODED_PX * MOON_DECODED_PX * 2);

  JPEGDEC moonJpeg;
  bool ok = false;
  if (moonJpeg.openRAM(jpegBuf, (int)fSize, moonJpegDraw)) {
    moonJpeg.setPixelType(RGB565_BIG_ENDIAN);
    ok = moonJpeg.decode(0, 0, JPEG_SCALE_QUARTER);
    moonJpeg.close();
  }
  heap_caps_free(jpegBuf);
  // Convert from JPEGDEC big-endian to native little-endian RGB565
  if (ok) {
    int npx = MOON_DECODED_PX * MOON_DECODED_PX;
    for (int i = 0; i < npx; i++) s_moonBuf[i] = __builtin_bswap16(s_moonBuf[i]);
    s_moonDrawn = false;  // force redraw
  }
  appendDiagLog("moon: frame=%d phase=%.2f decode=%s buf=%p\n",
                frameIdx, (double)phase, ok ? "ok" : "FAIL", (void*)s_moonBuf);
  return ok;
}

// Draw moon phase icon in the AMOLED bottom border (Y=431-502).
// s_moonBuf must already be converted to native little-endian RGB565.
// Uses s_moonDrawn flag: AMOLED retains pixels so we draw once.
static void drawMoonComplication() {
  if (!s_amoledOut || !s_moonBuf || s_hurricaneMode || s_moonDrawn) return;
  const int borderY = (AMOLED_HEIGHT - SCALED_H) / 2 + SCALED_H; // 71 + 360 = 431
  const int moonX = (SCALED_W - MOON_DECODED_PX) / 2;              // horizontally centered
  const int moonY = borderY + (AMOLED_HEIGHT - borderY - MOON_DECODED_PX) / 2; // vertically centered in border
  s_amoledOut->draw16bitRGBBitmap(moonX, moonY, s_moonBuf, MOON_DECODED_PX, MOON_DECODED_PX);
  s_moonDrawn = true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  End of Moon Phase section
// ═══════════════════════════════════════════════════════════════════════════

#endif  // BOARD_IS_AMOLED_206

// Render fresh timestamp bars for frameIdx into s_topBarBuf / s_botBarBuf.
// On AMOLED: renders directly at display resolution for crisp output.
// On other boards: draws into sprite bar rows then scales.
static void updateBarBufs(int frameIdx) {
  if (!s_topBarBuf || !s_botBarBuf) return;
#if BOARD_IS_AMOLED_206
  if (s_hurricaneMode) renderHurricaneStormBars(frameIdx);
  else renderBarsAtScaledRes(frameIdx);
#else
  if (!spriteReady) return;
  drawTimestamp(frameIdx, &sprite);
  scaleBarRowsToBuffer(s_topBarBuf, 0);
  scaleBarRowsToBuffer(s_botBarBuf, DISP_H - 14);
#endif
}

// Stamp the pre-rendered bar buffers over the top and bottom of buf (410×360).
// Called inside presentScaledBuf() so bars persist through all animation phases.
static void applyBarsToBuf(uint16_t* buf) {
  if (!buf || !s_topBarBuf || !s_botBarBuf) return;
  size_t topBarBytes = (size_t)SCALED_W * (size_t)SCALED_TOP_BAR_H * 2U;
  size_t botBarBytes = (size_t)SCALED_W * (size_t)SCALED_BAR_H * 2U;
  // Top bar: rows 0..SCALED_TOP_BAR_H-1 (flush against top of frame)
  memcpy(buf, s_topBarBuf, topBarBytes);
  // Bottom bar: rows SCALED_H-SCALED_BAR_H..SCALED_H-1 (flush against bottom)
  int botY = SCALED_H - SCALED_BAR_H;
  memcpy(buf + (size_t)botY * SCALED_W, s_botBarBuf, botBarBytes);
}

// Push a pre-scaled 410×360 canonical RGB565 buffer to the AMOLED display.
// src must point to SCALED_FRAME_BYTES of little-endian RGB565.
static void presentScaledBuf(uint16_t* src) {
#if BOARD_IS_AMOLED_206
  if (!s_amoledOut || !src) return;
  // Stamp persistent timestamp bars over whatever is in src.
  applyBarsToBuf(src);
  if (s_amoledClearBeforeNextPresent) {
    s_amoledOut->fillScreen(0x0000);
    s_amoledClearBeforeNextPresent = false;
    s_hurricaneHintDrawn = false;  // redraw hint after screen clear
    s_moonDrawn = false;           // redraw moon after screen clear
  }
  static const int CHUNK_ROWS = 8;
  const int outW = SCALED_W;   // 410
  const int outH = SCALED_H;   // 360
  const int outX = 0;
  const int outY = (AMOLED_HEIGHT - outH) / 2;  // (502-360)/2 = 71
  // PSRAM-allocated chunk buffer: keeps ~20 KB out of internal DRAM for WiFi/SSL use
  static uint16_t* s_chunkBuf = nullptr;
  if (!s_chunkBuf) s_chunkBuf = (uint16_t*)heap_caps_malloc((size_t)CHUNK_ROWS * SCALED_W * 2U, MALLOC_CAP_SPIRAM);
  if (!s_chunkBuf) return;
  for (int cy = 0; cy < outH; cy += CHUNK_ROWS) {
    int rows = CHUNK_ROWS;
    if (cy + rows > outH) rows = outH - cy;
    memcpy(s_chunkBuf, src + cy * outW, (size_t)rows * outW * 2U);
    s_amoledOut->draw16bitRGBBitmap(outX, outY + cy, s_chunkBuf, outW, rows);
  }
  if (s_hurricaneMode) drawHurricaneRebootHint(nullptr);
  drawMoonComplication();
#else
  tft.startWrite(); sprite.pushSprite(0, 0); tft.waitDMA(); tft.endWrite();
#endif
}

static bool presentSyntheticZoomStage(const char* path, int newestIdx) {
  if (!path || !s_frameDisplayBuf || !s_terrainDisplayBuf) return false;

  float ratio = 0.0f;
  if (strcmp(path, ZOOM1_FILE) == 0) ratio = 0.82f;
  else if (strcmp(path, ZOOM2_FILE) == 0) ratio = 0.72f;
  else if (strcmp(path, ZOOM3_FILE) == 0) ratio = 0.64f;
  else return false;

  memcpy(s_terrainDisplayBuf, s_frameDisplayBuf, SCALED_FRAME_BYTES);

  int cropW = (int)(SCALED_W * ratio + 0.5f);
  int cropH = (int)(SCALED_H * ratio + 0.5f);
  if (cropW < 8 || cropH < 8) return false;
  if (cropW > SCALED_W) cropW = SCALED_W;
  if (cropH > SCALED_H) cropH = SCALED_H;

  int cropX = (SCALED_W - cropW) / 2;
  int cropY = (SCALED_H - cropH) / 2;

  for (int y = 0; y < SCALED_H; y++) {
    int sy = cropY + ((y * cropH) / SCALED_H);
    if (sy < 0) sy = 0;
    if (sy >= SCALED_H) sy = SCALED_H - 1;
    uint16_t* dstRow = s_frameDisplayBuf + ((size_t)y * SCALED_W);
    const uint16_t* srcBase = s_terrainDisplayBuf + ((size_t)sy * SCALED_W);
    for (int x = 0; x < SCALED_W; x++) {
      int sx = cropX + ((x * cropW) / SCALED_W);
      if (sx < 0) sx = 0;
      if (sx >= SCALED_W) sx = SCALED_W - 1;
      dstRow[x] = srcBase[sx];
    }
  }

  updateBarBufs(newestIdx);
  presentScaledBuf(s_frameDisplayBuf);
  Serial.printf("zoom synth %s\n", path);
  return true;
}

static bool scaledFrameLooksFreezeBlockCorrupted() {
  if (!s_frameDisplayBuf) return false;

  constexpr int kBlock = 20;
  constexpr int kCols = (SCALED_W + kBlock - 1) / kBlock;  // 21
  constexpr int kUsableH = SCALED_H - SCALED_TOP_BAR_H - SCALED_BAR_H;  // 267
  constexpr int kRows = (kUsableH + kBlock - 1) / kBlock;  // 14

  bool zmap[kRows][kCols] = {};
  const int imageY0 = SCALED_TOP_BAR_H;
  const int imageY1 = SCALED_H - SCALED_BAR_H;

  for (int br = 0; br < kRows; ++br) {
    int y0 = imageY0 + br * kBlock;
    int y1 = y0 + kBlock;
    if (y0 >= imageY1) break;
    if (y1 > imageY1) y1 = imageY1;

    for (int bc = 0; bc < kCols; ++bc) {
      int x0 = bc * kBlock;
      int x1 = x0 + kBlock;
      if (x0 >= SCALED_W) break;
      if (x1 > SCALED_W) x1 = SCALED_W;

      bool allZero = true;
      for (int y = y0; y < y1 && allZero; y += 4) {
        const uint16_t* row = s_frameDisplayBuf + (size_t)y * (size_t)SCALED_W;
        for (int x = x0; x < x1; x += 4) {
          if (row[x] != 0) {
            allZero = false;
            break;
          }
        }
      }
      zmap[br][bc] = allZero;
    }
  }

  constexpr int kFullRowThreshold = kCols - 6;  // 15 of 21 blocks
  for (int br = 1; br < kRows - 1; ++br) {
    int zeroCount = 0;
    for (int bc = 1; bc < kCols - 1; ++bc) {
      if (zmap[br][bc]) zeroCount++;
    }
    if (zeroCount >= kFullRowThreshold) {
      Serial.printf("freezeband row=%d zeros=%d\n", br, zeroCount);
      return true;
    }
  }

  constexpr int kHalfRow = (kCols - 2) / 2;
  for (int br = 1; br < kRows - 2; ++br) {
    int z0 = 0;
    int z1 = 0;
    for (int bc = 1; bc < kCols - 1; ++bc) {
      if (zmap[br][bc]) z0++;
      if (zmap[br + 1][bc]) z1++;
    }
    if (z0 >= kHalfRow && z1 >= kHalfRow) {
      Serial.printf("freezeband2 rows=%d-%d z=%d,%d\n", br, br + 1, z0, z1);
      return true;
    }
  }

  for (int br = 1; br < kRows - 1; ++br) {
    for (int bc = 1; bc < kCols - 1; ++bc) {
      if (!zmap[br][bc]) continue;
      int validNeighbors = 0;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (!(dr | dc)) continue;
          if (!zmap[br + dr][bc + dc]) validNeighbors++;
        }
      }
      if (validNeighbors >= 4) {
        Serial.printf("freezeblk @(%d,%d) vn=%d\n", bc * kBlock, br * kBlock, validNeighbors);
        return true;
      }
    }
  }

  return false;
}

// Convenience: scale current sprite content then present.
// In dirty-rect mode (s_dirtyRectDstW > 0), restores only the clock region
// from s_terrainDisplayBuf then partial-scales instead of full rescale.
static void presentSpriteToDisplay() {
  if (s_dirtyRectDstW > 0 && s_frameDisplayBuf && s_terrainDisplayBuf) {
    // Restore clean base in dirty region, then scale only the dirty sprite rect.
    for (int r = 0; r < s_dirtyRectDstH; r++) {
      int row = s_dirtyRectDstY + r;
      memcpy(s_frameDisplayBuf  + row * SCALED_W + s_dirtyRectDstX,
             s_terrainDisplayBuf + row * SCALED_W + s_dirtyRectDstX,
             (size_t)s_dirtyRectDstW * 2U);
    }
    scaleSubrectTo410x360(s_frameDisplayBuf,
                          s_dirtyRectSrcX, s_dirtyRectSrcY,
                          s_dirtyRectSrcW, s_dirtyRectSrcH);
  } else {
    scaleSpriteTo410x360(s_frameDisplayBuf);
  }
  presentScaledBuf(s_frameDisplayBuf);
}

// ─────────────────────────────────────────────────────────────
//  Time helpers
// ─────────────────────────────────────────────────────────────
static float normalizeLon180(float lon) {
  while (lon > 180.0f) lon -= 360.0f;
  while (lon < -180.0f) lon += 360.0f;
  return lon;
}

static bool activeLayerIs(const char* layer) {
  return (layer && strcmp(s_activeGibsLayer, layer) == 0);
}

static int activeCadenceMin() {
  if (s_activeCadenceMin >= 5 && s_activeCadenceMin <= 60) return s_activeCadenceMin;
  return CADENCE_MIN;
}

static int activeLagHours() {
  if (s_activeLagHours >= 0 && s_activeLagHours <= 8) return s_activeLagHours;
  return GIBS_LAG_HOURS;
}

static int targetFrameCount() {
  int cadenceMin = activeCadenceMin();
  if (cadenceMin <= 0) cadenceMin = CADENCE_MIN;

  int hoursBack = HOURS_BACK;
  if (hoursBack <= 0) hoursBack = 24;

  int frames = (hoursBack * 60) / cadenceMin;
  if (frames < 1) frames = 1;
  if (frames > MAX_FRAMES) frames = MAX_FRAMES;
  return frames;
}

static void setActiveSatelliteProfile(const char* layer,
                                      int cadenceMin,
                                      int lagHours,
                                      const char* sourceLabel) {
  if (!layer || !sourceLabel) return;
  strlcpy(s_activeGibsLayer, layer, sizeof(s_activeGibsLayer));
  strlcpy(s_activeWeatherSource, sourceLabel, sizeof(s_activeWeatherSource));
  s_activeCadenceMin = cadenceMin;
  s_activeLagHours = lagHours;
}

static void selectSatelliteForLon(float lonDeg, bool force) {
  // Boundary tests for validation:
  //  - GOES-West/GOES-East split at ~-110° (verify both crossing directions)
  //  - GOES-East/Himawari split at ~+60° (APAC IR fallback)
  static constexpr float kGoesSplitLon = -110.0f;
  static constexpr float kApacSplitLon = 60.0f;
  static constexpr float kHystDeg = 2.0f;

  float lon = normalizeLon180(lonDeg);
  bool keepHimawari =
    !force && activeLayerIs(WEATHER_LAYER_HIMAWARI_IR) && (lon >= (kApacSplitLon - kHystDeg));
  bool enterHimawari = (lon >= (kApacSplitLon + kHystDeg));
  if (keepHimawari || enterHimawari) {
    setActiveSatelliteProfile(WEATHER_LAYER_HIMAWARI_IR, 10, 3, "Himawari-IR");
    return;
  }

  bool keepWest =
    !force && activeLayerIs(WEATHER_LAYER_GOES_WEST) && (lon < (kGoesSplitLon + kHystDeg));
  bool enterWest = (lon < (kGoesSplitLon - kHystDeg));
  if (keepWest || enterWest) {
    setActiveSatelliteProfile(WEATHER_LAYER_GOES_WEST, 10, 2, "GOES-West");
  } else {
    setActiveSatelliteProfile(WEATHER_LAYER_GOES_EAST, 10, 2, "GOES-East");
  }
}

// Round t down to the active source cadence boundary
static time_t roundToCadence(time_t t) {
  if (t <= 0) return 0;
  time_t cadenceSec = (time_t)activeCadenceMin() * 60;
  return t - (t % cadenceSec);  // zeroes seconds too
}

// Build an ISO-8601 UTC string for a given time_t
static void toISO(time_t t, char* out, size_t len) {
  struct tm* ti = gmtime(&t);
  strftime(out, len, "%Y-%m-%dT%H:%M:%SZ", ti);
}

static bool buildWeatherFrameUrl(char* out, size_t outLen,
                                 time_t t,
                                 float bboxWest, float bboxSouth,
                                 float bboxEast, float bboxNorth,
                                 int reqW = DISP_W,
                                 int reqH = DISP_H) {
  if (!out || outLen == 0) return false;
  const char* layer = s_activeGibsLayer[0] ? s_activeGibsLayer : WEATHER_LAYER_GOES_EAST;
  char timeISO[32];
  toISO(t, timeISO, sizeof(timeISO));
  int n = snprintf(out, outLen,
    "%s&LAYERS=%s&BBOX=%.1f,%.1f,%.1f,%.1f&WIDTH=%d&HEIGHT=%d&FORMAT=image%%2Fjpeg&TIME=%s",
    GIBS_WMS_BASE, layer,
    (double)bboxWest, (double)bboxSouth,
    (double)bboxEast, (double)bboxNorth,
    reqW, reqH, timeISO);
  return (n > 0 && (size_t)n < outLen);
}

static size_t jpegEffectiveLength(const uint8_t* data, size_t len) {
  if (!data || len < 4) return 0;
  if (!(data[0] == 0xFF && data[1] == 0xD8)) return 0;
  for (size_t i = len - 2; i > 0; --i) {
    if (data[i] == 0xFF && data[i + 1] == 0xD9) {
      return i + 2;
    }
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────
//  GIBS fetch → write JPEG bytes directly to an SD file
//  Returns bytes written, or 0 on failure.
// ─────────────────────────────────────────────────────────────

// Download buffer — reused for every frame fetch and for showFrame playback
#define DL_BUF_BYTES  MAX_JPEG_BYTES
static uint8_t s_dlBuf[DL_BUF_BYTES];

class FixedBufferWriteStream : public Stream {
public:
  FixedBufferWriteStream(uint8_t* buf, size_t cap)
  : m_buf(buf), m_cap(cap), m_pos(0), m_overflow(false) {}

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}

  size_t write(uint8_t b) override {
    return write(&b, 1);
  }

  size_t write(const uint8_t* data, size_t len) override {
    if (!m_buf || m_pos >= m_cap) {
      m_overflow = true;
      setWriteError();
      return 0;
    }
    size_t room = m_cap - m_pos;
    size_t n = (len < room) ? len : room;
    if (n > 0) {
      memcpy(m_buf + m_pos, data, n);
      m_pos += n;
    }
    if (n != len) {
      m_overflow = true;
      setWriteError();
    }
    return n;
  }

  size_t size() const { return m_pos; }
  bool overflowed() const { return m_overflow; }

private:
  uint8_t* m_buf;
  size_t   m_cap;
  size_t   m_pos;
  bool     m_overflow;
};

static bool readHttpJpegBodyToDlBuf(HTTPClient& http,
                                    const char* label,
                                    size_t* outJpegLen,
                                    size_t* outRawLen = nullptr) {
  if (outJpegLen) *outJpegLen = 0;
  if (outRawLen)  *outRawLen = 0;

  FixedBufferWriteStream sink(s_dlBuf, DL_BUF_BYTES);
  int wrote = http.writeToStream(&sink);
  http.end();

  if (wrote <= 0) {
    Serial.printf("%s BODY-ERR(%d)\n", label, wrote);
    return false;
  }

  size_t rawLen = (size_t)wrote;
  if (sink.overflowed() || rawLen == 0 || rawLen > DL_BUF_BYTES) {
    Serial.printf("%s TOO BIG(%u)\n", label, (unsigned)rawLen);
    return false;
  }

  size_t jpegLen = jpegEffectiveLength(s_dlBuf, rawLen);
  if (jpegLen == 0) {
    Serial.printf("%s BAD(%u)\n", label, (unsigned)rawLen);
    return false;
  }

  if (outRawLen)  *outRawLen = rawLen;
  if (outJpegLen) *outJpegLen = jpegLen;
  return true;
}

static void computeWeatherBboxFromCenter(float lat, float lon,
                                         float* west, float* south,
                                         float* east, float* north) {
  // Regional view tuned for the 410x220 display. Keep a fixed vertical span
  // and expand longitude by aspect/cos(latitude) so the tile stays useful at
  // higher latitudes.
  const float kHalfLatDeg = 4.5f;  // ~1000 km tall view
  // Hardcoded original aspect ratio — must not change when DISP_H changes or
  // the bbox will drift, invalidate view.meta, and wipe radar.meta.
  const float kAspect = 320.0f / 172.0f;

  if (lat > 85.0f) lat = 85.0f;
  if (lat < -85.0f) lat = -85.0f;
  while (lon > 180.0f) lon -= 360.0f;
  while (lon < -180.0f) lon += 360.0f;

  float cosLat = cosf(lat * 0.01745329252f);
  if (cosLat < 0.25f) cosLat = 0.25f;
  float halfLonDeg = (kHalfLatDeg * kAspect) / cosLat;
  if (halfLonDeg > 60.0f) halfLonDeg = 60.0f;

  float w = lon - halfLonDeg;
  float e = lon + halfLonDeg;
  float s = lat - kHalfLatDeg;
  float n = lat + kHalfLatDeg;

  if (s < -89.5f) s = -89.5f;
  if (n >  89.5f) n =  89.5f;
  if (w < -180.0f) w = -180.0f;
  if (e >  180.0f) e =  180.0f;

  if (west)  *west  = w;
  if (south) *south = s;
  if (east)  *east  = e;
  if (north) *north = n;
}

static void getActiveWeatherBbox(float* west, float* south, float* east, float* north) {
  if (s_hurricaneMode) {
    computeStormBbox(s_activeStorm.lat, s_activeStorm.lon, s_activeStorm.category,
                     west, south, east, north);
    return;
  }
  if (s_weatherGeoValid) {
    computeWeatherBboxFromCenter(s_weatherCenterLat, s_weatherCenterLon, west, south, east, north);
    return;
  }
  if (west)  *west  = BBOX_WEST;
  if (south) *south = BBOX_SOUTH;
  if (east)  *east  = BBOX_EAST;
  if (north) *north = BBOX_NORTH;
}

static void formatCurrentWeatherViewSignature(char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  float w, s, e, n;
  getActiveWeatherBbox(&w, &s, &e, &n);
  snprintf(out, outLen,
           "v=%d|bbox=%.4f,%.4f,%.4f,%.4f|layer=%s|cad=%d|lag=%d\n",
           WEATHER_VIEW_VERSION,
           (double)w, (double)s, (double)e, (double)n,
           s_activeGibsLayer, activeCadenceMin(), activeLagHours());
}

static bool parseWeatherViewSignature(const char* sig,
                                      float* west,
                                      float* south,
                                      float* east,
                                      float* north,
                                      char* layer,
                                      size_t layerLen,
                                      int* cadence,
                                      int* lag) {
  if (!sig) return false;

  const char* p = sig;
  if (strncmp(p, "v=", 2) == 0) {
    char* verEnd = nullptr;
    long ver = strtol(p + 2, &verEnd, 10);
    if (verEnd == p + 2 || ver != WEATHER_VIEW_VERSION) return false;
    p = verEnd;
    if (strncmp(p, "|bbox=", 6) != 0) return false;
    p += 6;
  } else
  if (strncmp(p, "bbox=", 5) != 0) return false;
  else p += 5;

  char* end = nullptr;
  float vals[4] = {};
  for (int i = 0; i < 4; i++) {
    vals[i] = strtof(p, &end);
    if (end == p) return false;
    p = end;
    if (i < 3) {
      if (*p != ',') return false;
      p++;
    }
  }

  if (strncmp(p, "|layer=", 7) != 0) return false;
  p += 7;
  const char* layerEnd = strchr(p, '|');
  if (!layerEnd) return false;
  if (layer && layerLen > 0) {
    size_t copyLen = (size_t)(layerEnd - p);
    if (copyLen >= layerLen) copyLen = layerLen - 1;
    memcpy(layer, p, copyLen);
    layer[copyLen] = '\0';
  }
  p = layerEnd;

  if (strncmp(p, "|cad=", 5) != 0) return false;
  p += 5;
  long cad = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;

  if (strncmp(p, "|lag=", 5) != 0) return false;
  p += 5;
  long lagVal = strtol(p, &end, 10);
  if (end == p) return false;

  if (west)  *west  = vals[0];
  if (south) *south = vals[1];
  if (east)  *east  = vals[2];
  if (north) *north = vals[3];
  if (cadence) *cadence = (int)cad;
  if (lag) *lag = (int)lagVal;
  return true;
}

static void writeCurrentWeatherViewMeta();

static bool currentWeatherViewMatchesCache() {
  char expected[192];
  formatCurrentWeatherViewSignature(expected, sizeof(expected));

  File f = SD.open(WEATHER_VIEW_META_FILE, FILE_READ);
  if (!f) return false;
  char actual[192] = {};
  f.readBytes(actual, sizeof(actual) - 1);
  f.close();
  if (strcmp(actual, expected) == 0) return true;

  float curW, curS, curE, curN;
  getActiveWeatherBbox(&curW, &curS, &curE, &curN);
  float oldW, oldS, oldE, oldN;
  char oldLayer[64] = {};
  int oldCad = 0;
  int oldLag = 0;
  if (!parseWeatherViewSignature(actual,
                                 &oldW, &oldS, &oldE, &oldN,
                                 oldLayer, sizeof(oldLayer),
                                 &oldCad, &oldLag)) {
    return false;
  }

  constexpr float kBboxToleranceDeg = 0.50f;
  bool approxMatch =
    (strcmp(oldLayer, s_activeGibsLayer) == 0) &&
    (oldCad == activeCadenceMin()) &&
    (oldLag == activeLagHours()) &&
    (fabsf(oldW - curW) <= kBboxToleranceDeg) &&
    (fabsf(oldS - curS) <= kBboxToleranceDeg) &&
    (fabsf(oldE - curE) <= kBboxToleranceDeg) &&
    (fabsf(oldN - curN) <= kBboxToleranceDeg);

  if (approxMatch) {
    writeCurrentWeatherViewMeta();
    return true;
  }
  return false;
}

static void writeCurrentWeatherViewMeta() {
  char sig[192];
  formatCurrentWeatherViewSignature(sig, sizeof(sig));
  SD.remove(WEATHER_VIEW_META_FILE);  // FILE_WRITE may append
  File f = SD.open(WEATHER_VIEW_META_FILE, FILE_WRITE);
  if (!f) return;
  f.print(sig);
  f.close();
}

static bool loadWeatherViewCenterFromCache() {
  File f = SD.open(WEATHER_VIEW_META_FILE, FILE_READ);
  if (!f) return false;
  char actual[192] = {};
  f.readBytes(actual, sizeof(actual) - 1);
  f.close();

  float oldW, oldS, oldE, oldN;
  if (!parseWeatherViewSignature(actual,
                                 &oldW, &oldS, &oldE, &oldN,
                                 nullptr, 0,
                                 nullptr, nullptr)) {
    return false;
  }

  s_weatherCenterLat = 0.5f * (oldS + oldN);
  s_weatherCenterLon = 0.5f * (oldW + oldE);
  s_weatherGeoValid = true;
  selectSatelliteForLon(s_weatherCenterLon, true);
  return true;
}

static bool fileExistsNonEmpty(const char* path) {
  if (!path || !path[0]) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  bool ok = (f.size() > 0);
  f.close();
  return ok;
}

static bool zoomSnapshotsMatchNewestFrame(time_t newestUtc) {
  if (newestUtc <= 0) return false;
  if (!fileExistsNonEmpty(ZOOM1_FILE) ||
      !fileExistsNonEmpty(ZOOM2_FILE) ||
      !fileExistsNonEmpty(ZOOM3_FILE)) {
    return false;
  }

  File f = SD.open(ZOOM_SNAPSHOT_META_FILE, FILE_READ);
  if (!f) return false;
  char buf[32] = {};
  f.readBytes(buf, sizeof(buf) - 1);
  f.close();
  int ver = 0;
  long long savedUtc = 0;
  if (sscanf(buf, "%d %lld", &ver, &savedUtc) != 2) return false;
  if (ver != ZOOM_META_VERSION) return false;
  return (savedUtc == (long long)newestUtc);
}

static void writeZoomSnapshotMeta(time_t newestUtc) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%d %lld\n", ZOOM_META_VERSION, (long long)newestUtc);
  SD.remove(ZOOM_SNAPSHOT_META_FILE);  // FILE_WRITE may append
  File f = SD.open(ZOOM_SNAPSHOT_META_FILE, FILE_WRITE);
  if (!f) return;
  f.print(buf);
  f.close();
}

static const char* zoomRawPathForJpeg(const char* jpegPath) {
  if (!jpegPath) return nullptr;
  if (strcmp(jpegPath, ZOOM1_FILE) == 0) return ZOOM1_RAW_FILE;
  if (strcmp(jpegPath, ZOOM2_FILE) == 0) return ZOOM2_RAW_FILE;
  if (strcmp(jpegPath, ZOOM3_FILE) == 0) return ZOOM3_RAW_FILE;
  return nullptr;
}

static bool zoomRawFileLooksUsable(const char* rawPath) {
  if (!rawPath || !rawPath[0]) return false;
  File f = SD.open(rawPath, FILE_READ);
  if (!f) return false;
  bool ok = ((size_t)f.size() == RAW_FRAME_BYTES);
  f.close();
  return ok;
}

static void applyGentleLowPassOnSprite() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px || DISP_W < 3 || DISP_H < 3) return;

  static uint16_t rowPrev[DISP_W];
  static uint16_t rowCurr[DISP_W];
  static uint16_t rowNext[DISP_W];
  static uint16_t rowOut[DISP_W];

  auto loadRowCanonical = [&](int y, uint16_t* dst) {
    uint16_t* src = px + ((size_t)y * (size_t)DISP_W);
    for (int x = 0; x < DISP_W; x++) {
      uint16_t c = src[x];
      dst[x] = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(c) : c;
    }
  };
  auto storeRowCanonical = [&](int y, const uint16_t* src) {
    uint16_t* dst = px + ((size_t)y * (size_t)DISP_W);
    for (int x = 0; x < DISP_W; x++) {
      uint16_t c = src[x];
      dst[x] = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(c) : c;
    }
  };

  loadRowCanonical(0, rowPrev);
  loadRowCanonical(1, rowCurr);
  for (int y = 1; y < DISP_H - 1; y++) {
    loadRowCanonical(y + 1, rowNext);
    for (int x = 0; x < DISP_W; x++) {
      uint16_t p = rowPrev[x], c = rowCurr[x], n = rowNext[x];
      int pr = (p >> 11) & 0x1F, pg = (p >> 5) & 0x3F, pb = p & 0x1F;
      int cr = (c >> 11) & 0x1F, cg = (c >> 5) & 0x3F, cb = c & 0x1F;
      int nr = (n >> 11) & 0x1F, ng = (n >> 5) & 0x3F, nb = n & 0x1F;

      // Vertical anti-banding low-pass: target horizontal line artifacts.
      int fr = (pr + (cr << 1) + nr + 2) >> 2;
      int fg = (pg + (cg << 1) + ng + 2) >> 2;
      int fb = (pb + (cb << 1) + nb + 2) >> 2;
      int br = fr;
      int bg = fg;
      int bb = fb;

      if (br > 31) br = 31;
      if (bg > 63) bg = 63;
      if (bb > 31) bb = 31;
      rowOut[x] = (uint16_t)((br << 11) | (bg << 5) | bb);
    }

    storeRowCanonical(y, rowOut);
    memcpy(rowPrev, rowCurr, sizeof(rowPrev));
    memcpy(rowCurr, rowNext, sizeof(rowCurr));
  }
}

static bool buildFilteredZoomRawFromJpeg(const char* jpegPath, const char* rawPath) {
  if (!jpegPath || !rawPath) return false;
  if (!decodeJpegPathToSprite(jpegPath)) return false;
  if (spriteLooksCompletelyBlack()) return false;
  if (spriteLooksPartialDecode()) return false;
  if (spriteLooksHorizontallyCorrupted() || spriteLooksVerticallyCorrupted()) return false;

  applyGentleLowPassOnSprite();

  uint16_t* src = (uint16_t*)sprite.getBuffer();
  if (!src) return false;
  SD.remove(rawPath);
  File wf = SD.open(rawPath, FILE_WRITE);
  if (!wf) return false;
  size_t wrote = wf.write((const uint8_t*)src, RAW_FRAME_BYTES);
  wf.flush();
  wf.close();
  if (wrote != RAW_FRAME_BYTES) {
    SD.remove(rawPath);
    return false;
  }
  return true;
}

static bool ensureFilteredZoomRaw(const char* jpegPath, bool forceRebuild = false) {
  const char* rawPath = zoomRawPathForJpeg(jpegPath);
  if (!rawPath) return false;
  if (!forceRebuild && zoomRawFileLooksUsable(rawPath)) return true;
  SD.remove(rawPath);
  return buildFilteredZoomRawFromJpeg(jpegPath, rawPath);
}

static void rebuildFilteredZoomRawsFromCache() {
  ensureFilteredZoomRaw(ZOOM1_FILE, true);
  ensureFilteredZoomRaw(ZOOM2_FILE, true);
  ensureFilteredZoomRaw(ZOOM3_FILE, true);
}

static void clearRadarMeta() {
  s_lastRadarUtc = 0;
  s_lastRadarUtcValid = false;
  SD.remove(RADAR_META_FILE);
}

static void writeRadarMeta(time_t radarUtc) {
  if (radarUtc <= 0) {
    clearRadarMeta();
    return;
  }

  s_lastRadarUtc = radarUtc;
  s_lastRadarUtcValid = true;
  SD.remove(RADAR_META_FILE);  // FILE_WRITE may append
  File f = SD.open(RADAR_META_FILE, FILE_WRITE);
  if (!f) return;
  char buf[32];
  snprintf(buf, sizeof(buf), "%lld\n", (long long)radarUtc);
  f.print(buf);
  f.close();
}

static void loadRadarMetaIfNeeded() {
  if (s_radarMetaLoaded) return;
  s_radarMetaLoaded = true;

  File f = SD.open(RADAR_META_FILE, FILE_READ);
  if (!f) return;
  char buf[32] = {};
  f.readBytes(buf, sizeof(buf) - 1);
  f.close();
  long long t = atoll(buf);
  if (t > 0) {
    s_lastRadarUtc = (time_t)t;
    s_lastRadarUtcValid = true;
  }
}

static bool zoomSnapshotFileLooksUsable(const char* path) {
  // Fast path: just check file size — avoids a full JPEG decode on startup.
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t sz = f.size();
  f.close();
  return sz >= 2048U;  // minimum valid JPEG size
}

static bool zoomWeatherSnapshotsPresentAndUsable() {
  if (!zoomSnapshotFileLooksUsable(ZOOM1_FILE)) return false;
  if (!zoomSnapshotFileLooksUsable(ZOOM2_FILE)) return false;
  if (!zoomSnapshotFileLooksUsable(ZOOM3_FILE)) return false;
  return true;
}

static bool zoomSnapshotsCurrentAndUsable(time_t newestUtc) {
  if (!zoomSnapshotsMatchNewestFrame(newestUtc)) return false;
  if (!zoomWeatherSnapshotsPresentAndUsable()) return false;
  // Both terrain textures must be present so live day/night switching works.
  if (!zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_DAY_FILE)) return false;
  if (!zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_NIGHT_FILE)) return false;
  return true;
}

// ─────────────────────────────────────────────────────────────
//  SD helpers
// ─────────────────────────────────────────────────────────────
static void appendDiagLog(const char* fmt, ...) {
  File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
  if (!diagF) return;
  char line[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  diagF.print(line);
  diagF.close();
}

// ─────────────────────────────────────────────────────────────
//  Pre-allocated frame store helpers
// ─────────────────────────────────────────────────────────────

static void initFrameStore() {
  SD.mkdir(FRAMES_DIR);

  // Delete legacy per-frame files if any exist
  bool hasLegacy = false;
  {
    File lf = SD.open(META_FILE, FILE_READ);
    if (lf) { lf.close(); hasLegacy = true; }
  }
  {
    File lf = SD.open(TIMES_FILE, FILE_READ);
    if (lf) { lf.close(); hasLegacy = true; }
  }
  if (hasLegacy) {
    appendDiagLog("initFrameStore: legacy files detected, cleaning up\n");
    // Remove old metadata files
    SD.remove(META_FILE);
    SD.remove(TIMES_FILE);
    SD.remove(RAW_CACHE_META_FILE);
    SD.remove(CACHE_VALIDATE_META_FILE);
    SD.remove(FRAME_DIM_FILE);
    // Remove old per-frame .jpg files
    File dir = SD.open(FRAMES_DIR);
    if (dir) {
      char names[MAX_FRAMES + 16][20];
      int nameCount = 0;
      while (nameCount < (int)(sizeof(names)/sizeof(names[0]))) {
        File entry = dir.openNextFile();
        if (!entry) break;
        const char* n = entry.name();
        size_t nlen = strlen(n);
        entry.close();
        if (nlen < 5 || nlen >= sizeof(names[0])) continue;
        if (strcmp(n + nlen - 4, ".jpg") != 0) continue;
        // Skip zoom files
        if (strncmp(n, "vz", 2) == 0) continue;
        if (strncmp(n, "terrain", 7) == 0) continue;
        strncpy(names[nameCount], n, sizeof(names[0]) - 1);
        names[nameCount][sizeof(names[0]) - 1] = '\0';
        nameCount++;
      }
      dir.close();
      for (int k = 0; k < nameCount; k++) {
        char path[40];
        snprintf(path, sizeof(path), "%s/%s", FRAMES_DIR, names[k]);
        SD.remove(path);
      }
    }
    // Also remove old stream.raw (wrong layout for new system)
    SD.remove(RAW_STREAM_FILE);
  }

  // Pre-allocate frames.bin (144 × 64KB = 9MB)
  const size_t framesBinSize = (size_t)MAX_FRAMES * JPEG_SLOT_BYTES;
  {
    File f = SD.open(FRAMES_BIN_FILE, FILE_READ);
    bool needCreate = !f;
    if (f) {
      if (f.size() != framesBinSize) needCreate = true;
      f.close();
    }
    if (needCreate) {
      showMessage("Allocating frames.bin...", "First boot setup");
      SD.remove(FRAMES_BIN_FILE);
      File wf = SD.open(FRAMES_BIN_FILE, FILE_WRITE);
      if (wf) {
        memset(s_dlBuf, 0, DL_BUF_BYTES);
        size_t written = 0;
        while (written < framesBinSize) {
          size_t chunk = framesBinSize - written;
          if (chunk > DL_BUF_BYTES) chunk = DL_BUF_BYTES;
          wf.write(s_dlBuf, chunk);
          written += chunk;
          int pct = (int)((written * 100ULL) / framesBinSize);
          showProgress(pct, 100, "frames.bin");
          yield();
        }
        wf.flush();
        wf.close();
        appendDiagLog("initFrameStore: frames.bin created %u bytes\n", (unsigned)framesBinSize);
      }
    }
  }

  // Pre-allocate stream.raw (144 × 295200 = ~42.5MB)
  const size_t streamRawSize = (size_t)MAX_FRAMES * SCALED_FRAME_BYTES;
  {
    File f = SD.open(RAW_STREAM_FILE, FILE_READ);
    bool needCreate = !f;
    if (f) {
      if (f.size() != streamRawSize) needCreate = true;
      f.close();
    }
    if (needCreate) {
      showMessage("Allocating stream.raw...", "First boot setup");
      SD.remove(RAW_STREAM_FILE);
      File wf = SD.open(RAW_STREAM_FILE, FILE_WRITE);
      if (wf) {
        memset(s_dlBuf, 0, DL_BUF_BYTES);
        size_t written = 0;
        while (written < streamRawSize) {
          size_t chunk = streamRawSize - written;
          if (chunk > DL_BUF_BYTES) chunk = DL_BUF_BYTES;
          wf.write(s_dlBuf, chunk);
          written += chunk;
          int pct = (int)((written * 100ULL) / streamRawSize);
          showProgress(pct, 100, "stream.raw");
          yield();
        }
        wf.flush();
        wf.close();
        appendDiagLog("initFrameStore: stream.raw created %u bytes\n", (unsigned)streamRawSize);
      }
    }
  }
}

static bool loadIndex() {
  memset(&s_idx, 0, sizeof(s_idx));
  memset(s_streamValid, 0, sizeof(s_streamValid));

  File f = SD.open(INDEX_BIN_FILE, FILE_READ);
  if (!f) {
    appendDiagLog("loadIndex: no index.bin\n");
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }
  if (f.size() != sizeof(FrameStoreIndex)) {
    f.close();
    appendDiagLog("loadIndex: bad size %u\n", (unsigned)f.size());
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }
  f.read((uint8_t*)&s_idx, sizeof(s_idx));
  f.close();

  if (s_idx.magic != INDEX_MAGIC) {
    appendDiagLog("loadIndex: bad magic 0x%08X\n", s_idx.magic);
    memset(&s_idx, 0, sizeof(s_idx));
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }

  // SOI sanity check: verify jpegValid slots have valid JPEG headers
  File fb = SD.open(FRAMES_BIN_FILE, FILE_READ);
  if (fb) {
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (!s_idx.jpegValid[i]) continue;
      int phys = ((int)s_idx.head + i) % MAX_FRAMES;
      uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;
      uint8_t hdr[2] = {0, 0};
      fb.seek(off);
      fb.read(hdr, 2);
      if (hdr[0] != 0xFF || hdr[1] != 0xD8) {
        appendDiagLog("loadIndex: SOI fail slot %d phys %d\n", i, phys);
        s_idx.jpegValid[i] = 0;
        s_idx.rawValid[i] = 0;
      }
    }
    fb.close();
  }

  // Populate compatibility arrays
  int validCount = 0;
  for (int i = 0; i < (int)s_idx.count && i < MAX_FRAMES; i++) {
    s_frameTimes[i] = s_idx.times[i];
    s_streamValid[i] = s_idx.rawValid[i];
    if (s_idx.rawValid[i]) validCount++;
  }
  for (int i = (int)s_idx.count; i < MAX_FRAMES; i++) {
    s_frameTimes[i] = 0;
    s_streamValid[i] = 0;
  }

  frameCount = (int)s_idx.count;
  framesReady = (frameCount > 0 && validCount > 0);
  s_timesLoaded = true;
  invalidateValidIdxCache();

  appendDiagLog("loadIndex: count=%d head=%d valid=%d\n",
                frameCount, (int)s_idx.head, validCount);
  return true;
}

static void writeIndex() {
  s_idx.magic = INDEX_MAGIC;

  // Write to tmp, flush, rename for atomic commit
  SD.remove(INDEX_TMP_FILE);
  File f = SD.open(INDEX_TMP_FILE, FILE_WRITE);
  if (!f) {
    appendDiagLog("writeIndex: open tmp fail\n");
    return;
  }
  f.write((const uint8_t*)&s_idx, sizeof(s_idx));
  f.flush();
  f.close();

  SD.remove(INDEX_BIN_FILE);
  if (!SD.rename(INDEX_TMP_FILE, INDEX_BIN_FILE)) {
    appendDiagLog("writeIndex: rename fail\n");
  }

  // Sync compatibility arrays
  for (int i = 0; i < (int)s_idx.count && i < MAX_FRAMES; i++) {
    s_frameTimes[i] = s_idx.times[i];
    s_streamValid[i] = s_idx.rawValid[i];
  }
  for (int i = (int)s_idx.count; i < MAX_FRAMES; i++) {
    s_frameTimes[i] = 0;
    s_streamValid[i] = 0;
  }
  frameCount = (int)s_idx.count;
  framesReady = (frameCount > 0);
  s_timesLoaded = true;
  invalidateValidIdxCache();
}

static bool readJpegFromSlot(int logicalIdx, uint8_t* buf, size_t* outLen) {
  if (logicalIdx < 0 || logicalIdx >= (int)s_idx.count) return false;
  if (!s_idx.jpegValid[logicalIdx]) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;
  size_t len = s_idx.jpegLen[logicalIdx];
  if (len == 0 || len > JPEG_SLOT_BYTES) return false;

  File f = SD.open(FRAMES_BIN_FILE, FILE_READ);
  if (!f) return false;
  f.seek(off);
  size_t got = f.read(buf, len);
  f.close();
  if (got != len) return false;
  if (outLen) *outLen = len;
  return true;
}

static bool writeJpegToSlot(int logicalIdx, const uint8_t* buf, size_t len) {
  if (logicalIdx < 0 || logicalIdx >= MAX_FRAMES) return false;
  if (len == 0 || len > JPEG_SLOT_BYTES) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;

  File f = SD.open(FRAMES_BIN_FILE, "r+");
  if (!f) return false;
  f.seek(off);
  size_t written = f.write(buf, len);
  f.flush();
  f.close();
  if (written != len) return false;

  s_idx.jpegLen[logicalIdx] = (uint32_t)len;
  s_idx.jpegValid[logicalIdx] = 1;
  return true;
}

static bool writeRawToSlot(int logicalIdx, const uint8_t* buf) {
  if (logicalIdx < 0 || logicalIdx >= MAX_FRAMES) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * (uint32_t)SCALED_FRAME_BYTES;

  File f = SD.open(RAW_STREAM_FILE, "r+");
  if (!f) return false;
  f.seek(off);
  size_t written = f.write(buf, SCALED_FRAME_BYTES);
  f.flush();
  f.close();
  if (written != SCALED_FRAME_BYTES) return false;

  s_idx.rawValid[logicalIdx] = 1;
  s_streamValid[logicalIdx] = 1;
  return true;
}

// Decode JPEG from s_dlBuf, validate all sprite checks, scale, write raw slot.
// Returns true if raw slot was successfully written.
static bool decodeAndWriteRawSlot(int logicalIdx, size_t jpegLen) {
  if (!ensureSprite() || !s_frameDisplayBuf) return false;

  LovyanGFX* prevTarget = g_drawTarget;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)jpegLen, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  g_drawTarget = prevTarget;

  if (!ok || !jpegDrawLooksFullFrame()) return false;
  if (spriteLooksCompletelyBlack()) return false;
  if (spriteLooksPartialDecode()) return false;
  if (spriteLooksHorizontallyCorrupted() || spriteLooksVerticallyCorrupted()) return false;
  if (spriteLooksHoldFrameBlockCorrupted()) return false;
  if (spriteLooksCyanWhiteBlockCorrupted()) return false;
  if (spriteLooksBottomBandJunkCorrupted()) return false;
  // slab detector disabled — GOES-West disk edge triggers false positives for limb regions

  scaleSpriteTo410x360(s_frameDisplayBuf);
  if (scaledFrameLooksFreezeBlockCorrupted() || scaledFrameLooksHoldBlockCorrupted()) return false;

  return writeRawToSlot(logicalIdx, (const uint8_t*)s_frameDisplayBuf);
}

static bool copyFrameFile(const char* srcPath, const char* dstPath) {
  File src = SD.open(srcPath, FILE_READ);
  if (!src) return false;

  SD.remove(dstPath);  // avoid accidental append if file exists
  File dst = SD.open(dstPath, FILE_WRITE);
  if (!dst) {
    src.close();
    return false;
  }

  bool ok = true;
  while (src.available()) {
    size_t n = src.read(s_dlBuf, sizeof(s_dlBuf));
    if (n == 0) {
      ok = false;
      break;
    }
    if (dst.write(s_dlBuf, n) != n) {
      ok = false;
      break;
    }
  }

  src.close();
  dst.flush();
  dst.close();

  if (!ok) SD.remove(dstPath);
  return ok;
}

static void removeObsoleteGifAssetsIfPresent() {
  bool removed = false;
  removed = SD.remove(SD_ROOT "/radar_gif_70/meta.txt") || removed;
  removed = SD.remove(SD_ROOT "/radar_gif_70/frames_le_rgb565.bin") || removed;
  removed = SD.remove(SD_ROOT "/radar_gif_70/delays_ms_u16.bin") || removed;
  if (removed) {
    SD.rmdir(SD_ROOT "/radar_gif_70");
    Serial.println("removed obsolete radar_gif_70 assets");
  }
}


// Simplified path-based JPEG installer for zoom/terrain downloads.
// Writes s_dlBuf to disk via tmp+rename with SOI/EOI verify.
static bool installValidatedWeatherJpegToPath(const char* finalPath,
                                              size_t jpegLen,
                                              const char* label = nullptr,
                                              bool skipStoredValidate = false) {
  (void)skipStoredValidate;
  if (!finalPath || jpegLen == 0 || jpegLen > DL_BUF_BYTES) return false;

  char tmpPath[64];
  snprintf(tmpPath, sizeof(tmpPath), "%s.part", finalPath);
  SD.remove(tmpPath);

  File f = SD.open(tmpPath, FILE_WRITE);
  size_t written = 0;
  if (f) {
    written = f.write(s_dlBuf, jpegLen);
    f.flush();
    f.close();
  }
  if (written != jpegLen) {
    SD.remove(tmpPath);
    if (label) appendDiagLog("inst: %s SD-ERR wr=%u want=%u\n", label, (unsigned)written, (unsigned)jpegLen);
    return false;
  }

  // Quick SOI/EOI verify on what was written
  File vf = SD.open(tmpPath, FILE_READ);
  if (!vf || (size_t)vf.size() != jpegLen) {
    if (vf) vf.close();
    SD.remove(tmpPath);
    return false;
  }
  uint8_t hdr[2]; vf.read(hdr, 2);
  vf.seek(jpegLen - 2); uint8_t trl[2]; vf.read(trl, 2);
  vf.close();
  if (hdr[0] != 0xFF || hdr[1] != 0xD8 || trl[0] != 0xFF || trl[1] != 0xD9) {
    SD.remove(tmpPath);
    return false;
  }

  SD.remove(finalPath);
  bool moved = SD.rename(tmpPath, finalPath);
  if (!moved) {
    moved = copyFrameFile(tmpPath, finalPath);
    if (moved) SD.remove(tmpPath);
  }
  if (!moved) {
    SD.remove(tmpPath);
    return false;
  }
  return true;
}

// Forward declarations for GIBS available times (defined later in file)
#ifndef MAX_GIBS_AVAIL
#define MAX_GIBS_AVAIL 160
#endif
static time_t s_gibsAvailTimes[MAX_GIBS_AVAIL];
static int s_gibsAvailCount = 0;

// Download a GIBS frame to an SD file path — used by zoom/terrain pipeline.
static bool downloadFrameToPathAtBbox(HTTPClient& http,
                                      WiFiClientSecure& client,
                                      time_t t,
                                      const char* sdPath,
                                      float bboxWest, float bboxSouth,
                                      float bboxEast, float bboxNorth,
                                      int reqW = DISP_W,
                                      int reqH = DISP_H,
                                      size_t* outBytes = nullptr,
                                      bool validateWeatherFrame = false) {
  char timeISO[32];
  char url[512];
  size_t jpegLen = 0;

  const int cadenceSec = max(60, activeCadenceMin() * 60);
  time_t snappedTime = snapToNearestGibsTime(t, 2 * cadenceSec);

  const time_t secondOffsets[] = {0, -60, 60,
                                  -(time_t)cadenceSec,   (time_t)cadenceSec,
                                  -2*(time_t)cadenceSec, 2*(time_t)cadenceSec};
  int stepCount;
  if (snappedTime > 0) {
    stepCount = 1;
  } else if (s_gibsAvailCount > 0) {
    return false;
  } else {
    stepCount = validateWeatherFrame ? 7 : 1;
  }

  for (int si = 0; si < stepCount; ++si) {
    time_t candT = (snappedTime > 0) ? snappedTime : (t + secondOffsets[si]);
    if (candT <= 0) continue;
    toISO(candT, timeISO, sizeof(timeISO));

    if (!buildWeatherFrameUrl(url, sizeof(url), candT,
                              bboxWest, bboxSouth, bboxEast, bboxNorth,
                              reqW, reqH)) {
      continue;
    }

    const int maxAttempts = 2;
    bool fetchedOk = false;
    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
      http.begin(client, url);
      http.setTimeout(5000);
      int code = http.GET();
      if (code != HTTP_CODE_OK) {
        http.end();
        continue;
      }
      if (!readHttpJpegBodyToDlBuf(http, "MISS", &jpegLen)) {
        continue;
      }
      if (validateWeatherFrame && !validateBufferedWeatherFrameJpeg(jpegLen, "MISS")) {
        continue;
      }
      if (jpegEffectiveLength(s_dlBuf, jpegLen) == 0) {
        appendDiagLog("BUG: decoder-corrupt s_dlBuf jpegLen=%u\n", (unsigned)jpegLen);
        continue;
      }
      fetchedOk = true;
      break;
    }
    if (!fetchedOk) continue;

    char verifyLabel[40];
    snprintf(verifyLabel, sizeof(verifyLabel), "MISS %s", timeISO);
    bool instOk = false;
    for (int ir = 0; ir < 3 && !instOk; ir++) {
      if (ir > 0) delay(10);
      instOk = installValidatedWeatherJpegToPath(sdPath, jpegLen, verifyLabel, validateWeatherFrame);
    }
    if (!instOk) continue;

    if (outBytes) *outBytes = jpegLen;
    return true;
  }
  return false;
}

static bool connectWifiForSync(bool required, const char* statusLine = "Connecting WiFi...") {
  (void)required;
  loadWifiPortalConfig();
  const char* title = statusLine ? statusLine : "Connecting WiFi...";
  s_wifiSyncInProgress = true;

  if (s_wifiPortalDnsRunning) {
    s_wifiPortalDns.stop();
    s_wifiPortalDnsRunning = false;
  }
  if (s_wifiPortalHttpRunning) {
    s_wifiPortalServer.stop();
    s_wifiPortalHttpRunning = false;
  }
  if (s_wifiPortalApActive) {
    WiFi.softAPdisconnect(true);
    s_wifiPortalApActive = false;
  }
  stopWifiPortalMdns();

  for (int slot = 0; slot < WIFI_CONFIG_SLOTS; ++slot) {
    if (s_wifiConfig[slot].ssid[0] == '\0') continue;

    showMessage(title, s_wifiConfig[slot].ssid);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_PORTAL_HOSTNAME);
    esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
      WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
    WiFi.begin(s_wifiConfig[slot].ssid, s_wifiConfig[slot].pass);

    int wtries = 0;
    while (WiFi.status() != WL_CONNECTED && wtries++ < 20) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("\nWiFi fail: %s\n", s_wifiConfig[slot].ssid);
      appendDiagLog("wifi: fail ssid=%s slot=%d millis=%lu ms\n", s_wifiConfig[slot].ssid, slot, millis());
      continue;
    }
    if (!wifiHasInternetConnectivity()) {
      Serial.printf("WiFi no internet: %s\n", s_wifiConfig[slot].ssid);
      appendDiagLog("wifi: no-internet ssid=%s slot=%d millis=%lu ms\n", s_wifiConfig[slot].ssid, slot, millis());
      WiFi.disconnect(true);
      continue;
    }

    Serial.printf("\nWiFi: %s (%s)\n",
                  WiFi.localIP().toString().c_str(),
                  s_wifiConfig[slot].ssid);
    appendDiagLog("wifi: ok ssid=%s ip=%s rssi=%d millis=%lu ms\n",
                  s_wifiConfig[slot].ssid,
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(), millis());
    refreshCachedWifiDisplayState();
    s_wifiSyncInProgress = false;
    return true;
  }

  Serial.println("\nWiFi failed");
  appendDiagLog("wifi: all-slots-failed millis=%lu ms\n", millis());
  s_wifiSyncInProgress = false;
  return false;
}

static bool syncClockFromNtpBestEffort(int maxTries = 10) {
  if (WiFi.status() != WL_CONNECTED) return false;
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm ti;
  int tries = 0;
  while (!getLocalTime(&ti, 500) && tries++ < maxTries) {
    delay(100);
  }
  time_t nowUtc = time(nullptr);
  bool ok = (nowUtc > 1700000000);
  if (!ok) {
    Serial.println("ntp best-effort sync pending");
    appendDiagLog("ntp: fail t=%lld millis=%lu ms\n", (long long)nowUtc, millis());
  } else {
    appendDiagLog("ntp: ok utc=%lld millis=%lu ms\n", (long long)nowUtc, millis());
    writePcf85063(nowUtc);
  }
  return ok;
}

// ── PCF85063A hardware RTC (I2C 0x51) ────────────────────────────────────
static uint8_t pcfBcdEnc(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static int pcfBcdDec(uint8_t b) { return ((b >> 4) & 0x0F) * 10 + (b & 0x0F); }

static bool writePcf85063(time_t t) {
  struct tm* gmt = gmtime(&t);
  if (!gmt) return false;
  // Halt oscillator during write
  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x20);  // STOP bit
  if (Wire.endTransmission() != 0) return false;
  // Write seconds through years (registers 0x02-0x08)
  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x02);
  Wire.write(pcfBcdEnc(gmt->tm_sec));
  Wire.write(pcfBcdEnc(gmt->tm_min));
  Wire.write(pcfBcdEnc(gmt->tm_hour));
  Wire.write(pcfBcdEnc(gmt->tm_mday));
  Wire.write((uint8_t)gmt->tm_wday);
  Wire.write(pcfBcdEnc(gmt->tm_mon + 1));
  Wire.write(pcfBcdEnc((gmt->tm_year + 1900 - 2000) % 100));
  if (Wire.endTransmission() != 0) return false;
  // Resume oscillator
  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  return Wire.endTransmission() == 0;
}

static bool readPcf85063(time_t* out) {
  if (!out) return false;
  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x51, (uint8_t)7) < 7) return false;
  uint8_t secRaw = Wire.read();
  uint8_t minRaw = Wire.read();
  uint8_t hrRaw  = Wire.read();
  uint8_t dayRaw = Wire.read();
  Wire.read();  // weekday — unused
  uint8_t monRaw = Wire.read();
  uint8_t yrRaw  = Wire.read();
  if (secRaw & 0x80) return false;  // OS flag: oscillator stopped, time invalid
  int sec = pcfBcdDec(secRaw & 0x7F);
  int min = pcfBcdDec(minRaw & 0x7F);
  int hr  = pcfBcdDec(hrRaw  & 0x3F);
  int day = pcfBcdDec(dayRaw & 0x3F);
  int mon = pcfBcdDec(monRaw & 0x1F);
  int yr  = pcfBcdDec(yrRaw) + 2000;
  if (yr < 2024 || mon < 1 || mon > 12 || day < 1 || day > 31) return false;
  // Compute UTC epoch without relying on mktime() timezone
  static const int kDpM[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int y = yr - 1970;
  int leaps = (y + 1) / 4 - (y + 69) / 100 + (y + 369) / 400;
  int yd = kDpM[mon - 1] + day - 1;
  if (mon > 2 && (yr % 4 == 0) && (yr % 100 != 0 || yr % 400 == 0)) yd++;
  time_t t = ((time_t)y * 365 + leaps + yd) * 86400
             + hr * 3600 + min * 60 + sec;
  if (t < 1700000000LL) return false;  // sanity: must be after Nov 2023
  *out = t;
  return true;
}

static bool tryApplyPcf85063Time() {
  time_t t = 0;
  if (!readPcf85063(&t)) return false;
  struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  appendDiagLog("rtc: pcf85063 t=%lld\n", (long long)t);
  return true;
}

// Persist last-known UTC offset and location label to NVS so they survive power-off
static void saveUtcOffsetToNvs(int32_t offsetSec) {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  prefs.putInt("utcoff", (int)offsetSec);
  prefs.putBool("utcvalid", true);
  prefs.end();
}

static int32_t loadUtcOffsetFromNvs() {
  Preferences prefs;
  if (!prefs.begin("satwatch", true)) return 0;
  int32_t v = (int32_t)prefs.getInt("utcoff", 0);
  prefs.end();
  return v;
}

static bool loadUtcOffsetValidFromNvs() {
  Preferences prefs;
  if (!prefs.begin("satwatch", true)) return false;
  bool v = prefs.getBool("utcvalid", false);
  prefs.end();
  return v;
}

static void saveLocationLabelToNvs(const char* label, const char* full) {
  Preferences prefs;
  if (!prefs.begin("satwatch", false)) return;
  if (label) prefs.putString("loclabel", label);
  if (full)  prefs.putString("locfull", full);
  prefs.end();
}

static void loadLocationLabelFromNvs(char* label, size_t labelLen, char* full, size_t fullLen) {
  Preferences prefs;
  if (!prefs.begin("satwatch", true)) return;
  if (label && labelLen > 0) {
    String s = prefs.getString("loclabel", "");
    if (s.length() > 0) strlcpy(label, s.c_str(), labelLen);
  }
  if (full && fullLen > 0) {
    String s = prefs.getString("locfull", "");
    if (s.length() > 0) strlcpy(full, s.c_str(), fullLen);
  }
  prefs.end();
}

// Poll for a "skip" tap: touch INT pin (TP_INT = GPIO 38, active-low) or AXP2101 PKEY.
// Used in the dismissible portal loop only — does NOT call serviceUserButtons().
static bool pollPortalSkip() {
  // Touch IC INT pin (GPIO 38) goes low when a touch is detected.
  // This works even when the IC is in auto-sleep — touch wakes it and asserts INT.
  if (digitalRead(38) == LOW) return true;
  // AXP2101 PKEY short-press interrupt (reg 0x49 bit 2) — intercept before restart
  uint8_t intSts = readAxp2101Register(0x49);
  if (intSts & 0x04) {
    writeAxp2101Register(0x49, intSts);  // clear
    return true;
  }
  return false;
}

static bool topButtonPressed() {
  return digitalRead(BOOT_BTN_GPIO) == LOW;
}

static void syncTopButtonStateNow() {
  uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
  bool physicalPressed = topButtonPressed();
  portENTER_CRITICAL(&s_topBtnStateMux);
  s_topBtnRawPressed = physicalPressed;
  s_topBtnStablePressed = physicalPressed;
  s_topBtnReleasePending = false;
  s_topBtnRawChangedMs = nowMs;
  s_topBtnPressStartMs = physicalPressed ? nowMs : 0U;
  s_topBtnReleaseMs = 0U;
  portEXIT_CRITICAL(&s_topBtnStateMux);
}

static void resetTopButtonStateAfterWake(bool ignoreUntilRelease) {
  uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
  bool physicalPressed = topButtonPressed();
  portENTER_CRITICAL(&s_topBtnStateMux);
  s_topBtnRawPressed = physicalPressed;
  s_topBtnStablePressed = physicalPressed;
  s_topBtnReleasePending = false;
  s_topBtnRawChangedMs = nowMs;
  s_topBtnPressStartMs = 0U;
  s_topBtnReleaseMs = 0U;
  portEXIT_CRITICAL(&s_topBtnStateMux);
  s_topBtnIgnoreUntilRelease = ignoreUntilRelease && physicalPressed;
}

static void topButtonPollTask(void*) {
  while (true) {
    bool physicalPressed = (gpio_get_level((gpio_num_t)BOOT_BTN_GPIO) == 0);
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);

    portENTER_CRITICAL(&s_topBtnStateMux);
    if (physicalPressed != s_topBtnRawPressed) {
      s_topBtnRawPressed = physicalPressed;
      s_topBtnRawChangedMs = nowMs;
    }

    if (physicalPressed != s_topBtnStablePressed &&
        (uint32_t)(nowMs - s_topBtnRawChangedMs) >= TOP_BTN_DEBOUNCE_MS) {
      s_topBtnStablePressed = physicalPressed;
      if (physicalPressed) {
        s_topBtnPressStartMs = nowMs;
      } else {
        s_topBtnReleaseMs = nowMs;
        s_topBtnReleasePending = true;
      }
    }
    portEXIT_CRITICAL(&s_topBtnStateMux);

    vTaskDelay(pdMS_TO_TICKS(TOP_BTN_POLL_MS));
  }
}

static void disconnectWifiAfterSync() {
  if (WiFi.status() == WL_CONNECTED) {
    startWifiPortalServer(false);
    return;
  }
  if (s_wifiPortalDnsRunning) {
    s_wifiPortalDns.stop();
    s_wifiPortalDnsRunning = false;
  }
  s_wifiPortalApActive = false;
  s_wifiPortalHttpRunning = false;
  stopWifiPortalMdns();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

static void refreshZoomSnapshotsForLatestFrame();
static void maybeRefreshPendingZoomSnapshots() {
  if (!s_zoomSnapshotsRefreshPending) return;
  if (frameCount <= 0) return;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("zoom refresh deferred (wifi offline)");
    return;
  }
  s_zoomSnapshotsRefreshPending = false;
  refreshZoomSnapshotsForLatestFrame();
}

static void drawTimestamp(int frameIdx, LovyanGFX* target);
static void logSourceBlackFrame(int idx, const char* phase);
static bool spriteLooksPartialDecode();
static void ensureStreamOpen();
static void closeStream();
static void getActiveWeatherBbox(float* west, float* south, float* east, float* north);
static bool currentWeatherViewMatchesCache();
static void writeCurrentWeatherViewMeta();

static void invalidateValidIdxCache() {
  s_validCount = -1;
}


static void logSourceBlackFrame(int idx, const char* phase) {
  (void)phase;
  if (idx < 0 || idx >= MAX_FRAMES) return;
  s_sourceBlackLogged[idx] = 1;
}

static bool decodeJpegFrameToSprite(int idx, bool rejectBlank) {
  if (idx < 0 || idx >= (int)s_idx.count) return false;
  if (!s_idx.jpegValid[idx]) return false;
  size_t readLen = 0;
  if (!readJpegFromSlot(idx, s_dlBuf, &readLen) || readLen == 0) return false;

  if (!ensureSprite()) return false;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)readLen, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  if (!ok || !jpegDrawLooksFullFrame()) return false;
  if (rejectBlank && spriteLooksCompletelyBlack()) return false;
  return true;
}

// Individual rNNN.raw read/write functions removed — replaced by stream.raw


// ─────────────────────────────────────────────────────────────
//  Display one frame from SD
// ─────────────────────────────────────────────────────────────

// Fast path: read frame from stream file directly into sprite and push to LCD.
// Runtime guards still reject black/partial/corrupted reads before display.
// Returns false if frame is invalid/missing (caller should hold previous frame).
static bool showFrame(int idx) {
  if (!s_streamReady || !s_streamFile || idx >= frameCount || !s_streamValid[idx]) return false;
  if (!s_frameDisplayBuf) return false;

  int phys = ((int)s_idx.head + idx) % MAX_FRAMES;
  uint32_t offset = (uint32_t)phys * (uint32_t)SCALED_FRAME_BYTES;
  bool seekOk = s_streamFile.seek(offset);
  size_t got = s_streamFile.read((uint8_t*)s_frameDisplayBuf, SCALED_FRAME_BYTES);
  if (!seekOk || got != SCALED_FRAME_BYTES) {
    invalidateStreamSlot(idx, "short-read");
    appendDiagLog("showFrame: read-fail idx=%d got=%u seek=%d\n",
                  idx, (unsigned)got, (int)seekOk);
    return false;
  }
  updateBarBufs(idx);
  presentScaledBuf(s_frameDisplayBuf);
  return true;
}

// Reject tiny WMS GeoColor JPEGs (placeholder/malformed composites) before SD install.
// 614-byte black placeholders and ~4.7KB yellow-wash outliers both fall below this floor.
static constexpr size_t MIN_WEATHER_JPEG_BYTES = 7000;

static bool validateBufferedWeatherFrameJpeg(size_t jpegLen, const char* label) {
  if (jpegLen == 0 || jpegLen > DL_BUF_BYTES) return false;
  if (jpegLen < MIN_WEATHER_JPEG_BYTES) {
    if (label) Serial.printf("%s TOO-SMALL %u\n", label, (unsigned)jpegLen);
    return false;
  }
  if (!ensureSprite()) return false;

  LovyanGFX* prevTarget = g_drawTarget;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)jpegLen, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  g_drawTarget = prevTarget;

  if (!ok || !jpegDrawLooksFullFrame()) {
    if (label) { Serial.printf("%s DEC-VERIFY\n", label); appendDiagLog("vld: %s DEC-VERIFY\n", label); }
    return false;
  }
  if (spriteLooksCompletelyBlack()) {
    if (label) { Serial.printf("%s SRC-BLACK\n", label); appendDiagLog("vld: %s SRC-BLACK\n", label); }
    return false;
  }
  if (spriteLooksPartialDecode()) {
    if (label) { Serial.printf("%s PARTIAL\n", label); appendDiagLog("vld: %s PARTIAL\n", label); }
    return false;
  }
  if (spriteLooksHorizontallyCorrupted() || spriteLooksVerticallyCorrupted()) {
    if (label) { Serial.printf("%s LINE-CORR\n", label); appendDiagLog("vld: %s LINE-CORR\n", label); }
    return false;
  }
  return true;
}

static bool captureWeatherSemanticSignatureFromSprite(WeatherSemanticSignature* out) {
  if (!out) return false;
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px) return false;

  memset(out, 0, sizeof(*out));

  constexpr int TILE_W = 16;
  constexpr int TILE_H = 16;
  constexpr int COLS = DISP_W / TILE_W;
  constexpr int ROWS = DISP_H / TILE_H;
  constexpr int START_Y = 14;

  uint32_t frameSumR = 0;
  uint32_t frameSumG = 0;
  uint32_t frameSumB = 0;
  uint32_t frameSamples = 0;

  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < COLS; ++bc) {
      uint32_t tileSumR = 0;
      uint32_t tileSumG = 0;
      uint32_t tileSumB = 0;
      int cyanish = 0;
      int whiteish = 0;
      int sampled = 0;

      for (int dy = 0; dy < TILE_H; ++dy) {
        int py = br * TILE_H + dy;
        if (py < START_Y || py >= DISP_H) continue;
        const uint16_t* row = px + py * DISP_W + bc * TILE_W;
        for (int dx = 0; dx < TILE_W; ++dx) {
          uint16_t p = row[dx];
          if (s_mainSpritePixelsByteSwapped) p = __builtin_bswap16(p);
          int r = (p >> 11) & 0x1F;
          int g = ((p >> 5) & 0x3F) >> 1;
          int b = p & 0x1F;

          tileSumR += (uint32_t)r;
          tileSumG += (uint32_t)g;
          tileSumB += (uint32_t)b;
          frameSumR += (uint32_t)r;
          frameSumG += (uint32_t)g;
          frameSumB += (uint32_t)b;
          frameSamples++;
          sampled++;

          bool isWhiteish = (r >= 22 && g >= 22 && b >= 22);
          bool isCyanish = (g >= 20 && b >= 15 && r <= 10 && (g + b - (r * 2) >= 20));
          if (isWhiteish) whiteish++;
          if (isCyanish) cyanish++;
        }
      }

      if (sampled <= 0) continue;
      out->tileMean[br][bc][0] = (uint8_t)(tileSumR / (uint32_t)sampled);
      out->tileMean[br][bc][1] = (uint8_t)(tileSumG / (uint32_t)sampled);
      out->tileMean[br][bc][2] = (uint8_t)(tileSumB / (uint32_t)sampled);

      int mixPct = ((cyanish + whiteish) * 100) / sampled;
      int cyanPct = (cyanish * 100) / sampled;
      int whitePct = (whiteish * 100) / sampled;
      if (mixPct >= 50 && cyanPct >= 10 && whitePct >= 6) {
        out->cyanTiles++;
      }
    }
  }

  if (frameSamples <= 0) return false;
  uint32_t gradSum = 0;
  uint32_t gradCnt = 0;
  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < COLS; ++bc) {
      if (bc + 1 < COLS) {
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][0] - (int)out->tileMean[br][bc + 1][0]);
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][1] - (int)out->tileMean[br][bc + 1][1]);
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][2] - (int)out->tileMean[br][bc + 1][2]);
        gradCnt++;
      }
      if (br + 1 < ROWS) {
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][0] - (int)out->tileMean[br + 1][bc][0]);
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][1] - (int)out->tileMean[br + 1][bc][1]);
        gradSum += (uint32_t)abs((int)out->tileMean[br][bc][2] - (int)out->tileMean[br + 1][bc][2]);
        gradCnt++;
      }
    }
  }
  out->meanR = (uint8_t)(frameSumR / frameSamples);
  out->meanG = (uint8_t)(frameSumG / frameSamples);
  out->meanB = (uint8_t)(frameSumB / frameSamples);
  uint32_t tex = (gradCnt > 0) ? (gradSum / gradCnt) : 0;
  if (tex > 255) tex = 255;
  out->texture = (uint8_t)tex;
  return true;
}

static int weatherSemanticDistance(const WeatherSemanticSignature& a,
                                   const WeatherSemanticSignature& b) {
  int total = 0;
  int tiles = 0;
  for (int br = 0; br < 11; ++br) {
    for (int bc = 0; bc < 20; ++bc) {
      total += abs((int)a.tileMean[br][bc][0] - (int)b.tileMean[br][bc][0]);
      total += abs((int)a.tileMean[br][bc][1] - (int)b.tileMean[br][bc][1]);
      total += abs((int)a.tileMean[br][bc][2] - (int)b.tileMean[br][bc][2]);
      tiles++;
    }
  }
  if (tiles <= 0) return 0;
  return total / tiles;
}


// ─────────────────────────────────────────────────────────────
//  GIBS DescribeDomains — query exact available timestamps
// ─────────────────────────────────────────────────────────────

static time_t parseISOToUtcEpoch(const char* p) {
  int Y, M, D, h, m, s;
  if (sscanf(p, "%d-%d-%dT%d:%d:%d", &Y, &M, &D, &h, &m, &s) != 6) return 0;
  if (Y < 2024 || M < 1 || M > 12 || D < 1 || D > 31) return 0;
  static const int kDpM[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int y = Y - 1970;
  int leaps = (y + 1) / 4 - (y + 69) / 100 + (y + 369) / 400;
  int yd = kDpM[M - 1] + D - 1;
  if (M > 2 && (Y % 4 == 0) && (Y % 100 != 0 || Y % 400 == 0)) yd++;
  return ((time_t)y * 365 + leaps + yd) * 86400 + h * 3600 + m * 60 + s;
}

static time_t snapToNearestGibsTime(time_t t, int maxOffsetSec) {
  if (s_gibsAvailCount == 0) return 0;
  int lo = 0, hi = s_gibsAvailCount - 1;
  while (lo < hi) {
    int mid = (lo + hi) / 2;
    if (s_gibsAvailTimes[mid] < t) lo = mid + 1;
    else hi = mid;
  }
  time_t best = 0;
  long bestDist = (long)maxOffsetSec + 1;
  for (int i = (lo > 0 ? lo - 1 : 0); i <= lo && i < s_gibsAvailCount; i++) {
    long dist = labs((long)(s_gibsAvailTimes[i] - t));
    if (dist < bestDist) {
      bestDist = dist;
      best = s_gibsAvailTimes[i];
    }
  }
  return (bestDist <= maxOffsetSec) ? best : 0;
}

static bool fetchGibsAvailableTimes(WiFiClientSecure& client,
                                    time_t rangeStart, time_t rangeEnd) {
  s_gibsAvailCount = 0;
  char startISO[32], endISO[32];
  toISO(rangeStart, startISO, sizeof(startISO));
  toISO(rangeEnd, endISO, sizeof(endISO));

  char url[256];
  snprintf(url, sizeof(url),
    "https://gibs.earthdata.nasa.gov/wmts/epsg4326/best/wmts.cgi"
    "?SERVICE=WMTS&REQUEST=DescribeDomains&Version=1.0.0"
    "&Layer=%s&TileMatrixSet=500m&Time=%s/%s",
    s_activeGibsLayer, startISO, endISO);

  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(10000);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    Serial.printf("DescribeDomains HTTP-%d\n", code);
    return false;
  }

  int contentLen = http.getSize();
  if (contentLen <= 0 || contentLen > (int)(DL_BUF_BYTES - 1)) {
    http.end();
    return false;
  }
  WiFiClient* stream = http.getStreamPtr();
  size_t rd = stream->readBytes((char*)s_dlBuf, contentLen);
  http.end();
  if ((int)rd != contentLen) return false;
  s_dlBuf[rd] = 0;

  const char* domStart = strstr((char*)s_dlBuf, "<Domain>");
  const char* domEnd = strstr((char*)s_dlBuf, "</Domain>");
  if (!domStart || !domEnd) return false;
  domStart += 8;

  const char* p = domStart;
  while (p < domEnd && s_gibsAvailCount < MAX_GIBS_AVAIL) {
    const char* comma = (const char*)memchr(p, ',', domEnd - p);
    if (!comma) comma = domEnd;

    const char* slash1 = (const char*)memchr(p, '/', comma - p);
    if (!slash1) { p = comma + 1; continue; }
    const char* slash2 = (const char*)memchr(slash1 + 1, '/', comma - slash1 - 1);
    if (!slash2) { p = comma + 1; continue; }

    char buf[32];
    int len = min((int)(slash1 - p), 31);
    memcpy(buf, p, len); buf[len] = 0;
    time_t tStart = parseISOToUtcEpoch(buf);

    len = min((int)(slash2 - slash1 - 1), 31);
    memcpy(buf, slash1 + 1, len); buf[len] = 0;
    time_t tEnd = parseISOToUtcEpoch(buf);

    if (tStart > 0 && tEnd >= tStart) {
      for (time_t tt = tStart; tt <= tEnd && s_gibsAvailCount < MAX_GIBS_AVAIL; tt += 600) {
        s_gibsAvailTimes[s_gibsAvailCount++] = tt;
      }
    }
    p = comma + 1;
  }

  Serial.printf("GIBS avail: %d times\n", s_gibsAvailCount);
  appendDiagLog("gibs: describeDomains count=%d\n", s_gibsAvailCount);
  return s_gibsAvailCount > 0;
}


static bool decodeJpegPathToSprite(const char* path, bool relaxedHeight) {
  if (!path || !ensureSprite()) return false;

  File f = SD.open(path, FILE_READ);
  if (!f) {
    appendDiagLog("dec-fail: %s no-file\n", path);
    return false;
  }
  size_t size = f.size();
  if (size == 0 || size > MAX_JPEG_BYTES) {
    appendDiagLog("dec-fail: %s sz=%u\n", path, (unsigned)size);
    Serial.printf("jpeg skip %s %u>%u\n",
                  path, (unsigned)size, (unsigned)MAX_JPEG_BYTES);
    f.close();
    return false;
  }

  size_t got = f.readBytes((char*)s_dlBuf, size);
  f.close();
  if (got < size) {
    appendDiagLog("dec-fail: %s short=%u/%u\n", path, (unsigned)got, (unsigned)size);
    return false;
  }
  size_t jpegLen = jpegEffectiveLength(s_dlBuf, got);
  if (jpegLen == 0) {
    // Diagnostic: log what was actually read vs what's on disk
    static int s_noEoiDiagCount = 0;
    if (s_noEoiDiagCount < 10) {
      uint8_t tail[2] = {0, 0};
      if (got >= 2) { tail[0] = s_dlBuf[got-2]; tail[1] = s_dlBuf[got-1]; }
      appendDiagLog("dec-fail: %s no-eoi sz=%u got=%u hdr=%02x%02x tail=%02x%02x\n",
        path, (unsigned)size, (unsigned)got,
        (unsigned)s_dlBuf[0], (unsigned)s_dlBuf[1],
        (unsigned)tail[0], (unsigned)tail[1]);
      // Re-read file independently to check if disk data differs from s_dlBuf
      File chk = SD.open(path, FILE_READ);
      if (chk) {
        uint8_t h2[2] = {0, 0}, t2[2] = {0, 0};
        chk.read(h2, 2);
        size_t csz = chk.size();
        if (csz >= 2) { chk.seek(csz - 2); chk.read(t2, 2); }
        chk.close();
        appendDiagLog("dec-reread: %s sz=%u hdr=%02x%02x tail=%02x%02x\n",
          path, (unsigned)csz, h2[0], h2[1], t2[0], t2[1]);
      }
      s_noEoiDiagCount++;
    }
    return false;
  }

  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  int jpegActualH = 0;  // non-zero when a shorter-than-DISP_H decode was accepted
  bool padShortBottomRows = false;
  if (jpeg.openRAM(s_dlBuf, (int)jpegLen, jpegDraw)) {
    int jw = jpeg.getWidth();
    int jh = jpeg.getHeight();
    int decodeOpt = -1;
    if (jw == DISP_W && jh == DISP_H) {
      decodeOpt = 0;
    } else if (relaxedHeight && jw == DISP_W && jh >= DISP_H - 8 && jh < DISP_H) {
      // NOAA Esri service returns slightly shorter JPEG due to Web Mercator reprojection
      // rounding (e.g. 320×172 when 320×176 was requested). Accept it — the bottom
      // rows of the sprite stay black (from fillScreen above), which is correct for radar.
      decodeOpt = 0;
      jpegActualH = jh;
    } else if (jw == (DISP_W * 2) && jh == (DISP_H * 2)) {
      decodeOpt = JPEG_SCALE_HALF;
    } else if (jw == (DISP_W * 2) && jh >= ((DISP_H - 8) * 2) && jh < (DISP_H * 2)) {
      // Supersampled zoom snapshots may come back a few rows shorter than the
      // logical 176px buffer height. Accept them at half-scale and pad the
      // missing bottom rows from the last decoded row.
      decodeOpt = JPEG_SCALE_HALF;
      jpegActualH = jh >> 1;
      padShortBottomRows = true;
    }
    if (decodeOpt >= 0) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, decodeOpt);
      if (!ok) appendDiagLog("dec-fail: %s decode\n", path);
    } else {
      appendDiagLog("dec-fail: %s dim=%dx%d\n", path, jw, jh);
    }
    jpeg.close();
  }
  if (ok) {
    if (jpegActualH > 0) {
      // Accept any full-width decode that didn't go out of bounds.
      // No s_jpegMaxY constraint — short JPEG ends before DISP_H and that's fine.
      ok = (s_jpegDrawCalls > 0 && !s_jpegDrawOutOfBounds &&
            s_jpegMinX == 0 && s_jpegMinY == 0 &&
            s_jpegMaxX == DISP_W);
      if (ok && padShortBottomRows && jpegActualH > 0 && jpegActualH < DISP_H) {
        uint16_t* px = (uint16_t*)sprite.getBuffer();
        if (!px) return false;
        const uint16_t* lastRow = px + ((jpegActualH - 1) * DISP_W);
        for (int y = jpegActualH; y < DISP_H; y++) {
          memcpy(px + (y * DISP_W), lastRow, (size_t)DISP_W * 2U);
        }
      }
    } else {
      ok = jpegDrawLooksFullFrame();
      if (!ok) appendDiagLog("dec-fail: %s not-full\n", path);
    }
  }
  return ok;
}

static void applyDayTerrainGreenBoostOnSprite() {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return;
  const size_t total = (size_t)DISP_W * (size_t)DISP_H;
  for (size_t i = 0; i < total; i++) {
    uint16_t c = px[i];
    uint16_t n = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(c) : c;
    int r = (n >> 11) & 0x1F;
    int g = (n >> 5) & 0x3F;
    int b = n & 0x1F;

    // Mild green lift to counter blue-leaning panel tint without washing out.
    g = g + (((63 - g) * 12 + 50) / 100) + 1;
    if (g > 63) g = 63;

    uint16_t out = (uint16_t)((r << 11) | (g << 5) | b);
    px[i] = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(out) : out;
  }
}

static uint32_t countRadarSignalPixelsInSprite(int minSaturation) {
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return 0U;
  uint32_t count = 0U;
  const size_t total = (size_t)DISP_W * (size_t)DISP_H;
  for (size_t i = 0; i < total; i++) {
    uint16_t c = px[i];
    uint16_t n = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(c) : c;
    int r = (n >> 11) & 0x1F;
    int g = (n >> 5) & 0x3F;
    int b = n & 0x1F;
    int g5 = (g + 1) >> 1;  // normalize to 5-bit channel scale

    int maxCh = r;
    if (g5 > maxCh) maxCh = g5;
    if (b > maxCh) maxCh = b;
    int minCh = r;
    if (g5 < minCh) minCh = g5;
    if (b < minCh) minCh = b;

    int sat = maxCh - minCh;
    int lum = r + g5 + b;
    if (sat >= minSaturation && lum >= 8) count++;
  }
  return count;
}

static bool decodeTerrainCompositeToSprite() {
  // Radar is optional and must never blank the terrain stage.
  bool radarRawReady = false;
  uint32_t radarSignalPixels = 0U;
  bool radarFileExists = fileExistsNonEmpty(ZOOM_TERRAIN_RADAR_FILE);
  size_t radarFileBytes = 0U;
  if (radarFileExists) {
    File rf = SD.open(ZOOM_TERRAIN_RADAR_FILE, FILE_READ);
    if (rf) { radarFileBytes = (size_t)rf.size(); rf.close(); }
  }
  SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);

  if (radarFileExists &&
      decodeJpegPathToSprite(ZOOM_TERRAIN_RADAR_FILE, true)) {
    radarSignalPixels = countRadarSignalPixelsInSprite(5);
    if (radarSignalPixels >= 1U) {
      uint16_t* radarPx = (uint16_t*)sprite.getBuffer();
      if (radarPx) {
        File wf = SD.open(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE, FILE_WRITE);
        if (wf) {
          size_t wrote = wf.write((const uint8_t*)radarPx, RAW_FRAME_BYTES);
          wf.close();
          radarRawReady = (wrote == RAW_FRAME_BYTES);
        }
      }
    }
  }

  // Base terrain is mandatory. Use active day/night first, then fall back to the
  // opposite terrain layer if the active JPEG is missing/corrupt.
  bool nightLayer = terrainUsesNightLayerForUtc(time(nullptr));
  const char* primaryTerrainJpeg = nightLayer ? ZOOM_TERRAIN_NIGHT_FILE : ZOOM_TERRAIN_DAY_FILE;
  const char* fallbackTerrainJpeg = nightLayer ? ZOOM_TERRAIN_DAY_FILE : ZOOM_TERRAIN_NIGHT_FILE;
  bool decodedBase = decodeJpegPathToSprite(primaryTerrainJpeg);
  if (!decodedBase) {
    if (decodeJpegPathToSprite(fallbackTerrainJpeg)) {
      nightLayer = !nightLayer;
      Serial.printf("terrain jpeg fallback %s\n", fallbackTerrainJpeg);
      decodedBase = true;
    }
  }
  if (!decodedBase) {
    SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
    Serial.println("Terrain base decode failed");
    return false;
  }

  // Day-only green lift to counter panel tint.
  if (!nightLayer) applyDayTerrainGreenBoostOnSprite();

  // No usable radar → return base terrain only.
  if (!radarRawReady) {
    SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
    if (!radarFileExists) {
      Serial.println("radar miss -> base");
    } else if (radarSignalPixels < 1U) {
      Serial.println("radar weak -> base");
    } else {
      Serial.println("radar tmp write fail -> base");
    }
    Serial.printf("radar diag e=%d b=%u s=%u r=%d\n",
                  (int)radarFileExists, (unsigned)radarFileBytes,
                  (unsigned)radarSignalPixels, (int)radarRawReady);
    return true;
  }

  File rf = SD.open(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE, FILE_READ);
  if (!rf || rf.size() != RAW_FRAME_BYTES) {
    if (rf) rf.close();
    SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
    Serial.println("radar raw read fail -> base");
    return true;
  }

  uint16_t* dst = (uint16_t*)sprite.getBuffer();
  if (!dst) {
    rf.close();
    SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
    return true;
  }

  const size_t rowBytes = (size_t)DISP_W * 2U;
  uint16_t radarRow[DISP_W];
  uint8_t prevMask[DISP_W] = {};
  uint8_t currMask[DISP_W] = {};
  uint8_t nextMask[DISP_W] = {};
  uint16_t currColor[DISP_W] = {};
  uint16_t nextColor[DISP_W] = {};
  uint8_t currAlpha[DISP_W] = {};
  uint8_t nextAlpha[DISP_W] = {};
  uint32_t blendedPixels = 0U;

  auto classifyRadarRow = [&](uint8_t* outMask, uint16_t* outColor, uint8_t* outAlpha) -> bool {
    if (rf.read((uint8_t*)radarRow, rowBytes) != (int)rowBytes) return false;
    for (int x = 0; x < DISP_W; x++) {
      uint16_t radarMem = radarRow[x];
      uint16_t radar565 = s_mainSpritePixelsByteSwapped ? __builtin_bswap16(radarMem) : radarMem;

      int r = (radar565 >> 11) & 0x1F;
      int g = (radar565 >> 5) & 0x3F;
      int b = radar565 & 0x1F;
      int g5 = (g + 1) >> 1;  // normalize to 5-bit scale
      int maxCh = r;
      if (g5 > maxCh) maxCh = g5;
      if (b > maxCh) maxCh = b;
      int minCh = r;
      if (g5 < minCh) minCh = g5;
      if (b < minCh) minCh = b;
      int sat = maxCh - minCh;
      int lum = r + g5 + b;
      if (sat < 5 || lum < 8) {
        outMask[x] = 0;
        outAlpha[x] = 0;
        outColor[x] = 0;
        continue;
      }

      uint16_t mapped = 0x07E0;  // green
      if (g5 >= r && g5 >= b) {
        if (r > 20 && g5 > 20) mapped = 0xFFE0;       // yellow
        else if (r > 12 && g5 > 16) mapped = 0xFD20;  // orange
        else if (g5 > 22) mapped = 0x07E0;            // bright green
        else mapped = 0x03E0;                         // dark green
      } else if (r >= g5 && r >= b) {
        if (b > 12) mapped = 0xF81F;                  // magenta
        else if (g5 > 16) mapped = 0xFD20;            // orange
        else mapped = 0xF800;                         // red
      } else {
        mapped = (r > 10) ? 0xF81F : 0x07FF;          // magenta / cyan
      }

      int strength = sat + maxCh;                     // 0..62
      uint8_t alpha = (uint8_t)(150 + ((strength * 80) / 62)); // ~150..230
      if (alpha > 230) alpha = 230;

      outMask[x] = 1;
      outColor[x] = mapped;
      outAlpha[x] = alpha;
    }
    return true;
  };

  if (!classifyRadarRow(currMask, currColor, currAlpha)) {
    rf.close();
    SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
    return true;
  }
  if (DISP_H > 1) {
    if (!classifyRadarRow(nextMask, nextColor, nextAlpha)) {
      memset(nextMask, 0, sizeof(nextMask));
      memset(nextAlpha, 0, sizeof(nextAlpha));
      memset(nextColor, 0, sizeof(nextColor));
    }
  }

  for (int y = 0; y < DISP_H; y++) {
    uint16_t* dstRow = dst + ((size_t)y * (size_t)DISP_W);
    for (int x = 0; x < DISP_W; x++) {
      if (!currMask[x]) continue;

      int x0 = (x > 0) ? (x - 1) : x;
      int x1 = (x + 1 < DISP_W) ? (x + 1) : x;
      int neighbors = 0;
      for (int xx = x0; xx <= x1; xx++) {
        if (prevMask[xx]) neighbors++;
        if (nextMask[xx]) neighbors++;
      }
      if (x > 0 && currMask[x - 1]) neighbors++;
      if (x + 1 < DISP_W && currMask[x + 1]) neighbors++;
      if (neighbors <= 1) continue;  // despeckle isolated hits

      uint16_t fg = currColor[x];
      uint8_t alpha = currAlpha[x];
      if (!s_mainSpritePixelsByteSwapped) {
        dstRow[x] = blend565(dstRow[x], fg, alpha);
      } else {
        uint16_t bg16 = __builtin_bswap16(dstRow[x]);
        dstRow[x] = __builtin_bswap16(blend565(bg16, fg, alpha));
      }
      blendedPixels++;
    }

    if (y == DISP_H - 1) break;
    memcpy(prevMask, currMask, sizeof(prevMask));
    memcpy(currMask, nextMask, sizeof(currMask));
    memcpy(currColor, nextColor, sizeof(currColor));
    memcpy(currAlpha, nextAlpha, sizeof(currAlpha));

    if (y + 2 < DISP_H) {
      if (!classifyRadarRow(nextMask, nextColor, nextAlpha)) {
        memset(nextMask, 0, sizeof(nextMask));
        memset(nextColor, 0, sizeof(nextColor));
        memset(nextAlpha, 0, sizeof(nextAlpha));
      }
    } else {
      memset(nextMask, 0, sizeof(nextMask));
      memset(nextColor, 0, sizeof(nextColor));
      memset(nextAlpha, 0, sizeof(nextAlpha));
    }
  }

  rf.close();
  SD.remove(ZOOM_TERRAIN_RADAR_RAW_TMP_FILE);
  Serial.printf("radar blend e=%d b=%u s=%u n=%u\n",
                (int)radarFileExists, (unsigned)radarFileBytes,
                (unsigned)radarSignalPixels, (unsigned)blendedPixels);
  return true;
}

static bool installValidatedZoomSnapshotAtBbox(HTTPClient& http,
                                               WiFiClientSecure& client,
                                               time_t t,
                                               const char* finalPath,
                                               float bboxWest, float bboxSouth,
                                               float bboxEast, float bboxNorth,
                                               int reqW = DISP_W,
                                               int reqH = DISP_H,
                                               size_t* outBytes = nullptr) {
  const char* tmpPath = SD_ROOT "/frames/.zoomtmp.jpg";
  SD.remove(tmpPath);

  size_t tmpBytes = 0;
  if (!downloadFrameToPathAtBbox(http, client, t, tmpPath,
                                 bboxWest, bboxSouth, bboxEast, bboxNorth,
                                 reqW, reqH,
                                 &tmpBytes)) {
    return false;
  }

  bool valid = decodeJpegPathToSprite(tmpPath);
  if (valid && spriteLooksCompletelyBlack()) valid = false;
  if (valid && spriteLooksPartialDecode()) valid = false;
  if (valid && (spriteLooksHorizontallyCorrupted() || spriteLooksVerticallyCorrupted())) valid = false;
  if (!valid) {
    SD.remove(tmpPath);
    Serial.println("zoom rej bad");
    return false;
  }

  SD.remove(finalPath);
  bool moved = SD.rename(tmpPath, finalPath);
  if (!moved) {
    moved = copyFrameFile(tmpPath, finalPath);
    if (moved) SD.remove(tmpPath);
  }
  if (!moved) {
    SD.remove(tmpPath);
    Serial.println("zoom install fail");
    return false;
  }

  const char* rawPath = zoomRawPathForJpeg(finalPath);
  if (rawPath) SD.remove(rawPath);  // force low-pass raw rebuild for new JPEG

  if (outBytes) *outBytes = tmpBytes;
  return true;
}

static bool extractRadarTimeExtentEndMs(const String& body, unsigned long long* outMs) {
  if (!outMs) return false;
  *outMs = 0ULL;

  const char* s = body.c_str();
  if (!s) return false;
  const char* p = strstr(s, "\"timeExtent\"");
  if (!p) return false;
  p = strchr(p, '[');
  if (!p) return false;
  p++;

  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
  while (*p >= '0' && *p <= '9') p++;   // skip start time
  p = strchr(p, ',');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
  if (!(*p >= '0' && *p <= '9')) return false;

  unsigned long long v = 0ULL;
  while (*p >= '0' && *p <= '9') {
    v = (v * 10ULL) + (unsigned long long)(*p - '0');
    p++;
  }
  if (v == 0ULL) return false;
  *outMs = v;
  return true;
}

static bool extractRadarLatestValidTimeMsFromQuery(const String& body, unsigned long long* outMs) {
  if (!outMs) return false;
  *outMs = 0ULL;

  const char* s = body.c_str();
  if (!s) return false;

  // Query responses include a fields[] schema where "idp_validtime" appears as a
  // string value. Parse the key token with colon to avoid matching that schema text.
  const char* p = strstr(s, "\"idp_validtime\":");
  if (!p) return false;
  p += strlen("\"idp_validtime\":");

  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
  if (!(*p >= '0' && *p <= '9')) return false;

  unsigned long long v = 0ULL;
  while (*p >= '0' && *p <= '9') {
    v = (v * 10ULL) + (unsigned long long)(*p - '0');
    p++;
  }
  if (v == 0ULL) return false;
  *outMs = v;
  return true;
}

static bool fetchRadarLatestTimeMs(HTTPClient& http,
                                   WiFiClientSecure& client,
                                   unsigned long long* outMs) {
  if (!outMs) return false;
  *outMs = 0ULL;

  unsigned long long cacheBust = 0ULL;
  time_t nowUtc = time(nullptr);
  if (nowUtc > 1000000000) {
    cacheBust = ((unsigned long long)nowUtc * 1000ULL) + (unsigned long long)(millis() & 0x7FFU);
  } else {
    cacheBust = (unsigned long long)millis();
  }
  if (cacheBust == 0ULL) cacheBust = 1ULL;

  char queryUrl[512];
  snprintf(queryUrl, sizeof(queryUrl),
    "https://mapservices.weather.noaa.gov/eventdriven/rest/services/radar/"
    "radar_base_reflectivity_time/ImageServer/query"
    "?where=1%%3D1&outFields=idp_validtime&returnGeometry=false"
    "&orderByFields=idp_validtime%%20DESC&resultRecordCount=1&f=pjson&_ts=%llu",
    cacheBust);

  http.begin(client, queryUrl);
  http.setTimeout(7000);
  http.addHeader("Cache-Control", "no-cache");
  http.addHeader("Pragma", "no-cache");
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    String body = http.getString();
    http.end();
    if (extractRadarLatestValidTimeMsFromQuery(body, outMs) && *outMs > 0ULL) {
      return true;
    }
  } else {
    http.end();
  }

  char metaUrl[320];
  snprintf(metaUrl, sizeof(metaUrl),
    "https://mapservices.weather.noaa.gov/eventdriven/rest/services/radar/"
    "radar_base_reflectivity_time/ImageServer?f=pjson&_ts=%llu",
    cacheBust + 1ULL);

  http.begin(client, metaUrl);
  http.setTimeout(7000);
  http.addHeader("Cache-Control", "no-cache");
  http.addHeader("Pragma", "no-cache");
  code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();
  return extractRadarTimeExtentEndMs(body, outMs);
}

static float wrapHours24(float h) {
  while (h < 0.0f) h += 24.0f;
  while (h >= 24.0f) h -= 24.0f;
  return h;
}

static float wrapDegrees360(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

static bool computeSunEventLocalHour(int dayOfYear,
                                     float latDeg, float lonDeg,
                                     bool sunrise,
                                     float utcOffsetHours,
                                     float* outLocalHour) {
  if (!outLocalHour) return false;
  if (dayOfYear < 1 || dayOfYear > 366) return false;
  if (latDeg < -89.8f || latDeg > 89.8f) return false;
  if (lonDeg < -180.0f || lonDeg > 180.0f) return false;

  const float degToRad = 0.01745329252f;
  const float radToDeg = 57.2957795131f;
  const float zenithDeg = 90.833f;  // apparent sunrise/sunset

  float lngHour = lonDeg / 15.0f;
  float t = (float)dayOfYear + (((sunrise ? 6.0f : 18.0f) - lngHour) / 24.0f);
  float M = (0.9856f * t) - 3.289f;
  float L = M
          + (1.916f * sinf(M * degToRad))
          + (0.020f * sinf(2.0f * M * degToRad))
          + 282.634f;
  L = wrapDegrees360(L);

  float RA = atanf(0.91764f * tanf(L * degToRad)) * radToDeg;
  RA = wrapDegrees360(RA);

  float Lquadrant = floorf(L / 90.0f) * 90.0f;
  float RAquadrant = floorf(RA / 90.0f) * 90.0f;
  RA += (Lquadrant - RAquadrant);
  RA /= 15.0f;

  float sinDec = 0.39782f * sinf(L * degToRad);
  float cosDec = cosf(asinf(sinDec));
  float sinLat = sinf(latDeg * degToRad);
  float cosLat = cosf(latDeg * degToRad);
  if (fabsf(cosDec) < 1e-6f || fabsf(cosLat) < 1e-6f) return false;

  float cosH = (cosf(zenithDeg * degToRad) - (sinDec * sinLat)) / (cosDec * cosLat);
  if (cosH > 1.0f || cosH < -1.0f) return false;  // polar day/night case

  float H = acosf(cosH) * radToDeg;
  if (sunrise) H = 360.0f - H;
  H /= 15.0f;

  float T = H + RA - (0.06571f * t) - 6.622f;
  float UT = wrapHours24(T - lngHour);
  *outLocalHour = wrapHours24(UT + utcOffsetHours);
  return true;
}

static bool terrainUsesNightLayerForUtc(time_t weatherFrameUtc) {
  // Terrain day/night should track the user's *current* local light cycle, not
  // the weather frame capture timestamp (which can lag by hours).
  time_t nowUtc = time(nullptr);
  if (nowUtc < 1000000000) nowUtc = weatherFrameUtc;

  struct tm tmTerrainLocal;
  if (!localTimeForDisplay(nowUtc, &tmTerrainLocal)) return false;

  float localHour = (float)tmTerrainLocal.tm_hour
                  + ((float)tmTerrainLocal.tm_min / 60.0f)
                  + ((float)tmTerrainLocal.tm_sec / 3600.0f);

  // Fallback threshold if geo data isn't available yet.
  if (!s_weatherGeoValid) {
    return (localHour >= 19.0f || localHour < 6.0f);
  }

  int dayOfYear = tmTerrainLocal.tm_yday + 1;  // tm_yday is 0-based
  float utcOffsetHours = ((float)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600))) / 3600.0f;

  float sunriseHour = 0.0f, sunsetHour = 0.0f;
  bool haveSunrise = computeSunEventLocalHour(dayOfYear, s_weatherCenterLat, s_weatherCenterLon,
                                               true, utcOffsetHours, &sunriseHour);
  bool haveSunset  = computeSunEventLocalHour(dayOfYear, s_weatherCenterLat, s_weatherCenterLon,
                                               false, utcOffsetHours, &sunsetHour);

  if (!haveSunrise || !haveSunset) {
    return (localHour >= 19.0f || localHour < 6.0f);
  }

  if (sunriseHour <= sunsetHour) {
    return (localHour < sunriseHour || localHour >= sunsetHour);
  }
  return !(localHour >= sunsetHour && localHour < sunriseHour);
}

// Return the terrain JPEG / raw paths appropriate for the current wall-clock time.
// ~0.1ms of float math — safe to call once per terrain stage, no FPS impact.
static const char* activeTerrainJpegPath() {
  return terrainUsesNightLayerForUtc(time(nullptr)) ? ZOOM_TERRAIN_NIGHT_FILE : ZOOM_TERRAIN_DAY_FILE;
}
static const char* activeTerrainRawPath() {
  return terrainUsesNightLayerForUtc(time(nullptr)) ? ZOOM_TERRAIN_NIGHT_RAW : ZOOM_TERRAIN_DAY_RAW;
}

// Downloads both day (BlueMarble) and night (Black Marble) terrain JPEGs for the
// given bbox. Storing both allows live day/night switching without re-downloading.
// Returns true only when BOTH textures are successfully refreshed.
static bool downloadTerrainSnapshotToPathAtBbox(HTTPClient& http,
                                                WiFiClientSecure& client,
                                                time_t weatherFrameUtc,
                                                float bboxWest, float bboxSouth,
                                                float bboxEast, float bboxNorth) {
  bool haveExistingDay = zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_DAY_FILE);
  bool haveExistingNight = zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_NIGHT_FILE);

  auto downloadJpegUrlToPath = [&](const char* url, const char* outPath, const char* label, size_t* bytesOut) -> bool {
    http.begin(client, url);
    http.setTimeout(8000);
    http.addHeader("Cache-Control", "no-cache");
    http.addHeader("Pragma", "no-cache");
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
      http.end();
      Serial.printf("%s HTTP-%d\n", label, code);
      return false;
    }

    size_t jpegLen = 0;
    if (!readHttpJpegBodyToDlBuf(http, label, &jpegLen)) {
      return false;
    }
    SD.remove(outPath);
    File f = SD.open(outPath, FILE_WRITE);
    size_t written = f ? f.write(s_dlBuf, jpegLen) : 0;
    if (f) f.close();
    if (written != jpegLen) {
      SD.remove(outPath);
      Serial.printf("%s SD-ERR\n", label);
      return false;
    }
    if (bytesOut) *bytesOut = written;
    return true;
  };

  // Always download both textures so day/night can switch live without re-downloading.
  char terrainUrl[640];
  const char* dayTmpPath = SD_ROOT "/frames/.terrain_day.tmp.jpg";
  const char* nightTmpPath = SD_ROOT "/frames/.terrain_night.tmp.jpg";
  SD.remove(dayTmpPath);
  SD.remove(nightTmpPath);
  auto downloadTerrainLayer2x = [&](const char* layer,
                                    const char* outPath,
                                    const char* label,
                                    const char* timeParam = nullptr) -> bool {
    if (timeParam) {
      snprintf(terrainUrl, sizeof(terrainUrl),
        "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
        "&LAYERS=%s"
        "&STYLES=&SRS=EPSG:4326"
        "&BBOX=%.6f,%.6f,%.6f,%.6f&WIDTH=%d&HEIGHT=%d"
        "&FORMAT=image%%2Fjpeg&TIME=%s",
        layer,
        (double)bboxWest, (double)bboxSouth,
        (double)bboxEast, (double)bboxNorth,
        TERRAIN_FETCH_W, TERRAIN_FETCH_H, timeParam);
    } else {
      snprintf(terrainUrl, sizeof(terrainUrl),
        "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
        "&LAYERS=%s"
        "&STYLES=&SRS=EPSG:4326"
        "&BBOX=%.6f,%.6f,%.6f,%.6f&WIDTH=%d&HEIGHT=%d"
        "&FORMAT=image%%2Fjpeg",
        layer,
        (double)bboxWest, (double)bboxSouth,
        (double)bboxEast, (double)bboxNorth,
        TERRAIN_FETCH_W, TERRAIN_FETCH_H);
    }
    return downloadJpegUrlToPath(terrainUrl, outPath, label, nullptr);
  };

  // Day: BlueMarble
  bool dayOk = downloadTerrainLayer2x(
    "BlueMarble_NextGeneration",
    dayTmpPath,
    "Terrain day");

  // Night: VIIRS Black Marble (static composite, fixed date)
  bool nightOk = downloadTerrainLayer2x(
    "VIIRS_Black_Marble",
    nightTmpPath,
    "Terrain night",
    "2016-01-01");

  auto promoteTerrainTmp = [&](const char* tmpPath, const char* finalPath, const char* label) -> bool {
    SD.remove(finalPath);
    bool moved = SD.rename(tmpPath, finalPath);
    if (!moved) {
      moved = copyFrameFile(tmpPath, finalPath);
      if (moved) SD.remove(tmpPath);
    }
    if (!moved) {
      Serial.printf("%s promote fail\n", label);
    }
    return moved;
  };
  if (dayOk) {
    if (!promoteTerrainTmp(dayTmpPath, ZOOM_TERRAIN_DAY_FILE, "Terrain day")) dayOk = false;
  }
  if (nightOk) {
    if (!promoteTerrainTmp(nightTmpPath, ZOOM_TERRAIN_NIGHT_FILE, "Terrain night")) nightOk = false;
  }
  SD.remove(dayTmpPath);
  SD.remove(nightTmpPath);

  bool dayAvailable = dayOk || haveExistingDay || zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_DAY_FILE);
  bool nightAvailable = nightOk || haveExistingNight || zoomSnapshotFileLooksUsable(ZOOM_TERRAIN_NIGHT_FILE);

  if (!dayAvailable && nightAvailable) {
    dayAvailable = copyFrameFile(ZOOM_TERRAIN_NIGHT_FILE, ZOOM_TERRAIN_DAY_FILE);
  } else if (!nightAvailable && dayAvailable) {
    nightAvailable = copyFrameFile(ZOOM_TERRAIN_DAY_FILE, ZOOM_TERRAIN_NIGHT_FILE);
  }

  if (!(dayAvailable || nightAvailable)) {
    Serial.printf("terrain unavailable d=%d n=%d\n", (int)dayOk, (int)nightOk);
    appendDiagLog("terrain: unavailable d=%d n=%d keep=%d/%d\n",
                  (int)dayOk, (int)nightOk,
                  (int)haveExistingDay, (int)haveExistingNight);
    return false;
  }

  if (!dayAvailable || !nightAvailable) {
    appendDiagLog("terrain: partial d=%d n=%d keep=%d/%d\n",
                  (int)dayAvailable, (int)nightAvailable,
                  (int)haveExistingDay, (int)haveExistingNight);
  } else {
    Serial.println("terrain pair ok");
    appendDiagLog("terrain: ok d=%d n=%d\n", (int)dayAvailable, (int)nightAvailable);
  }

  char radarUrl[640];
  const char* radarTryPath = SD_ROOT "/frames/.radar_try.jpg";
  const char* radarBestPath = SD_ROOT "/frames/.radar_best.jpg";
  const uint32_t minUsableSignalPx = 5U;
  bool hadExistingRadar = fileExistsNonEmpty(ZOOM_TERRAIN_RADAR_FILE);
  SD.remove(radarTryPath);
  SD.remove(radarBestPath);

  auto radarSignalFromJpegPath = [&](const char* path) -> uint32_t {
    if (!decodeJpegPathToSprite(path, true)) return 0U;
    return countRadarSignalPixelsInSprite(5);
  };

  auto promoteRadarTryToBest = [&]() {
    SD.remove(radarBestPath);
    bool moved = SD.rename(radarTryPath, radarBestPath);
    if (!moved) {
      moved = copyFrameFile(radarTryPath, radarBestPath);
      if (moved) SD.remove(radarTryPath);
    }
    return moved;
  };

  bool radarOk = false;
  bool anyDownloadSucceeded = false;  // HTTP 200 + valid JPEG received (regardless of signal)
  uint32_t bestSignal = 0U;
  unsigned long long radarLatestMs = 0ULL;
  unsigned long long selectedRadarMs = 0ULL;
  unsigned long long radarReqBust = 0ULL;
  time_t nowUtc = time(nullptr);
  if (nowUtc > 1000000000) {
    radarReqBust = ((unsigned long long)nowUtc * 1000ULL) + (unsigned long long)(millis() & 0x7FFU);
  } else {
    radarReqBust = (unsigned long long)millis();
  }
  if (radarReqBust == 0ULL) radarReqBust = 1ULL;
  if (fetchRadarLatestTimeMs(http, client, &radarLatestMs) && radarLatestMs > 0ULL) {
    long radarAgeMin = (long)(difftime(time(nullptr), (time_t)(radarLatestMs / 1000ULL)) / 60.0);
    if (radarAgeMin < 0) radarAgeMin = 0;
    Serial.printf("radar age %ld m\n", radarAgeMin);
    snprintf(radarUrl, sizeof(radarUrl),
      "https://mapservices.weather.noaa.gov/eventdriven/rest/services/radar/"
      "radar_base_reflectivity_time/ImageServer/exportImage"
      "?bbox=%.6f,%.6f,%.6f,%.6f&bboxSR=4326&size=%d,%d&format=jpg&f=image&time=%llu&_cb=%llu",
      (double)bboxWest, (double)bboxSouth,
      (double)bboxEast, (double)bboxNorth,
      TERRAIN_FETCH_W, TERRAIN_FETCH_H, radarLatestMs, radarReqBust);
    if (downloadJpegUrlToPath(radarUrl, radarTryPath, "Terrain radar", nullptr)) {
      anyDownloadSucceeded = true;
      bestSignal = radarSignalFromJpegPath(radarTryPath);
      Serial.printf("[radar] dl1 sig=%u min=%u\n", (unsigned)bestSignal, (unsigned)minUsableSignalPx);
      if (bestSignal >= minUsableSignalPx) {
        radarOk = promoteRadarTryToBest();
        if (radarOk) selectedRadarMs = radarLatestMs;
      } else {
        Serial.printf("radar latest weak %u\n", (unsigned)bestSignal);
      }
    } else {
      Serial.println("[radar] dl1 failed");
    }
  }

  if (!radarOk) {
    snprintf(radarUrl, sizeof(radarUrl),
      "https://mapservices.weather.noaa.gov/eventdriven/rest/services/radar/"
      "radar_base_reflectivity_time/ImageServer/exportImage"
      "?bbox=%.6f,%.6f,%.6f,%.6f&bboxSR=4326&size=%d,%d&format=jpg&f=image&_cb=%llu",
      (double)bboxWest, (double)bboxSouth,
      (double)bboxEast, (double)bboxNorth,
      TERRAIN_FETCH_W, TERRAIN_FETCH_H, radarReqBust + 1ULL);
    if (downloadJpegUrlToPath(radarUrl, radarTryPath, "Terrain radar", nullptr)) {
      anyDownloadSucceeded = true;
      uint32_t fallbackSignal = radarSignalFromJpegPath(radarTryPath);
      if (fallbackSignal > bestSignal) bestSignal = fallbackSignal;
      Serial.printf("[radar] dl2 sig=%u min=%u\n", (unsigned)fallbackSignal, (unsigned)minUsableSignalPx);
      if (fallbackSignal >= minUsableSignalPx) {
        radarOk = promoteRadarTryToBest();
        if (radarOk) {
          selectedRadarMs = (radarLatestMs > 0ULL)
                          ? radarLatestMs
                          : ((unsigned long long)time(nullptr) * 1000ULL);
        }
      } else {
        Serial.printf("radar fb weak %u\n", (unsigned)fallbackSignal);
      }
    } else {
      Serial.println("[radar] dl2 failed");
    }
  }

  if (radarOk) {
    SD.remove(ZOOM_TERRAIN_RADAR_FILE);
    bool moved = SD.rename(radarBestPath, ZOOM_TERRAIN_RADAR_FILE);
    if (!moved) moved = copyFrameFile(radarBestPath, ZOOM_TERRAIN_RADAR_FILE);
    radarOk = moved;
  }
  SD.remove(radarTryPath);
  SD.remove(radarBestPath);

  if (!radarOk) {
    if (hadExistingRadar) {
      Serial.println("radar weak -> keep prev");
      // Keep existing radar file and timestamp; don't update status flags.
    } else if (anyDownloadSucceeded) {
      // Download reached the service and got a valid JPEG, but no precipitation detected.
      s_radarNoSignatures = true;
      s_radarDownloadFailed = false;
      s_lastRadarCheckUtc = time(nullptr);
      SD.remove(ZOOM_TERRAIN_RADAR_FILE);
      clearRadarMeta();
      Serial.println("radar clear -> base");
    } else {
      // Download itself failed — HTTP error or service unreachable (outside coverage).
      s_radarNoSignatures = false;
      s_radarDownloadFailed = true;
      SD.remove(ZOOM_TERRAIN_RADAR_FILE);
      clearRadarMeta();
      Serial.println("radar unavail -> base");
    }
  } else {
    s_radarNoSignatures = false;
    s_radarDownloadFailed = false;
    if (selectedRadarMs > 0ULL) {
      writeRadarMeta((time_t)(selectedRadarMs / 1000ULL));
    }
    Serial.printf("radar ok %u\n", (unsigned)bestSignal);
  }
  return true;
}

static bool showZoomSnapshotFrame(const char* path, int newestIdx) {
  serviceUserButtons();
  if (!path) return false;
  bool isTerrainStage = (strcmp(path, ZOOM_TERRAIN_DAY_FILE) == 0 ||
                         strcmp(path, ZOOM_TERRAIN_NIGHT_FILE) == 0);
  const char* rawPath = zoomRawPathForJpeg(path);
  auto loadRawStage = [&](const char* rp) -> bool {
    if (!rp || !ensureSprite()) return false;
    File rf = SD.open(rp, FILE_READ);
    if (!rf) return false;
    bool ok = ((size_t)rf.size() == RAW_FRAME_BYTES);
    if (!ok) { rf.close(); return false; }
    uint16_t* dst = (uint16_t*)sprite.getBuffer();
    if (!dst) { rf.close(); return false; }
    size_t got = rf.read((uint8_t*)dst, RAW_FRAME_BYTES);
    rf.close();
    return (got == RAW_FRAME_BYTES);
  };
  auto decodeStage = [&]() -> bool {
    if (isTerrainStage) {
      return decodeTerrainCompositeToSprite();
    }
    if (loadRawStage(rawPath)) return true;
    if (ensureFilteredZoomRaw(path, true) && loadRawStage(rawPath)) return true;
    return decodeJpegPathToSprite(path);
  };

  if (!decodeStage()) {
    if (!isTerrainStage && presentSyntheticZoomStage(path, newestIdx)) {
      Serial.printf("zoom synth-fallback %s (decode fail)\n", path);
      return true;
    }
    Serial.printf("zoom decode-fail %s\n", path);
    appendDiagLog("zoom: decode-fail %s\n", path);
    return false;
  }
  // If decoded zoom has black block artifacts (common on GOES-West limb),
  // fall through to synthetic zoom instead of showing the corrupt frame.
  if (!isTerrainStage && spriteLooksHoldFrameBlockCorrupted()) {
    if (presentSyntheticZoomStage(path, newestIdx)) {
      Serial.printf("zoom synth-fallback %s (blk-corr)\n", path);
      return true;
    }
    Serial.printf("zoom blk-corr %s\n", path);
    appendDiagLog("zoom: blk-corr %s\n", path);
    return false;
  }
  bool prevTopBarRadarMode = s_topBarUseRadarScanTime;
  if (isTerrainStage) s_topBarUseRadarScanTime = true;
  updateBarBufs(newestIdx);
  s_topBarUseRadarScanTime = prevTopBarRadarMode;
  presentSpriteToDisplay();
  serviceUserButtons();
  return true;
}

static void computeBboxFromCenterKm(float centerLat, float centerLon,
                                    float widthKm, float heightKm,
                                    float* west, float* south,
                                    float* east, float* north) {
  if (widthKm < 1.0f) widthKm = 1.0f;
  if (heightKm < 1.0f) heightKm = 1.0f;
  if (centerLat > 85.0f) centerLat = 85.0f;
  if (centerLat < -85.0f) centerLat = -85.0f;
  while (centerLon > 180.0f) centerLon -= 360.0f;
  while (centerLon < -180.0f) centerLon += 360.0f;

  float cosLat = cosf(centerLat * 0.01745329252f);
  if (cosLat < 0.2f) cosLat = 0.2f;
  float halfLatDeg = (heightKm * 0.5f) / 111.32f;
  float halfLonDeg = (widthKm  * 0.5f) / (111.32f * cosLat);

  float w = centerLon - halfLonDeg;
  float e = centerLon + halfLonDeg;
  float s = centerLat - halfLatDeg;
  float n = centerLat + halfLatDeg;
  if (s < -89.5f) s = -89.5f;
  if (n >  89.5f) n =  89.5f;
  if (w < -180.0f) w = -180.0f;
  if (e >  180.0f) e =  180.0f;

  if (west)  *west = w;
  if (south) *south = s;
  if (east)  *east = e;
  if (north) *north = n;
}

static void drawScaledOutlineRect(uint16_t* buf,
                                  int x, int y, int w, int h,
                                  uint16_t color) {
  if (!buf || w <= 1 || h <= 1) return;
  int x0 = x;
  int y0 = y;
  int x1 = x + w - 1;
  int y1 = y + h - 1;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= SCALED_W) x1 = SCALED_W - 1;
  if (y1 >= SCALED_H) y1 = SCALED_H - 1;
  if (x1 <= x0 || y1 <= y0) return;

  const int thickness = 2;
  for (int t = 0; t < thickness; ++t) {
    int yt = y0 + t;
    int yb = y1 - t;
    if (yt <= y1) {
      uint16_t* row = buf + (size_t)yt * (size_t)SCALED_W;
      for (int xi = x0; xi <= x1; ++xi) row[xi] = color;
    }
    if (yb >= y0 && yb != yt) {
      uint16_t* row = buf + (size_t)yb * (size_t)SCALED_W;
      for (int xi = x0; xi <= x1; ++xi) row[xi] = color;
    }
    int xl = x0 + t;
    int xr = x1 - t;
    if (xl <= x1) {
      for (int yi = y0; yi <= y1; ++yi) buf[(size_t)yi * (size_t)SCALED_W + (size_t)xl] = color;
    }
    if (xr >= x0 && xr != xl) {
      for (int yi = y0; yi <= y1; ++yi) buf[(size_t)yi * (size_t)SCALED_W + (size_t)xr] = color;
    }
  }
}

static bool computeZoom3LocatorRectScaled(int* outX, int* outY, int* outW, int* outH) {
  if (!outX || !outY || !outW || !outH) return false;
  if (!s_weatherGeoValid) return false;

  float baseW, baseS, baseE, baseN;
  getActiveWeatherBbox(&baseW, &baseS, &baseE, &baseN);
  float lonSpan = baseE - baseW;
  float latSpan = baseN - baseS;
  if (lonSpan <= 0.0001f || latSpan <= 0.0001f) return false;

  float zoomW, zoomS, zoomE, zoomN;
  computeBboxFromCenterKm(s_weatherCenterLat, s_weatherCenterLon,
                          ZOOM3_FINAL_W_KM, ZOOM3_FINAL_H_KM,
                          &zoomW, &zoomS, &zoomE, &zoomN);

  auto clamp01 = [](float v) -> float {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  };

  float fx0 = clamp01((zoomW - baseW) / lonSpan);
  float fx1 = clamp01((zoomE - baseW) / lonSpan);
  float fy0 = clamp01((baseN - zoomN) / latSpan);
  float fy1 = clamp01((baseN - zoomS) / latSpan);
  if (fx1 <= fx0 || fy1 <= fy0) return false;

  int x0 = (int)floorf(fx0 * (float)SCALED_W);
  int x1 = (int)ceilf (fx1 * (float)SCALED_W) - 1;
  int y0 = (int)floorf(fy0 * (float)SCALED_H);
  int y1 = (int)ceilf (fy1 * (float)SCALED_H) - 1;

  if (x0 < 0) x0 = 0;
  if (x1 >= SCALED_W) x1 = SCALED_W - 1;
  int minY = SCALED_TOP_BAR_H;
  int maxY = SCALED_H - SCALED_BAR_H - 1;
  if (y0 < minY) y0 = minY;
  if (y1 > maxY) y1 = maxY;
  if (x1 - x0 < 3 || y1 - y0 < 3) return false;

  *outX = x0;
  *outY = y0;
  *outW = (x1 - x0) + 1;
  *outH = (y1 - y0) + 1;
  return true;
}

static bool scaledFrameLooksHoldBlockCorrupted() {
  if (!s_frameDisplayBuf) return false;

  const int mcu = 16;
  const int topSkip = SCALED_TOP_BAR_H;
  const int botSkip = SCALED_BAR_H;
  const int usableY0 = topSkip;
  const int usableY1 = SCALED_H - botSkip;
  const int usableH = usableY1 - usableY0;
  if (usableH < mcu * 3 || SCALED_W < mcu * 3) return false;

  const int bCols = SCALED_W / mcu;
  const int bRows = usableH / mcu;
  if (bCols < 3 || bRows < 3) return false;

  bool zmap[32][32] = {};
  if (bRows > 32 || bCols > 32) return false;

  for (int br = 1; br < bRows - 1; ++br) {
    for (int bc = 1; bc < bCols - 1; ++bc) {
      bool allZero = true;
      for (int dy = 0; dy < mcu && allZero; ++dy) {
        int py = usableY0 + br * mcu + dy;
        const uint16_t* row = s_frameDisplayBuf + (size_t)py * (size_t)SCALED_W + (size_t)(bc * mcu);
        for (int dx = 0; dx < mcu; ++dx) {
          if (row[dx] != 0) { allZero = false; break; }
        }
      }
      zmap[br][bc] = allZero;
    }
  }

  const int kFullRowThreshold = bCols - 4;
  for (int br = 1; br < bRows - 1; ++br) {
    int zeroCount = 0;
    for (int bc = 1; bc < bCols - 1; ++bc) {
      if (zmap[br][bc]) zeroCount++;
    }
    if (zeroCount >= kFullRowThreshold) return true;
  }

  const int kHalfRow = (bCols - 2) / 2;
  for (int br = 1; br < bRows - 2; ++br) {
    int z0 = 0, z1 = 0;
    for (int bc = 1; bc < bCols - 1; ++bc) {
      if (zmap[br][bc]) z0++;
      if (zmap[br + 1][bc]) z1++;
    }
    if (z0 >= kHalfRow && z1 >= kHalfRow) return true;
  }

  for (int br = 1; br < bRows - 1; ++br) {
    for (int bc = 1; bc < bCols - 1; ++bc) {
      if (!zmap[br][bc]) continue;
      int validNeighbors = 0;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if ((dr | dc) == 0) continue;
          if (!zmap[br + dr][bc + dc]) validNeighbors++;
        }
      }
      if (validNeighbors >= 4) return true;
    }
  }

  return false;
}

static bool currentScaledPlaybackFrameLooksCorrupted() {
  return scaledFrameLooksFreezeBlockCorrupted() ||
         scaledFrameLooksHoldBlockCorrupted();
}

static bool currentScaledFreezeFrameLooksCorrupted() {
  if (scaledFrameLooksFreezeBlockCorrupted()) {
    appendDiagLog("freeze-check: freeze-block\n");
    return true;
  }
  if (scaledFrameLooksHoldBlockCorrupted()) {
    appendDiagLog("freeze-check: hold-block\n");
    return true;
  }
  return false;
}

static bool tryShowCleanFreezeFrameByIdx(int idx) {
  if (idx < 0 || idx >= frameCount) return false;
  if (showFrame(idx) && !currentScaledFreezeFrameLooksCorrupted()) {
    return true;
  }
  return false;
}

static void runFreezeZoom3LocatorCue(uint32_t holdMs) {
  if (holdMs == 0) return;

  const uint32_t initialDelayMs = (holdMs >= 1000U) ? 1000U : holdMs;
  delay(initialDelayMs);
  if (holdMs <= initialDelayMs) return;

  uint32_t remainingMs = holdMs - initialDelayMs;
  if (!s_frameDisplayBuf) {
    delay(remainingMs);
    return;
  }

  int rx, ry, rw, rh;
  if (!computeZoom3LocatorRectScaled(&rx, &ry, &rw, &rh)) {
    delay(remainingMs);
    return;
  }
  size_t saveBytes = (size_t)rw * (size_t)rh * 2U;
  if (saveBytes == 0 || saveBytes > DL_BUF_BYTES) {
    delay(remainingMs);
    return;
  }
  uint16_t* saved = (uint16_t*)s_dlBuf;
  for (int row = 0; row < rh; ++row) {
    memcpy(saved + (size_t)row * (size_t)rw,
           s_frameDisplayBuf + (size_t)(ry + row) * (size_t)SCALED_W + (size_t)rx,
           (size_t)rw * 2U);
  }

  const uint32_t flashSlotMs = (remainingMs >= 500U) ? 125U : (remainingMs / 4U);
  if (flashSlotMs == 0) {
    delay(remainingMs);
    return;
  }

  auto restoreRect = [&]() {
    for (int row = 0; row < rh; ++row) {
      memcpy(s_frameDisplayBuf + (size_t)(ry + row) * (size_t)SCALED_W + (size_t)rx,
             saved + (size_t)row * (size_t)rw,
             (size_t)rw * 2U);
    }
  };

  uint32_t consumedMs = 0;
  for (int i = 0; i < 4; ++i) {
    restoreRect();
    if ((i & 1) == 0) {
      drawScaledOutlineRect(s_frameDisplayBuf, rx, ry, rw, rh, 0xF800);
    }
    presentScaledBuf(s_frameDisplayBuf);
    delay(flashSlotMs);
    consumedMs += flashSlotMs;
  }

  restoreRect();
  presentScaledBuf(s_frameDisplayBuf);
  if (remainingMs > consumedMs) {
    delay(remainingMs - consumedMs);
  }
}

static void refreshZoomSnapshotsForLatestFrame() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (frameCount <= 0) return;
  if (!s_timesLoaded) return;
  bool useUnifiedProgress = syncProgressIsActive();
  if (useUnifiedProgress) syncProgressBeginPhase("zoom/terrain", 12);
  uint32_t zoomStepDone = 0;
  auto zoomStepAdvance = [&](uint32_t inc) {
    if (!useUnifiedProgress) return;
    zoomStepDone += inc;
    if (zoomStepDone > 12U) zoomStepDone = 12U;
    syncProgressSetPhaseProgress((int)zoomStepDone, 12);
  };

  time_t newestUtc = 0;
  int newestIdx = -1;
  for (int i = frameCount - 1; i >= 0; --i) {
    if (s_idx.jpegValid[i] && s_frameTimes[i] > 0) { newestUtc = s_frameTimes[i]; newestIdx = i; break; }
  }
  if (newestUtc <= 0) return;
  bool zoomWeatherAssetsUsable = zoomWeatherSnapshotsPresentAndUsable();
  bool refreshWeatherZooms = s_zoomWeatherRefreshNeeded || !zoomWeatherAssetsUsable;
  if (!refreshWeatherZooms) {
    Serial.println("zoom current; terrain only");
  } else {
    Serial.printf("zoom refresh weather=%d assets=%d\n",
                  (int)s_zoomWeatherRefreshNeeded, (int)zoomWeatherAssetsUsable);
  }
  bool geoFallback = !s_weatherGeoValid;
  if (geoFallback) {
    Serial.println("zoom refresh: no geo, using weather bbox center");
  }

  float baseW, baseS, baseE, baseN;
  getActiveWeatherBbox(&baseW, &baseS, &baseE, &baseN);
  float centerLat = s_weatherCenterLat;
  float centerLon = s_weatherCenterLon;
  if (geoFallback) {
    centerLat = (baseS + baseN) * 0.5f;
    centerLon = (baseW + baseE) * 0.5f;
  }
  float cosLat = cosf(centerLat * 0.01745329252f);
  if (cosLat < 0.2f) cosLat = 0.2f;
  float baseWKm = fabsf(baseE - baseW) * 111.32f * cosLat;
  float baseHKm = fabsf(baseN - baseS) * 111.32f;
  if (baseWKm < 25.0f) baseWKm = 25.0f;
  if (baseHKm < 25.0f) baseHKm = 25.0f;

  // Final zoom floor: keep source-zoom snapshots within a more realistic GOES
  // resolution range to avoid extreme pixelation on the 320x172 display.
  const float finalWKm = ZOOM3_FINAL_W_KM;
  const float finalHKm = ZOOM3_FINAL_H_KM;
  float zoom2WKm = sqrtf(baseWKm * finalWKm);
  float zoom2HKm = sqrtf(baseHKm * finalHKm);
  float zoom1WKm = sqrtf(baseWKm * zoom2WKm);
  float zoom1HKm = sqrtf(baseHKm * zoom2HKm);

  struct ZoomJob { const char* path; float wKm; float hKm; };
  ZoomJob jobs[3] = {
    { ZOOM1_FILE, zoom1WKm, zoom1HKm },
    { ZOOM2_FILE, zoom2WKm, zoom2HKm },
    { ZOOM3_FILE, finalWKm, finalHKm },
  };

  // Keep existing snapshots until replacements download successfully. The fetch
  // helpers overwrite individual files atomically enough for our use (remove+write),
  // which preserves prior zoom stages if a later fetch fails.

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setReuse(true);

  bool zoomsRefreshedOk = true;
  if (refreshWeatherZooms) {
    const int maxBackSteps = 1;
    const time_t cadenceSec = (time_t)activeCadenceMin() * 60;
    bool installed = false;
    for (int back = 0; back <= maxBackSteps && !installed; back++) {
      time_t candidateUtc = newestUtc - ((time_t)back * cadenceSec);
      bool thisCandidateOk = true;
      for (int i = 0; i < 3; ++i) {
        float w, s, e, n;
        computeBboxFromCenterKm(centerLat, centerLon, jobs[i].wKm, jobs[i].hKm, &w, &s, &e, &n);
        size_t outBytes = 0;
        bool stageOk = false;
        const int widths[2]  = { ZOOM_FETCH_W, DISP_W };
        const int heights[2] = { ZOOM_FETCH_H, DISP_H };
        for (int pass = 0; pass < 2 && !stageOk; ++pass) {
          stageOk = installValidatedZoomSnapshotAtBbox(http, client, candidateUtc,
                                                       jobs[i].path, w, s, e, n,
                                                       widths[pass], heights[pass],
                                                       &outBytes);
          if (!stageOk && pass == 0) {
            Serial.printf("zoom z%d fallback 1x\n", i + 1);
          }
        }
        if (!stageOk) {
          Serial.printf("zoom z%d fail b=%d\n", i + 1, back);
          appendDiagLog("zoom: z%d fail back=%d\n", i + 1, back);
          thisCandidateOk = false;
          break;
        }
        zoomStepAdvance(2);
      }
      if (thisCandidateOk) {
        installed = true;
        if (back > 0) {
          Serial.printf("zoom ts fallback b=%d cad=%d\n", back, (int)activeCadenceMin());
          appendDiagLog("zoom: ts-fallback back=%d cad=%d\n", back, (int)activeCadenceMin());
        }
      }
    }
    zoomsRefreshedOk = installed;
    if (refreshWeatherZooms) {
      appendDiagLog("zoom: weather %s millis=%lu ms\n", zoomsRefreshedOk ? "ok" : "fail", millis());
    }
  }
  if (refreshWeatherZooms && zoomsRefreshedOk) {
    writeZoomSnapshotMeta(newestUtc);
    s_zoomWeatherRefreshNeeded = false;
  } else if (refreshWeatherZooms) {
    // Keep retry flag set so next sync can retry weather zooms.
    s_zoomWeatherRefreshNeeded = true;
  } else {
    s_zoomWeatherRefreshNeeded = false;
  }
  if (refreshWeatherZooms) zoomStepAdvance(1);

  // Precompute filtered raw zoom frames (gentle low-pass) once per snapshot set.
  if (zoomsRefreshedOk) {
    ensureFilteredZoomRaw(ZOOM1_FILE, refreshWeatherZooms);
    zoomStepAdvance(1);
    ensureFilteredZoomRaw(ZOOM2_FILE, refreshWeatherZooms);
    zoomStepAdvance(1);
    ensureFilteredZoomRaw(ZOOM3_FILE, refreshWeatherZooms);
    zoomStepAdvance(1);
  }

  // Download terrain z3 (BlueMarble day / VIIRS Black Marble night) for the
  // same bbox as weather z3, used by the terrain transition before clock overlay.
  {
    float tw, ts, te, tn;
    computeBboxFromCenterKm(centerLat, centerLon, finalWKm, finalHKm, &tw, &ts, &te, &tn);
    bool terrainSnapOk = downloadTerrainSnapshotToPathAtBbox(http, client, newestUtc, tw, ts, te, tn);
    appendDiagLog("terrain-snap: %s millis=%lu ms\n", terrainSnapOk ? "ok" : "fail", millis());
    if (terrainSnapOk) {
      SD.remove(ZOOM_TERRAIN_DAY_RAW);    // invalidate raws; rebuild at playback
      SD.remove(ZOOM_TERRAIN_NIGHT_RAW);
    }
    zoomStepAdvance(3);
  }
  if (useUnifiedProgress) syncProgressCompletePhase();

}

// ─────────────────────────────────────────────────────────────
//  Download phase — sequential with connection reuse.
//  One SSL handshake for the full weather window (frame count derives from
//  HOURS_BACK and active cadence, clamped to MAX_FRAMES).
// ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────
//  Unified weather frame sync — one function replaces downloadFrames
//  + syncFramesRolling + all branch logic.
// ─────────────────────────────────────────────────────────────
static void syncWeatherFrames() {
  if (!syncProgressIsActive()) showMessage("Syncing time...", "pool.ntp.org");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm ti;
  int tries = 0;
  while (!getLocalTime(&ti, 1000) && tries++ < 20) {}
  time_t now = time(nullptr);
  Serial.printf("UTC: %s\n", ctime(&now));
  appendDiagLog("sync: ntp utc=%lld millis=%lu ms\n", (long long)now, millis());
  refreshDisplayLocationTimeFromIpInfo();
  if (syncProgressIsActive()) syncProgressCompletePhase();

  const int cadenceMin = activeCadenceMin();
  const int lagHours = activeLagHours();
  const int totalFrames = targetFrameCount();
  const time_t cadenceSec = (time_t)cadenceMin * 60;
  const time_t fetchEnd = roundToCadence(now - (time_t)(lagHours * 3600));
  const time_t fetchStart = fetchEnd - (time_t)((totalFrames - 1) * cadenceMin * 60);

  // Load current index
  loadIndex();
  int oldCount = (int)s_idx.count;

  // Check view mismatch
  bool viewMatches = true;
  if (s_idx.count > 0) {
    viewMatches = currentWeatherViewMatchesCache();
  }

  // Compute shift and slot reuse
  int shift = 0;
  bool fullRefresh = false;
  if (!viewMatches && s_idx.count > 0) {
    fullRefresh = true;
    appendDiagLog("sync: full-refresh reason=view\n");
  } else if (s_idx.count > 0 && s_idx.times[0] != 0) {
    time_t delta = fetchStart - s_idx.times[0];
    if (delta > 0 && cadenceSec > 0 && (delta % cadenceSec) == 0) {
      shift = (int)(delta / cadenceSec);
      if (shift >= (int)s_idx.count) fullRefresh = true;
    } else if (delta != 0) {
      fullRefresh = true;
    }
  } else if (s_idx.count == 0) {
    fullRefresh = true;
  }

  if (fullRefresh) {
    memset(&s_idx, 0, sizeof(s_idx));
    s_idx.magic = INDEX_MAGIC;
    s_idx.head = 0;
    s_idx.count = 0;
    shift = 0;
  }

  // Determine which slots need download
  bool needsDownload[MAX_FRAMES];
  memset(needsDownload, 0, sizeof(needsDownload));
  int downloadCount = 0;

  if (fullRefresh) {
    // Everything needs download
    s_idx.count = (uint16_t)totalFrames;
    for (int i = 0; i < totalFrames; i++) {
      s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
      s_idx.jpegValid[i] = 0;
      s_idx.rawValid[i] = 0;
      s_idx.jpegLen[i] = 0;
      needsDownload[i] = true;
      downloadCount++;
    }
  } else if (shift > 0) {
    // Advance head, keep overlapping slots
    int oldUsable = (int)s_idx.count - shift;
    if (oldUsable < 0) oldUsable = 0;

    // Build new index
    FrameStoreIndex newIdx;
    memset(&newIdx, 0, sizeof(newIdx));
    newIdx.magic = INDEX_MAGIC;
    newIdx.head = (s_idx.head + (uint16_t)shift) % MAX_FRAMES;
    newIdx.count = (uint16_t)totalFrames;

    for (int i = 0; i < totalFrames; i++) {
      time_t t = fetchStart + (time_t)(i * cadenceSec);
      newIdx.times[i] = t;

      // Check if this slot was in old index
      int oldLogical = i + shift - ((int)s_idx.count - oldUsable);
      // Actually: old slot for this time = shift + i if within old range
      int oldSlot = shift + i;
      if (oldSlot >= 0 && oldSlot < (int)s_idx.count && i < oldUsable &&
          s_idx.times[oldSlot] == t && s_idx.jpegValid[oldSlot]) {
        // Reuse this slot — physical slot is the same (circular)
        newIdx.jpegLen[i] = s_idx.jpegLen[oldSlot];
        newIdx.jpegValid[i] = 1;
        newIdx.rawValid[i] = s_idx.rawValid[oldSlot];
      } else {
        needsDownload[i] = true;
        downloadCount++;
      }
    }
    memcpy(&s_idx, &newIdx, sizeof(s_idx));
  } else {
    // shift == 0, check for gaps and update count if needed
    if ((int)s_idx.count < totalFrames) {
      // Need more frames at the tail
      for (int i = (int)s_idx.count; i < totalFrames; i++) {
        s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
        s_idx.jpegValid[i] = 0;
        s_idx.rawValid[i] = 0;
        needsDownload[i] = true;
        downloadCount++;
      }
      s_idx.count = (uint16_t)totalFrames;
    }
    // Check for invalid slots
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (!s_idx.jpegValid[i]) {
        s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
        needsDownload[i] = true;
        downloadCount++;
      }
    }
  }

  appendDiagLog("sync: totalFrames=%d shift=%d downloadCount=%d oldCount=%d\n",
                totalFrames, shift, downloadCount, oldCount);

  if (downloadCount == 0) {
    // Check if raw is all built
    bool allRawValid = true;
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (s_idx.jpegValid[i] && !s_idx.rawValid[i]) { allRawValid = false; break; }
    }
    writeIndex();
    s_zoomWeatherRefreshNeeded = false;
    if (s_idx.count > 0) {
      s_zoomSnapshotsRefreshPending = !zoomSnapshotsCurrentAndUsable(s_idx.times[s_idx.count - 1]);
    }
    if (allRawValid) {
      if (syncProgressIsActive()) {
        syncProgressBeginPhase("cache", (uint32_t)totalFrames);
        syncProgressCompletePhase();
        syncProgressBeginPhase("raw", (uint32_t)totalFrames);
        syncProgressCompletePhase();
      }
      showMessage("Cache current", "No downloads needed");
      delay(700);
      appendDiagLog("sync: cache current, no downloads\n");
      return;
    }
    // Need raw rebuild
    if (syncProgressIsActive()) {
      syncProgressBeginPhase("cache", (uint32_t)totalFrames);
      syncProgressCompletePhase();
    }
    rebuildRawFromStored();
    appendDiagLog("sync: cache current, rebuilt raw\n");
    return;
  }

  // Fetch GIBS available times
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setReuse(true);
  fetchGibsAvailableTimes(client, fetchStart, fetchEnd);

  // Download frames
  if (!syncProgressIsActive()) showMessage("Updating cache...", "NASA GIBS");
  if (syncProgressIsActive()) syncProgressBeginPhase("cache", (uint32_t)totalFrames);

  float bboxWest, bboxSouth, bboxEast, bboxNorth;
  getActiveWeatherBbox(&bboxWest, &bboxSouth, &bboxEast, &bboxNorth);

  int saved = 0;
  int skipped = 0;
  int dlFail = 0;
  char url[512];

  for (int i = 0; i < (int)s_idx.count; i++) {
    showProgress(i + 1, (int)s_idx.count, "cache");
    yield();

    if (!needsDownload[i]) {
      saved++;
      // If JPEG valid but raw invalid, decode from RAM
      if (s_idx.jpegValid[i] && !s_idx.rawValid[i]) {
        size_t readLen = 0;
        if (readJpegFromSlot(i, s_dlBuf, &readLen) && readLen > 0) {
          decodeAndWriteRawSlot(i, readLen);
        }
      }
      continue;
    }

    time_t t = s_idx.times[i];
    if (t <= 0) { skipped++; continue; }

    // Build URL and fetch
    const int fetchCadenceSec = max(60, cadenceMin * 60);
    time_t snappedTime = snapToNearestGibsTime(t, 2 * fetchCadenceSec);
    const time_t secondOffsets[] = {0, -60, 60,
                                    -(time_t)fetchCadenceSec, (time_t)fetchCadenceSec,
                                    -2*(time_t)fetchCadenceSec, 2*(time_t)fetchCadenceSec};
    int stepCount;
    if (snappedTime > 0) {
      stepCount = 1;
    } else if (s_gibsAvailCount > 0) {
      skipped++;
      continue;
    } else {
      stepCount = 7;
    }

    bool fetchedOk = false;
    size_t jpegLen = 0;
    for (int si = 0; si < stepCount && !fetchedOk; ++si) {
      time_t candT = (snappedTime > 0) ? snappedTime : (t + secondOffsets[si]);
      if (candT <= 0) continue;
      if (!buildWeatherFrameUrl(url, sizeof(url), candT,
                                bboxWest, bboxSouth, bboxEast, bboxNorth)) continue;

      for (int attempt = 0; attempt < 2; ++attempt) {
        http.begin(client, url);
        http.setTimeout(5000);
        int code = http.GET();
        if (code != HTTP_CODE_OK) { http.end(); continue; }
        if (!readHttpJpegBodyToDlBuf(http, "SYNC", &jpegLen)) continue;
        if (!validateBufferedWeatherFrameJpeg(jpegLen, "SYNC")) continue;
        if (jpegEffectiveLength(s_dlBuf, jpegLen) == 0) continue;
        fetchedOk = true;
        break;
      }
    }

    if (!fetchedOk) {
      dlFail++;
      continue;
    }

    // Additional validators
    if (spriteLooksHoldFrameBlockCorrupted() ||
        spriteLooksCyanWhiteBlockCorrupted() ||
        spriteLooksBottomBandJunkCorrupted()) {
      dlFail++;
      continue;
    }

    // Size outlier check using index jpegLen
    if ((int)s_idx.count >= 3) {
      int refA = -1, refB = -1;
      if (i <= 0) { refA = 1; refB = 2; }
      else if (i >= (int)s_idx.count - 1) { refA = (int)s_idx.count - 2; refB = (int)s_idx.count - 3; }
      else { refA = i - 1; refB = i + 1; }
      if (refA >= 0 && refB >= 0 && refA < (int)s_idx.count && refB < (int)s_idx.count &&
          s_idx.jpegValid[refA] && s_idx.jpegValid[refB]) {
        int sizeA = (int)s_idx.jpegLen[refA];
        int sizeB = (int)s_idx.jpegLen[refB];
        if (sizeA > 0 && sizeB > 0 && abs(sizeA - sizeB) <= 1500) {
          int refSize = (sizeA + sizeB) / 2;
          if ((int)jpegLen + 900 < refSize) {
            Serial.printf("SYNC sz-outlier i=%d sz=%u ref=%d\n", i, (unsigned)jpegLen, refSize);
            dlFail++;
            continue;
          }
        }
      }
    }

    // Write JPEG to slot first (so we can read it back after semantic check)
    if (!writeJpegToSlot(i, s_dlBuf, jpegLen)) {
      dlFail++;
      continue;
    }

    // Semantic outlier check — compare candidate sprite signature with neighbors.
    // Candidate sprite is still in the sprite buffer from validateBufferedWeatherFrameJpeg.
    // Neighbor reads will clobber s_dlBuf and sprite, but the candidate JPEG is
    // already safe in frames.bin.
    bool semanticReject = false;
    if ((int)s_idx.count >= 3 && ensureSprite()) {
      WeatherSemanticSignature candSig = {};
      if (captureWeatherSemanticSignatureFromSprite(&candSig)) {
        int refA = -1, refB = -1;
        if (i <= 0) { refA = 1; refB = 2; }
        else if (i >= (int)s_idx.count - 1) { refA = (int)s_idx.count - 2; refB = (int)s_idx.count - 3; }
        else { refA = i - 1; refB = i + 1; }
        if (refA >= 0 && refB >= 0 && refA < (int)s_idx.count && refB < (int)s_idx.count &&
            s_idx.jpegValid[refA] && s_idx.jpegValid[refB]) {
          // Decode neighbors from frames.bin to capture their signatures
          WeatherSemanticSignature refSigA = {}, refSigB = {};
          bool gotA = false, gotB = false;
          auto decodeSlotSig = [&](int slot, WeatherSemanticSignature* sig) -> bool {
            size_t rLen = 0;
            if (!readJpegFromSlot(slot, s_dlBuf, &rLen) || rLen == 0) return false;
            sprite.fillScreen(TFT_BLACK);
            LovyanGFX* prev = g_drawTarget;
            g_drawTarget = &sprite;
            resetJpegDrawStats();
            JPEGDEC tj;
            bool ok = false;
            if (tj.openRAM(s_dlBuf, (int)rLen, jpegDraw)) {
              tj.setPixelType(RGB565_BIG_ENDIAN);
              ok = tj.decode(0, 0, 0);
              tj.close();
            }
            g_drawTarget = prev;
            return ok && captureWeatherSemanticSignatureFromSprite(sig);
          };
          gotA = decodeSlotSig(refA, &refSigA);
          gotB = decodeSlotSig(refB, &refSigB);

          if (gotA && gotB) {
            int candA = weatherSemanticDistance(candSig, refSigA);
            int candB = weatherSemanticDistance(candSig, refSigB);
            int refAB = weatherSemanticDistance(refSigA, refSigB);
            int refR = ((int)refSigA.meanR + (int)refSigB.meanR) / 2;
            int refG = ((int)refSigA.meanG + (int)refSigB.meanG) / 2;
            int refBM = ((int)refSigA.meanB + (int)refSigB.meanB) / 2;
            int candColorJump = abs((int)candSig.meanR - refR) +
                                abs((int)candSig.meanG - refG) +
                                abs((int)candSig.meanB - refBM);
            int refMotion = abs((int)refSigA.meanR - (int)refSigB.meanR) +
                            abs((int)refSigA.meanG - (int)refSigB.meanG) +
                            abs((int)refSigA.meanB - (int)refSigB.meanB);
            bool strongDist = (candA >= max(8, refAB + 7) && candB >= max(8, refAB + 7));
            bool moderateDist = (candA >= max(6, refAB + 5) && candB >= max(6, refAB + 5));
            int refCyan = max((int)refSigA.cyanTiles, (int)refSigB.cyanTiles);
            bool colorLift = (candSig.meanG >= refG + 3 && candSig.meanB >= refBM + 4);
            bool cyanLift = (candSig.cyanTiles >= refCyan + 2 && candSig.cyanTiles >= 3);
            bool colorJumpOut = (candColorJump >= max(10, refMotion + 6));
            int refSize = ((int)s_idx.jpegLen[refA] + (int)s_idx.jpegLen[refB]) / 2;
            bool sizeOut = (refSize > 0 && (int)jpegLen + 900 < refSize);
            if (moderateDist && (colorLift && (sizeOut || strongDist))) semanticReject = true;
            if (moderateDist && (cyanLift || colorJumpOut)) semanticReject = true;
            if (semanticReject) {
              Serial.printf("SYNC sem-outlier i=%d cA=%d cB=%d rr=%d cj=%d rm=%d\n",
                            i, candA, candB, refAB, candColorJump, refMotion);
              appendDiagLog("sync: sem-outlier i=%d\n", i);
            }
          }
        }
      }
    }
    if (semanticReject) {
      // Undo the JPEG write by marking slot invalid
      s_idx.jpegValid[i] = 0;
      s_idx.jpegLen[i] = 0;
      dlFail++;
      continue;
    }

    // Re-read candidate from slot and decode for raw (s_dlBuf/sprite were clobbered
    // by semantic check neighbor reads, or are still valid if check was skipped)
    {
      size_t rereadLen = 0;
      if (!readJpegFromSlot(i, s_dlBuf, &rereadLen) || rereadLen == 0) {
        s_idx.jpegValid[i] = 0;
        dlFail++;
        continue;
      }
      decodeAndWriteRawSlot(i, rereadLen);
    }

    saved++;
    Serial.printf("SYNC %d/%d OK %u B\n", i + 1, (int)s_idx.count, (unsigned)jpegLen);
    delay(20);  // pace requests
  }
  if (syncProgressIsActive()) syncProgressCompletePhase();

  // Gap-fill invalid raw slots by copying raw data from nearest valid neighbor
  int filled = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) continue;
    // Find nearest valid neighbor
    int src = -1;
    for (int d = 1; d < (int)s_idx.count && src < 0; d++) {
      if (i - d >= 0 && s_idx.rawValid[i - d]) src = i - d;
      else if (i + d < (int)s_idx.count && s_idx.rawValid[i + d]) src = i + d;
    }
    if (src >= 0 && s_frameDisplayBuf) {
      // Copy raw data from neighbor's physical slot to this slot
      int srcPhys = ((int)s_idx.head + src) % MAX_FRAMES;
      int dstPhys = ((int)s_idx.head + i)   % MAX_FRAMES;
      uint32_t srcOff = (uint32_t)srcPhys * SCALED_FRAME_BYTES;
      uint32_t dstOff = (uint32_t)dstPhys * SCALED_FRAME_BYTES;

      File sf = SD.open(SD_ROOT "/frames/stream.raw", FILE_READ);
      if (sf) {
        sf.seek(srcOff);
        size_t got = sf.read((uint8_t*)s_frameDisplayBuf, SCALED_FRAME_BYTES);
        sf.close();
        if (got == SCALED_FRAME_BYTES) {
          if (writeRawToSlot(i, (const uint8_t*)s_frameDisplayBuf)) {
            filled++;
          }
        }
      }
    }
  }

  // Commit index
  writeIndex();
  writeCurrentWeatherViewMeta();

  s_zoomWeatherRefreshNeeded = (downloadCount > 0 && saved > 0);
  if (s_idx.count > 0) {
    s_zoomSnapshotsRefreshPending = !zoomSnapshotsCurrentAndUsable(s_idx.times[s_idx.count - 1]);
  }

  // Rebuild filtered zoom raws
  rebuildFilteredZoomRawsFromCache();

  if (syncProgressIsActive()) {
    syncProgressBeginPhase("raw", 1);
    syncProgressCompletePhase();
  }

  Serial.printf("sync done saved=%d skip=%d fail=%d fill=%d\n", saved, skipped, dlFail, filled);
  appendDiagLog("sync: done saved=%d skip=%d fail=%d fill=%d total=%d millis=%lu ms\n",
                saved, skipped, dlFail, filled, (int)s_idx.count, millis());
}

static void rebuildRawFromStored() {
  if (!ensureSprite() || !s_frameDisplayBuf) return;
  if (s_idx.count <= 0) return;

  bool useUnifiedProgress = syncProgressIsActive();
  if (useUnifiedProgress) {
    syncProgressBeginPhase("raw", (uint32_t)s_idx.count);
  } else {
    showMessage("Building raw cache...", "Predecode for smooth playback");
  }

  if (s_streamFile) { s_streamFile.close(); }
  s_streamReady = false;
  invalidateValidIdxCache();

  int built = 0;
  int decFail = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    showProgress(i + 1, (int)s_idx.count, "raw");
    yield();

    if (s_idx.rawValid[i]) { built++; continue; }
    if (!s_idx.jpegValid[i]) continue;

    size_t readLen = 0;
    if (!readJpegFromSlot(i, s_dlBuf, &readLen) || readLen == 0) {
      decFail++;
      continue;
    }

    if (decodeAndWriteRawSlot(i, readLen)) {
      built++;
    } else {
      decFail++;
    }
  }

  if (useUnifiedProgress) syncProgressCompletePhase();

  // Gap-fill
  int filled = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) continue;
    int src = -1;
    for (int d = 1; d < (int)s_idx.count && src < 0; d++) {
      if (i - d >= 0 && s_idx.rawValid[i - d]) src = i - d;
      else if (i + d < (int)s_idx.count && s_idx.rawValid[i + d]) src = i + d;
    }
    if (src >= 0) {
      s_idx.rawValid[i] = 1;
      s_streamValid[i] = 1;
      filled++;
    }
  }

  writeIndex();
  rebuildFilteredZoomRawsFromCache();

  Serial.printf("raw-rebuild: built=%d dec=%d fill=%d\n", built, decFail, filled);
  appendDiagLog("raw-rebuild: built=%d dec=%d fill=%d count=%d\n",
                built, decFail, filled, (int)s_idx.count);
}

// ─────────────────────────────────────────────────────────────
//  Enter sleep (deep sleep if BOOT pin supports it; light sleep fallback on C6)
// ─────────────────────────────────────────────────────────────
static void goToSleep(bool buttonOnly = false) {
  Serial.printf("Sleeping %d h...\n", SLEEP_HOURS);
  s_buttonSleepTransition = true;
  closeStream();

  // Mute amp before sleep to prevent wake-pop. The I2S clock stops during
  // light sleep; with amp enabled the bias collapse on wake injects a pop.
  // Clearing s_audioPathPrimed forces a clean re-prime (silence→enable→settle)
  // on the next cue after wake.
  if (s_audioReady) {
    es8311_voice_mute(s_audioCodec, true);
    digitalWrite(46, LOW);
    s_audioPathPrimed = false;
  }

  // Dim backlight before sleep
  if (buttonOnly) {
#if BOARD_IS_AMOLED_206
    if (s_amoledOut) s_amoledOut->setBrightness(0);
#else
    tft.setBrightness(0);
#endif
  } else {
    const int fadeStep = 5;
    const uint32_t fadeDelayMs = 10U;
    for (int b = 255; b >= 0; b -= fadeStep) {
#if BOARD_IS_AMOLED_206
      if (s_amoledOut) s_amoledOut->setBrightness((uint8_t)b);
#else
      tft.setBrightness(b);
#endif
      delay(fadeDelayMs);
    }
  }
#if BOARD_IS_AMOLED_206
  if (s_amoledOut) s_amoledOut->fillScreen(0x0000);
  if (s_amoledOut) s_amoledOut->displayOff();
#else
  tft.fillScreen(TFT_BLACK);
#endif

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);

  if (!buttonOnly && s_autoUpdateInSleep) {
    uint64_t sleepUs = (uint64_t)SLEEP_HOURS * 3600ULL * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleepUs);
  }

#if BOARD_IS_AMOLED_206
  gpio_num_t bootPin = (gpio_num_t)BOOT_BTN_GPIO;
  gpio_pullup_en(bootPin);
  gpio_wakeup_enable(bootPin, GPIO_INTR_LOW_LEVEL);
  if (s_shakeToWakeEnabled && s_qmiInitialized && s_sleepModeEnabled) {
    // QMI8658 WoM interrupt TOGGLES on each event — it does not latch high or low.
    // Level is arbitrary ("jump transition state" per the official example).
    // Strategy: read current INT1 level, arm the OPPOSITE level.
    // The next wrist flick toggles INT1 to the armed level → ESP32 wakes.
    gpio_num_t imuPin = (gpio_num_t)QMI8658_INT1;
    // GPIO21 is shared with AMOLED_PWR_EN. Firmware holds it OUTPUT HIGH for
    // display power — QMI8658 push-pull output cannot toggle an actively-driven
    // output pin, and digitalRead() on an output returns the ESP32's own register,
    // not the physical level. Release the driver so QMI8658 can freely drive the
    // pin and digitalRead() reflects the actual state.
    gpio_reset_pin(imuPin);
    pinMode(QMI8658_INT1, INPUT_PULLUP);
    int int1Level = digitalRead(QMI8658_INT1);
    gpio_int_type_t wakeOn = (int1Level == LOW) ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL;
    gpio_wakeup_enable(imuPin, wakeOn);
    appendDiagLog("qmi: shake-wake armed INT1=%d wakeOn=%s\n",
                  int1Level, wakeOn == GPIO_INTR_HIGH_LEVEL ? "HIGH" : "LOW");
  }
  esp_sleep_enable_gpio_wakeup();

  // Flush and unmount SD before the SDMMC clock stops during light sleep.
  // Stopping the clock with the filesystem still mounted can leave the FAT
  // dirty, causing large runs of files to return wrong sector data on wake.
  SD_MMC.end();

  // Record when we enter sleep so we can measure elapsed time on WoM wake.
  // esp_timer_get_time() continues counting through light sleep via the RTC.
  uint64_t sleepEntryUs = esp_timer_get_time();

  esp_sleep_wakeup_cause_t wake;
  do {
    esp_err_t sleepErr = esp_light_sleep_start();
    if (sleepErr != ESP_OK) {
      Serial.printf("Light sleep failed (%d)\n", (int)sleepErr);
    }
    wake = esp_sleep_get_wakeup_cause();
    Serial.printf("Wake cause: %d\n", (int)wake);

    // Shake settle window: ignore WoM wakes that happen within
    // s_shakeEntryIgnoreMs of entering sleep. Pressing the sleep button
    // always involves arm motion — without this window the device wakes
    // immediately from that settling motion. After the window any single
    // wrist flick wakes normally. Button press (topButtonPressed()) always
    // bypasses this regardless of timing.
    bool isShakeWake = (wake == ESP_SLEEP_WAKEUP_GPIO) && !topButtonPressed();
    if (isShakeWake && s_shakeToWakeEnabled && s_qmiInitialized) {
      int int1Now = digitalRead(QMI8658_INT1);
      gpio_int_type_t reArmOn = (int1Now == LOW) ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL;

      // Layer 1 — settle window: ignore motion within s_shakeEntryIgnoreMs of
      // entering sleep (covers arm-settling from the button press).
      uint64_t elapsedMs = (esp_timer_get_time() - sleepEntryUs) / 1000ULL;
      if (s_shakeEntryIgnoreMs > 0 && elapsedMs < (uint64_t)s_shakeEntryIgnoreMs) {
        Serial.printf("qmi: shake ignored settle-window elapsed=%llums\n", elapsedMs);
        gpio_wakeup_enable((gpio_num_t)QMI8658_INT1, reArmOn);
        esp_sleep_enable_gpio_wakeup();
        continue;
      }

      // Layer 2 — confirm window: require a 2nd WoM event (INT1 re-toggle)
      // within s_shakeConfirmMs. An elbow bump is a single transient and
      // never produces a 2nd event. A deliberate raise-to-wake motion lasts
      // >250 ms (the hardware blanking period) and reliably fires twice.
      // s_shakeConfirmMs=0 skips this and wakes on any single event.
      if (s_shakeConfirmMs > 0) {
        bool confirmed = false;
        uint32_t deadline = (uint32_t)(esp_timer_get_time() / 1000ULL) + s_shakeConfirmMs;
        while ((int32_t)(deadline - (uint32_t)(esp_timer_get_time() / 1000ULL)) > 0) {
          if (digitalRead(QMI8658_INT1) != int1Now) {
            confirmed = true;
            break;
          }
          delay(5);
        }
        if (!confirmed) {
          Serial.println("qmi: shake no-confirm re-arm");
          gpio_wakeup_enable((gpio_num_t)QMI8658_INT1, reArmOn);
          esp_sleep_enable_gpio_wakeup();
          continue;
        }
      }
    }
    break;  // real wake — fall through to post-wake handling
  } while (true);

  // Remount SD before any file operations (logs, sync, playback).
  SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT);

  if (!buttonOnly && s_autoUpdateInSleep && wake == ESP_SLEEP_WAKEUP_TIMER) {
    if (s_hurricaneMode && s_hurricaneWatchEnabled) {
      hurricaneRecheckAndUpdate();
    }
    if (connectWifiForSync(false)) {
      // Hurricane check on timer wake (if not already in mode)
      if (s_hurricaneWatchEnabled && !s_hurricaneMode) {
        HurricaneInfo hStorms[4]; int hCount = 0;
        if (pollNoaaForHurricane(hStorms, 4, &hCount)) {
          cleanupSuppressedStorms(hStorms, hCount);
          for (int hi = 0; hi < hCount; hi++) {
            if (!isStormSuppressed(hStorms[hi].id)) {
              enterHurricaneMode(hStorms[hi]);
              break;
            }
          }
        }
      }
      syncWeatherFrames();
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(buttonOnly);

  // Restore AMOLED_PWR_EN strong output driver (released for QMI8658 INT1 sharing).
  // INPUT_PULLUP kept the pin ~HIGH during sleep so display register state is intact.
  pinMode(LCD_PWR, OUTPUT);
  digitalWrite(LCD_PWR, HIGH);

  if (s_amoledOut) s_amoledOut->displayOn();
  if (s_amoledOut) s_amoledOut->setBrightness(0xFF);
  s_buttonSleepTransition = false;
  return;
#endif

#if !BOARD_HAS_PHYSICAL_BOOT_WAKE
  esp_deep_sleep_start();
  return;
#else
  // Wake on BOOT button press (active LOW). Some ESP32 targets support direct
  // GPIO deep-sleep wake, others only support the lighter gpio wake path in
  // this sketch. Use deep sleep only when the target exposes the required API.
  gpio_num_t bootPin = (gpio_num_t)BOOT_BTN_GPIO;
  gpio_pullup_en(bootPin);

#if defined(GPIO_IS_DEEP_SLEEP_WAKEUP_VALID_GPIO) && defined(ESP_GPIO_WAKEUP_GPIO_LOW)
  if (GPIO_IS_DEEP_SLEEP_WAKEUP_VALID_GPIO(BOOT_BTN_GPIO)) {
    esp_err_t wakeErr = esp_deep_sleep_enable_gpio_wakeup(1ULL << BOOT_BTN_GPIO,
                                                          ESP_GPIO_WAKEUP_GPIO_LOW);
    if (wakeErr != ESP_OK) {
      Serial.printf("deep wake setup fail %d\n", (int)wakeErr);
    }
    esp_deep_sleep_start();
    // Never returns
    return;
  }
#endif

  Serial.printf("gpio%d no deep wake; light sleep\n",
                BOOT_BTN_GPIO);
  gpio_wakeup_enable(bootPin, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_err_t sleepErr = esp_light_sleep_start();
  if (sleepErr != ESP_OK) {
    Serial.printf("Light sleep failed (%d)\n", (int)sleepErr);
  }

  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake cause: %d\n", (int)wake);

  if (s_autoUpdateInSleep && wake == ESP_SLEEP_WAKEUP_TIMER) {
    if (connectWifiForSync(false)) {
      syncWeatherFrames();
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(false);

#if BOARD_IS_AMOLED_206
  if (s_amoledOut) s_amoledOut->displayOn();
  if (s_amoledOut) s_amoledOut->setBrightness(0xFF);
#else
  tft.setBrightness(255);
#endif
  s_buttonSleepTransition = false;
#endif
}

static void serviceUserButtons() {
  if (s_buttonSleepTransition) return;

#if BOARD_IS_AMOLED_206
  static uint32_t lastPekPollMs = 0;
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastPekPollMs) >= 80U) {
    lastPekPollMs = nowMs;
    int irq2 = readAxp2101Register(0x49);  // INTSTS2
    if (irq2 > 0) {
      writeAxp2101Register(0x49, (uint8_t)irq2);
      if (irq2 & 0x08) {  // PKEY short press
        SD_MMC.end();
        ESP.restart();
      }
    }
  }
#endif

  static bool longHandled = false;
  static uint32_t suppressUntilMs = 0;

  uint32_t now = millis();
  bool allowAction = ((int32_t)(now - suppressUntilMs) >= 0);

  bool stablePressed = false;
  bool releasePending = false;
  uint32_t pressStartMs = 0;
  uint32_t releaseMs = 0;
  portENTER_CRITICAL(&s_topBtnStateMux);
  stablePressed = s_topBtnStablePressed;
  releasePending = s_topBtnReleasePending;
  pressStartMs = s_topBtnPressStartMs;
  releaseMs = s_topBtnReleaseMs;
  if (releasePending) s_topBtnReleasePending = false;
  portEXIT_CRITICAL(&s_topBtnStateMux);

  if (s_topBtnIgnoreUntilRelease) {
    if (!stablePressed) {
      s_topBtnIgnoreUntilRelease = false;
      suppressUntilMs = now + TOP_BTN_SUPPRESS_MS;
    }
    longHandled = false;
    return;
  }

  if (stablePressed && allowAction && !longHandled &&
      pressStartMs > 0 &&
      (uint32_t)(now - pressStartMs) >= TOP_BTN_LONG_PRESS_MS) {
    s_sleepModeEnabled = !s_sleepModeEnabled;
    saveSleepModePreference(s_sleepModeEnabled);
    // Init WoM when toggling INTO sleep mode; it was skipped at boot in always-on mode.
    if (s_sleepModeEnabled && s_shakeToWakeEnabled && !s_qmiInitialized) {
      initQmiShakeToWake();
    }
    longHandled = true;
    suppressUntilMs = now + TOP_BTN_SUPPRESS_MS;
  } else if (releasePending) {
    uint32_t heldMs = (pressStartMs > 0 && releaseMs >= pressStartMs) ? (releaseMs - pressStartMs) : 0U;
    if (allowAction && !longHandled && heldMs >= TOP_BTN_SHORT_PRESS_MS) {
      goToSleep(true);
      suppressUntilMs = millis() + TOP_BTN_SUPPRESS_MS;
    }
    longHandled = false;
  } else if (!stablePressed) {
    longHandled = false;
  }
}

static void applySt7789WarmGamma() {
  static const uint8_t pos[14] = {
    0xD0, 0x02, 0x04, 0x08, 0x0C, 0x2A, 0x36,
    0x48, 0x45, 0x08, 0x12, 0x16, 0x18, 0x1A
  };
  static const uint8_t neg[14] = {
    0xD0, 0x02, 0x04, 0x08, 0x0C, 0x2A, 0x35,
    0x56, 0x4A, 0x10, 0x1E, 0x19, 0x1D, 0x20
  };

  tft.startWrite();
  tft.writecommand(0xE0);
  for (int i = 0; i < 14; ++i) tft.writedata(pos[i]);
  tft.writecommand(0xE1);
  for (int i = 0; i < 14; ++i) tft.writedata(neg[i]);
  tft.endWrite();
}

#if defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
static void applyWaveshareS3St7789Init() {
  static const uint8_t b0[] = {0x00, 0xE8};
  static const uint8_t b2[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
  static const uint8_t d0[] = {0xA4, 0xA1};
  static const uint8_t e0[] = {0xF0, 0x00, 0x04, 0x04, 0x04, 0x05, 0x29, 0x33, 0x3E, 0x38, 0x12, 0x12, 0x28, 0x30};
  static const uint8_t e1[] = {0xF0, 0x07, 0x0A, 0x0D, 0x0B, 0x07, 0x28, 0x33, 0x3E, 0x36, 0x14, 0x14, 0x29, 0x32};

  auto writeList = [](uint8_t cmd, const uint8_t* data, int len) {
    tft.writecommand(cmd);
    for (int i = 0; i < len; ++i) tft.writedata(data[i]);
  };

  tft.startWrite();
  tft.writecommand(0x3A); tft.writedata(0x05);
  writeList(0xB0, b0, (int)(sizeof(b0)));
  writeList(0xB2, b2, (int)(sizeof(b2)));
  tft.writecommand(0xB7); tft.writedata(0x35);
  tft.writecommand(0xBB); tft.writedata(0x35);
  tft.writecommand(0xC0); tft.writedata(0x2C);
  tft.writecommand(0xC2); tft.writedata(0x01);
  tft.writecommand(0xC3); tft.writedata(0x13);
  tft.writecommand(0xC4); tft.writedata(0x20);
  tft.writecommand(0xC6); tft.writedata(0x0F);
  writeList(0xD0, d0, (int)(sizeof(d0)));
  tft.writecommand(0xD6); tft.writedata(0xA1);
  writeList(0xE0, e0, (int)(sizeof(e0)));
  writeList(0xE1, e1, (int)(sizeof(e1)));
  tft.writecommand(0x21);
  tft.endWrite();
}
#endif

// The SSL handshake + lwIP TCP ACK path runs ~35 frames deep and overflows
// the default 8 KB loopTask stack. blendRadarIntoTerrainRaw() alone allocates
// ~7 KB of local arrays (radarRow[640] + 7x mask/color arrays). 32 KB needed.
size_t getArduinoLoopTaskStackSize() { return 32768; }

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);  // wait for USB host to open port
  delay(500);
  Wire.begin(SDA, SCL);  // SDA=15, SCL=14 — shared I2C bus (AXP2101/touch/RTC/IMU)
  loadWifiPortalConfig();  // needed before QMI decision below
  if (s_shakeToWakeEnabled && s_sleepModeEnabled) {
    initQmiShakeToWake();
  }
  configureAxp2101PowerKey();
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());
  Serial.printf("free heap: %u\n", (unsigned)ESP.getFreeHeap());
  Serial.println("\n== Geo Weather Loop ==");

  // ── Display ───────────────────────────────────────────────
  // Display bus setup is handled by LovyanGFX above.
#if !BOARD_IS_AMOLED_206
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
#endif
#if BOARD_IS_AMOLED_206
  if (s_amoledOut) {
    s_amoledOut->begin();
    s_amoledOut->setRotation(0);
    s_amoledOut->setBrightness(0xFF);
    s_amoledOut->fillScreen(0x0000);
  }
#elif defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
  tft.init();
  applyWaveshareS3St7789Init();
  tft.setRotation(3);          // landscape 320×172 (rotated 180°)
#else
  tft.init();
  applySt7789WarmGamma();
  tft.setRotation(3);          // landscape 320×172 (rotated 180°)
#endif
#if !BOARD_IS_AMOLED_206
  tft.setBrightness(255);
  tft.fillScreen(TFT_BLACK);
#endif

  loadWifiPortalConfig();

#if BOARD_IS_AMOLED_206 || BOARD_HAS_PHYSICAL_BOOT_WAKE
  pinMode(BOOT_BTN_GPIO, INPUT_PULLUP);  // keep BOOT ready for wake-from-sleep
  syncTopButtonStateNow();
  if (!s_topBtnPollTaskHandle) {
    BaseType_t taskOk = xTaskCreatePinnedToCore(
      topButtonPollTask,
      "topbtn",
      2048,
      nullptr,
      3,
      &s_topBtnPollTaskHandle,
      1
    );
    if (taskOk != pdPASS) {
      s_topBtnPollTaskHandle = nullptr;
    }
  }
#endif

  Serial.printf("Free heap: %u\n", (unsigned)ESP.getFreeHeap());

  // ── SD card ───────────────────────────────────────────────
  showMessage("Mounting SD...", nullptr);
#if BOARD_IS_AMOLED_206
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0);
  bool sdOk = SD_MMC.begin("/sdcard", /*mode1bit=*/true, /*format_if_failed=*/false,
                            SDMMC_FREQ_DEFAULT);
#elif defined(ARDUINO_WAVESHARE_ESP32_S3_LCD_147)
  bool sdOk = SD.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN, SD_D1_PIN, SD_D2_PIN, SD_D3_PIN)
           && SD.begin("/sdcard", true, false);
#else
  bool sdOk = SD.begin(SD_CS, SPI, 20000000);  // 20 MHz: within SD SPI spec, improves throughput
#endif
  if (!sdOk) {
    showMessage("SD FAILED", "Insert card and reset");
    Serial.println("SD mount failed!");
    while (1) delay(5000);
  }
  Serial.printf("SD OK  %llu MB\n", SD.totalBytes() / (1024ULL * 1024ULL));
  removeObsoleteGifAssetsIfPresent();
  initFrameStore();
#if BOARD_IS_AMOLED_206
  ensureAudioCueWorker();
  preloadSelectedCueToPsram(false);
#endif

  // ── Allocate pre-scaled display PSRAM buffers ─────────────────────────────
  if (!s_frameDisplayBuf) {
    s_frameDisplayBuf  = (uint16_t*)heap_caps_malloc(SCALED_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    s_terrainDisplayBuf = (uint16_t*)heap_caps_malloc(SCALED_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    size_t topBarBytes = (size_t)SCALED_W * (size_t)SCALED_TOP_BAR_H * 2U;
    size_t botBarBytes = (size_t)SCALED_W * (size_t)SCALED_BAR_H * 2U;
    s_topBarBuf = (uint16_t*)heap_caps_calloc(1, topBarBytes, MALLOC_CAP_SPIRAM);
    s_botBarBuf = (uint16_t*)heap_caps_calloc(1, botBarBytes, MALLOC_CAP_SPIRAM);
    Serial.printf("PSRAM bufs: %s\n",
      (s_frameDisplayBuf && s_terrainDisplayBuf && s_topBarBuf && s_botBarBuf) ? "OK" : "FAIL");
  }

  // ── Determine boot type ───────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool hardBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);
  bool timerWake = (cause == ESP_SLEEP_WAKEUP_TIMER);

  if (hardBoot) {
    Serial.println("hard boot; keep SD");
    framesReady = false;
    loopsDone   = 0;
    frameCount  = 0;
    s_timesLoaded = false;
    s_displayUtcOffsetSec = loadUtcOffsetFromNvs();  // restore from NVS; 0 if never synced
    s_displayUtcOffsetValid = loadUtcOffsetValidFromNvs();
    // Keep a deterministic fallback location so pin/zoom/terrain paths still
    // work even if IP geolocation is temporarily unavailable.
    s_weatherCenterLat = (BBOX_SOUTH + BBOX_NORTH) * 0.5f;
    s_weatherCenterLon = (BBOX_WEST + BBOX_EAST) * 0.5f;
    s_weatherGeoValid = true;
    selectSatelliteForLon(s_weatherCenterLon, true);
    s_lastRadarUtc = 0;
    s_lastRadarUtcValid = false;
    s_radarMetaLoaded = false;
    // Restore location labels from NVS; fall back to config default if never saved
    strlcpy(s_displayLocationLabel, DISPLAY_TZ_LABEL, sizeof(s_displayLocationLabel));
    strlcpy(s_displayLocationFull,  DISPLAY_TZ_LABEL, sizeof(s_displayLocationFull));
    loadLocationLabelFromNvs(s_displayLocationLabel, sizeof(s_displayLocationLabel),
                             s_displayLocationFull,  sizeof(s_displayLocationFull));
    // Hurricane: on hard boot, suppress the active storm (user rebooted to dismiss)
    // and clear hurricane mode.
    {
      Preferences hprefs;
      if (hprefs.begin("satwatch", false)) {
        char hwact[12] = {};
        hprefs.getString("hwact", hwact, sizeof(hwact));
        if (hwact[0] != '\0') {
          suppressStorm(hwact);
          hprefs.putString("hwact", "");
          Serial.printf("hurricane: hard boot — suppressed %s\n", hwact);
        }
        hprefs.end();
      }
    }
    s_hurricaneMode = false;
    memset(&s_activeStorm, 0, sizeof(s_activeStorm));
#ifdef HURRICANE_TEST_MODE
    // Clear suppression so test storm always triggers
    {
      Preferences tprefs;
      if (tprefs.begin("satwatch", false)) {
        tprefs.putString("hwsup", "");
        tprefs.putString("hwact", "");
        tprefs.end();
      }
    }
#endif
  } else {
    Serial.printf("wake loops=%d ready=%d n=%d\n",
                  loopsDone, framesReady, frameCount);
    if (s_weatherGeoValid) {
      selectSatelliteForLon(s_weatherCenterLon, true);
    } else {
      float fallbackLon = (BBOX_WEST + BBOX_EAST) * 0.5f;
      selectSatelliteForLon(fallbackLon, true);
    }
  }

  loadRadarMetaIfNeeded();

  // Load frame index from SD
  loadIndex();
  if (frameCount > 0) {
    Serial.printf("SD index: %d frames\n", frameCount);
    if (hardBoot) {
      loadWeatherViewCenterFromCache();
    }
  }

  // ── Diagnostic log to SD ─────────────────────────────────────────────────
  {
    File diagF = SD.open(SD_ROOT "/diag.txt", FILE_WRITE);
    if (diagF) {
      int8_t batPct = readAxp2101BatPct();
      int chargeState = readAxp2101ChargeState();
      diagF.printf("LogID=%lu date=pending millis=%lu ms\n", (unsigned long)esp_random(), millis());
      diagF.printf("boot: millis=%lu ms\n", millis());
      diagF.printf("idxCount=%d framesReady=%d frameCount=%d\n", (int)s_idx.count, (int)framesReady, frameCount);
      diagF.printf("bat=%d%% chargeState=0x%02X resetReason=%d\n",
                   (int)batPct, (unsigned)(chargeState < 0 ? 0xFF : chargeState),
                   (int)esp_reset_reason());
      diagF.printf("hurricaneWatch=%d\n", (int)s_hurricaneWatchEnabled);
      diagF.close();
    }
  }

  // ── Refresh cache on hard boot / timer wake / empty cache ───────────────
  bool skipNextSyncOnce = consumeSkipNextSyncOnce();
  bool hardBootSyncDue = false;
  if (hardBoot && !skipNextSyncOnce) {
    // Always sync on a hard boot regardless of update mode.
    // Manual mode means no timer-wake syncs, not "ignore stale frames on reboot".
    hardBootSyncDue = true;
  }
  bool needSync = (hardBootSyncDue || timerWake || !framesReady || frameCount == 0);
  {
    File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
    if (diagF) {
      diagF.printf("hardBoot=%d timerWake=%d framesReady=%d frameCount=%d skipOnce=%d needSync=%d\n",
                   (int)hardBoot, (int)timerWake, (int)framesReady, frameCount,
                   (int)skipNextSyncOnce, (int)needSync);
      diagF.close();
    }
  }
  if (skipNextSyncOnce && framesReady && frameCount > 0) {
    bool wifiOk = connectWifiForSync(false);
    { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("skip-sync wifiOk=%d\n", (int)wifiOk); diagF.close(); } }
    if (!wifiOk) {
      tryApplyPcf85063Time();
      runWifiConfigPortal(framesReady && frameCount > 0);
    } else {
      (void)syncClockFromNtpBestEffort(8);
      noteSuccessfulScanNow();
      disconnectWifiAfterSync();
    }
  }
  if (needSync) {
    bool wifiOk = connectWifiForSync(false);
    { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("wifiOk=%d\n", (int)wifiOk); diagF.close(); } }
    if (wifiOk) {
      uint32_t frameBudget = (uint32_t)targetFrameCount();
      if (frameBudget < 1U) frameBudget = 1U;
      if (frameBudget > MAX_FRAMES) frameBudget = MAX_FRAMES;
      // Unified sync pipeline budget:
      // sync + cache/roll + raw(remap/rebuild) + finalize + zoom/terrain refresh.
      uint32_t totalBudget = 20U + frameBudget + (frameBudget + 8U) + 4U + 12U;
      syncProgressBegin(totalBudget, "Updating cache...", "NASA GIBS");
      syncProgressBeginPhase("sync", 20U);

      // Hurricane watch: check NOAA before sync so bbox is storm-centered if needed
      if (s_hurricaneWatchEnabled && !s_hurricaneMode) {
        HurricaneInfo hStorms[4]; int hCount = 0;
        if (pollNoaaForHurricane(hStorms, 4, &hCount)) {
          cleanupSuppressedStorms(hStorms, hCount);
          for (int hi = 0; hi < hCount; hi++) {
            if (!isStormSuppressed(hStorms[hi].id)) {
              enterHurricaneMode(hStorms[hi]);
              break;
            }
          }
        }
      }

      syncWeatherFrames();
      { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("syncWeatherFrames done fc=%d rdy=%d\n", frameCount, (int)framesReady); diagF.close(); } }
      downloadMoonFramesIfMissing();
      maybeRefreshPendingZoomSnapshots();
      noteSuccessfulScanNow();
      disconnectWifiAfterSync();
    } else {
      tryApplyPcf85063Time();
      runWifiConfigPortal(framesReady && frameCount > 0);
    }
    {
      File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
      if (diagF) {
        diagF.printf("post-sync: framesReady=%d frameCount=%d\n", (int)framesReady, frameCount);
        diagF.close();
      }
    }
  }

  if (!needSync && hardBoot && framesReady && frameCount > 0) {
    bool wifiOk = connectWifiForSync(false);
    {
      File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
      if (diagF) {
        diagF.printf("boot-lite wifiOk=%d\n", (int)wifiOk);
        diagF.close();
      }
    }
    if (wifiOk) {
      (void)syncClockFromNtpBestEffort(8);
      refreshDisplayLocationTimeFromIpInfo();
      time_t newestUtc = 0;
      for (int i = frameCount - 1; i >= 0; --i) {
        if (s_frameTimes[i] > 0) { newestUtc = s_frameTimes[i]; break; }
      }
      (void)newestUtc;
      s_zoomSnapshotsRefreshPending = true;
      s_zoomWeatherRefreshNeeded = false;
      downloadMoonFramesIfMissing();
      maybeRefreshPendingZoomSnapshots();
      noteSuccessfulScanNow();
      disconnectWifiAfterSync();
    } else {
      tryApplyPcf85063Time();
      runWifiConfigPortal(true);  // frames guaranteed by branch condition
    }
  }

  // Open stream for playback
  if (framesReady && frameCount > 0) {
    ensureStreamOpen();
    if (!s_streamReady || !s_streamFile) {
      appendDiagLog("setup: stream open failed -> rebuild raw\n");
      showMessage("Building...", "Preparing frames");
      rebuildRawFromStored();
      ensureStreamOpen();
      appendDiagLog("setup: stream ready after rebuild=%d\n",
                    (int)(s_streamReady && (bool)s_streamFile));
    }
  }

  if (syncProgressIsActive()) {
    syncProgressEnd();
  }

  if ((needSync || hardBoot) && framesReady && frameCount > 0) {
    s_startCuePending = true;
  }

  // ── Decode moon phase for bottom border complication ──────
#if BOARD_IS_AMOLED_206
  decodeMoonPhase();
#endif

  // ── Restore brightness after download screen ──────────────
#if BOARD_IS_AMOLED_206
  if (s_amoledOut) s_amoledOut->setBrightness(0xFF);
#else
  tft.setBrightness(255);
#endif
}

// ─────────────────────────────────────────────────────────────
//  Timestamp overlay — drawn on top of each decoded frame
// ─────────────────────────────────────────────────────────────
// Load frame timestamps from SD (needed after wake-from-sleep when RTC is intact
// but the s_frameTimes array in RAM has been cleared)


static void drawTimestamp(int frameIdx, LovyanGFX* target) {
  if (frameIdx >= frameCount || !s_timesLoaded || !target) return;
  loadRadarMetaIfNeeded();
  time_t frameUtc = s_frameTimes[frameIdx];
  if (frameUtc == 0) return;
  time_t nowForAge = currentUtcForAgeMetrics();
  bool useRadarTopTime = s_topBarUseRadarScanTime && s_lastRadarUtcValid && s_lastRadarUtc > 0;
  time_t topUtc = useRadarTopTime ? s_lastRadarUtc : frameUtc;

  // Frame timestamps follow the same IP-based local offset as the clock overlay.
  time_t localFrame = topUtc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
  struct tm tmLocal;
  memset(&tmLocal, 0, sizeof(tmLocal));
  gmtime_r(&localFrame, &tmLocal);

  // How stale is this frame?
  double hoursAgo = difftime(nowForAge, frameUtc) / 3600.0;
  if (hoursAgo < 0) hoursAgo = 0;
  int minsAgo = (int)(difftime(nowForAge, topUtc) / 60.0 + 0.5);
  if (minsAgo < 0) minsAgo = 0;

  char dateBuf[16];
  strftime(dateBuf, sizeof(dateBuf), "%b %d %Y", &tmLocal);
  char timeBuf[16];
  strftime(timeBuf, sizeof(timeBuf), "%I:%M %p", &tmLocal);

  const char* loc = (s_displayLocationFull[0] != '\0') ? s_displayLocationFull : DISPLAY_TZ_LABEL;

  // Radar age string
  char radarBuf[32];
  if (s_lastRadarUtcValid && s_lastRadarUtc > 0) {
    int radarMins = (int)(difftime(nowForAge, s_lastRadarUtc) / 60.0 + 0.5);
    if (radarMins < 0) radarMins = 0;
    snprintf(radarBuf, sizeof(radarBuf), "Radar: %d min", radarMins);
  } else if (s_radarNoSignatures) {
    if (s_lastRadarCheckUtc > 0) {
      int clearMins = (int)(difftime(nowForAge, s_lastRadarCheckUtc) / 60.0 + 0.5);
      if (clearMins < 0) clearMins = 0;
      snprintf(radarBuf, sizeof(radarBuf), "Radar: Clear %dm", clearMins);
    } else {
      snprintf(radarBuf, sizeof(radarBuf), "Radar: Clear");
    }
  } else if (s_radarDownloadFailed) {
    snprintf(radarBuf, sizeof(radarBuf), "Radar: no sig");
  } else {
    snprintf(radarBuf, sizeof(radarBuf), "Radar: n/a");
  }

  target->setTextColor(TFT_WHITE);
  target->setTextSize(1);

  // Top bar: date + time + hours ago, centered
  char topBuf[64];
  if (useRadarTopTime) {
    snprintf(topBuf, sizeof(topBuf), "%s  %s  %d min ago", dateBuf, timeBuf, minsAgo);
  } else {
    snprintf(topBuf, sizeof(topBuf), "%s  %s  %.1f h ago", dateBuf, timeBuf, hoursAgo);
  }
  {
    int tw = target->textWidth(topBuf);
    int tx = (DISP_W - tw) / 2;
    if (tx < 0) tx = 0;
    target->fillRect(0, 0, DISP_W, 14, 0x0000);
    target->setCursor(tx, 3);
    target->print(topBuf);
  }

  // Bottom bar: location + radar, centered
  char botBuf[80];
  snprintf(botBuf, sizeof(botBuf), "%s  %s", loc, radarBuf);
  {
    int bw = target->textWidth(botBuf);
    int bx = (DISP_W - bw) / 2;
    if (bx < 0) bx = 0;
    target->fillRect(0, DISP_H - 14, DISP_W, 14, 0x0000);
    target->setCursor(bx, DISP_H - 11);
    target->print(botBuf);
  }
}

static int findNewestValidStreamFrameIndex(const int* validIdx, int validCount) {
  if (!validIdx || validCount <= 0) return -1;
  if (!s_timesLoaded) return validIdx[validCount - 1];

  int best = validIdx[validCount - 1];
  time_t bestTime = (best >= 0 && best < frameCount) ? s_frameTimes[best] : 0;
  for (int i = 0; i < validCount; i++) {
    int idx = validIdx[i];
    if (idx < 0 || idx >= frameCount) continue;
    time_t t = s_frameTimes[idx];
    if (t > bestTime) {
      bestTime = t;
      best = idx;
    }
  }
  return best;
}

// Map a timeline slot (0..frameCount-1) to a playable frame index.
// Prefer exact slot if valid, otherwise nearest older valid frame so playback
// progresses smoothly through time instead of jumping by sparse validIdx spacing.
static int findPlayableFrameForSlot(int slot, const int* validIdx, int validCount) {
  if (frameCount <= 0) return -1;
  if (slot < 0) slot = 0;
  if (slot >= frameCount) slot = frameCount - 1;

  if (slot < MAX_FRAMES && s_streamValid[slot]) return slot;

  for (int i = slot - 1; i >= 0; --i) {
    if (i < MAX_FRAMES && s_streamValid[i]) return i;
  }
  for (int i = slot + 1; i < frameCount; ++i) {
    if (i < MAX_FRAMES && s_streamValid[i]) return i;
  }
  if (validCount > 0 && validIdx) return validIdx[0];
  return -1;
}

static bool spriteLooksHoldFrameBlockCorrupted() {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px) return false;

  constexpr int MCU   = 16;
  constexpr int BCols = DISP_W / MCU;
  constexpr int BRows = DISP_H / MCU;

  bool zmap[BRows][BCols] = {};
  for (int br = 0; br < BRows; br++) {
    for (int bc = 0; bc < BCols; bc++) {
      bool allZero = true;
      for (int dy = 0; dy < MCU && allZero; dy++) {
        const uint16_t* row = px + (br * MCU + dy) * DISP_W + bc * MCU;
        for (int dx = 0; dx < MCU; dx++) {
          if (row[dx] != 0) { allZero = false; break; }
        }
      }
      zmap[br][bc] = allZero;
    }
  }

  constexpr int kFullRowThreshold = BCols - 4;
  for (int br = 1; br < BRows - 1; br++) {
    int zeroCount = 0;
    for (int bc = 1; bc < BCols - 1; bc++) {
      if (zmap[br][bc]) zeroCount++;
    }
    if (zeroCount >= kFullRowThreshold) return true;
  }

  constexpr int kHalfRow = (BCols - 2) / 2;
  for (int br = 1; br < BRows - 2; br++) {
    int z0 = 0, z1 = 0;
    for (int bc = 1; bc < BCols - 1; bc++) {
      if (zmap[br][bc]) z0++;
      if (zmap[br + 1][bc]) z1++;
    }
    if (z0 >= kHalfRow && z1 >= kHalfRow) return true;
  }

  for (int br = 1; br < BRows - 1; br++) {
    for (int bc = 1; bc < BCols - 1; bc++) {
      if (!zmap[br][bc]) continue;
      int validNeighbors = 0;
      for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++)
          if ((dr | dc) && !zmap[br + dr][bc + dc]) validNeighbors++;
      if (validNeighbors >= 4) return true;
    }
  }
  return false;
}

// Open the stream file for playback (called once, stays open across loops)
static void ensureStreamOpen() {
  if (s_streamReady && s_streamFile) return;
  if (s_idx.count == 0 || !framesReady) return;

  // Check if any raw frames are valid
  bool anyRaw = false;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) { anyRaw = true; break; }
  }
  if (!anyRaw) {
    // Try to rebuild from stored JPEGs
    rebuildRawFromStored();
    anyRaw = false;
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (s_idx.rawValid[i]) { anyRaw = true; break; }
    }
    if (!anyRaw) return;
  }

#if !BOARD_IS_AMOLED_206
  tft.waitDMA();
#endif
  s_streamFile = SD.open(RAW_STREAM_FILE, FILE_READ);
  s_streamReady = (bool)s_streamFile;
  if (s_streamReady) {
    uint32_t actual = (uint32_t)s_streamFile.size();
    uint32_t expected = (uint32_t)MAX_FRAMES * (uint32_t)SCALED_FRAME_BYTES;
    Serial.printf("stream open %u exp %u\n", (unsigned)actual, (unsigned)expected);
  }
}

static void closeStream() {
  if (s_streamFile) { s_streamFile.close(); }
  s_streamReady = false;
}

static uint16_t gray565(uint8_t g) {
  return (uint16_t)(((g & 0xF8) << 8) | ((g & 0xFC) << 3) | (g >> 3));
}

static bool localTimeForDisplay(time_t utc, struct tm* outTm) {
  if (!outTm) return false;
  time_t local = utc + (time_t)(s_displayUtcOffsetValid ? s_displayUtcOffsetSec : (-4 * 3600));
  memset(outTm, 0, sizeof(*outTm));
  gmtime_r(&local, outTm);
  return true;
}

static bool jsonExtractIntField(const String& body, const char* quotedKey, int32_t* out) {
  if (!quotedKey || !out) return false;
  const char* s = body.c_str();
  const char* p = strstr(s, quotedKey);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  if (*p < '0' || *p > '9') return false;
  int32_t v = 0;
  while (*p >= '0' && *p <= '9') {
    v = (v * 10) + (int32_t)(*p - '0');
    p++;
  }
  *out = neg ? -v : v;
  return true;
}

static bool jsonExtractFloatField(const String& body, const char* quotedKey, float* out) {
  if (!quotedKey || !out) return false;
  const char* s = body.c_str();
  const char* p = strstr(s, quotedKey);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  else if (*p == '+') { p++; }

  if ((*p < '0' || *p > '9') && *p != '.') return false;

  uint32_t whole = 0;
  bool haveDigits = false;
  while (*p >= '0' && *p <= '9') {
    haveDigits = true;
    whole = (whole * 10U) + (uint32_t)(*p - '0');
    p++;
  }

  uint32_t frac = 0;
  uint32_t scale = 1;
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9') {
      haveDigits = true;
      if (scale < 1000000U) {
        frac = (frac * 10U) + (uint32_t)(*p - '0');
        scale *= 10U;
      }
      p++;
    }
  }

  if (!haveDigits) return false;
  float v = (float)whole + ((scale > 1U) ? ((float)frac / (float)scale) : 0.0f);
  *out = neg ? -v : v;
  return true;
}

static bool jsonExtractStringField(const String& body, const char* quotedKey, char* out, size_t outLen) {
  if (!quotedKey || !out || outLen == 0) return false;
  out[0] = '\0';
  const char* s = body.c_str();
  const char* p = strstr(s, quotedKey);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;
  p++;
  size_t n = 0;
  while (*p && *p != '"') {
    char c = *p++;
    if (c == '\\' && *p) c = *p++;
    if ((uint8_t)c < 32) continue;
    if (n + 1 < outLen) out[n++] = c;
  }
  out[n] = '\0';
  return (n > 0);
}

// Attempts Starlink dish GPS via gRPC on 192.168.100.1:9200.
// Returns true and sets *outLat/*outLon if successful.
// Prerequisite: enable once in Starlink app:
//   SETTINGS → ADVANCED → DEBUG DATA → STARLINK LOCATION → "allow access on local network"
//
// NOTE: kReq[] must be pre-computed by connecting a laptop to Starlink and running:
//   grpcurl -plaintext 192.168.100.1:9200 describe SpaceX.API.Device.Request
// then encoding HTTP/2 client preface + SETTINGS + HEADERS + DATA frames offline.
// Until that is done, this function stubs out and the BSSID path is used instead.
static bool starlinkGeoLocate(float* outLat, float* outLon) {
  WiFiClient tcp;
  tcp.setTimeout(2000);
  if (!tcp.connect(IPAddress(192, 168, 100, 1), 9200)) return false;
  tcp.stop();
  // kReq[] not yet computed — stub returns false so BSSID path is used.
  // To enable: compute kReq[], send via tcp, parse HTTP/2 DATA frame response,
  // strip 5-byte gRPC length prefix, parse protobuf location.lla.lat / .lon (sint64 ×1e-8).
  Serial.println("starlink-geo: dish reachable but kReq[] not yet embedded — skipping");
  return false;
}

// Scans visible WiFi APs and queries Apple's BSSID geo API.
// Returns true and sets *outLat/*outLon if successful.
// Works on home WiFi, iPhone hotspot, or Starlink — scans RF environment, not WAN IP.
static bool bssidGeoLocate(float* outLat, float* outLon) {
  // 1. Scan visible APs
  int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
  if (n <= 0) {
    Serial.printf("bssid-geo: no APs found (%d)\n", n);
    WiFi.scanDelete();
    return false;
  }
  if (n > 20) n = 20;  // Apple ignores extras; clamp to 20 strongest

  // 2. Build protobuf request body
  // Schema: outer field 13 (repeated WifiAP), each containing field1=bssid string + field3=rssi int32
  // Worst-case size: 20 APs × (1 outer tag + 1 len + 19 bssid + 1 rssi tag + 10 varint) = 640 bytes
  uint8_t body[1024];
  int pos = 0;

  auto writeVarint = [&](uint64_t v) {
    while (v > 0x7F) { body[pos++] = (uint8_t)((v & 0x7F) | 0x80); v >>= 7; }
    body[pos++] = (uint8_t)v;
  };

  for (int i = 0; i < n; i++) {
    String mac = WiFi.BSSIDstr(i);  // "aa:bb:cc:dd:ee:ff"
    int32_t rssi = WiFi.RSSI(i);

    // Build inner WifiAP sub-message
    uint8_t ap[64]; int ap_pos = 0;
    // field 1 = bssid (wire type 2 = length-delimited string)
    ap[ap_pos++] = 0x0a;
    ap[ap_pos++] = (uint8_t)mac.length();
    memcpy(ap + ap_pos, mac.c_str(), mac.length()); ap_pos += mac.length();
    // field 3 = rssi (wire type 0 = varint, negative encoded as 64-bit two's complement)
    ap[ap_pos++] = 0x18;
    uint64_t v = (uint64_t)(int64_t)rssi;
    do {
      ap[ap_pos++] = (uint8_t)((v & 0x7F) | (v > 0x7F ? 0x80 : 0));
      v >>= 7;
    } while (v);

    // Wrap in outer field 13 (wire type 2)
    writeVarint((13 << 3) | 2);
    writeVarint(ap_pos);
    memcpy(body + pos, ap, ap_pos); pos += ap_pos;
  }
  WiFi.scanDelete();

  // WiFi.scanNetworks() temporarily disconnects from the AP while scanning channels.
  // Wait for the driver to reassociate before making an HTTPS request.
  {
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 5000) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("bssid-geo: WiFi lost after scan");
      return false;
    }
  }

  // 3. POST to Apple BSSID geo API
  WiFiClientSecure appleClient;
  appleClient.setInsecure();
  HTTPClient http;
  http.begin(appleClient, "https://gs-loc.apple.com/clls/wloc");
  http.setTimeout(6000);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST(body, pos);
  if (code != HTTP_CODE_OK) {
    Serial.printf("bssid-geo: HTTP-%d\n", code);
    http.end();
    return false;
  }

  // 4. Read response
  WiFiClient* stream = http.getStreamPtr();
  int rlen = http.getSize();
  if (rlen <= 0 || rlen > 128) { http.end(); return false; }
  uint8_t resp[128]; int ri = 0;
  unsigned long t0 = millis();
  while (ri < rlen && millis() - t0 < 3000) {
    if (stream->available()) resp[ri++] = stream->read();
  }
  http.end();

  // 5. Parse protobuf response
  // Schema: field1 = Location { field1=lat sint64 ×1e-8, field2=lon sint64 ×1e-8 }
  auto readVarint = [&](int& p) -> uint64_t {
    uint64_t result = 0; int shift = 0;
    while (p < ri) {
      uint8_t b = resp[p++];
      result |= (uint64_t)(b & 0x7F) << shift;
      if (!(b & 0x80)) break;
      shift += 7;
    }
    return result;
  };
  auto zigzag = [](uint64_t n) -> int64_t { return (int64_t)((n >> 1) ^ -(n & 1)); };

  float lat = 0, lon = 0; bool gotLat = false, gotLon = false;
  for (int p = 0; p < ri; ) {
    uint64_t tag = readVarint(p);
    int field = (int)(tag >> 3), wtype = (int)(tag & 7);
    if (field == 1 && wtype == 2) {        // outer Location message
      int mlen = (int)readVarint(p);
      int mend = p + mlen;
      while (p < mend) {
        uint64_t itag = readVarint(p);
        int ifield = (int)(itag >> 3), iwtype = (int)(itag & 7);
        if ((ifield == 1 || ifield == 2) && iwtype == 0) {
          float deg = (float)(zigzag(readVarint(p)) * 1e-8);
          if (ifield == 1) { lat = deg; gotLat = true; }
          else             { lon = deg; gotLon = true; }
        } else if (iwtype == 0) { readVarint(p); }
        else break;
      }
      p = mend;
    } else if (wtype == 0) { readVarint(p); }
    else if (wtype == 2)   { int l = (int)readVarint(p); p += l; }
    else break;
  }

  if (!gotLat || !gotLon || lat < -90 || lat > 90 || lon < -180 || lon > 180) {
    Serial.println("bssid-geo: parse fail");
    return false;
  }
  *outLat = lat; *outLon = lon;
  Serial.printf("bssid-geo: lat=%ld lon=%ld\n",
                (long)(lat * 10000), (long)(lon * 10000));
  return true;
}

static void refreshDisplayLocationTimeFromIpInfo() {
  if (WiFi.status() != WL_CONNECTED) return;

  // ── Phase 1: Lat/lon via Starlink GPS or Apple BSSID multi-AP scan ──
  // These run independently of any IP API and are the primary geo source.
  float lat = 0.0f, lon = 0.0f;
  bool gotGeo = starlinkGeoLocate(&lat, &lon);
  if (gotGeo) Serial.println("geo: starlink GPS");
  if (!gotGeo) {
    gotGeo = bssidGeoLocate(&lat, &lon);
    if (gotGeo) Serial.println("geo: Apple BSSID multi-AP");
  }

  // ── Phase 2: Timezone offset + location labels via ip-api.com (HTTP) ──
  // Free tier: 45 req/min, no API key, HTTP only. More reliable than ipapi.co.
  // ip-api.com fields: offset (seconds), regionName, countryCode, timezone, lat, lon
  bool gotIpApi = false;
  String body;
  {
    WiFiClient plainClient;
    HTTPClient http;
    if (http.begin(plainClient, "http://ip-api.com/json/?fields=status,lat,lon,timezone,regionName,countryCode,offset")) {
      http.setTimeout(5000);
      int code = http.GET();
      if (code == HTTP_CODE_OK) {
        body = http.getString();
        // Verify status field
        char status[16] = {};
        if (jsonExtractStringField(body, "\"status\"", status, sizeof(status))
            && strcmp(status, "success") == 0) {
          gotIpApi = true;
        }
      } else {
        Serial.printf("ip-api HTTP-%d\n", code);
      }
      http.end();
    }
  }

  // Extract timezone offset from ip-api.com (integer seconds, not a string)
  if (gotIpApi) {
    int32_t offsetSec = 0;
    if (jsonExtractIntField(body, "\"offset\"", &offsetSec)) {
      s_displayUtcOffsetSec = offsetSec;
      s_displayUtcOffsetValid = true;
      saveUtcOffsetToNvs(offsetSec);
    }
  }

  // If BSSID/Starlink failed, use ip-api.com lat/lon as fallback
  if (!gotGeo && gotIpApi) {
    gotGeo = jsonExtractFloatField(body, "\"lat\"", &lat) &&
             jsonExtractFloatField(body, "\"lon\"", &lon) &&
             lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
    if (gotGeo) Serial.println("geo: fell back to ip-api.com lat/lon");
  }

  // Apply geo position — skip in hurricane mode to preserve storm-centered bbox
  if (s_hurricaneMode) {
    // Still extract timezone offset above, but don't override center/satellite
  } else if (gotGeo) {
    float stableLat = roundf(lat * 100.0f) * 0.01f;
    float stableLon = roundf(lon * 100.0f) * 0.01f;
    // Suppress cache invalidation for small drifts (BSSID geo is accurate to tens of metres).
    if (s_weatherGeoValid) {
      float dLat = fabsf(stableLat - s_weatherCenterLat);
      float dLon = fabsf(normalizeLon180(stableLon - s_weatherCenterLon));
      if (dLat < 0.15f && dLon < 0.15f) {
        stableLat = s_weatherCenterLat;
        stableLon = s_weatherCenterLon;
      }
    }
    s_weatherCenterLat = stableLat;
    s_weatherCenterLon = stableLon;
    s_weatherGeoValid = true;
    selectSatelliteForLon(s_weatherCenterLon);
  } else {
    s_weatherGeoValid = false;
    selectSatelliteForLon(s_weatherCenterLon, true);
  }

  // Extract location labels from ip-api.com
  // ip-api.com: regionName = full region name, countryCode = 2-letter country
  if (gotIpApi) {
    char regionName[40] = {};
    char countryCode[8] = {};
    bool haveRegion = jsonExtractStringField(body, "\"regionName\"", regionName, sizeof(regionName)) && regionName[0];
    bool haveCountry = jsonExtractStringField(body, "\"countryCode\"", countryCode, sizeof(countryCode)) && countryCode[0];

    // Compact label: "BC, CA" style
    if (haveRegion && haveCountry) {
      // Abbreviate regionName to first 2 chars if it's long, else use as-is
      char regionShort[12] = {};
      strlcpy(regionShort, regionName, sizeof(regionShort));
      snprintf(s_displayLocationLabel, sizeof(s_displayLocationLabel), "%s, %s", regionShort, countryCode);
    } else if (haveCountry) {
      strlcpy(s_displayLocationLabel, countryCode, sizeof(s_displayLocationLabel));
    } else {
      strlcpy(s_displayLocationLabel, "Local", sizeof(s_displayLocationLabel));
    }

    // Full label: "British Columbia, CA"
    if (haveRegion && haveCountry) {
      snprintf(s_displayLocationFull, sizeof(s_displayLocationFull), "%s, %s", regionName, countryCode);
    } else if (haveRegion) {
      strlcpy(s_displayLocationFull, regionName, sizeof(s_displayLocationFull));
    } else if (haveCountry) {
      strlcpy(s_displayLocationFull, countryCode, sizeof(s_displayLocationFull));
    } else {
      strlcpy(s_displayLocationFull, s_displayLocationLabel, sizeof(s_displayLocationFull));
    }

    saveLocationLabelToNvs(s_displayLocationLabel, s_displayLocationFull);
  }

  char tzName[40] = {};
  if (gotIpApi) jsonExtractStringField(body, "\"timezone\"", tzName, sizeof(tzName));
  Serial.printf("geo: off=%ld loc=%s tz=%s lat=%ld lon=%ld g=%d bssid=%d ipapi=%d\n",
                (long)s_displayUtcOffsetSec,
                s_displayLocationLabel,
                tzName[0] ? tzName : "?",
                (long)(s_weatherCenterLat * 10000.0f),
                (long)(s_weatherCenterLon * 10000.0f),
                (int)s_weatherGeoValid,
                (int)gotGeo, (int)gotIpApi);
  Serial.printf("wx src=%s layer=%s cad=%d lag=%d\n",
                s_activeWeatherSource, s_activeGibsLayer,
                activeCadenceMin(), activeLagHours());
  appendDiagLog("geo: off=%ld loc=%s lat=%.4f lon=%.4f src=%s cad=%d g=%d bssid=%d ipapi=%d millis=%lu ms\n",
                (long)s_displayUtcOffsetSec, s_displayLocationLabel,
                (double)s_weatherCenterLat, (double)s_weatherCenterLon,
                s_activeWeatherSource, activeCadenceMin(),
                (int)s_weatherGeoValid, (int)gotGeo, (int)gotIpApi, millis());
}

static void formatDisplayLocalClockNow(char* out, size_t len) {
  if (!out || len == 0) return;
  struct tm tmLocal;
  time_t nowUtc = time(nullptr);
  if (!localTimeForDisplay(nowUtc, &tmLocal)) {
    out[0] = '\0';
    return;
  }
  if (s_clockUse12Hour) {
    int hr12 = tmLocal.tm_hour % 12;
    if (hr12 == 0) hr12 = 12;
    const char* ampm = (tmLocal.tm_hour < 12) ? "am" : "pm";
    snprintf(out, len, "%d:%02d%s", hr12, tmLocal.tm_min, ampm);
  } else {
    snprintf(out, len, "%02d:%02d", tmLocal.tm_hour, tmLocal.tm_min);
  }
}

static ClockOverlayLayout makeClockOverlayLayout() {
  ClockOverlayLayout l = {};

  s_clockFxSprite.setFont(&fonts::DejaVu56);
  s_clockFxSprite.setTextSize(1);
  s_clockFxSprite.setTextDatum(top_left);

  int textW = s_clockFxSprite.textWidth(s_clockUse12Hour ? "00:00pm" : "00:00");
  int textH = s_clockFxSprite.fontHeight();
  if (textW <= 0) textW = 235;
  if (textH <= 0) textH = 76;

  l.textSize = 1;
  l.textW = textW;
  l.textH = textH;
  l.textX = (SCALED_W - l.textW) / 2;
  l.textY = ((SCALED_H - SCALED_BAR_H) - l.textH) / 2 + SCALED_BAR_H;
  if (l.textY < SCALED_BAR_H + 4) l.textY = SCALED_BAR_H + 4;
  l.bgX = l.textX - 4; if (l.bgX < 0) l.bgX = 0;
  l.bgY = l.textY - 4; if (l.bgY < SCALED_BAR_H) l.bgY = SCALED_BAR_H;
  l.bgW = l.textW + 10; if (l.bgX + l.bgW > SCALED_W) l.bgW = SCALED_W - l.bgX;
  l.bgH = l.textH + 12; if (l.bgY + l.bgH > SCALED_H) l.bgH = SCALED_H - l.bgY;
  return l;
}

static bool saveSpriteRegionToDlBuf(const ClockOverlayLayout& l) {
  // Read from s_frameDisplayBuf (SCALED_W stride, always correct-format RGB565 —
  // the scaler already de-byteswapped sprite pixels when filling this buffer).
  const uint16_t* px = s_frameDisplayBuf;
  if (!px || l.bgW <= 0 || l.bgH <= 0) return false;
  size_t bytes = (size_t)l.bgW * (size_t)l.bgH * 2U;
  if (bytes > DL_BUF_BYTES) return false;
  uint16_t* dst = (uint16_t*)s_dlBuf;
  for (int row = 0; row < l.bgH; row++) {
    memcpy(dst + (row * l.bgW),
           px + ((l.bgY + row) * SCALED_W + l.bgX),
           (size_t)l.bgW * 2U);
  }
  return true;
}

static void restoreSpriteRegionFromDlBuf(const ClockOverlayLayout& l) {
  // Write back to s_frameDisplayBuf (SCALED_W stride, same format as saved).
  uint16_t* px = s_frameDisplayBuf;
  if (!px || l.bgW <= 0 || l.bgH <= 0) return;
  const uint16_t* src = (const uint16_t*)s_dlBuf;
  for (int row = 0; row < l.bgH; row++) {
    memcpy(px + ((l.bgY + row) * SCALED_W + l.bgX),
           src + (row * l.bgW),
           (size_t)l.bgW * 2U);
  }
}

static bool ensureClockFxSpriteForLayout(const ClockOverlayLayout& l) {
  if (l.bgW <= 0 || l.bgH <= 0) return false;
  if (s_clockFxSpriteReady && s_clockFxSpriteW == l.bgW && s_clockFxSpriteH == l.bgH) return true;

  s_clockFxSprite.deleteSprite();
  s_clockFxSpriteReady = false;
  s_clockFxSpriteW = 0;
  s_clockFxSpriteH = 0;

  s_clockFxSprite.setColorDepth(16);
  s_clockFxSprite.setSwapBytes(false);
#if BOARD_HAS_PSRAM_SPRITES
  s_clockFxSprite.setPsram(psramFound());
#endif
  s_clockFxSpriteReady = (bool)s_clockFxSprite.createSprite(l.bgW, l.bgH);
  if (s_clockFxSpriteReady) {
    s_clockFxSpriteW = l.bgW;
    s_clockFxSpriteH = l.bgH;
    // Probe how LovyanGFX stores 16-bit pixels in this sprite buffer so we can
    // do correct RGB565 channel blending during alpha fades.
    uint16_t* probe = (uint16_t*)s_clockFxSprite.getBuffer();
    if (probe) {
      s_clockFxSprite.fillScreen(0);
      s_clockFxSprite.drawPixel(0, 0, 0xF800);  // pure red
      s_clockFxSpritePixelsByteSwapped = (probe[0] == 0x00F8);
    } else {
      s_clockFxSpritePixelsByteSwapped = false;
    }
  }
  return s_clockFxSpriteReady;
}

// Copy the clock FX sprite buffer into s_frameDisplayBuf at display resolution.
// clockFxSprite may store pixels byteswapped; always write correct-format pixels
// to s_frameDisplayBuf so presentScaledBuf can memcpy to the display directly.
static void copyClockFxSpriteToMainSprite(const ClockOverlayLayout& l) {
  uint16_t* dst = s_frameDisplayBuf;
  const uint16_t* src = (const uint16_t*)s_clockFxSprite.getBuffer();
  if (!dst || !src) return;
  for (int row = 0; row < l.bgH; row++) {
    uint16_t* d = dst + (l.bgY + row) * SCALED_W + l.bgX;
    const uint16_t* s = src + row * l.bgW;
    if (s_clockFxSpritePixelsByteSwapped) {
      for (int col = 0; col < l.bgW; col++) d[col] = __builtin_bswap16(s[col]);
    } else {
      memcpy(d, s, (size_t)l.bgW * 2U);
    }
  }
}

static bool loadSavedClockBgIntoFxSprite(const ClockOverlayLayout& l) {
  if (!ensureClockFxSpriteForLayout(l)) return false;
  uint16_t* dst = (uint16_t*)s_clockFxSprite.getBuffer();
  const uint16_t* src = (const uint16_t*)s_dlBuf;
  if (!dst || !src) return false;
  // s_dlBuf holds correct-format (non-byteswapped) RGB565 from s_frameDisplayBuf.
  // clockFxSprite may store pixels byteswapped; convert on load if needed.
  size_t n = (size_t)l.bgW * (size_t)l.bgH;
  if (s_clockFxSpritePixelsByteSwapped) {
    for (size_t i = 0; i < n; i++) dst[i] = __builtin_bswap16(src[i]);
  } else {
    memcpy(dst, src, n * 2U);
  }
  return true;
}

static uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t alpha) {
  if (alpha == 0) return bg;
  if (alpha >= 255) return fg;

  int br = (bg >> 11) & 0x1F;
  int bgc = (bg >> 5) & 0x3F;
  int bb = bg & 0x1F;
  int fr = (fg >> 11) & 0x1F;
  int fgc = (fg >> 5) & 0x3F;
  int fb = fg & 0x1F;

  int r = br + (((fr - br) * (int)alpha + 127) / 255);
  int g = bgc + (((fgc - bgc) * (int)alpha + 127) / 255);
  int b = bb + (((fb - bb) * (int)alpha + 127) / 255);
  if (r < 0) r = 0; else if (r > 31) r = 31;
  if (g < 0) g = 0; else if (g > 63) g = 63;
  if (b < 0) b = 0; else if (b > 31) b = 31;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

static uint8_t smoothstep8(uint8_t t) {
  uint32_t x = t;
  uint32_t x2 = ((x * x) + 127U) / 255U;
  uint32_t x3 = ((x2 * x) + 127U) / 255U;
  int32_t y = (int32_t)(3U * x2) - (int32_t)(2U * x3); // 3x^2 - 2x^3
  if (y <= 0) return 0;
  if (y >= 255) return 255;
  return (uint8_t)y;
}

static bool ensureTerrainCrossfadeRawFromJpeg() {
  if (!s_terrainDisplayBuf) return false;

  // Pick the raw/jpeg path for the current wall-clock time (day or night).
  const char* rawPath = activeTerrainRawPath();

  // Only rebuild if the active raw doesn't exist at the correct pre-scaled size.
  {
    File existing = SD.open(rawPath, FILE_READ);
    if (existing && existing.size() == SCALED_FRAME_BYTES) { existing.close(); return true; }
    if (existing) existing.close();
  }
  SD.remove(rawPath);

  // decodeTerrainCompositeToSprite() internally calls activeTerrainJpegPath(),
  // so it reads the same day/night JPEG that activeTerrainRawPath() corresponds to.
  if (!decodeTerrainCompositeToSprite()) return false;

  // Scale 320×176 terrain composite to 410×360 canonical RGB565
  scaleSpriteTo410x360(s_terrainDisplayBuf);

  File wf = SD.open(rawPath, FILE_WRITE);
  if (!wf) return false;
  size_t wrote = wf.write((const uint8_t*)s_terrainDisplayBuf, SCALED_FRAME_BYTES);
  wf.close();
  if (wrote != SCALED_FRAME_BYTES) {
    SD.remove(rawPath);
    return false;
  }
  return true;
}

static bool runTerrainCrossfadeSegment(int newestIdx, bool baseAlreadyShown) {
  const uint32_t totalMs = 1000U;
  if (!baseAlreadyShown) return false;
  if (!ensureSprite()) return false;
  if (!s_frameDisplayBuf || !s_terrainDisplayBuf) return false;

  // Lock the intended terrain layer for this segment start, then allow a
  // fallback to the opposite layer if the primary raw is missing/invalid.
  const char* primaryRawPath = activeTerrainRawPath();
  const char* fallbackRawPath =
    (strcmp(primaryRawPath, ZOOM_TERRAIN_DAY_RAW) == 0) ? ZOOM_TERRAIN_NIGHT_RAW : ZOOM_TERRAIN_DAY_RAW;

  // Rebuild terrain raw if stale (size-checked, no unconditional wipe)
  if (!ensureTerrainCrossfadeRawFromJpeg()) {
    Serial.println("terrain ensure fail");
  }

  // Pre-load the active terrain raw (day or night) into PSRAM in one sequential SD read.
  {
    File terrainRaw = SD.open(primaryRawPath, FILE_READ);
    bool usingFallbackRaw = false;
    if (!terrainRaw || terrainRaw.size() != SCALED_FRAME_BYTES) {
      if (terrainRaw) terrainRaw.close();
      terrainRaw = SD.open(fallbackRawPath, FILE_READ);
      usingFallbackRaw = true;
      if (!terrainRaw || terrainRaw.size() != SCALED_FRAME_BYTES) {
        if (terrainRaw) terrainRaw.close();
        Serial.printf("terrain raw missing p=%s f=%s\n", primaryRawPath, fallbackRawPath);
        return false;
      }
    }
    size_t tGot = terrainRaw.read((uint8_t*)s_terrainDisplayBuf, SCALED_FRAME_BYTES);
    terrainRaw.close();
    if (tGot != SCALED_FRAME_BYTES) {
      Serial.println("terrain raw short read");
      return false;
    }
    if (usingFallbackRaw) {
      Serial.printf("terrain raw fallback %s\n", fallbackRawPath);
    }
  }

  // Terrain wipe should show radar scan time + minutes-old in the top bar.
  bool prevTopBarRadarMode = s_topBarUseRadarScanTime;
  s_topBarUseRadarScanTime = true;
  updateBarBufs(newestIdx);
  s_topBarUseRadarScanTime = prevTopBarRadarMode;

  // s_frameDisplayBuf holds ZOOM3+timestamp from the preceding showZoomSnapshotFrame() call.
  // Wipe terrain rows progressively from top (startY) into s_frameDisplayBuf.
  // Both buffers are canonical little-endian RGB565 — no bswap needed.
  const int startY = 14;
  const int usableRows = SCALED_H - startY;
  const int steps = 18;
  const int featherRows = 24;
  int prevFullRows = 0;
  int prevTargetRows = 0;
  uint32_t startMs = millis();

  for (int step = 1; step <= steps; step++) {
    uint8_t stepProgressLin = (uint8_t)(((uint32_t)step * 255U) / (uint32_t)steps);
    uint8_t stepProgressEased = smoothstep8(stepProgressLin);
    int targetRows = (step == steps)
                   ? usableRows
                   : (int)(((uint32_t)stepProgressEased * (uint32_t)usableRows + 127U) / 255U);
    if (targetRows < prevTargetRows) targetRows = prevTargetRows;
    if (targetRows > usableRows) targetRows = usableRows;

    int fullRows = (step == steps) ? targetRows : (targetRows - featherRows);
    if (fullRows < 0) fullRows = 0;
    if (fullRows < prevFullRows) fullRows = prevFullRows;
    if (fullRows > usableRows) fullRows = usableRows;

    // Full rows: memcpy entire rows from terrain PSRAM into frame PSRAM
    for (int r = prevFullRows; r < fullRows; r++) {
      int y = startY + r;
      memcpy(s_frameDisplayBuf + y * SCALED_W, s_terrainDisplayBuf + y * SCALED_W, SCALED_W * 2U);
    }

    // Feathered band: alpha-blend terrain into frame (both in PSRAM, no SD seeks)
    if (fullRows < targetRows) {
      int bandLen = targetRows - fullRows;
      for (int r = fullRows; r < targetRows; r++) {
        int y = startY + r;
        uint16_t* dst = s_frameDisplayBuf + y * SCALED_W;
        const uint16_t* ter = s_terrainDisplayBuf + y * SCALED_W;
        uint8_t alphaLin = (uint8_t)(((uint32_t)(r - fullRows + 1) * 255U) / (uint32_t)(bandLen + 1));
        uint8_t alpha = smoothstep8(alphaLin);
        if (alpha == 0) continue;
        for (int x = 0; x < SCALED_W; x++) {
          dst[x] = blend565(dst[x], ter[x], alpha);
        }
      }
    }

    prevTargetRows = targetRows;
    prevFullRows = fullRows;
    presentScaledBuf(s_frameDisplayBuf);

    uint32_t stepDeadline = startMs + (uint32_t)(((uint64_t)step * totalMs) / (uint64_t)steps);
    int32_t waitMs = (int32_t)(stepDeadline - millis());
    if (waitMs > 0) delay((uint32_t)waitMs);
  }

  // Refresh sprite with terrain (used by fallback clock overlay path)
  decodeTerrainCompositeToSprite();

  return true;
}

static uint8_t directionalFadeAlphaForColumn(int col, int width, uint8_t progress, bool rightToLeft) {
  if (width <= 1) return progress;
  if (progress == 0) return 0;
  if (progress >= 255) return 255;

  if (rightToLeft) col = (width - 1) - col;
  if (col < 0) col = 0;
  if (col >= width) col = width - 1;

  // Wide feather so the text feels pre-existing and is revealed by opacity,
  // not "created" at a hard wipe edge.
  int feather = width;
  if (feather < 24) feather = 24;

  int32_t front = -feather
                + (int32_t)(((uint32_t)progress * (uint32_t)((width - 1) + (2 * feather))) / 255U);
  int32_t dist = front - col;
  if (dist <= 0) return 0;
  if (dist >= feather) return 255;

  uint8_t t = (uint8_t)(((uint32_t)dist * 255U) / (uint32_t)feather); // 0..255
  // Ease for smoother edge falloff (less visible banding on bitmap glyphs).
  return (uint8_t)((((uint32_t)t * (uint32_t)t) + 254U) / 255U);
}

static bool drawApproxLocationPinOnClockFxSprite(const ClockOverlayLayout& l,
                                                 int* outX, int* outY,
                                                 int* outW, int* outH) {
  if (!s_weatherGeoValid) return false;
  if (!s_clockFxSprite.getBuffer()) return false;

  int tipX = (SCALED_W / 2) - l.bgX;
  int tipY = (SCALED_H / 2) - l.bgY;
  int r = 5;
  int headCy = tipY - 7;

  int minX = tipX - r - 1;
  int minY = headCy - r - 1;
  int maxX = tipX + r + 1;
  int maxY = tipY + 1;
  if (maxX < 0 || maxY < 0 || minX >= l.bgW || minY >= l.bgH) return false;

  s_clockFxSprite.fillCircle(tipX, headCy, r, 0xF800);
  s_clockFxSprite.fillTriangle(tipX - (r - 1), headCy + 2,
                               tipX + (r - 1), headCy + 2,
                               tipX, tipY, 0xF800);
  s_clockFxSprite.fillCircle(tipX, headCy, 2, 0xFFFF);

  if (outX && outY && outW && outH) {
    int x = minX;
    int y = minY;
    int w = (maxX - minX) + 1;
    int h = (maxY - minY) + 1;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > l.bgW) w = l.bgW - x;
    if (y + h > l.bgH) h = l.bgH - y;
    if (w <= 0 || h <= 0) return false;
    *outX = x; *outY = y; *outW = w; *outH = h;
  }
  return true;
}

static void drawCurrentTimeSweepOverlayFrame(const ClockOverlayLayout& l,
                                             const char* clockBuf,
                                             uint32_t segElapsedMs) {
  if (!clockBuf || !loadSavedClockBgIntoFxSprite(l)) return;

  const uint32_t inMs   = 2000U;
  const uint32_t holdMs = 3000U;
  const uint32_t outMs  = 2000U;
  const uint32_t totalMs = inMs + holdMs + outMs;
  if (segElapsedMs > totalMs) segElapsedMs = totalMs;

  // Opacity progress — fade in, hold, fade out.
  uint8_t alphaProgress;
  bool fadeInPhase = false;
  bool fadeOutPhase = false;
  if (segElapsedMs < inMs) {
    alphaProgress = (uint8_t)((segElapsedMs * 255U) / inMs);  // linear 0 → 255 over 2s
    fadeInPhase = true;
  } else if (segElapsedMs < inMs + holdMs) {
    alphaProgress = 255;
  } else {
    uint32_t outElapsed = segElapsedMs - (inMs + holdMs);
    if (outElapsed > outMs) outElapsed = outMs;
    alphaProgress = (uint8_t)(255U - ((outElapsed * 255U) / outMs)); // linear 255 → 0 over 2s
    fadeOutPhase = true;
  }

  if (alphaProgress == 0) {
    return;
  }

  s_clockFxSprite.setFont(&fonts::DejaVu56);
  s_clockFxSprite.setTextSize(l.textSize);
  s_clockFxSprite.setTextDatum(top_left);
  int actualTextW = s_clockFxSprite.textWidth(clockBuf);
  if (actualTextW <= 0 || actualTextW > l.bgW) actualTextW = l.textW;
  int textLocalX = (l.bgW - actualTextW) / 2;
  if (textLocalX < 0) textLocalX = 0;
  const int textLocalY = l.textY - l.bgY;

  // Drop shadow — always present for consistent depth at all opacity levels.
  s_clockFxSprite.setTextColor(gray565(160));
  s_clockFxSprite.drawString(clockBuf, textLocalX + 1, textLocalY + 1);

  // Main text (white).
  s_clockFxSprite.setTextColor(gray565(255));
  s_clockFxSprite.drawString(clockBuf, textLocalX, textLocalY);

  // Blend only the text+pin region against the saved background.
  // Hold phase is fully opaque; fade phases use directional alpha so the fade
  // sweeps across the text instead of changing uniformly.
  uint16_t* fx = (uint16_t*)s_clockFxSprite.getBuffer();
  const uint16_t* bg = (const uint16_t*)s_dlBuf;
  if (fx && bg) {
    int regionX = textLocalX;
    int regionY = textLocalY;
    int regionW = actualTextW + 2;
    int regionH = l.textH + 2;
    if (regionX < 0) { regionW += regionX; regionX = 0; }
    if (regionY < 0) { regionH += regionY; regionY = 0; }
    if (regionX + regionW > l.bgW) regionW = l.bgW - regionX;
    if (regionY + regionH > l.bgH) regionH = l.bgH - regionY;

    if (regionW > 0 && regionH > 0) {
      for (int row = 0; row < regionH; row++) {
        int base = (regionY + row) * l.bgW + regionX;
        for (int col = 0; col < regionW; col++) {
          int idx = base + col;
          if (bg[idx] == fx[idx]) continue;

          uint8_t pxAlpha = alphaProgress;
          if (fadeInPhase || fadeOutPhase) {
            pxAlpha = directionalFadeAlphaForColumn(col, regionW, alphaProgress, false);
          }

          // bg[] comes from s_dlBuf saved from s_frameDisplayBuf — always
          // correct-format (non-byteswapped) RGB565. fx[] is in clockFxSprite
          // which may store pixels byteswapped. Unify before blending.
          if (pxAlpha == 0) {
            // Restore bg to fxSprite (converting to fxSprite byte order)
            fx[idx] = s_clockFxSpritePixelsByteSwapped ? __builtin_bswap16(bg[idx]) : bg[idx];
          } else if (pxAlpha < 255) {
            if (!s_clockFxSpritePixelsByteSwapped) {
              fx[idx] = blend565(bg[idx], fx[idx], pxAlpha);
            } else {
              // bg is correct-format; bswap only fx to get correct value, blend, store bswapped
              uint16_t f16 = __builtin_bswap16(fx[idx]);
              fx[idx] = __builtin_bswap16(blend565(bg[idx], f16, pxAlpha));
            }
          }
        }
      }
    }
  }

  // Copy blended clock region into s_frameDisplayBuf at display resolution,
  // then push the full frame. Restoring s_dlBuf after each push keeps the
  // background clean for the next tick without re-reading from SD.
  copyClockFxSpriteToMainSprite(l);    // blit into s_frameDisplayBuf (SCALED coords)
  presentScaledBuf(s_frameDisplayBuf); // full-frame push at display resolution
  restoreSpriteRegionFromDlBuf(l);     // restore clean bg in s_frameDisplayBuf
}

static void drawClockOverlayFallbackFrame(const ClockOverlayLayout& layout, const char* clockBuf) {
  if (!clockBuf || clockBuf[0] == '\0') return;

  sprite.setFont(&fonts::DejaVu56);
  sprite.setTextSize(layout.textSize);
  sprite.setTextDatum(top_left);
  int actualTextW = sprite.textWidth(clockBuf);
  if (actualTextW <= 0 || actualTextW > layout.bgW) actualTextW = layout.textW;
  int drawX = layout.bgX + ((layout.bgW - actualTextW) / 2);
  if (drawX < layout.bgX) drawX = layout.bgX;
  sprite.fillRect(layout.bgX, layout.bgY, layout.bgW, layout.bgH, 0x0000);
  sprite.setTextColor(gray565(255));
  sprite.drawString(clockBuf, drawX, layout.textY);
  if (s_weatherGeoValid) {
    int tipX = DISP_W / 2;
    int tipY = DISP_H / 2;
    int r = 5;
    int headCy = tipY - 7;
    sprite.fillCircle(tipX, headCy, r, 0xF800);
    sprite.fillTriangle(tipX - (r - 1), headCy + 2,
                        tipX + (r - 1), headCy + 2,
                        tipX, tipY, 0xF800);
    sprite.fillCircle(tipX, headCy, 2, 0xFFFF);
  }
  presentSpriteToDisplay();
}

static void runCurrentTimeSweepOverlaySegment(int newestIdx, bool baseAlreadyShown) {
  const uint32_t totalMs = 7000U;   // 2s in + 3s hold + 2s out
  const uint32_t tickMs  = 31U;     // avoid 33ms (~2x60Hz) phase-lock with panel scanout
  // Future UX knob: make the pin lead-in configurable from the UI/settings layer.
  const uint32_t pinLeadMs = 0U;    // show pin before the time animation starts

  if (!baseAlreadyShown) {
    if (!showFrame(newestIdx) &&
        !showZoomSnapshotFrame(ZOOM3_FILE, newestIdx) &&
        !showZoomSnapshotFrame(activeTerrainJpegPath(), newestIdx)) {
      return;
    }
  }
  ClockOverlayLayout layout = makeClockOverlayLayout();
  if (!saveSpriteRegionToDlBuf(layout)) {
    Serial.println("clock fb: save fail");
    char clockBuf[16] = {};
    formatDisplayLocalClockNow(clockBuf, sizeof(clockBuf));
    drawClockOverlayFallbackFrame(layout, clockBuf);
    delay(totalMs);
    return;
  }
  if (!ensureClockFxSpriteForLayout(layout)) {
    Serial.println("clock fb: fx alloc fail");
    char clockBuf[16] = {};
    formatDisplayLocalClockNow(clockBuf, sizeof(clockBuf));
    drawClockOverlayFallbackFrame(layout, clockBuf);
    delay(totalMs);
    return;
  }

  char clockBuf[16] = {};
  formatDisplayLocalClockNow(clockBuf, sizeof(clockBuf));
  if (clockBuf[0] == '\0') {
    delay(totalMs);
    return;
  }

  // Pin lead-in frame before time animation.
  // copyClockFxSpriteToMainSprite now writes to s_frameDisplayBuf directly,
  // so present via presentScaledBuf instead of presentSpriteToDisplay.
  if (loadSavedClockBgIntoFxSprite(layout)) {
    drawApproxLocationPinOnClockFxSprite(layout, nullptr, nullptr, nullptr, nullptr);
    copyClockFxSpriteToMainSprite(layout);
    presentScaledBuf(s_frameDisplayBuf);
    delay(pinLeadMs);
    // Re-save so pin is part of the animation background (no text-fade flicker).
    saveSpriteRegionToDlBuf(layout);
  }

  uint32_t startMs = millis();
  uint32_t nextTickMs = startMs;
  while (true) {
    serviceUserButtons();
    uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - startMs;
    if (elapsedMs > totalMs) elapsedMs = totalMs;

    if (elapsedMs > 0 && elapsedMs < totalMs) {
      drawCurrentTimeSweepOverlayFrame(layout, clockBuf, elapsedMs);
    }
    if (elapsedMs >= totalMs) break;

    nextTickMs += tickMs;
    int32_t waitMs = (int32_t)(nextTickMs - millis());
    if (waitMs > 0) {
      delay((uint32_t)waitMs);
    } else {
      nextTickMs = millis();
    }
  }
  // Ensure segment exits on a clean terrain frame (no overlay).
  restoreSpriteRegionFromDlBuf(layout);
  presentScaledBuf(s_frameDisplayBuf);
}

// ─────────────────────────────────────────────────────────────
//  Bottom-band / black-slab corruption detectors
// ─────────────────────────────────────────────────────────────
static bool spriteLooksBottomBandJunkCorrupted() {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px) return false;

  const int startY = DISP_H * 2 / 3;
  const int MCU    = 8;
  const int stepX  = 4;
  int numMcuRows   = (DISP_H - startY) / MCU;
  if (numMcuRows < 4) return false;

  int transitions = 0;
  int prevCls     = -1;

  for (int mr = 0; mr < numMcuRows; mr++) {
    int y0 = startY + mr * MCU;
    int y1 = (y0 + MCU < DISP_H) ? (y0 + MCU) : DISP_H;
    int zeros = 0, total = 0;
    for (int y = y0; y < y1; y++) {
      int row = y * DISP_W;
      for (int x = 0; x < DISP_W; x += stepX) {
        total++;
        uint16_t p = px[row + x];
        if (s_mainSpritePixelsByteSwapped) p = __builtin_bswap16(p);
        int lum = (((p >> 11) & 0x1F) * 3 + ((p >> 5) & 0x3F) + (p & 0x1F) * 2) / 6;
        if (lum <= 4) zeros++;
      }
    }
    if (total == 0) continue;
    int zeroPct = zeros * 100 / total;
    int cls = (zeroPct > 65) ? 0 : (zeroPct < 30) ? 1 : -1;
    if (cls < 0) continue;
    if (prevCls >= 0 && cls != prevCls) transitions++;
    prevCls = cls;
  }
  if (transitions >= 2) {
    appendDiagLog("botband: tr=%d\n", transitions);
    Serial.printf("botband transitions=%d\n", transitions);
    return true;
  }
  return false;
}

static bool spriteLooksBlackSlabCorrupted() {
  const uint16_t* px = (const uint16_t*)sprite.getBuffer();
  if (!px) return false;

  constexpr int TILE_W = 16;
  constexpr int TILE_H = 16;
  constexpr int COLS = DISP_W / TILE_W;
  constexpr int ROWS = DISP_H / TILE_H;
  constexpr int START_Y = 14;

  bool suspect[ROWS][COLS] = {};

  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < COLS; ++bc) {
      int blackish = 0;
      int sampled = 0;
      for (int dy = 0; dy < TILE_H; ++dy) {
        int py = br * TILE_H + dy;
        if (py < START_Y || py >= DISP_H) continue;
        const uint16_t* row = px + py * DISP_W + bc * TILE_W;
        for (int dx = 0; dx < TILE_W; ++dx) {
          uint16_t p = row[dx];
          if (s_mainSpritePixelsByteSwapped) p = __builtin_bswap16(p);
          int r = (p >> 11) & 0x1F;
          int g6 = (p >> 5) & 0x3F;
          int b = p & 0x1F;
          int g = g6 >> 1;
          int maxCh = r;
          if (g > maxCh) maxCh = g;
          if (b > maxCh) maxCh = b;
          int minCh = r;
          if (g < minCh) minCh = g;
          if (b < minCh) minCh = b;
          if (maxCh <= 6 && (maxCh - minCh) <= 2) blackish++;
          sampled++;
        }
      }
      if (sampled > 0 && ((blackish * 100) / sampled) >= 50) {
        suspect[br][bc] = true;
      }
    }
  }

  {
    int darkTiles = 0;
    for (int br = 0; br < ROWS; ++br)
      for (int bc = 0; bc < COLS; ++bc)
        if (suspect[br][bc]) darkTiles++;
    if (darkTiles * 100 / (ROWS * COLS) >= 20) {
      Serial.printf("slab skip: darkTiles=%d/%d\n", darkTiles, ROWS * COLS);
      appendDiagLog("slab: skip darkTiles=%d/%d\n", darkTiles, ROWS * COLS);
      return false;
    }
  }

  bool seen[ROWS][COLS] = {};
  int qx[ROWS * COLS];
  int qy[ROWS * COLS];
  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < COLS; ++bc) {
      if (!suspect[br][bc] || seen[br][bc]) continue;
      int head = 0, tail = 0;
      qx[tail] = bc;
      qy[tail] = br;
      tail++;
      seen[br][bc] = true;
      int comp = 0;
      while (head < tail) {
        int cx = qx[head];
        int cy = qy[head];
        head++;
        comp++;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= COLS || ny >= ROWS) continue;
            if (seen[ny][nx] || !suspect[ny][nx]) continue;
            seen[ny][nx] = true;
            qx[tail] = nx;
            qy[tail] = ny;
            tail++;
          }
        }
      }
      if (comp >= 50) {
        Serial.printf("slab comp=%d\n", comp);
        appendDiagLog("slab: comp=%d\n", comp);
        return true;
      }
    }
  }

  auto regionDarkPct = [&](int x0, int x1, int y0, int y1) -> int {
    int dark = 0;
    int total = 0;
    if (x0 < 0) x0 = 0;
    if (x1 > DISP_W) x1 = DISP_W;
    if (y0 < START_Y) y0 = START_Y;
    if (y1 > DISP_H) y1 = DISP_H;
    for (int y = y0; y < y1; y += 2) {
      const uint16_t* row = px + y * DISP_W;
      for (int x = x0; x < x1; x += 2) {
        uint16_t p = row[x];
        if (s_mainSpritePixelsByteSwapped) p = __builtin_bswap16(p);
        int r = (p >> 11) & 0x1F;
        int g = ((p >> 5) & 0x3F) >> 1;
        int b = p & 0x1F;
        int maxCh = r;
        if (g > maxCh) maxCh = g;
        if (b > maxCh) maxCh = b;
        int minCh = r;
        if (g < minCh) minCh = g;
        if (b < minCh) minCh = b;
        if (maxCh <= 5 && (maxCh - minCh) <= 3) dark++;
        total++;
      }
    }
    return (total > 0) ? (dark * 100 / total) : 0;
  };

  const int y0 = START_Y;
  const int y1 = DISP_H;
  const int leftDark = regionDarkPct(0, DISP_W / 5, y0, y1);
  const int rightDark = regionDarkPct(DISP_W - (DISP_W / 5), DISP_W, y0, y1);
  const int midDark = regionDarkPct(DISP_W / 3, (DISP_W * 2) / 3, y0, y1);
  appendDiagLog("slab: darkPct l=%d r=%d m=%d\n", leftDark, rightDark, midDark);
  if ((leftDark >= 70 && midDark <= 18) || (rightDark >= 70 && midDark <= 18)) {
    Serial.printf("slab mass l=%d r=%d m=%d\n", leftDark, rightDark, midDark);
    appendDiagLog("slab: mass l=%d r=%d m=%d\n", leftDark, rightDark, midDark);
    return true;
  }

  constexpr int EDGE_COLS = 4;
  int leftHits = 0;
  int rightHits = 0;
  int leftTotal = 0;
  int rightTotal = 0;
  for (int br = 0; br < ROWS; ++br) {
    for (int bc = 0; bc < EDGE_COLS; ++bc) {
      if (bc < COLS) {
        leftTotal++;
        if (suspect[br][bc]) leftHits++;
      }
    }
    for (int bc = COLS - EDGE_COLS; bc < COLS; ++bc) {
      if (bc >= 0) {
        rightTotal++;
        if (suspect[br][bc]) rightHits++;
      }
    }
  }
  int leftPct = (leftTotal > 0) ? (leftHits * 100 / leftTotal) : 0;
  int rightPct = (rightTotal > 0) ? (rightHits * 100 / rightTotal) : 0;
  appendDiagLog("slab: edgePct l=%d r=%d\n", leftPct, rightPct);
  if ((leftPct >= 75 && rightPct <= 15) || (rightPct >= 75 && leftPct <= 15)) {
    Serial.printf("slab edge l=%d r=%d\n", leftPct, rightPct);
    appendDiagLog("slab: edge l=%d r=%d\n", leftPct, rightPct);
    return true;
  }

  for (int bc = 0; bc < COLS; ++bc) {
    int count = 0;
    int bestRun = 0;
    int run = 0;
    for (int br = 0; br < ROWS; ++br) {
      if (suspect[br][bc]) {
        count++;
        run++;
        if (run > bestRun) bestRun = run;
      } else {
        run = 0;
      }
    }
    if (count >= 9 && bestRun >= 7) {
      Serial.printf("slab col=%d cnt=%d run=%d\n", bc, count, bestRun);
      appendDiagLog("slab: col=%d cnt=%d run=%d\n", bc, count, bestRun);
      return true;
    }
  }

  for (int br = 0; br < ROWS; ++br) {
    int count = 0;
    int bestRun = 0;
    int run = 0;
    for (int bc = 0; bc < COLS; ++bc) {
      if (suspect[br][bc]) {
        count++;
        run++;
        if (run > bestRun) bestRun = run;
      } else {
        run = 0;
      }
    }
    if (count >= 12 && bestRun >= 8) {
      Serial.printf("slab row=%d cnt=%d run=%d\n", br, count, bestRun);
      appendDiagLog("slab: row=%d cnt=%d run=%d\n", br, count, bestRun);
      return true;
    }
  }

  return false;
}

// ─────────────────────────────────────────────────────────────
//  loop() — animate frames, sleep after LOOPS_BEFORE_SLEEP
// ─────────────────────────────────────────────────────────────
void loop() {
  serviceWifiPortalServer();
  serviceUserButtons();
  if (!framesReady || frameCount == 0) {
    showMessage("No frames", "Reset to retry download");
    delay(5000);
    return;
  }

  ensureStreamOpen();

  if (!s_streamReady || !s_streamFile) {
    rebuildRawFromStored();
    ensureStreamOpen();
  }

  if (!s_streamReady || !s_streamFile) {
    // Stream failed to open — show each variable on its own large line
    char l1[32], l2[32];
    snprintf(l1, sizeof(l1), "NO STREAM fc=%d", frameCount);
    snprintf(l2, sizeof(l2), "rdy=%d meta=%d", (int)s_streamReady, s_idx.count);
    showMessage(l1, nullptr); delay(4000);
    showMessage(l2, nullptr); delay(4000);
    return;
  }

  // Lazily rebuild the valid-frame index cache
  if (s_validCount < 0) {
    s_validCount = 0;
    for (int i = 0; i < frameCount && s_validCount < MAX_FRAMES; i++) {
      if (s_streamValid[i]) s_validIdx[s_validCount++] = i;
    }
    s_newestCachedIdx = findNewestValidStreamFrameIndex(s_validIdx, s_validCount);
    if (s_newestCachedIdx < 0 && s_validCount > 0)
      s_newestCachedIdx = s_validIdx[s_validCount - 1];
  }
  const int* validIdx = s_validIdx;
  const int  validCount = s_validCount;
  const int  newestIdx = s_newestCachedIdx;
  if (validCount == 0) {
    int validInBitmap = 0;
    for (int i = 0; i < frameCount; i++) if (s_streamValid[i]) validInBitmap++;
    char l1[32], l2[32];
    snprintf(l1, sizeof(l1), "0 frames fc=%d", frameCount);
    snprintf(l2, sizeof(l2), "vbm=%d meta=%d", validInBitmap, s_idx.count);
    showMessage(l1, nullptr); delay(4000);
    showMessage(l2, nullptr); delay(4000);
    return;
  }

  if (autoUpdateDueNow()) {
    showMessage("Auto update", "Rebooting to rescan");
    delay(300);
    SD_MMC.end();
    ESP.restart();
  }

  // Loop timing: 10s animation + 2s hold + 3×1s zoom + ~1s terrain + 7s clock ≈ 24s.
  // animationDurationMs is enforced with a break — frames that run long don't overrun.
  const uint32_t animationDurationMs = 10000U;
  const uint32_t latestFrameHoldMs   = 2000U;
  const uint32_t zoomPreviewStepMs   = 1000U;
  const uint32_t terrainTransitionMs = 1000U;
  const uint32_t clockOverlayMs      = 7000U;
  uint32_t targetFrameDelayMs = (FRAME_DELAY_MS > 0) ? (uint32_t)FRAME_DELAY_MS : 1U;
  // No slots cap — time-based frame selection self-paces with pre-scaled reads (~120ms/frame)
  bool startCueArmed = s_startCuePending;

  static bool s_animStartLogged = false;
  if (!s_animStartLogged) {
    s_animStartLogged = true;
    appendDiagLog("anim-start: millis=%lu ms\n", millis());
  }

  uint32_t loopStartMs = millis();
  bool preFreezeShown = false;
  int lastDisplayedFrameIdx = -1;
  // Time-based frame selection over the valid-frame list.
  // This keeps playback monotonic even when the raw validity map has holes.
  for (;;) {
    serviceUserButtons();
    uint32_t elapsed = millis() - loopStartMs;
    if (elapsed >= animationDurationMs) break;

    uint32_t srcPos = (uint32_t)(((uint64_t)elapsed * (uint64_t)validCount) / animationDurationMs);
    if (srcPos >= (uint32_t)validCount) srcPos = (uint32_t)validCount - 1U;

    // Pre-freeze: when ≤2 frame-times remain, stop advancing and let the hold
    // use the actual final animation frame, not a synthetic "newest" frame.
    uint32_t remaining = animationDurationMs - elapsed;
    if (remaining <= targetFrameDelayMs * 2U + 50U) {
      int freezeFrameIdx = validIdx[srcPos];
      if (lastDisplayedFrameIdx != freezeFrameIdx) {
        if (showFrame(freezeFrameIdx)) {
          if (startCueArmed) {
            if (playStartCueIfEnabled()) {
              s_startCuePending = false;
              startCueArmed = false;
            }
          }
          lastDisplayedFrameIdx = freezeFrameIdx;
          preFreezeShown = true;
          // Walk back if the freeze frame looks slab-corrupted (e.g. a GIBS
          // partial composite at the lag boundary that passed size/decode checks
          // but has a large near-zero black region).
          if (currentScaledFreezeFrameLooksCorrupted()) {
            // Evict the corrupt frame from index so next sync re-downloads it.
            {
              appendDiagLog("raw-evict: idx=%d freeze-corrupt\n", freezeFrameIdx);
              if (freezeFrameIdx < MAX_FRAMES) {
                s_streamValid[freezeFrameIdx] = 0;
                s_idx.rawValid[freezeFrameIdx] = 0;
              }
            }
            bool foundClean = false;
            for (int back = 1; back <= 12 && (int)srcPos - back >= 0; ++back) {
              int backIdx = validIdx[(int)srcPos - back];
              if (showFrame(backIdx)) {
                if (!currentScaledFreezeFrameLooksCorrupted()) {
                  lastDisplayedFrameIdx = backIdx;
                  foundClean = true;
                  appendDiagLog("loop: freeze-back=%d idx=%d\n", back, backIdx);
                  break;
                }
                // Also corrupt — evict this one too.
                appendDiagLog("raw-evict: idx=%d freeze-corrupt\n", backIdx);
                if (backIdx < MAX_FRAMES) {
                  s_streamValid[backIdx] = 0;
                  s_idx.rawValid[backIdx] = 0;
                }
              }
            }
            if (!foundClean) {
              // No clean alternative — re-show the original freeze frame.
              showFrame(freezeFrameIdx);
              appendDiagLog("loop: freeze-back-fail\n");
            }
            // Write updated index so evicted slots get re-downloaded next sync
            writeIndex();
            appendDiagLog("idx: updated after freeze-corrupt eviction\n");
            // Force valid-frame index rebuild next loop so evicted frames
            // are excluded for the remainder of this session.
            invalidateValidIdxCache();
          }
        }
      } else if (lastDisplayedFrameIdx >= 0) {
        preFreezeShown = true;
      }
      uint32_t el2 = millis() - loopStartMs;
      if (el2 < animationDurationMs) delay(animationDurationMs - el2);
      break;
    }

    if (showFrame(validIdx[srcPos])) {
      if (startCueArmed) {
        if (playStartCueIfEnabled()) {
          s_startCuePending = false;
          startCueArmed = false;
        }
      }
      lastDisplayedFrameIdx = validIdx[srcPos];
      preFreezeShown = true;
    }
  }

  // Freeze on the already-displayed final animation frame.
  // Avoid any extra frame push at freeze entry (that's where transfer tear occurs).
  int holdFrameIdx = (lastDisplayedFrameIdx >= 0) ? lastDisplayedFrameIdx : newestIdx;
  bool holdFrameShown = preFreezeShown;
  if (!holdFrameShown) holdFrameShown = true;  // keep current panel frame as freeze base
  if (holdFrameShown && s_frameDisplayBuf) {
    // Re-present the exact current frame buffer once so the start of the freeze
    // and the locator-cue redraws are guaranteed to use the same image.
    presentScaledBuf(s_frameDisplayBuf);
  }
  runFreezeZoom3LocatorCue(latestFrameHoldMs);

  // Zoom + terrain stages — each step is wall-clock governed so decode time is
  // included in the step budget, keeping the total loop time predictable.
  bool baseForClockOverlay = holdFrameShown;
  if (holdFrameShown) {
    uint32_t zStepStart = millis();
    bool z1Shown = showZoomSnapshotFrame(ZOOM1_FILE, newestIdx);
    if (z1Shown) {
      int32_t zRemain = (int32_t)zoomPreviewStepMs - (int32_t)(millis() - zStepStart);
      if (zRemain > 0) delay((uint32_t)zRemain);

      zStepStart = millis();
      bool z2Shown = showZoomSnapshotFrame(ZOOM2_FILE, newestIdx);
      if (z2Shown) {
        zRemain = (int32_t)zoomPreviewStepMs - (int32_t)(millis() - zStepStart);
        if (zRemain > 0) delay((uint32_t)zRemain);

        zStepStart = millis();
        bool z3Shown = showZoomSnapshotFrame(ZOOM3_FILE, newestIdx);
        if (z3Shown) {
          zRemain = (int32_t)zoomPreviewStepMs - (int32_t)(millis() - zStepStart);
          if (zRemain > 0) delay((uint32_t)zRemain);

          bool terrainShownForClock = false;
          if (s_hurricaneMode) {
            // Skip terrain crossfade in hurricane mode — not relevant
            baseForClockOverlay = true;
          } else if (runTerrainCrossfadeSegment(newestIdx, true)) {
            terrainShownForClock = true;
            baseForClockOverlay = terrainShownForClock;
          } else {
            if (showZoomSnapshotFrame(activeTerrainJpegPath(), newestIdx)) {
              terrainShownForClock = true;
              delay(terrainTransitionMs);
            } else if (showFrame(holdFrameIdx)) {
              terrainShownForClock = true;
              delay(terrainTransitionMs);
            }
            baseForClockOverlay = terrainShownForClock;
          }
        } else {
          baseForClockOverlay = true;  // keep ZOOM2 as the base if ZOOM3 is unavailable
        }
      } else {
        baseForClockOverlay = true;    // keep ZOOM1 as the base if ZOOM2 is unavailable
      }
    } else {
      baseForClockOverlay = true;      // keep freeze frame if zoom snapshots are unavailable
    }
  }

  if (!baseForClockOverlay) {
    Serial.println("zoom/terrain/clock stages skipped — holding weather frame");
  }

  runCurrentTimeSweepOverlaySegment(newestIdx, baseForClockOverlay);

  // Periodic NOAA re-check during hurricane mode
  if (s_hurricaneMode && ++s_hurricaneLoopsSinceCheck >= 3) {
    hurricaneRecheckAndUpdate();
    s_hurricaneLoopsSinceCheck = 0;
    // If hurricane mode was exited by recheck, sync new local frames
    if (!s_hurricaneMode && !framesReady) {
      if (connectWifiForSync(false)) {
        syncWeatherFrames();
        disconnectWifiAfterSync();
      }
    }
  }

  loopsDone++;
  uint32_t loopElapsedMs = millis() - loopStartMs;
  uint32_t loopExpectedMs = animationDurationMs + latestFrameHoldMs
                            + (3U * zoomPreviewStepMs) + terrainTransitionMs + clockOverlayMs;
  int32_t loopDriftMs = (int32_t)loopElapsedMs - (int32_t)loopExpectedMs;
  Serial.printf("loop %d/%d v=%d elapsed=%ums expected=%ums drift=%dms\n",
                loopsDone, s_loopsBeforeSleep,
                validCount, loopElapsedMs, loopExpectedMs, loopDriftMs);
  appendDiagLog("loop: %d/%d valid=%d fc=%d newest=%d elapsed=%u expected=%u drift=%d ms\n",
                loopsDone, s_loopsBeforeSleep, validCount, frameCount, newestIdx,
                loopElapsedMs, loopExpectedMs, loopDriftMs);

  // Prevent sleep during hurricane mode
  if (s_sleepModeEnabled && !s_hurricaneMode && loopsDone >= s_loopsBeforeSleep) {
    loopsDone = 0;
    goToSleep(false);
    return;
  }
}
