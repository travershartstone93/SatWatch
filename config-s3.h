#pragma once

// =============================================================
//  WiFi - edit these
// =============================================================
#define WIFI_SSID          ""
#define WIFI_PASS          ""

// =============================================================
//  Fetch window
// =============================================================
#define HOURS_BACK          24    // hours of imagery to download
#define CADENCE_MIN         10    // GOES-East full-disk cadence (minutes)
#define GIBS_LAG_HOURS       2    // GIBS lags real-time; start 2 h before now

// Fallback weather bounding box (EPSG:4326, lon/lat degrees)
// Used only if IP geolocation is unavailable.
#define BBOX_WEST          -85.0f
#define BBOX_SOUTH           2.0f
#define BBOX_EAST          -57.0f
#define BBOX_NORTH          25.0f

// =============================================================
//  Animation
// =============================================================
#define FRAME_DELAY_MS       33   // ms per frame slot; used for pre-freeze timing
#define LOOPS_BEFORE_SLEEP   10   // full loops to play before sleeping

// =============================================================
//  Display local time (set for installation location)
// =============================================================
// POSIX TZ string - overridden at runtime by user location
#define DISPLAY_TZ_POSIX    "AST4"
#define DISPLAY_TZ_LABEL    ""

// =============================================================
//  Deep sleep
// =============================================================
#define SLEEP_HOURS          6    // hours to sleep between play sessions

#define ENABLE_SERIAL_DIAG 0

// Set to 0 to disable writing diag.txt to SD card (production builds)
#define ENABLE_DIAG_LOG    1

// Uncomment for off-season hurricane watch testing (injects fake Cat 3 storm)
// #define HURRICANE_TEST_MODE 1

// =============================================================
//  Forecast / Nowcast
// =============================================================
#define NOWCAST_BBOX_HALF_DEG   0.8f    // ~90km half-width for radar fetch
#define NOWCAST_FRAME_COUNT     6       // radar snapshots to analyze (30 min span)
#define NOWCAST_FRAME_STEP_MS   (5ULL * 60ULL * 1000ULL)  // 5 min between frames
#define NOWCAST_ANALYSIS_PATCH  10      // 10x10 pixel sample patch
#define NOWCAST_UPWIND_KM       80.0f   // how far upwind to sample
#define FORECAST_ICON_PX        27      // weather/bar icon size in pixels (max for ~31px bar)
#define NOWCAST_PIXEL_SAT_MIN   10      // min saturation to classify as radar signal (was 5)
#define NOWCAST_PIXEL_LUM_MIN   20      // min luminance to classify as radar signal (was 8)
#define NOWCAST_RAIN_INTENSITY  30      // min avg intensity to declare rain (was 15)
