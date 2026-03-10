#pragma once

// =============================================================
//  WiFi — edit these
// =============================================================
#define WIFI_SSID          "TELUS7556"
#define WIFI_PASS          "PhdRxmb5Cz5G"

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
#define FRAME_DELAY_MS       33   // ms per frame slot; ~30fps target (pre-scaled frames read fast)
#define LOOPS_BEFORE_SLEEP   10   // full loops to play before sleeping

// =============================================================
//  Display local time (set for installation location)
// =============================================================
// POSIX TZ string — overridden at runtime by user location
#define DISPLAY_TZ_POSIX    "AST4"
#define DISPLAY_TZ_LABEL    ""

// =============================================================
//  Deep sleep
// =============================================================
#define SLEEP_HOURS          6    // hours to sleep between play sessions

#define ENABLE_SERIAL_DIAG 1

// Uncomment for off-season hurricane watch testing (injects fake Cat 3 storm)
// #define HURRICANE_TEST_MODE 1
