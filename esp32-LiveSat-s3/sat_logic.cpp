#include "sat_logic.h"

#ifdef ARDUINO
#include <cstring>
#else
#include <cstring>
#include <cstdlib>
#endif

// Layer name constants (must match firmware #defines)
static const char LAYER_GOES_EAST[]    = "GOES-East_ABI_GeoColor";
static const char LAYER_GOES_WEST[]    = "GOES-West_ABI_GeoColor";
static const char LAYER_HIMAWARI_IR[]  = "Himawari_AHI_Band13_Clean_Infrared";
static const char LAYER_MTG_GEOCOLOR[] = "mtg_fd:rgb_geocolour";

static float normalizeLon180(float lon) {
    while (lon > 180.0f) lon -= 360.0f;
    while (lon < -180.0f) lon += 360.0f;
    return lon;
}

SatProfile selectSatelliteForLonPure(float lonDeg, const char* currentLayer, bool force) {
    static constexpr float kGoesSplitLon = -110.0f;
    static constexpr float kMtgWestLon   = -15.0f;
    static constexpr float kApacSplitLon = 80.0f;
    static constexpr float kHystDeg      = 2.0f;

    float lon = normalizeLon180(lonDeg);

    auto layerIs = [&](const char* layer) -> bool {
        return currentLayer && layer && strcmp(currentLayer, layer) == 0;
    };

    // Himawari: > +80
    bool keepHimawari =
        !force && layerIs(LAYER_HIMAWARI_IR) && (lon >= (kApacSplitLon - kHystDeg));
    bool enterHimawari = (lon >= (kApacSplitLon + kHystDeg));
    if (keepHimawari || enterHimawari) {
        return { LAYER_HIMAWARI_IR, 10, 3, "Himawari-IR", false };
    }

    // MTG GeoColor: -15 to +80
    bool keepMtg =
        !force && layerIs(LAYER_MTG_GEOCOLOR) &&
        (lon >= (kMtgWestLon - kHystDeg)) && (lon < (kApacSplitLon + kHystDeg));
    bool enterMtg = (lon >= (kMtgWestLon + kHystDeg)) && (lon < (kApacSplitLon - kHystDeg));
    if (keepMtg || enterMtg) {
        return { LAYER_MTG_GEOCOLOR, 10, 1, "MTG-GeoColor", true };
    }

    // GOES-West: < -110
    bool keepWest =
        !force && layerIs(LAYER_GOES_WEST) && (lon < (kGoesSplitLon + kHystDeg));
    bool enterWest = (lon < (kGoesSplitLon - kHystDeg));
    if (keepWest || enterWest) {
        return { LAYER_GOES_WEST, 10, 2, "GOES-West", false };
    }

    // GOES-East: fallback
    return { LAYER_GOES_EAST, 10, 2, "GOES-East", false };
}

bool isProgressiveJpeg(const uint8_t* data, size_t len) {
    for (size_t i = 0; i + 1 < len; i++) {
        if (data[i] == 0xFF) {
            if (data[i + 1] == 0xC2) return true;   // SOF2 = progressive
            if (data[i + 1] == 0xC0) return false;   // SOF0 = baseline
            if (data[i + 1] == 0xDA) return false;   // SOS = past all SOF markers
        }
    }
    return false;
}

time_t snapToNearestTime(const time_t* times, int count, time_t t, int maxOffsetSec) {
    if (count == 0 || !times) return 0;
    int lo = 0, hi = count - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (times[mid] < t) lo = mid + 1;
        else hi = mid;
    }
    time_t best = 0;
    long bestDist = (long)maxOffsetSec + 1;
    for (int i = (lo > 0 ? lo - 1 : 0); i <= lo && i < count; i++) {
        long dist = labs((long)(times[i] - t));
        if (dist < bestDist) {
            bestDist = dist;
            best = times[i];
        }
    }
    return (bestDist <= maxOffsetSec) ? best : 0;
}

int generateFrameTimes(time_t* out, int maxFrames, time_t fetchEnd, int cadenceSec, int totalFrames) {
    if (!out || maxFrames <= 0 || cadenceSec <= 0 || totalFrames <= 0) return 0;
    int count = totalFrames < maxFrames ? totalFrames : maxFrames;
    time_t fetchStart = fetchEnd - (time_t)((count - 1) * cadenceSec);
    for (int i = 0; i < count; i++) {
        out[i] = fetchStart + (time_t)(i * cadenceSec);
    }
    return count;
}
