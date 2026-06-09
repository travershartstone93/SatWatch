#include "sat_logic.h"

// Layer name constants (must match firmware #defines)
static const char LAYER_GOES_EAST[]   = "GOES-East_ABI_GeoColor";
static const char LAYER_GOES_WEST[]   = "GOES-West_ABI_GeoColor";
static const char LAYER_HIMAWARI_IR[] = "Himawari_AHI_Band13_Clean_Infrared";
static const char LAYER_MTG_GEOCOLOR[] = "mtg_fd:rgb_geocolour";

static float normalizeLon180(float lon) {
    while (lon > 180.0f) lon -= 360.0f;
    while (lon < -180.0f) lon += 360.0f;
    return lon;
}

static SatProfile makeProfile(const char* layer, int cadenceMin, int lagHours,
                               const char* source, bool eumetview) {
    SatProfile p;
    memset(&p, 0, sizeof(p));
    strncpy(p.layer, layer, sizeof(p.layer) - 1);
    strncpy(p.source, source, sizeof(p.source) - 1);
    p.cadenceMin = cadenceMin;
    p.lagHours = lagHours;
    p.isEumetview = eumetview;
    return p;
}

SatProfile selectSatelliteForLon(float lonDeg) {
    static constexpr float kGoesSplitLon = -110.0f;
    static constexpr float kMtgWestLon   = -15.0f;
    static constexpr float kApacSplitLon = 80.0f;
    // Hysteresis not applicable for stateless version — use exact boundaries
    // (hysteresis requires knowing the previous active layer, which is a global)

    float lon = normalizeLon180(lonDeg);

    // Himawari: >= +80
    if (lon >= kApacSplitLon) {
        return makeProfile(LAYER_HIMAWARI_IR, 10, 3, "Himawari-IR", false);
    }

    // MTG GeoColor: -15 to +80
    if (lon >= kMtgWestLon && lon < kApacSplitLon) {
        return makeProfile(LAYER_MTG_GEOCOLOR, 10, 1, "MTG-GeoColor", true);
    }

    // GOES-West: < -110
    if (lon < kGoesSplitLon) {
        return makeProfile(LAYER_GOES_WEST, 10, 2, "GOES-West", false);
    }

    // GOES-East: -110 to -15
    return makeProfile(LAYER_GOES_EAST, 10, 2, "GOES-East", false);
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

time_t snapToNearestGibsTime(const time_t* times, int count, time_t t, int maxOffsetSec) {
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
