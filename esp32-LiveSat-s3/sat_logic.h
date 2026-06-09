#pragma once
#include <cstdint>
#include <cstddef>
#include <ctime>
#include <cmath>
#include <cstring>

struct SatProfile {
    char layer[48];
    char source[20];
    int cadenceMin;
    int lagHours;
    bool isEumetview;
};

// Select satellite source based on longitude
SatProfile selectSatelliteForLon(float lonDeg);

// Detect progressive JPEG from SOF2 marker
bool isProgressiveJpeg(const uint8_t* data, size_t len);

// Snap time to nearest available GIBS time within tolerance
time_t snapToNearestGibsTime(const time_t* times, int count, time_t t, int maxOffsetSec);
