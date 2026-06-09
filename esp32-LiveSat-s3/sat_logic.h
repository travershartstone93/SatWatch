#pragma once
#include <stdint.h>
#include <stddef.h>
#include <time.h>

// Satellite source selection
struct SatProfile {
    const char* layer;
    int cadenceMin;
    int lagHours;
    const char* label;
    bool isEumetview;
};
SatProfile selectSatelliteForLonPure(float lonDeg, const char* currentLayer, bool force);

// Progressive JPEG detection
bool isProgressiveJpeg(const uint8_t* data, size_t len);

// Time snapping
time_t snapToNearestTime(const time_t* times, int count, time_t t, int maxOffsetSec);

// Frame time generation
int generateFrameTimes(time_t* out, int maxFrames, time_t fetchEnd, int cadenceSec, int totalFrames);
