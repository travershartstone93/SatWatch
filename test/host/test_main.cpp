#include "microtest.h"
#include "sat_logic.h"
#include <cstring>

// ── selectSatelliteForLon ──────────────────────────────────────

TEST(goes_east_center) {
    SatProfile p = selectSatelliteForLon(-75.0f);
    EXPECT_EQ(strcmp(p.source, "GOES-East"), 0);
    EXPECT_EQ(p.isEumetview, false);
    EXPECT_EQ(p.cadenceMin, 10);
    EXPECT_EQ(p.lagHours, 2);
}

TEST(goes_west_center) {
    SatProfile p = selectSatelliteForLon(-140.0f);
    EXPECT_EQ(strcmp(p.source, "GOES-West"), 0);
    EXPECT_EQ(p.isEumetview, false);
}

TEST(mtg_center) {
    SatProfile p = selectSatelliteForLon(20.0f);
    EXPECT_EQ(strcmp(p.source, "MTG-GeoColor"), 0);
    EXPECT_EQ(p.isEumetview, true);
    EXPECT_EQ(p.lagHours, 1);
}

TEST(himawari_center) {
    SatProfile p = selectSatelliteForLon(130.0f);
    EXPECT_EQ(strcmp(p.source, "Himawari-IR"), 0);
    EXPECT_EQ(p.isEumetview, false);
    EXPECT_EQ(p.lagHours, 3);
}

// Boundary at -110 exactly
TEST(boundary_minus110_exact) {
    SatProfile p = selectSatelliteForLon(-110.0f);
    EXPECT_EQ(strcmp(p.source, "GOES-East"), 0);
}

TEST(boundary_minus110_minus_epsilon) {
    SatProfile p = selectSatelliteForLon(-110.01f);
    EXPECT_EQ(strcmp(p.source, "GOES-West"), 0);
}

TEST(boundary_minus110_plus_epsilon) {
    SatProfile p = selectSatelliteForLon(-109.99f);
    EXPECT_EQ(strcmp(p.source, "GOES-East"), 0);
}

// Boundary at -15
TEST(boundary_minus15_exact) {
    SatProfile p = selectSatelliteForLon(-15.0f);
    EXPECT_EQ(strcmp(p.source, "MTG-GeoColor"), 0);
}

TEST(boundary_minus15_minus_epsilon) {
    SatProfile p = selectSatelliteForLon(-15.01f);
    EXPECT_EQ(strcmp(p.source, "GOES-East"), 0);
}

TEST(boundary_minus15_plus_epsilon) {
    SatProfile p = selectSatelliteForLon(-14.99f);
    EXPECT_EQ(strcmp(p.source, "MTG-GeoColor"), 0);
}

// Boundary at +80
TEST(boundary_plus80_exact) {
    SatProfile p = selectSatelliteForLon(80.0f);
    EXPECT_EQ(strcmp(p.source, "Himawari-IR"), 0);
}

TEST(boundary_plus80_minus_epsilon) {
    SatProfile p = selectSatelliteForLon(79.99f);
    EXPECT_EQ(strcmp(p.source, "MTG-GeoColor"), 0);
}

TEST(boundary_plus80_plus_epsilon) {
    SatProfile p = selectSatelliteForLon(80.01f);
    EXPECT_EQ(strcmp(p.source, "Himawari-IR"), 0);
}

// Longitude normalization
TEST(lon_wrap_positive) {
    SatProfile p = selectSatelliteForLon(400.0f);  // 400 - 360 = 40 → MTG
    EXPECT_EQ(strcmp(p.source, "MTG-GeoColor"), 0);
}

TEST(lon_wrap_negative) {
    SatProfile p = selectSatelliteForLon(-400.0f);  // -400 + 360 = -40 → GOES-East
    EXPECT_EQ(strcmp(p.source, "GOES-East"), 0);
}

// ── isProgressiveJpeg ──────────────────────────────────────────

TEST(jpeg_baseline_sof0) {
    // SOI + SOF0
    uint8_t data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0xFF, 0xC0};
    EXPECT_EQ(isProgressiveJpeg(data, sizeof(data)), false);
}

TEST(jpeg_progressive_sof2) {
    // SOI + SOF2
    uint8_t data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0xFF, 0xC2};
    EXPECT_EQ(isProgressiveJpeg(data, sizeof(data)), true);
}

TEST(jpeg_truncated_no_sof) {
    // SOI only, no SOF marker at all
    uint8_t data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10};
    EXPECT_EQ(isProgressiveJpeg(data, sizeof(data)), false);
}

TEST(jpeg_empty) {
    EXPECT_EQ(isProgressiveJpeg(nullptr, 0), false);
}

TEST(jpeg_single_byte) {
    uint8_t data[] = {0xFF};
    EXPECT_EQ(isProgressiveJpeg(data, 1), false);
}

TEST(jpeg_sos_before_sof) {
    // SOI + SOS (0xDA) — should return false (past marker region)
    uint8_t data[] = {0xFF, 0xD8, 0xFF, 0xDA, 0xFF, 0xC2};
    EXPECT_EQ(isProgressiveJpeg(data, sizeof(data)), false);
}

// ── snapToNearestGibsTime ──────────────────────────────────────

TEST(snap_exact_hit) {
    time_t times[] = {1000, 1600, 2200, 2800};
    time_t result = snapToNearestGibsTime(times, 4, 1600, 300);
    EXPECT_EQ(result, 1600);
}

TEST(snap_nearest_within_tolerance) {
    time_t times[] = {1000, 1600, 2200, 2800};
    // t=1550, closest is 1600 (dist=50), within tolerance 300
    time_t result = snapToNearestGibsTime(times, 4, 1550, 300);
    EXPECT_EQ(result, 1600);
}

TEST(snap_nearest_before) {
    time_t times[] = {1000, 1600, 2200, 2800};
    // t=1250, closest is 1000 (dist=250) vs 1600 (dist=350), within tolerance 300
    time_t result = snapToNearestGibsTime(times, 4, 1250, 300);
    EXPECT_EQ(result, 1000);
}

TEST(snap_outside_tolerance) {
    time_t times[] = {1000, 2000};
    // t=1500, closest is either 1000 or 2000 (dist=500), tolerance=100
    time_t result = snapToNearestGibsTime(times, 2, 1500, 100);
    EXPECT_EQ(result, (time_t)0);
}

TEST(snap_empty_table) {
    time_t result = snapToNearestGibsTime(nullptr, 0, 1000, 300);
    EXPECT_EQ(result, (time_t)0);
}

TEST(snap_single_entry_match) {
    time_t times[] = {5000};
    time_t result = snapToNearestGibsTime(times, 1, 5050, 100);
    EXPECT_EQ(result, 5000);
}

TEST(snap_single_entry_miss) {
    time_t times[] = {5000};
    time_t result = snapToNearestGibsTime(times, 1, 5200, 100);
    EXPECT_EQ(result, (time_t)0);
}

TEST(snap_first_element) {
    time_t times[] = {1000, 2000, 3000};
    time_t result = snapToNearestGibsTime(times, 3, 1010, 100);
    EXPECT_EQ(result, 1000);
}

TEST(snap_last_element) {
    time_t times[] = {1000, 2000, 3000};
    time_t result = snapToNearestGibsTime(times, 3, 2990, 100);
    EXPECT_EQ(result, 3000);
}

TEST_MAIN()
