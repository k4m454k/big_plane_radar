#pragma once

#ifndef PLANE_RADAR_RGB_PCLK_HZ
#define PLANE_RADAR_RGB_PCLK_HZ (13 * 1000 * 1000)
#endif

#ifndef PLANE_RADAR_RGB_7B_PCLK_HZ
#define PLANE_RADAR_RGB_7B_PCLK_HZ (30 * 1000 * 1000)
#endif

// Refresh rate is pclk / (htotal * vtotal), and this panel's porches are much
// larger than the Waveshare one's, so the same clock buys far less refresh:
//
//   Waveshare  (800+4+8+8)  * (480+4+8+8)   = 410,000  -> 13 MHz = 31.7 Hz
//   CrowPanel  (800+48+40+40)*(480+31+13+1) = 487,200  -> 13 MHz = 26.7 Hz
//                                                         18 MHz = 36.9 Hz
//                                                         21 MHz = 43.1 Hz
//
// Measured on hardware: 13 MHz (26.7 Hz) and 16 MHz (32.8 Hz) both flicker
// badly on this TN panel; 18 MHz (36.9 Hz) cured the flicker but showed
// artefacts, because at that rate the panel streams 36 MB/s out of PSRAM and
// contends with XIP instruction fetch.
//
// The two symptoms pull in opposite directions -- flicker wants a HIGHER clock,
// artefacts want a LOWER one -- so trading between them cannot win. The way out
// is to stop competing for PSRAM: build with REQUIRE_HIGH_PERF=0 so code runs
// from flash rather than PSRAM, which frees the bandwidth to run the panel fast
// enough to be flicker-free. Bounce lines cannot help here; 30 starves TLS.
#ifndef PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ
#define PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ (21 * 1000 * 1000)
#endif

// 0: auto, 7: ESP32-S3-Touch-LCD-7, 8: ESP32-S3-Touch-LCD-7B,
// 9: Elecrow CrowPanel 7.0 (DIS08070H). Profile 9 is compile-time only:
// it cannot participate in autodetection because its I2C bus sits on pins
// that are RGB data lines on the Waveshare boards.
#ifndef PLANE_RADAR_DISPLAY_PROFILE
#define PLANE_RADAR_DISPLAY_PROFILE 0
#endif

#if PLANE_RADAR_DISPLAY_PROFILE != 0 && \
    PLANE_RADAR_DISPLAY_PROFILE != 7 && \
    PLANE_RADAR_DISPLAY_PROFILE != 8 && \
    PLANE_RADAR_DISPLAY_PROFILE != 9
#error "PLANE_RADAR_DISPLAY_PROFILE must be 0, 7, 8, or 9"
#endif

#define PLANE_RADAR_BOARD_CROWPANEL7 (PLANE_RADAR_DISPLAY_PROFILE == 9)

#ifndef PLANE_RADAR_RGB_BOUNCE_LINES
#define PLANE_RADAR_RGB_BOUNCE_LINES 10
#endif

// Bounce buffers live in internal DRAM and decouple the panel from PSRAM
// latency, so more lines means more tolerance for bandwidth contention. The
// cost is two buffers of (width * lines * 2) bytes -- at 800 px wide, 32 KB per
// step of 10.
//
// 30 lines is NOT viable on the CrowPanel and was tried: it leaves ~134 KB of
// internal heap, and the TLS handshake for the ADS-B fetch then fails with
// "SSL - Memory allocation failed" on every poll, emptying the aircraft list.
// Boot looks entirely healthy, because the allocation that fails happens later.
// 20 is the ceiling here; reach for a lower pixel clock instead.
#if PLANE_RADAR_RGB_BOUNCE_LINES != 10 && PLANE_RADAR_RGB_BOUNCE_LINES != 20
#error "PLANE_RADAR_RGB_BOUNCE_LINES must be 10 or 20"
#endif
