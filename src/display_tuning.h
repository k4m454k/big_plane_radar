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
// The LI0704122Z datasheet rates DCLK at min 20 MHz, typ 33.3 MHz, max 50 MHz,
// with a typical horizontal total of 928 and vertical total of 525 -- exactly
// the porches configured for this board. 33.3 MHz therefore gives the panel's
// designed 68 Hz refresh.
//
// Everything below 20 MHz is out of spec, which is why 13 MHz (26.7 Hz) and
// 16 MHz (32.8 Hz) flickered so badly: the panel was being driven under its
// rated minimum, not merely tuned low. Elecrow's own demo runs 15 MHz and is
// out of spec too, so it is not a safe reference for this value.
//
// The cost is PSRAM bandwidth: the panel streams pclk * 2 bytes/s, so 33.3 MHz
// is ~67 MB/s. If artefacts appear, step down toward 25 MHz rather than below
// 20. Raising PLANE_RADAR_RGB_BOUNCE_LINES is not an option -- 30 starves the
// TLS handshake and kills the ADS-B feed.
#ifndef PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ
#define PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ (33300000)
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
