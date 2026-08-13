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
// The panel's rating is only half the constraint: every pixel is streamed out of
// PSRAM, so the clock also has to fit the SoC's memory bandwidth.
//
//   pclk * 2 bytes/pixel = sustained PSRAM read
//   33.3 MHz -> 66.6 MB/s   measured on hardware: the image drifts sideways.
//   21.0 MHz -> 42.0 MB/s   stable.
//
// Octal PSRAM at 80 MHz gives roughly 40-60 MB/s in practice, and this build
// also fetches code and rodata from PSRAM (XIP), competing for the same bus.
// Past that ceiling the RGB DMA underruns, lines start at the wrong horizontal
// offset and the error accumulates as a sideways walk.
//
// So the usable window is bounded at both ends: below 20 MHz is outside the
// panel's spec and flickers, above roughly 21 MHz the SoC cannot keep the
// peripheral fed. Do not raise this without checking for drift on real
// hardware -- it looks fine in a framebuffer screenshot either way.
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
// 30 was previously impossible: it leaves ~134 KB of internal heap, and the TLS
// handshake for the ADS-B fetch then failed with "SSL - Memory allocation
// failed" on every poll, emptying the aircraft list -- while boot looked
// entirely healthy, because the allocation that fails happens later under load.
//
// With ADS-B, routes and map tiles all served over plain HTTP by pi-feed there
// is no TLS context to allocate, which frees roughly the same amount again. 30
// is therefore viable when a local feed is configured. Verify the aircraft list
// still populates after changing this, not just that the device boots.
#if PLANE_RADAR_RGB_BOUNCE_LINES != 10 && \
    PLANE_RADAR_RGB_BOUNCE_LINES != 20 && \
    PLANE_RADAR_RGB_BOUNCE_LINES != 30
#error "PLANE_RADAR_RGB_BOUNCE_LINES must be 10, 20, or 30"
#endif
