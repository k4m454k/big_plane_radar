#pragma once

#ifndef PLANE_RADAR_RGB_PCLK_HZ
#define PLANE_RADAR_RGB_PCLK_HZ (13 * 1000 * 1000)
#endif

#ifndef PLANE_RADAR_RGB_7B_PCLK_HZ
#define PLANE_RADAR_RGB_7B_PCLK_HZ (30 * 1000 * 1000)
#endif

// Elecrow's own demo runs this panel at 15 MHz, but that demo draws a static
// LVGL UI. The radar streams a continuously redrawn frame out of PSRAM, and the
// Waveshare profile above was deliberately tuned down to 13 MHz for exactly
// that reason. Start conservative so a bad first image means "wrong pin map"
// rather than "marginal timing"; raise to 15 MHz once a picture appears.
#ifndef PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ
#define PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ (13 * 1000 * 1000)
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

#if PLANE_RADAR_RGB_BOUNCE_LINES != 10 && PLANE_RADAR_RGB_BOUNCE_LINES != 20
#error "PLANE_RADAR_RGB_BOUNCE_LINES must be 10 or 20"
#endif
