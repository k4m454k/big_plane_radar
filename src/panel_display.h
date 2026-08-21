#pragma once

#include <Arduino.h>

// Glyph constants and the bitmap font live here so tools/preview can share them.
#include "panel_font.h"

static constexpr uint16_t TFT_BLACK = 0x0000;
static constexpr uint16_t TFT_BLUE = 0x001F;
static constexpr uint16_t TFT_GREEN = 0x07E0;
static constexpr uint16_t TFT_RED = 0xF800;
static constexpr uint16_t TFT_WHITE = 0xFFFF;

enum class textdatum_t {
    top_left,
    top_right,
    middle_center,
};

namespace PanelDisplay {

enum class Model : uint8_t {
    TouchLcd7,
    TouchLcd7B,
    TouchLcd5,
    CrowPanel7,
};


class Canvas {
public:
    bool begin();
    bool present();
    bool readTouch(uint16_t *x, uint16_t *y);
    const uint16_t *displayedFrameBuffer() const;

    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) const;
    void fillScreen(uint16_t color);
    void fillRect(int x, int y, int w, int h, uint16_t color);
    void drawPixel(int x, int y, uint16_t color);
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
    void drawWideLine(int x0, int y0, int x1, int y1, float width, uint16_t color);
    void drawCircle(int x0, int y0, int r, uint16_t color);
    void fillCircle(int x0, int y0, int r, uint16_t color);
    void fillSmoothCircle(int x0, int y0, int r, uint16_t color);
    void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color);
    void blitRGB565(int x, int y, int w, int h, const uint16_t *pixels, int stride);
    void blendAlphaMask4(
        int x,
        int y,
        int w,
        int h,
        const uint8_t *packedAlpha,
        uint16_t color
    );

    void setTextSize(uint8_t size);
    void setTextColor(uint16_t fg);
    void setTextColor(uint16_t fg, uint16_t bg);
    void setTextDatum(textdatum_t datum);
    int textWidth(const char *text) const;
    int textWidth(const String &text) const;
    int mediumTextWidth(const char *text) const;
    int mediumTextWidth(const String &text) const;
    void drawString(const String &text, int x, int y);
    void drawString(const char *text, int x, int y);
    void drawMediumString(const String &text, int x, int y);
    void drawMediumString(const char *text, int x, int y);

    // Anti-aliased text from the generated atlas, drawn through the same
    // 4-bit alpha blend the rotated aircraft icons use. Datum-aware like
    // drawString: top_left, top_right and middle_center place the AA baseline.
    enum class AaFace : uint8_t { Small, Large };
    int aaTextWidth(const char *text, AaFace face) const;
    int aaTextWidth(const String &text, AaFace face) const;
    void drawAaString(const char *text, int x, int y, AaFace face, uint16_t color);
    void drawAaString(const String &text, int x, int y, AaFace face, uint16_t color);
    // RGB565 has no alpha, so a card is a solid block; the radius is what keeps
    // it from reading as a spreadsheet row.
    void fillRoundRect(int x, int y, int w, int h, int r, uint16_t color);

    int width() const { return _width; }
    int height() const { return _height; }
    // Pre-clamp controller values, for diagnosing coordinate scaling.
    int lastRawTouchX() const { return _lastRawTouchX; }
    int lastRawTouchY() const { return _lastRawTouchY; }
    uint32_t touchReadCount() const { return _touchReadCount; }
    Model model() const { return _model; }
    const char *modelName() const;
    uint32_t pixelClockHz() const;
    int getRotation() const { return 0; }
    void startWrite() {}
    void endWrite() {}

private:
    uint16_t *_fb = nullptr;
    uint16_t *_driverFb[2] = {nullptr, nullptr};
    int _lastRawTouchX = -1;
    int _lastRawTouchY = -1;
    uint32_t _touchReadCount = 0;
    int _width = 800;
    int _height = 480;
    Model _model = Model::TouchLcd7;
    uint8_t _drawFbIndex = 0;
    bool _usingDriverFrameBuffers = false;
    uint8_t _textSize = 1;
    uint16_t _textFg = TFT_WHITE;
    uint16_t _textBg = TFT_BLACK;
    textdatum_t _datum = textdatum_t::top_left;

    void drawChar(char ch, int x, int y);
    void drawMediumChar(char ch, int x, int y);
};

extern Canvas screen;

} // namespace PanelDisplay
