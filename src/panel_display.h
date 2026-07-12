#pragma once

#include <Arduino.h>

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

static constexpr int WIDTH = 800;
static constexpr int HEIGHT = 480;

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

    void setTextSize(uint8_t size);
    void setTextColor(uint16_t fg);
    void setTextColor(uint16_t fg, uint16_t bg);
    void setTextDatum(textdatum_t datum);
    int textWidth(const char *text) const;
    int textWidth(const String &text) const;
    void drawString(const String &text, int x, int y);
    void drawString(const char *text, int x, int y);

    int width() const { return WIDTH; }
    int height() const { return HEIGHT; }
    int getRotation() const { return 0; }
    void startWrite() {}
    void endWrite() {}

private:
    uint16_t *_fb = nullptr;
    uint16_t *_driverFb[2] = {nullptr, nullptr};
    uint8_t _drawFbIndex = 0;
    bool _usingDriverFrameBuffers = false;
    uint8_t _textSize = 1;
    uint16_t _textFg = TFT_WHITE;
    uint16_t _textBg = TFT_BLACK;
    textdatum_t _datum = textdatum_t::top_left;

    void drawChar(char ch, int x, int y);
};

extern Canvas screen;

} // namespace PanelDisplay
