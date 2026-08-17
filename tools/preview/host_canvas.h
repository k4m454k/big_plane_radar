#pragma once

// A Canvas-compatible software renderer that draws into host memory instead of
// an RGB panel, so panel layout can be previewed without building and flashing.
//
// It deliberately reuses src/panel_font.h rather than carrying its own glyphs:
// the point of the preview is to show what the device will actually draw,
// including the font's gaps. A second copy would drift and quietly lie.

#include "../../src/panel_font.h"

#include "../../src/panel_font_aa.inc"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

enum class textdatum_t { top_left, top_right, middle_center };

namespace preview {

class HostCanvas {
public:
    HostCanvas(int w, int h) : _w(w), _h(h), _fb(static_cast<size_t>(w) * h, 0) {}

    int width() const { return _w; }
    int height() const { return _h; }

    static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    }

    void fillScreen(uint16_t c) { std::fill(_fb.begin(), _fb.end(), c); }

    void drawPixel(int x, int y, uint16_t c) {
        if (x < 0 || y < 0 || x >= _w || y >= _h) return;
        _fb[static_cast<size_t>(y) * _w + x] = c;
    }

    void fillRect(int x, int y, int w, int h, uint16_t c) {
        for (int yy = y; yy < y + h; yy++)
            for (int xx = x; xx < x + w; xx++) drawPixel(xx, yy, c);
    }

    void drawLine(int x0, int y0, int x1, int y1, uint16_t c) {
        int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
        for (;;) {
            drawPixel(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void drawWideLine(int x0, int y0, int x1, int y1, float, uint16_t c) {
        drawLine(x0, y0, x1, y1, c);
    }

    void fillCircle(int cx, int cy, int r, uint16_t c) {
        for (int y = -r; y <= r; y++)
            for (int x = -r; x <= r; x++)
                if (x * x + y * y <= r * r) drawPixel(cx + x, cy + y, c);
    }
    void fillSmoothCircle(int cx, int cy, int r, uint16_t c) { fillCircle(cx, cy, r, c); }
    void drawCircle(int cx, int cy, int r, uint16_t c) {
        for (int a = 0; a < 360; a++)
            drawPixel(cx + int(r * std::cos(a * M_PI / 180)),
                      cy + int(r * std::sin(a * M_PI / 180)), c);
    }

    void setTextSize(uint8_t s) { _size = s ? s : 1; }
    void setTextColor(uint16_t fg) { _fg = fg; _bg = fg; _opaque = false; }
    void setTextColor(uint16_t fg, uint16_t bg) { _fg = fg; _bg = bg; _opaque = true; }
    void setTextDatum(textdatum_t d) { _datum = d; }

    int textWidth(const char *t) const {
        if (!t) return 0;
        int n = static_cast<int>(strlen(t));
        return n ? (n * PanelDisplay::FONT_ADVANCE - 1) * _size : 0;
    }
    int textWidth(const std::string &t) const { return textWidth(t.c_str()); }
    int mediumTextWidth(const char *t) const {
        if (!t) return 0;
        int n = static_cast<int>(strlen(t));
        return n ? n * PanelDisplay::MEDIUM_FONT_ADVANCE - 2 : 0;
    }
    int mediumTextWidth(const std::string &t) const { return mediumTextWidth(t.c_str()); }

    void drawString(const char *t, int x, int y) {
        if (!t) return;
        int sx = x, sy = y;
        if (_datum == textdatum_t::top_right) sx = x - textWidth(t);
        else if (_datum == textdatum_t::middle_center) {
            sx = x - textWidth(t) / 2;
            sy = y - PanelDisplay::FONT_H * _size / 2;
        }
        for (size_t i = 0; t[i]; i++)
            drawChar(t[i], sx + int(i) * PanelDisplay::FONT_ADVANCE * _size, sy);
    }
    void drawString(const std::string &t, int x, int y) { drawString(t.c_str(), x, y); }

    void drawMediumString(const char *t, int x, int y) {
        if (!t) return;
        int sx = x, sy = y;
        if (_datum == textdatum_t::top_right) sx = x - mediumTextWidth(t);
        else if (_datum == textdatum_t::middle_center) {
            sx = x - mediumTextWidth(t) / 2;
            sy = y - PanelDisplay::MEDIUM_FONT_H / 2;
        }
        for (size_t i = 0; t[i]; i++)
            drawMediumChar(t[i], sx + int(i) * PanelDisplay::MEDIUM_FONT_ADVANCE, sy);
    }
    void drawMediumString(const std::string &t, int x, int y) { drawMediumString(t.c_str(), x, y); }

    // Anti-aliased text, using the generated atlas and the same 4-bit alpha
    // blend the firmware's blendAlphaMask4 performs.
    enum class Face { Small, Large };

    int aaTextWidth(const char *t, Face f) const {
        if (!t) return 0;
        const auto *G = (f == Face::Large) ? PanelFontAa::kLargeGlyphs
                                           : PanelFontAa::kSmallGlyphs;
        int w = 0;
        for (size_t i = 0; t[i]; i++) {
            unsigned c = static_cast<unsigned char>(t[i]);
            if (c < PanelFontAa::kFirstChar || c > PanelFontAa::kLastChar) continue;
            w += G[c - PanelFontAa::kFirstChar].advance;
        }
        return w;
    }

    void drawAaString(const char *t, int x, int y, Face f, uint16_t color) {
        if (!t) return;
        const auto *G = (f == Face::Large) ? PanelFontAa::kLargeGlyphs
                                           : PanelFontAa::kSmallGlyphs;
        const uint8_t *B = (f == Face::Large) ? PanelFontAa::kLargeBitmap
                                              : PanelFontAa::kSmallBitmap;
        int pen = x;
        if (_datum == textdatum_t::top_right) pen = x - aaTextWidth(t, f);
        for (size_t i = 0; t[i]; i++) {
            unsigned c = static_cast<unsigned char>(t[i]);
            if (c < PanelFontAa::kFirstChar || c > PanelFontAa::kLastChar) continue;
            const auto &g = G[c - PanelFontAa::kFirstChar];
            if (g.width && g.height)
                blendAlpha4(pen + g.bearingX, y + g.bearingY,
                            g.width, g.height, B + g.offset, color);
            pen += g.advance;
        }
    }

    // Writes a PNG with no external dependency, so the tool stays buildable with
    // nothing but a compiler.
    bool writePng(const char *path) const;

    // Mirrors Canvas::blendAlphaMask4 exactly: 4-bit alpha, two pixels per byte,
    // high nibble first.
    void blendAlpha4(int x, int y, int w, int h, const uint8_t *packed, uint16_t color) {
        int sr = (color >> 11) & 0x1F, sg = (color >> 5) & 0x3F, sb = color & 0x1F;
        for (int row = 0; row < h; row++) {
            for (int col = 0; col < w; col++) {
                size_t idx = static_cast<size_t>(row) * w + col;
                uint8_t byte = packed[idx >> 1];
                uint8_t a = (idx & 1) ? (byte & 0x0F) : (byte >> 4);
                if (!a) continue;
                int px = x + col, py = y + row;
                if (px < 0 || py < 0 || px >= _w || py >= _h) continue;
                uint16_t bgc = _fb[static_cast<size_t>(py) * _w + px];
                int ia = 15 - a;
                int r = (sr * a + ((bgc >> 11) & 0x1F) * ia + 7) / 15;
                int gg = (sg * a + ((bgc >> 5) & 0x3F) * ia + 7) / 15;
                int b = (sb * a + (bgc & 0x1F) * ia + 7) / 15;
                _fb[static_cast<size_t>(py) * _w + px] =
                    static_cast<uint16_t>((r << 11) | (gg << 5) | b);
            }
        }
    }

private:
    void drawChar(char ch, int x, int y) {
        const uint8_t *g = PanelDisplay::glyphFor(ch);
        for (int row = 0; row < PanelDisplay::FONT_H; row++)
            for (int col = 0; col < PanelDisplay::FONT_W; col++) {
                bool on = g[row] & (1 << (PanelDisplay::FONT_W - 1 - col));
                if (!on && !_opaque) continue;
                uint16_t c = on ? _fg : _bg;
                for (int sy = 0; sy < _size; sy++)
                    for (int sx = 0; sx < _size; sx++)
                        drawPixel(x + col * _size + sx, y + row * _size + sy, c);
            }
    }

    void drawMediumChar(char ch, int x, int y) {
        const uint8_t *g = PanelDisplay::glyphFor(ch);
        int oy = 0;
        for (int row = 0; row < PanelDisplay::FONT_H; row++) {
            int rh = PanelDisplay::MEDIUM_ROW_HEIGHTS[row];
            int ox = 0;
            for (int col = 0; col < PanelDisplay::FONT_W; col++) {
                int cw = PanelDisplay::MEDIUM_COLUMN_WIDTHS[col];
                bool on = g[row] & (1 << (PanelDisplay::FONT_W - 1 - col));
                if (on || _opaque)
                    for (int yy = 0; yy < rh; yy++)
                        for (int xx = 0; xx < cw; xx++)
                            drawPixel(x + ox + xx, y + oy + yy, on ? _fg : _bg);
                ox += cw;
            }
            oy += rh;
        }
    }

    int _w, _h;
    std::vector<uint16_t> _fb;
    uint8_t _size = 1;
    uint16_t _fg = 0xFFFF, _bg = 0x0000;
    bool _opaque = true;
    textdatum_t _datum = textdatum_t::top_left;
};

}  // namespace preview
