// Host-side layout preview for Big Plane Radar's side panel.
//
// Every UI change otherwise costs a build, a flash and a sixty-second boot, and
// the result still cannot be inspected without a screenshot endpoint. This
// renders the same panel geometry with the same font natively, in about a
// second, straight to PNG.
//
//   ./build.sh && ./preview out.png
//
// It is a layout previewer, not an emulator: it draws the panel chrome, rows,
// detail pane and settings screen using the firmware's constants and font so
// spacing, overflow and glyph coverage are faithful. Anything depending on live
// data or the radar projection still needs the device.

#include "host_canvas.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <zlib.h>

using preview::HostCanvas;

// ---- geometry, mirroring src/main.cpp -------------------------------------
static constexpr int SCREEN_W = 800;
static constexpr int SCREEN_H = 480;
static constexpr int PANEL_X = 520;
static constexpr int PANEL_LIST_TOP = 42;
static constexpr int PANEL_ROW_H = 54;
static constexpr int DETAIL_PANE_H = 112;
static constexpr int SETTINGS_TOP = 58;
static constexpr int SETTINGS_ROW_H = 52;
static constexpr int PANEL_PAD = 10;

static uint16_t colorBg, colorGrid, colorText, colorDim, colorRunway, colorWarn, colorSelectedRow;

static void initPalette() {
    colorBg          = HostCanvas::color565(2, 8, 7);
    colorGrid        = HostCanvas::color565(8, 46, 33);
    colorText        = HostCanvas::color565(235, 255, 238);
    colorDim         = HostCanvas::color565(110, 190, 145);
    colorRunway      = HostCanvas::color565(66, 210, 210);
    colorWarn        = HostCanvas::color565(255, 220, 70);
    colorSelectedRow = HostCanvas::color565(5, 28, 19);
}

struct Plane {
    const char *callsign, *type, *hex, *alt, *vsi, *squawk, *route;
    float distMi, gs;
    int hdg;
};

static const std::vector<Plane> kFleet = {
    {"UAL1235","B739","a28947","29000 FT","\x1f""1200","4721","SAN FRANCISCO - CHICAGO",8.4f,438,271},
    {"SWA3877","B737","abb1c2","1975 FT","\x1f""448","6765","LAS VEGAS - SAN JOSE",2.7f,264,183},
    {"N850CE","TBM7","aba78f","2100 FT","\x1f""128","2371",nullptr,1.1f,153,265},
    {"EJA605","C68A","add7c9","14975 FT","^2368","1200","VAN NUYS - LAKELAND",18.5f,360,44},
    {"AAL1728","B38M","acdbef","36025 FT","LVL","7250",nullptr,22.1f,470,74},
    {"N11AL","SF50","a1b8a6","675 FT","\x1f""640","1200",nullptr,7.6f,89,12},
};

// ---- panel drawing --------------------------------------------------------

static void drawPanelChrome(HostCanvas &g, const char *range) {
    g.fillRect(PANEL_X, 0, SCREEN_W - PANEL_X, SCREEN_H, colorBg);
    g.drawWideLine(PANEL_X - 8, 18, PANEL_X - 8, SCREEN_H - 18, 1.0f, colorGrid);
    g.setTextDatum(textdatum_t::top_right);
    g.setTextSize(2);
    g.setTextColor(colorDim, colorBg);
    char t[32];
    snprintf(t, sizeof(t), "RANGE %s", range);
    g.drawString(t, SCREEN_W - 10, 10);
}

static void drawRow(HostCanvas &g, const Plane &p, int rowY, bool selected) {
    uint16_t rowBg = selected ? colorSelectedRow : colorBg;
    if (selected) {
        g.fillRect(PANEL_X + 1, rowY - 2, SCREEN_W - PANEL_X - 2, PANEL_ROW_H - 1, rowBg);
        g.fillRect(PANEL_X + 2, rowY - 2, 3, PANEL_ROW_H - 1, colorWarn);
    }
    const int textX = PANEL_X + 42;
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorText, rowBg);
    g.drawString(p.callsign, textX, rowY);

    g.setTextSize(1);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s %.1fMI %s %s %.0fKT",
             p.type, p.distMi, p.alt, p.vsi, p.gs);
    g.setTextColor(colorDim, rowBg);
    g.drawString(detail, textX, rowY + 20);

    if (p.route) {
        g.setTextColor(colorRunway, rowBg);
        g.drawString(p.route, textX, rowY + 32);
    }
    g.drawWideLine(PANEL_X + PANEL_PAD, rowY + PANEL_ROW_H - 4,
                   SCREEN_W - 10, rowY + PANEL_ROW_H - 4, 1.0f, colorGrid);
}

static void drawDetailPane(HostCanvas &g, const Plane &p) {
    const int X = PANEL_X + 1, W = SCREEN_W - PANEL_X - 2;
    const int Y = SCREEN_H - DETAIL_PANE_H;
    g.fillRect(X, Y, W, DETAIL_PANE_H, colorBg);
    g.drawWideLine(X, Y, X + W, Y, 1.0f, colorGrid);
    g.fillRect(X, Y + 2, 3, DETAIL_PANE_H - 2, colorWarn);

    int tx = X + 12, ty = Y + 6;
    char line[80];
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorText, colorBg);
    g.drawString(p.callsign, tx, ty);
    ty += 24;

    g.setTextSize(1);
    g.setTextColor(colorDim, colorBg);
    snprintf(line, sizeof(line), "%s  %s", p.type, p.hex);
    g.drawString(line, tx, ty); ty += 14;

    g.setTextColor(colorText, colorBg);
    snprintf(line, sizeof(line), "%s  %s", p.alt, p.vsi);
    g.drawString(line, tx, ty); ty += 13;
    snprintf(line, sizeof(line), "%.0fKT  HDG %03d  %.1fMI", p.gs, p.hdg, p.distMi);
    g.drawString(line, tx, ty); ty += 13;
    snprintf(line, sizeof(line), "SQUAWK %s", p.squawk);
    g.setTextColor(colorDim, colorBg);
    g.drawString(line, tx, ty); ty += 13;
    if (p.route) {
        g.setTextColor(colorRunway, colorBg);
        g.drawString(p.route, tx, ty);
    }
}

static void drawScrollbar(HostCanvas &g, int rows, int total, int offset, int top) {
    if (total <= rows) return;
    int trackH = rows * PANEL_ROW_H;
    int barX = SCREEN_W - 7;
    g.fillRect(barX, top, 5, trackH, colorGrid);
    int thumbH = std::max(24, trackH * rows / total);
    int maxScroll = total - rows;
    int thumbY = top + (maxScroll > 0 ? (trackH - thumbH) * offset / maxScroll : 0);
    g.fillRect(barX, thumbY, 5, thumbH, colorText);
}

struct SettingRow { const char *label, *value; bool stepper, action; };

static void drawSettings(HostCanvas &g) {
    static const SettingRow rows[] = {
        {"DISTANCE UNITS","MILES",false,false},
        {"SHOW RUNWAYS","ON",false,false},
        {"AIRCRAFT SYMBOLS","DETAILED",false,false},
        {"AIRPORT COUNT","3",true,false},
        {"AIRPORT RADIUS","200 KM",true,false},
        {"MAP BRIGHTNESS","100 PCT",true,false},
        {"RADAR RANGE","62MI",true,false},
        {"ADS-B SOURCE","LOCAL",false,false},
        {"WEB PORTAL","START",false,true},
        {"SAVE / CLOSE","",false,true},
    };
    g.fillScreen(colorBg);
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorText, colorBg);
    g.drawString("SETTINGS", 24, 14);
    g.setTextSize(1);
    g.setTextColor(colorDim, colorBg);
    g.drawString("DRAG TO SCROLL / TAP TO CHANGE", 190, 24);
    g.drawWideLine(16, 48, SCREEN_W - 16, 48, 1.0f, colorGrid);

    const int minusX = SCREEN_W - 236, plusX = SCREEN_W - 142, btnW = 68, btnH = 36;
    int maxRows = std::max(1, (SCREEN_H - SETTINGS_TOP - 10) / SETTINGS_ROW_H);
    for (int i = 0; i < maxRows && i < (int)(sizeof(rows) / sizeof(rows[0])); i++) {
        const SettingRow &r = rows[i];
        int rowY = SETTINGS_TOP + i * SETTINGS_ROW_H;
        if (r.action) {
            g.fillRect(16, rowY, SCREEN_W - 32, SETTINGS_ROW_H - 6, colorSelectedRow);
            g.fillRect(16, rowY, 3, SETTINGS_ROW_H - 6, colorWarn);
        }
        g.setTextDatum(textdatum_t::top_left);
        g.setTextColor(r.action ? colorWarn : colorText, r.action ? colorSelectedRow : colorBg);
        g.drawMediumString(r.label, 30, rowY + 14);
        if (r.value && r.value[0]) {
            g.setTextDatum(textdatum_t::top_right);
            g.setTextColor(colorDim, r.action ? colorSelectedRow : colorBg);
            g.drawMediumString(r.value, r.stepper ? minusX - 16 : SCREEN_W - 30, rowY + 14);
        }
        if (r.stepper) {
            int btnY = rowY + 4;
            g.fillRect(minusX, btnY, btnW, btnH, colorSelectedRow);
            g.fillRect(plusX, btnY, btnW, btnH, colorSelectedRow);
            g.setTextDatum(textdatum_t::top_left);
            g.setTextColor(colorText, colorSelectedRow);
            g.drawMediumString("-", minusX + btnW / 2 - 4, btnY + 12);
            g.drawMediumString("+", plusX + btnW / 2 - 4, btnY + 12);
        }
        g.drawWideLine(24, rowY + SETTINGS_ROW_H - 5, SCREEN_W - 24,
                       rowY + SETTINGS_ROW_H - 5, 1.0f, colorGrid);
    }
}

// Renders every printable character so missing glyphs are visible at a glance.
static void drawFontCoverage(HostCanvas &g) {
    g.fillScreen(colorBg);
    g.setTextDatum(textdatum_t::top_left);
    g.setTextColor(colorText, colorBg);
    g.setTextSize(2);
    g.drawString("FONT COVERAGE - '?' MEANS NO GLYPH", 20, 14);
    g.setTextSize(2);
    int x = 20, y = 60;
    for (int c = 32; c < 127; c++) {
        char s[3] = {static_cast<char>(c), 0, 0};
        g.setTextColor(colorDim, colorBg);
        g.drawString(s, x, y);
        x += 26;
        if (x > SCREEN_W - 40) { x = 20; y += 30; }
    }
    g.setTextSize(1);
    g.setTextColor(colorWarn, colorBg);
    g.drawString("KNOWN GAPS: PERCENT AND AMPERSAND RENDER AS QUESTION MARK", 20, y + 50);
}

int main(int argc, char **argv) {
    initPalette();
    const char *out = argc > 1 ? argv[1] : "preview.png";
    std::string base(out);
    if (base.size() > 4 && base.substr(base.size() - 4) == ".png") base.resize(base.size() - 4);

    {   // panel with a selection: list shrinks to make room for the detail pane
        HostCanvas g(SCREEN_W, SCREEN_H);
        g.fillScreen(HostCanvas::color565(20, 22, 20));
        drawPanelChrome(g, "16MI");
        int listBottom = SCREEN_H - DETAIL_PANE_H;
        int maxRows = std::max(1, (listBottom - PANEL_LIST_TOP - 2) / PANEL_ROW_H);
        for (int i = 0; i < maxRows && i < (int)kFleet.size(); i++)
            drawRow(g, kFleet[i], PANEL_LIST_TOP + i * PANEL_ROW_H, i == 0);
        drawScrollbar(g, maxRows, 12, 2, PANEL_LIST_TOP - 2);
        drawDetailPane(g, kFleet[0]);
        g.writePng((base + "_panel_selected.png").c_str());
    }
    {   // panel with nothing selected: full-height list
        HostCanvas g(SCREEN_W, SCREEN_H);
        g.fillScreen(HostCanvas::color565(20, 22, 20));
        drawPanelChrome(g, "62MI");
        int maxRows = std::max(1, (SCREEN_H - PANEL_LIST_TOP - 2) / PANEL_ROW_H);
        for (int i = 0; i < maxRows && i < (int)kFleet.size(); i++)
            drawRow(g, kFleet[i], PANEL_LIST_TOP + i * PANEL_ROW_H, false);
        drawScrollbar(g, maxRows, 12, 0, PANEL_LIST_TOP - 2);
        g.writePng((base + "_panel_list.png").c_str());
    }
    {   HostCanvas g(SCREEN_W, SCREEN_H); drawSettings(g);
        g.writePng((base + "_settings.png").c_str()); }
    {   HostCanvas g(SCREEN_W, SCREEN_H); drawFontCoverage(g);
        g.writePng((base + "_font.png").c_str()); }

    printf("wrote %s_{panel_selected,panel_list,settings,font}.png\n", base.c_str());
    return 0;
}

// ---- PNG writer -----------------------------------------------------------
namespace preview {
static void be32(std::vector<uint8_t> &v, uint32_t x) {
    v.push_back(x >> 24); v.push_back(x >> 16); v.push_back(x >> 8); v.push_back(x);
}
static void chunk(std::vector<uint8_t> &o, const char *tag, const std::vector<uint8_t> &d) {
    be32(o, static_cast<uint32_t>(d.size()));
    std::vector<uint8_t> td(tag, tag + 4);
    td.insert(td.end(), d.begin(), d.end());
    o.insert(o.end(), td.begin(), td.end());
    be32(o, static_cast<uint32_t>(crc32(0, td.data(), td.size())));
}

bool HostCanvas::writePng(const char *path) const {
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(_h) * (1 + _w * 3));
    for (int y = 0; y < _h; y++) {
        raw.push_back(0);
        for (int x = 0; x < _w; x++) {
            uint16_t c = _fb[static_cast<size_t>(y) * _w + x];
            raw.push_back(static_cast<uint8_t>(((c >> 11) & 0x1F) * 255 / 31));
            raw.push_back(static_cast<uint8_t>(((c >> 5) & 0x3F) * 255 / 63));
            raw.push_back(static_cast<uint8_t>((c & 0x1F) * 255 / 31));
        }
    }
    uLongf clen = compressBound(raw.size());
    std::vector<uint8_t> comp(clen);
    if (compress2(comp.data(), &clen, raw.data(), raw.size(), 9) != Z_OK) return false;
    comp.resize(clen);

    std::vector<uint8_t> png = {0x89,'P','N','G','\r','\n',0x1A,'\n'};
    std::vector<uint8_t> ihdr;
    be32(ihdr, _w); be32(ihdr, _h);
    ihdr.push_back(8); ihdr.push_back(2); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", comp);
    chunk(png, "IEND", {});

    FILE *f = fopen(path, "wb");
    if (!f) return false;
    fwrite(png.data(), 1, png.size(), f);
    fclose(f);
    return true;
}
}  // namespace preview
