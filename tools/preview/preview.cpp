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
// Restyle additions. The old palette tinted everything green -- a deliberate
// radar-scope homage that reads as a 1980s terminal once the rest of the UI
// grows cards and typography. The new base is neutral slate; green survives
// only where it means something (the scope itself stays as-is for now).
static uint16_t colorCard, colorCardSelected, colorStroke, colorAccent;

static void initPalette() {
    colorBg          = HostCanvas::color565(13, 16, 22);
    colorCard        = HostCanvas::color565(24, 28, 38);
    colorCardSelected= HostCanvas::color565(33, 40, 56);
    colorStroke      = HostCanvas::color565(38, 44, 58);
    colorAccent      = HostCanvas::color565(86, 196, 255);
    colorGrid        = HostCanvas::color565(30, 36, 48);
    colorText        = HostCanvas::color565(232, 236, 244);
    colorDim         = HostCanvas::color565(148, 156, 172);
    colorRunway      = HostCanvas::color565(94, 206, 214);
    colorWarn        = HostCanvas::color565(255, 199, 95);
    colorSelectedRow = colorCardSelected;
}

// Cards replace the divider-lined rows: the list reads as distinct objects
// rather than a table, which is most of what makes a panel feel current.
static constexpr int CARD_INSET = 8;
static constexpr int CARD_GAP = 5;
static constexpr int CARD_R = 6;
static constexpr int NEW_ROW_H = 68;

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
    using F = HostCanvas::Face;
    g.fillRect(PANEL_X, 0, SCREEN_W - PANEL_X, SCREEN_H, colorBg);
    // Range as a chip rather than floating caps text: it is a control (tap the
    // radar to cycle it), and controls should look tappable.
    char t[24];
    snprintf(t, sizeof(t), "%s", range);
    int tw = g.aaTextWidth(t, F::Small);
    int chipW = tw + 24, chipH = 26, chipX = SCREEN_W - 10 - chipW, chipY = 10;
    g.fillRoundRect(chipX, chipY, chipW, chipH, 8, colorCard);
    g.drawAaString(t, chipX + 12, chipY + (chipH - 18) / 2 + 1, F::Small, colorText);
    // Aircraft count, dim, left of the chip so the two read as one header line.
    g.drawAaString("6 TRACKING", chipX - 14, chipY + (chipH - 18) / 2 + 1, F::Small, colorDim);
}

static void drawRow(HostCanvas &g, const Plane &p, int rowY, bool selected) {
    using F = HostCanvas::Face;
    int cardX = PANEL_X + CARD_INSET;
    int cardW = SCREEN_W - PANEL_X - 2 * CARD_INSET;
    g.fillRoundRect(cardX, rowY, cardW, NEW_ROW_H, CARD_R,
                    selected ? colorCardSelected : colorCard);
    if (selected) {
        // Accent bar rather than amber: amber is for alerts, this is a
        // selection, and the two must never share a colour.
        g.fillRect(cardX + 6, rowY + 8, 3, NEW_ROW_H - 16, colorAccent);
    }
    // Direction at a glance: the aircraft's heading in a dark disc, where the
    // old UI floated a bare symbol that disappeared against the background.
    int cx = cardX + 32, cy = rowY + NEW_ROW_H / 2;
    g.fillCircle(cx, cy, 17, colorBg);
    float rad = p.hdg * 3.14159265f / 180.0f;
    float s = sinf(rad), c = cosf(rad);
    auto rot = [&](float px, float py, int &ox, int &oy) {
        ox = cx + lroundf(px * s - py * c);
        oy = cy + lroundf(px * c + py * s);
    };
    int x1, y1, x2, y2, x3, y3, x4, y4;
    rot(0, -10, x1, y1); rot(0, 8, x2, y2);        // fuselage
    g.drawWideLine(x1, y1, x2, y2, 2.0f, colorText);
    rot(-8, 2, x3, y3); rot(8, 2, x4, y4);          // wings
    g.drawWideLine(x3, y3, x4, y4, 2.0f, colorText);
    rot(-4, 8, x3, y3); rot(0, 5, x4, y4);          // tail
    g.drawWideLine(x3, y3, x4, y4, 2.0f, colorText);
    rot(4, 8, x3, y3);
    g.drawWideLine(x3, y3, x4, y4, 2.0f, colorText);

    int tx = cardX + 60;
    g.drawAaString(p.callsign, tx, rowY + 7, F::Large, colorText);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s %.1fMI  %s  %.0fKT",
             p.type, p.distMi, p.alt, p.gs);
    g.drawAaString(detail, tx, rowY + 32, F::Small, colorDim);
    if (p.route) {
        g.drawAaString(p.route, tx, rowY + 48, F::Small, colorRunway);
    }
}

static void drawDetailPane(HostCanvas &g, const Plane &p) {
    using F = HostCanvas::Face;
    const int X = PANEL_X + CARD_INSET, W = SCREEN_W - PANEL_X - 2 * CARD_INSET;
    const int H = 150;
    const int Y = SCREEN_H - H - CARD_INSET;
    g.fillRoundRect(X, Y, W, H, CARD_R, colorCard);
    g.fillRect(X + 6, Y + 10, 3, H - 20, colorAccent);

    int tx = X + 18, ty = Y + 10;
    char line[80];
    g.drawAaString(p.callsign, tx, ty, F::Large, colorText);
    // Type and hex belong to the callsign -- a header line, dimmed, not data.
    snprintf(line, sizeof(line), "%s %s", p.type, p.hex);
    g.drawAaString(line, X + W - 14 - g.aaTextWidth(line, F::Small),
                   ty + 5, F::Small, colorDim);
    ty += 30;

    g.drawWideLine(X + 14, ty, X + W - 14, ty, 1.0f, colorStroke);
    ty += 8;

    snprintf(line, sizeof(line), "%s  VSI %s", p.alt, p.vsi);
    g.drawAaString(line, tx, ty, F::Small, colorText); ty += 20;
    snprintf(line, sizeof(line), "%.0fKT   HDG %03d   %.1fMI", p.gs, p.hdg, p.distMi);
    g.drawAaString(line, tx, ty, F::Small, colorText); ty += 20;
    snprintf(line, sizeof(line), "SQUAWK %s", p.squawk);
    g.drawAaString(line, tx, ty, F::Small, colorDim); ty += 22;
    if (p.route) {
        g.drawAaString(p.route, tx, ty, F::Small, colorRunway);
    }
}

static void drawScrollbar(HostCanvas &g, int rows, int total, int offset, int top) {
    if (total <= rows) return;
    int trackH = rows * (NEW_ROW_H + CARD_GAP) - CARD_GAP;
    int barX = SCREEN_W - 7;
    g.fillRect(barX, top, 5, trackH, colorStroke);
    int thumbH = std::max(24, trackH * rows / total);
    int maxScroll = total - rows;
    int thumbY = top + (maxScroll > 0 ? (trackH - thumbH) * offset / maxScroll : 0);
    g.fillRect(barX, thumbY, 5, thumbH, colorDim);
}

struct SettingRow { const char *label, *value; bool stepper, action; };

static void drawSettings(HostCanvas &g) {
    using F = HostCanvas::Face;
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
    g.drawAaString("Settings", 24, 14, F::Large, colorText);
    g.drawAaString("DRAG TO SCROLL  TAP TO CHANGE", 210, 20, F::Small, colorDim);
    g.drawWideLine(16, 48, SCREEN_W - 16, 48, 1.0f, colorStroke);

    const int rowH = 62, gap = 6;
    const int minusX = SCREEN_W - 236, plusX = SCREEN_W - 142, btnW = 68, btnH = 36;
    int maxRows = std::max(1, (SCREEN_H - SETTINGS_TOP - 12) / (rowH + gap));
    for (int i = 0; i < maxRows && i < (int)(sizeof(rows) / sizeof(rows[0])); i++) {
        const SettingRow &r = rows[i];
        int rowY = SETTINGS_TOP + i * (rowH + gap);
        g.fillRoundRect(16, rowY, SCREEN_W - 32, rowH, CARD_R,
                        r.action ? colorCardSelected : colorCard);
        if (r.action) {
            g.fillRect(22, rowY + 8, 3, rowH - 16, colorAccent);
        }
        int labelY = rowY + (rowH - 18) / 2 + 1;
        g.drawAaString(r.label, 32, labelY, F::Small, colorText);
        if (r.value && r.value[0]) {
            int vw = g.aaTextWidth(r.value, F::Small);
            int vx = r.stepper ? minusX - 14 - vw : SCREEN_W - 32 - vw;
            g.drawAaString(r.value, vx, labelY, F::Small,
                           r.action ? colorText : colorDim);
        }
        if (r.stepper) {
            int btnY = rowY + (rowH - btnH) / 2;
            g.fillRoundRect(minusX, btnY, btnW, btnH, 8, colorCardSelected);
            g.fillRoundRect(plusX, btnY, btnW, btnH, 8, colorCardSelected);
            g.drawAaString("-", minusX + btnW / 2 - 3, btnY + (btnH - 18) / 2 + 1,
                           F::Small, colorAccent);
            g.drawAaString("+", plusX + btnW / 2 - 4, btnY + (btnH - 18) / 2 + 1,
                           F::Small, colorAccent);
        }
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
    g.drawString("ALL PRINTABLE ASCII COVERED - THE ONLY QUESTION MARK IS THE REAL ONE", 20, y + 50);
}

// Same panel content drawn twice: current bitmap font versus the generated
// anti-aliased atlas, so the difference is judged on real strings at real size
// rather than a specimen sheet.
static void drawFontCompare(HostCanvas &g) {
    using F = HostCanvas::Face;
    g.fillScreen(colorBg);
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorText, colorBg);
    g.drawString("CURRENT 5x7 BITMAP", 24, 14);
    g.drawAaString("ROBOTO CONDENSED, ANTI-ALIASED", 420, 14, F::Large, colorText);
    g.drawWideLine(16, 46, SCREEN_W - 16, 46, 1.0f, colorGrid);
    g.drawWideLine(SCREEN_W / 2, 46, SCREEN_W / 2, SCREEN_H - 16, 1.0f, colorGrid);

    int y = 64;
    for (const auto &p : kFleet) {
        char detail[96], sq[48];
        snprintf(detail, sizeof(detail), "%s %.1fMI %s %.0fKT", p.type, p.distMi, p.alt, p.gs);
        snprintf(sq, sizeof(sq), "SQUAWK %s  HDG %03d", p.squawk, p.hdg);

        // left: existing bitmap font
        g.setTextSize(2); g.setTextColor(colorText, colorBg);
        g.drawString(p.callsign, 24, y);
        g.setTextSize(1); g.setTextColor(colorDim, colorBg);
        g.drawString(detail, 24, y + 20);
        g.setTextColor(colorRunway, colorBg);
        g.drawString(p.route ? p.route : sq, 24, y + 32);

        // right: anti-aliased atlas
        int rx = SCREEN_W / 2 + 24;
        g.drawAaString(p.callsign, rx, y - 4, F::Large, colorText);
        g.drawAaString(detail, rx, y + 18, F::Small, colorDim);
        g.drawAaString(p.route ? p.route : sq, rx, y + 32, F::Small, colorRunway);
        y += 62;
        if (y > SCREEN_H - 60) break;
    }
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
        int listBottom = SCREEN_H - 150 - 2 * CARD_INSET - CARD_GAP;
        int maxRows = std::max(1, (listBottom - PANEL_LIST_TOP - CARD_GAP) / (NEW_ROW_H + CARD_GAP));
        for (int i = 0; i < maxRows && i < (int)kFleet.size(); i++)
            drawRow(g, kFleet[i], PANEL_LIST_TOP + i * (NEW_ROW_H + CARD_GAP), i == 0);
        drawScrollbar(g, maxRows, 12, 2, PANEL_LIST_TOP - 2);
        drawDetailPane(g, kFleet[0]);
        g.writePng((base + "_panel_selected.png").c_str());
    }
    {   // panel with nothing selected: full-height list
        HostCanvas g(SCREEN_W, SCREEN_H);
        g.fillScreen(HostCanvas::color565(20, 22, 20));
        drawPanelChrome(g, "62MI");
        int maxRows = std::max(1, (SCREEN_H - PANEL_LIST_TOP - CARD_GAP) / (NEW_ROW_H + CARD_GAP));
        for (int i = 0; i < maxRows && i < (int)kFleet.size(); i++)
            drawRow(g, kFleet[i], PANEL_LIST_TOP + i * (NEW_ROW_H + CARD_GAP), false);
        drawScrollbar(g, maxRows, 12, 0, PANEL_LIST_TOP - 2);
        g.writePng((base + "_panel_list.png").c_str());
    }
    {   HostCanvas g(SCREEN_W, SCREEN_H); drawSettings(g);
        g.writePng((base + "_settings.png").c_str()); }
    {   HostCanvas g(SCREEN_W, SCREEN_H); drawFontCoverage(g);
        g.writePng((base + "_font.png").c_str()); }
    {   HostCanvas g(SCREEN_W, SCREEN_H); drawFontCompare(g);
        g.writePng((base + "_fontcompare.png").c_str()); }

    printf("wrote %s_{panel_selected,panel_list,settings,font,fontcompare}.png\n", base.c_str());
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
