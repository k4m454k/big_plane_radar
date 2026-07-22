#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <algorithm>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <math.h>

#include "airports.h"
#include "airports_iata.h"
#include "map_background.h"
#include "panel_display.h"

#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID ""
#endif
#ifndef DEFAULT_WIFI_PASSWORD
#define DEFAULT_WIFI_PASSWORD ""
#endif
#ifndef DEFAULT_LAT
#define DEFAULT_LAT 51.507400
#endif
#ifndef DEFAULT_LON
#define DEFAULT_LON -0.127800
#endif
#ifndef DEFAULT_MAP_PROVIDER
#define DEFAULT_MAP_PROVIDER 0
#endif
#ifndef DEFAULT_STADIA_API_KEY
#define DEFAULT_STADIA_API_KEY ""
#endif

enum class MapProvider : uint8_t {
    None = 0,
    Stadia = 1,
};

static constexpr int SCREEN_W = 800;
static constexpr int SCREEN_H = 480;
static constexpr int RADAR_CX = 260;
static constexpr int RADAR_CY = SCREEN_H / 2;
static constexpr int RADAR_RADIUS = 218;
static constexpr int PANEL_X = 520;
static constexpr int MAP_EDGE_MARKER_MARGIN = 5;
static constexpr int PANEL_PAD = 10;
static constexpr int PANEL_TEXT_X = PANEL_X + 42;
static constexpr int PANEL_RIGHT = SCREEN_W - 10;
static constexpr int PANEL_LIST_TOP = 42;
static constexpr int PANEL_ROW_H = 54;
static constexpr uint32_t WIFI_CONNECT_ATTEMPT_MS = 15000;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 12000;
static constexpr uint32_t ADSB_FETCH_INTERVAL_MS = 5000;
static constexpr uint32_t RADAR_DRAW_INTERVAL_MS = 0;
static constexpr uint32_t AIRCRAFT_EXTRAPOLATE_MAX_MS = 30000;
static constexpr uint32_t ROUTE_LOOKUP_INTERVAL_MS = 5000;
static constexpr uint32_t ROUTE_LOOKUP_RETRY_MS = 600000;
static constexpr uint32_t ROUTE_CACHE_STALE_MS = 60000;
static constexpr uint32_t ROUTE_HTTP_TIMEOUT_MS = 2500;
static constexpr uint32_t BOOT_SETUP_WINDOW_MS = 4000;
static constexpr uint32_t TOUCH_LONG_PRESS_MS = 1200;
static constexpr uint32_t TOUCH_TAP_MIN_MS = 50;
static constexpr uint32_t CONFIG_HOLD_NOTICE_MS = 900;
static constexpr float KM_PER_NM = 1.852f;
static constexpr float KM_PER_DEG = 111.0f;
static constexpr size_t MAX_AIRCRAFT = 64;
static constexpr size_t MAX_ROUTE_CACHE = 40;

static auto &screen = PanelDisplay::screen;
static WebServer server(80);
static Preferences prefs;

struct AppConfig {
    String ssid;
    String password;
    double lat = DEFAULT_LAT;
    double lon = DEFAULT_LON;
    bool miles = false;
    bool showRunways = true;
    MapProvider mapProvider = DEFAULT_MAP_PROVIDER == 1
        ? MapProvider::Stadia
        : MapProvider::None;
    String stadiaApiKey = DEFAULT_STADIA_API_KEY;
    bool configured = false;
};

struct Aircraft {
    float lat = 0;
    float lon = 0;
    float noseDeg = 0;
    float trackDeg = 0;
    float gsKnots = 0;
    float verticalRateFpm = 0;
    char callsign[10] = {};
    char type[8] = {};
    char category[4] = {};
    char squawk[5] = {};
    char alt[14] = {};
    char vsi[12] = {};
    float distanceKm = 0;
    float renderLat = 0;
    float renderLon = 0;
    int screenX = 0;
    int screenY = 0;
    uint32_t positionMs = 0;
    bool inside = false;
    bool hasFlight = false;
};

struct RouteCacheEntry {
    char callsign[10] = {};
    char originIata[4] = {};
    char destinationIata[4] = {};
    uint32_t lastSeenMs = 0;
    uint32_t lastLookupMs = 0;
    bool active = false;
    bool hasRoute = false;
    bool lookupDone = false;
};

static AppConfig config;
static Aircraft aircraft[MAX_AIRCRAFT];
static RouteCacheEntry routeCache[MAX_ROUTE_CACHE];
static Aircraft renderAircraft[MAX_AIRCRAFT];
static RouteCacheEntry renderRouteCache[MAX_ROUTE_CACHE];
static size_t aircraftCount = 0;
static String statusText = "BOOT";
static String lastFetchText = "NO DATA";
static bool portalActive = false;
static bool mdnsStarted = false;
static bool webServerStarted = false;
static bool wifiReconnectInProgress = false;
static bool wifiWasConnected = false;
static bool forceAdsbFetch = false;
static bool mapRuntimeReady = false;
static uint32_t wifiReconnectStartedMs = 0;
static uint32_t lastReconnectMs = 0;
static uint32_t lastFetchMs = 0;
static uint32_t lastDrawMs = 0;
static uint32_t lastRouteLookupMs = 0;
static StaticSemaphore_t stateMutexStorage;
static SemaphoreHandle_t stateMutex = nullptr;
static TaskHandle_t networkTaskHandle = nullptr;
static volatile bool networkDataDirty = false;
static bool touchWasDown = false;
static uint32_t touchDownMs = 0;
static bool longPressHandled = false;
static bool configNoticeShown = false;

static void lockState() {
    if (stateMutex != nullptr) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
    }
}

static void unlockState() {
    if (stateMutex != nullptr) {
        xSemaphoreGive(stateMutex);
    }
}

static void presentScreenOrRestart() {
    if (screen.present()) {
        return;
    }
    Serial.println("[display] unrecoverable framebuffer synchronization failure; restarting");
    Serial.flush();
    delay(100);
    ESP.restart();
}

struct RangePreset {
    float outerKm;
    const char *kmLabel;
    const char *miLabel;
};

static const RangePreset ranges[] = {
    {6.7f, "5km", "3mi"},
    {13.3f, "10km", "6mi"},
    {20.0f, "15km", "9mi"},
    {33.3f, "25km", "16mi"},
};
static constexpr size_t RANGE_COUNT = sizeof(ranges) / sizeof(ranges[0]);
static size_t rangeIndex = 1;

static uint16_t colorBg;
static uint16_t colorGrid;
static uint16_t colorText;
static uint16_t colorDim;
static uint16_t colorPlane;
static uint16_t colorRunway;
static uint16_t colorWarn;

enum class BootStatus : uint8_t {
    Pending,
    Running,
    Ok,
    Fail,
    Skip,
    NoKey,
};

enum BootStageId : uint8_t {
    BOOT_LCD,
    BOOT_PALETTE,
    BOOT_CONFIG,
    BOOT_WIFI,
    BOOT_MAP,
    BOOT_SERVICES,
    BOOT_DATA,
    BOOT_INTERFACE,
    BOOT_STAGE_COUNT,
};

struct BootStage {
    const char *label;
    BootStatus status;
    char detail[12];
};

static BootStage bootStages[BOOT_STAGE_COUNT] = {
    {"LCD INIT", BootStatus::Pending, {}},
    {"PALETTE", BootStatus::Pending, {}},
    {"CONFIG LOAD", BootStatus::Pending, {}},
    {"WIFI CONNECTION", BootStatus::Pending, {}},
    {"MAP CACHE", BootStatus::Pending, {}},
    {"WEB SERVICES", BootStatus::Pending, {}},
    {"ADSB DATA", BootStatus::Pending, {}},
    {"INTERFACE", BootStatus::Pending, {}},
};
static bool bootScreenActive = false;

static void logLine(const String &message) {
    Serial.println(message);
    Serial.flush();
}

static void logStep(const char *step) {
    Serial.print("[boot] ");
    Serial.print(millis());
    Serial.print(" ms ");
    Serial.println(step);
    Serial.flush();
}

static float activeOuterKm() {
    return ranges[rangeIndex].outerKm;
}

static const char *rangeLabel() {
    return config.miles ? ranges[rangeIndex].miLabel : ranges[rangeIndex].kmLabel;
}

static void formatDistanceLabel(float km, char *out, size_t outLen) {
    if (outLen == 0) return;
    if (config.miles) {
        snprintf(out, outLen, "%.1fMI", km * 0.621371f);
    } else {
        snprintf(out, outLen, "%.1fKM", km);
    }
}

static void formatSpeedLabel(float knots, char *out, size_t outLen) {
    if (outLen == 0) return;
    out[0] = '\0';
    if (knots <= 1.0f) {
        return;
    }
    snprintf(out, outLen, "%dKT", static_cast<int>(lroundf(knots)));
}

static void setStatus(const String &text) {
    lockState();
    statusText = text;
    networkDataDirty = true;
    unlockState();
    Serial.println("[status] " + text);
}

static const char *bootStatusLabel(BootStatus status) {
    switch (status) {
    case BootStatus::Running: return "RUN";
    case BootStatus::Ok: return "OK";
    case BootStatus::Fail: return "FAIL";
    case BootStatus::Skip: return "SKIP";
    case BootStatus::NoKey: return "NO KEY";
    case BootStatus::Pending:
    default: return "WAIT";
    }
}

static uint16_t bootStatusColor(BootStatus status) {
    switch (status) {
    case BootStatus::Running: return screen.color565(255, 220, 70);
    case BootStatus::Ok: return screen.color565(68, 255, 122);
    case BootStatus::Fail: return screen.color565(255, 75, 90);
    case BootStatus::Skip: return screen.color565(80, 130, 105);
    case BootStatus::NoKey: return screen.color565(255, 180, 70);
    case BootStatus::Pending:
    default: return screen.color565(70, 100, 85);
    }
}

static void drawBootStageLine(const BootStage &stage, int y) {
    const int left = 54;
    const int statusX = 650;
    uint16_t bootBg = screen.color565(1, 6, 5);
    uint16_t bootText = screen.color565(230, 255, 235);
    uint16_t bootDots = screen.color565(35, 75, 55);

    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(2);
    screen.setTextColor(bootText, bootBg);
    screen.drawString(stage.label, left, y);

    int dotX = left + screen.textWidth(stage.label) + 16;
    for (int x = dotX; x < statusX - 18; x += 14) {
        screen.fillRect(x, y + 12, 4, 4, bootDots);
    }

    screen.setTextColor(bootStatusColor(stage.status), bootBg);
    screen.drawString(stage.detail[0] != '\0' ? stage.detail : bootStatusLabel(stage.status), statusX, y);
}

static void drawBootScreen() {
    uint16_t bootBg = screen.color565(1, 6, 5);
    uint16_t bootTitle = screen.color565(68, 255, 122);
    uint16_t bootDim = screen.color565(95, 165, 125);
    uint16_t bootLine = screen.color565(18, 48, 35);

    screen.fillScreen(bootBg);
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(4);
    screen.setTextColor(bootTitle, bootBg);
    screen.drawString("PLANE RADAR", 48, 34);
    screen.setTextSize(2);
    screen.setTextColor(bootDim, bootBg);
    screen.drawString("BOOT SEQUENCE", 54, 90);
    screen.drawWideLine(54, 122, 746, 122, 1.0f, bootLine);

    for (uint8_t i = 0; i < BOOT_STAGE_COUNT; i++) {
        drawBootStageLine(bootStages[i], 142 + i * 36);
    }

    screen.setTextSize(1);
    screen.setTextColor(bootDim, bootBg);
    screen.drawString("LONG PRESS SCREEN FOR SETUP", 54, 444);
    presentScreenOrRestart();
}

static void resetBootScreen() {
    for (uint8_t i = 0; i < BOOT_STAGE_COUNT; i++) {
        bootStages[i].status = BootStatus::Pending;
        bootStages[i].detail[0] = '\0';
    }
    bootScreenActive = true;
    drawBootScreen();
}

static void setBootStage(BootStageId id, BootStatus status, const char *detail = nullptr) {
    bootStages[id].status = status;
    if (detail != nullptr) {
        strlcpy(bootStages[id].detail, detail, sizeof(bootStages[id].detail));
    } else {
        bootStages[id].detail[0] = '\0';
    }
    if (bootScreenActive) {
        drawBootScreen();
    }
}

static void setUnavailableMapBootStatus() {
    if (config.mapProvider == MapProvider::None) {
        setBootStage(BOOT_MAP, BootStatus::Skip);
    } else if (config.stadiaApiKey.isEmpty()) {
        setBootStage(BOOT_MAP, BootStatus::NoKey);
    } else {
        setBootStage(BOOT_MAP, BootStatus::Skip);
    }
}

static bool preloadMapCache() {
    if (config.mapProvider == MapProvider::None) {
        setBootStage(BOOT_MAP, BootStatus::Skip);
        return true;
    }
    if (config.stadiaApiKey.isEmpty()) {
        setBootStage(BOOT_MAP, BootStatus::NoKey);
        return true;
    }
    if (!mapRuntimeReady || WiFi.status() != WL_CONNECTED) {
        setBootStage(BOOT_MAP, BootStatus::Fail);
        return false;
    }

    bool allLoaded = true;
    for (size_t i = 0; i < RANGE_COUNT; i++) {
        char progress[12];
        snprintf(progress, sizeof(progress), "%u/%u",
                 static_cast<unsigned>(i + 1),
                 static_cast<unsigned>(RANGE_COUNT));
        setBootStage(BOOT_MAP, BootStatus::Running, progress);
        bool loaded = RadarMap::background.fetchStadia(
            config.lat,
            config.lon,
            ranges[i].outerKm,
            RADAR_RADIUS,
            config.stadiaApiKey,
            i
        );
        allLoaded = loaded && allLoaded;
    }
    setBootStage(BOOT_MAP, allLoaded ? BootStatus::Ok : BootStatus::Fail);
    return allLoaded;
}

static void drawBootSetupHint(const char *text, uint16_t color) {
    uint16_t bootBg = screen.color565(1, 6, 5);
    screen.fillRect(54, 436, 420, 26, bootBg);
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(1);
    screen.setTextColor(color, bootBg);
    screen.drawString(text, 54, 444);
    presentScreenOrRestart();
}

static void startPortal();

static bool waitForBootSetupHold(uint32_t windowMs) {
    uint32_t start = millis();
    uint32_t pressStart = 0;
    bool wasDown = false;
    bool hintChanged = false;
    uint16_t bootDim = screen.color565(95, 165, 125);
    uint16_t bootWarn = screen.color565(255, 220, 70);
    drawBootSetupHint("HOLD SCREEN FOR SETUP", bootDim);

    while (millis() - start < windowMs) {
        server.handleClient();
        uint16_t x = 0;
        uint16_t y = 0;
        bool down = screen.readTouch(&x, &y);
        uint32_t now = millis();

        if (down && !wasDown) {
            pressStart = now;
            hintChanged = false;
        }
        if (down && !hintChanged && now - pressStart >= CONFIG_HOLD_NOTICE_MS) {
            hintChanged = true;
            drawBootSetupHint("KEEP HOLDING FOR SETUP", bootWarn);
        }
        if (down && now - pressStart >= TOUCH_LONG_PRESS_MS) {
            startPortal();
            touchWasDown = true;
            longPressHandled = true;
            configNoticeShown = false;
            return true;
        }
        wasDown = down;
        delay(20);
    }

    touchWasDown = false;
    longPressHandled = false;
    configNoticeShown = false;
    return false;
}

static String htmlEscape(const String &input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); i++) {
        char c = input[i];
        if (c == '&') out += F("&amp;");
        else if (c == '<') out += F("&lt;");
        else if (c == '>') out += F("&gt;");
        else if (c == '"') out += F("&quot;");
        else out += c;
    }
    return out;
}

static void writeLe16(uint8_t *dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void writeLe32(uint8_t *dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static uint8_t expand5(uint16_t value) {
    return static_cast<uint8_t>((value * 255 + 15) / 31);
}

static uint8_t expand6(uint16_t value) {
    return static_cast<uint8_t>((value * 255 + 31) / 63);
}

static void loadConfig() {
    prefs.begin("plane-radar", false);
    config.ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
    config.password = prefs.getString("pass", DEFAULT_WIFI_PASSWORD);
    config.lat = prefs.getDouble("lat", DEFAULT_LAT);
    config.lon = prefs.getDouble("lon", DEFAULT_LON);
    config.miles = prefs.getBool("miles", false);
    config.showRunways = prefs.getBool("runways", true);
    uint8_t storedMapProvider = prefs.getUChar("map", DEFAULT_MAP_PROVIDER);
    config.mapProvider = storedMapProvider == static_cast<uint8_t>(MapProvider::Stadia)
        ? MapProvider::Stadia
        : MapProvider::None;
    config.stadiaApiKey = prefs.getString("stadiaKey", DEFAULT_STADIA_API_KEY);
    config.configured = prefs.getBool("configured", config.ssid.length() > 0);
    rangeIndex = std::min<size_t>(prefs.getUChar("range", 1), RANGE_COUNT - 1);
}

static void saveConfig() {
    prefs.putString("ssid", config.ssid);
    prefs.putString("pass", config.password);
    prefs.putDouble("lat", config.lat);
    prefs.putDouble("lon", config.lon);
    prefs.putBool("miles", config.miles);
    prefs.putBool("runways", config.showRunways);
    prefs.putUChar("map", static_cast<uint8_t>(config.mapProvider));
    prefs.putString("stadiaKey", config.stadiaApiKey);
    prefs.putBool("configured", config.ssid.length() > 0);
    config.configured = config.ssid.length() > 0;
}

static void saveRange() {
    prefs.putUChar("range", static_cast<uint8_t>(rangeIndex));
}

static void drawStatusScreen(const String &title, const String &body) {
    logLine("[ui] drawStatusScreen: " + title);
    screen.fillScreen(TFT_BLACK);
    screen.setTextColor(TFT_GREEN, TFT_BLACK);
    screen.setTextSize(4);
    screen.setTextDatum(textdatum_t::top_left);
    screen.drawString(title, 28, 28);
    screen.setTextColor(TFT_WHITE, TFT_BLACK);
    screen.setTextSize(2);
    screen.drawString(body, 28, 84);
    presentScreenOrRestart();
}

static void drawDisplayDiagnostics() {
    logStep("display diagnostics start");
    Serial.printf("[display] width=%d height=%d rotation=%d\n", screen.width(), screen.height(), screen.getRotation());
    Serial.flush();

    screen.fillScreen(TFT_RED);
    presentScreenOrRestart();
    delay(350);
    screen.fillScreen(TFT_GREEN);
    presentScreenOrRestart();
    delay(350);
    screen.fillScreen(TFT_BLUE);
    presentScreenOrRestart();
    delay(350);
    screen.fillScreen(TFT_BLACK);
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextColor(TFT_GREEN, TFT_BLACK);
    screen.setTextSize(4);
    screen.drawString("PLANE RADAR DIAG", 24, 24);
    screen.setTextSize(2);
    screen.setTextColor(TFT_WHITE, TFT_BLACK);
    screen.drawString("IF YOU SEE THIS, DISPLAY INIT WORKS.", 24, 82);
    presentScreenOrRestart();
    delay(900);
    logStep("display diagnostics end");
}

static void handleRoot() {
    String body;
    body.reserve(3400);
    body += F("<!doctype html><html><head><meta charset='utf-8'>");
    body += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    body += F("<title>Plane Radar Setup</title>");
    body += F("<style>body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;background:#050805;color:#e8ffe8;margin:24px}label{display:block;margin:14px 0 6px;color:#73ff8a}input,select{width:100%;box-sizing:border-box;padding:10px;background:#111;border:1px solid #295;color:#fff}button{margin-top:18px;padding:12px 18px;background:#19d45a;border:0;color:#001b08;font-weight:700}small{color:#8a9}</style>");
    body += F("</head><body><h1>Plane Radar Setup</h1>");
    body += F("<form method='POST' action='/save'>");
    body += F("<label>Wi-Fi SSID</label><input name='ssid' value='");
    body += htmlEscape(config.ssid);
    body += F("'>");
    body += F("<label>Wi-Fi password</label><input name='pass' type='password' value='");
    body += htmlEscape(config.password);
    body += F("'>");
    body += F("<label>Latitude</label><input name='lat' value='");
    body += String(config.lat, 6);
    body += F("'>");
    body += F("<label>Longitude</label><input name='lon' value='");
    body += String(config.lon, 6);
    body += F("'>");
    body += F("<label><input type='checkbox' name='miles' ");
    if (config.miles) body += F("checked");
    body += F("> Display distances in miles</label>");
    body += F("<label><input type='checkbox' name='runways' ");
    if (config.showRunways) body += F("checked");
    body += F("> Show airports/runways</label>");
    body += F("<label>Map background</label><select name='map'>");
    body += F("<option value='0'");
    if (config.mapProvider == MapProvider::None) body += F(" selected");
    body += F(">None</option><option value='1'");
    if (config.mapProvider == MapProvider::Stadia) body += F(" selected");
    body += F(">Stadia Alidade Smooth Dark</option></select>");
    body += F("<label>Stadia Maps API key</label><input name='stadia_key' type='password' value='");
    body += htmlEscape(config.stadiaApiKey);
    body += F("'>");
    body += F("<small>The radar continues without a map if this is empty or the map request fails.</small>");
    body += F("<button type='submit'>Save and reboot</button></form>");
    body += F("<p><a href='/screenshot.bmp'>Download current screen BMP</a></p>");
    body += F("<p><small>Short tap on radar: range preset. Long press: setup portal. Range is saved.</small></p>");
    body += F("<p><small>Current IP: ");
    body += WiFi.localIP().toString();
    body += F(" AP: 192.168.4.1 Host: plane-radar.local</small></p>");
    body += F("</body></html>");
    server.send(200, "text/html", body);
}

static void handleScreenshot() {
    const uint16_t *fb = screen.displayedFrameBuffer();
    if (fb == nullptr) {
        server.send(503, "text/plain", "Framebuffer unavailable");
        return;
    }

    static constexpr uint32_t rowBytes = SCREEN_W * 3;
    static constexpr uint32_t pixelBytes = rowBytes * SCREEN_H;
    static constexpr uint32_t fileBytes = 54 + pixelBytes;

    uint8_t header[54] = {};
    header[0] = 'B';
    header[1] = 'M';
    writeLe32(header + 2, fileBytes);
    writeLe32(header + 10, 54);
    writeLe32(header + 14, 40);
    writeLe32(header + 18, SCREEN_W);
    writeLe32(header + 22, SCREEN_H);
    writeLe16(header + 26, 1);
    writeLe16(header + 28, 24);
    writeLe32(header + 34, pixelBytes);

    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Content-Disposition", "inline; filename=\"plane-radar.bmp\"");
    server.setContentLength(fileBytes);
    server.send(200, "image/bmp", "");

    WiFiClient client = server.client();
    client.write(header, sizeof(header));

    uint8_t row[rowBytes];
    for (int y = SCREEN_H - 1; y >= 0 && client.connected(); y--) {
        const uint16_t *src = fb + static_cast<size_t>(y) * SCREEN_W;
        for (int x = 0; x < SCREEN_W; x++) {
            uint16_t px = src[x];
            row[x * 3] = expand5(px & 0x1F);
            row[x * 3 + 1] = expand6((px >> 5) & 0x3F);
            row[x * 3 + 2] = expand5((px >> 11) & 0x1F);
        }
        client.write(row, rowBytes);
        delay(1);
    }
}

static void handleSave() {
    String ssid = server.arg("ssid");
    String password = server.arg("pass");
    double lat = server.arg("lat").toDouble();
    double lon = server.arg("lon").toDouble();
    bool miles = server.hasArg("miles");
    bool showRunways = server.hasArg("runways");
    MapProvider mapProvider = server.arg("map").toInt() == 1
        ? MapProvider::Stadia
        : MapProvider::None;
    String stadiaApiKey = server.arg("stadia_key");

    lockState();
    config.ssid = ssid;
    config.password = password;
    config.lat = lat;
    config.lon = lon;
    config.miles = miles;
    config.showRunways = showRunways;
    config.mapProvider = mapProvider;
    config.stadiaApiKey = stadiaApiKey;
    saveConfig();
    unlockState();
    server.send(200, "text/html", "<html><body><h1>Saved</h1><p>Rebooting...</p></body></html>");
    delay(500);
    ESP.restart();
}

static void handleNotFound() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

static void startWebServer() {
    if (webServerStarted) {
        return;
    }
    server.on("/", HTTP_GET, handleRoot);
    server.on("/screenshot", HTTP_GET, handleScreenshot);
    server.on("/screenshot.bmp", HTTP_GET, handleScreenshot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();
    webServerStarted = true;
}

static void startPortal() {
    lockState();
    bool alreadyActive = portalActive;
    if (!alreadyActive) {
        portalActive = true;
    }
    unlockState();
    if (alreadyActive) {
        return;
    }
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("PlaneRadar-Setup");
    startWebServer();
    if (!mdnsStarted && MDNS.begin("plane-radar")) {
        mdnsStarted = true;
    }
    drawStatusScreen("PLANE RADAR SETUP", "Connect to Wi-Fi AP: PlaneRadar-Setup\nOpen http://192.168.4.1\nSet Wi-Fi and radar location.");
    setStatus("SETUP PORTAL");
}

static bool connectWifiOnce(uint32_t timeoutMs) {
    if (config.ssid.length() == 0) {
        return false;
    }
    WiFi.mode(portalActive ? WIFI_AP_STA : WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(config.ssid.c_str(), config.password.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        server.handleClient();
        delay(50);
    }
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    if (!mdnsStarted && MDNS.begin("plane-radar")) {
        mdnsStarted = true;
    }
    if (!portalActive) {
        startWebServer();
    }
    wifiWasConnected = true;
    wifiReconnectInProgress = false;
    Serial.printf("[wifi] connected ip=%s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    setStatus("WIFI OK " + WiFi.localIP().toString());
    return true;
}

static void serviceWifiReconnect(uint32_t now) {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiWasConnected) {
            wifiWasConnected = true;
            wifiReconnectInProgress = false;
            lockState();
            forceAdsbFetch = true;
            unlockState();
            setStatus("WIFI OK " + WiFi.localIP().toString());
            Serial.printf("[wifi] reconnected ip=%s rssi=%d\n",
                          WiFi.localIP().toString().c_str(),
                          WiFi.RSSI());
        }
        return;
    }

    wifiWasConnected = false;
    lockState();
    bool canReconnect = config.configured && config.ssid.length() > 0;
    unlockState();
    if (!canReconnect) {
        return;
    }

    if (wifiReconnectInProgress) {
        if (now - wifiReconnectStartedMs < WIFI_CONNECT_ATTEMPT_MS) {
            return;
        }
        WiFi.disconnect(false, false);
        wifiReconnectInProgress = false;
        lastReconnectMs = now;
        setStatus("WIFI RETRY");
        return;
    }

    if (now - lastReconnectMs < WIFI_RECONNECT_INTERVAL_MS) {
        return;
    }

    String ssid;
    String password;
    bool keepPortal = false;
    lockState();
    ssid = config.ssid;
    password = config.password;
    keepPortal = portalActive;
    unlockState();

    WiFi.mode(keepPortal ? WIFI_AP_STA : WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid.c_str(), password.c_str());
    wifiReconnectInProgress = true;
    wifiReconnectStartedMs = now;
    lastReconnectMs = now;
    setStatus("WIFI CONNECTING");
}

static bool readJsonFloat(const JsonObject &obj, const char *key, float &out) {
    if (obj[key].is<float>() || obj[key].is<int>() || obj[key].is<const char *>()) {
        out = obj[key].as<float>();
        return true;
    }
    return false;
}

static float pickHeading(const JsonObject &plane, bool track) {
    float v = 0;
    if (track && readJsonFloat(plane, "track", v)) return v;
    if (readJsonFloat(plane, "true_heading", v)) return v;
    if (readJsonFloat(plane, "mag_heading", v)) return v;
    if (readJsonFloat(plane, "track", v)) return v;
    if (readJsonFloat(plane, "dir", v)) return v;
    return 0;
}

static float pickSpeed(const JsonObject &plane) {
    float v = 0;
    if (readJsonFloat(plane, "gs", v)) return v;
    if (readJsonFloat(plane, "tas", v)) return v;
    if (readJsonFloat(plane, "ias", v)) return v;
    return 0;
}

static float pickVerticalRate(const JsonObject &plane) {
    float v = 0;
    if (readJsonFloat(plane, "baro_rate", v)) return v;
    if (readJsonFloat(plane, "geom_rate", v)) return v;
    return 0;
}

static void copyJsonStringTrimmed(const JsonObject &obj, const char *key, char *out, size_t outLen) {
    if (outLen == 0) return;
    out[0] = '\0';
    if (!obj[key].is<const char *>()) return;
    const char *s = obj[key].as<const char *>();
    size_t n = strnlen(s, outLen - 1);
    while (n > 0 && s[n - 1] == ' ') {
        n--;
    }
    memcpy(out, s, n);
    out[n] = '\0';
}

static void copySquawkCode(const JsonObject &obj, char *out, size_t outLen) {
    if (outLen < 5) return;
    out[0] = '\0';

    char raw[12] = {};
    if (obj["squawk"].is<const char *>()) {
        copyJsonStringTrimmed(obj, "squawk", raw, sizeof(raw));
    } else if (obj["squawk"].is<int>()) {
        snprintf(raw, sizeof(raw), "%04d", obj["squawk"].as<int>());
    } else {
        return;
    }

    size_t len = 0;
    for (size_t i = 0; raw[i] != '\0' && len < 4; i++) {
        char c = raw[i];
        if (c < '0' || c > '7') continue;
        out[len++] = c;
    }
    out[len] = '\0';
    if (len != 4) {
        out[0] = '\0';
    }
}

static const char *squawkAlertLabel(const char *squawk) {
    if (squawk == nullptr || squawk[0] == '\0') return nullptr;
    if (strcmp(squawk, "7700") == 0) return "EMERGENCY";
    if (strcmp(squawk, "7600") == 0) return "NO RADIO";
    if (strcmp(squawk, "7500") == 0) return "HIJACK";
    return nullptr;
}

static bool normalizeCallsign(const char *input, char *out, size_t outLen) {
    if (outLen == 0) return false;
    out[0] = '\0';
    if (input == nullptr) return false;

    size_t len = 0;
    bool hasAlpha = false;
    for (size_t i = 0; input[i] != '\0' && len + 1 < outLen; i++) {
        unsigned char c = static_cast<unsigned char>(input[i]);
        if (!isalnum(c)) continue;
        char normalized = static_cast<char>(toupper(c));
        if (isalpha(static_cast<unsigned char>(normalized))) {
            hasAlpha = true;
        }
        out[len++] = normalized;
    }
    out[len] = '\0';
    return len >= 3 && hasAlpha;
}

static bool copyIataCode(const JsonObject &obj, const char *key, char *out, size_t outLen) {
    if (outLen < 4) return false;
    out[0] = '\0';
    if (!obj[key].is<const char *>()) return false;
    const char *value = obj[key].as<const char *>();
    size_t len = 0;
    for (size_t i = 0; value[i] != '\0' && len < 3; i++) {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (!isalpha(c)) continue;
        out[len++] = static_cast<char>(toupper(c));
    }
    out[len] = '\0';
    return len == 3;
}

static bool copyAirportIata(const JsonObject &airport, char *out, size_t outLen) {
    return copyIataCode(airport, "iata_code", out, outLen) ||
           copyIataCode(airport, "iata", out, outLen) ||
           copyIataCode(airport, "iataCode", out, outLen);
}

static const char *cityForIata(const char *iata) {
    if (iata == nullptr || strlen(iata) != 3) return nullptr;
    for (size_t i = 0; i < kIataAirportCityCount; i++) {
        if (strcmp(kIataAirportCities[i].iata, iata) == 0) {
            return kIataAirportCities[i].city;
        }
    }
    return nullptr;
}

static RouteCacheEntry *findRouteCacheEntry(const char *callsign) {
    for (size_t i = 0; i < MAX_ROUTE_CACHE; i++) {
        if (routeCache[i].active && strcmp(routeCache[i].callsign, callsign) == 0) {
            return &routeCache[i];
        }
    }
    return nullptr;
}

static RouteCacheEntry *oldestRouteCacheEntry() {
    RouteCacheEntry *oldest = &routeCache[0];
    for (size_t i = 1; i < MAX_ROUTE_CACHE; i++) {
        if (!routeCache[i].active) {
            return &routeCache[i];
        }
        if (routeCache[i].lastSeenMs < oldest->lastSeenMs) {
            oldest = &routeCache[i];
        }
    }
    return oldest;
}

static RouteCacheEntry *touchRouteCacheEntry(const char *callsign, uint32_t now) {
    char normalized[10];
    if (!normalizeCallsign(callsign, normalized, sizeof(normalized))) {
        return nullptr;
    }

    RouteCacheEntry *entry = findRouteCacheEntry(normalized);
    if (entry == nullptr) {
        entry = oldestRouteCacheEntry();
        *entry = RouteCacheEntry();
        strlcpy(entry->callsign, normalized, sizeof(entry->callsign));
        entry->active = true;
    }
    entry->lastSeenMs = now;
    return entry;
}

static void pruneRouteCache(uint32_t now) {
    for (size_t i = 0; i < MAX_ROUTE_CACHE; i++) {
        if (routeCache[i].active && now - routeCache[i].lastSeenMs > ROUTE_CACHE_STALE_MS) {
            routeCache[i] = RouteCacheEntry();
        }
    }
}

static void syncRouteCacheFromAircraft(uint32_t now) {
    for (size_t i = 0; i < aircraftCount; i++) {
        if (aircraft[i].hasFlight) {
            touchRouteCacheEntry(aircraft[i].callsign, now);
        }
    }

    pruneRouteCache(now);
}

static bool lookupRouteForCallsign(RouteCacheEntry &entry) {
    String url = "https://api.adsbdb.com/v0/callsign/";
    url += entry.callsign;

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        return false;
    }
    http.setTimeout(ROUTE_HTTP_TIMEOUT_MS);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        http.end();
        return false;
    }

    JsonDocument filter;
    filter["response"]["flightroute"]["origin"]["iata_code"] = true;
    filter["response"]["flightroute"]["origin"]["iata"] = true;
    filter["response"]["flightroute"]["origin"]["iataCode"] = true;
    filter["response"]["flightroute"]["destination"]["iata_code"] = true;
    filter["response"]["flightroute"]["destination"]["iata"] = true;
    filter["response"]["flightroute"]["destination"]["iataCode"] = true;
    JsonDocument doc;
    DeserializationError err = deserializeJson(
        doc,
        http.getStream(),
        DeserializationOption::Filter(filter)
    );
    http.end();
    if (err) {
        return false;
    }

    JsonObject route = doc["response"]["flightroute"].as<JsonObject>();
    if (route.isNull()) {
        return false;
    }

    char origin[4] = {};
    char destination[4] = {};
    if (!copyAirportIata(route["origin"].as<JsonObject>(), origin, sizeof(origin)) ||
        !copyAirportIata(route["destination"].as<JsonObject>(), destination, sizeof(destination))) {
        return false;
    }

    strlcpy(entry.originIata, origin, sizeof(entry.originIata));
    strlcpy(entry.destinationIata, destination, sizeof(entry.destinationIata));
    entry.hasRoute = true;
    return true;
}

static bool serviceRouteLookup() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    uint32_t now = millis();
    if (now - lastRouteLookupMs < ROUTE_LOOKUP_INTERVAL_MS) {
        return false;
    }

    RouteCacheEntry lookupEntry;
    bool hasCandidate = false;
    lockState();
    if (aircraftCount == 0) {
        unlockState();
        return false;
    }

    for (size_t i = 0; i < MAX_ROUTE_CACHE; i++) {
        RouteCacheEntry &entry = routeCache[i];
        if (!entry.active || entry.hasRoute) continue;
        if (now - entry.lastSeenMs > ROUTE_CACHE_STALE_MS) continue;
        if (entry.lookupDone && now - entry.lastLookupMs < ROUTE_LOOKUP_RETRY_MS) continue;

        lastRouteLookupMs = now;
        entry.lastLookupMs = now;
        entry.lookupDone = true;
        lookupEntry = entry;
        hasCandidate = true;
        break;
    }
    unlockState();

    if (!hasCandidate) {
        return false;
    }

    bool ok = lookupRouteForCallsign(lookupEntry);

    lockState();
    RouteCacheEntry *entry = findRouteCacheEntry(lookupEntry.callsign);
    if (entry != nullptr) {
        if (ok) {
            strlcpy(entry->originIata, lookupEntry.originIata, sizeof(entry->originIata));
            strlcpy(entry->destinationIata, lookupEntry.destinationIata, sizeof(entry->destinationIata));
            entry->hasRoute = true;
            networkDataDirty = true;
        }
        Serial.printf("[route] callsign=%s ok=%d origin=%s destination=%s\n",
                      entry->callsign,
                      ok ? 1 : 0,
                      entry->originIata,
                      entry->destinationIata);
    }
    unlockState();
    Serial.flush();
    return ok;
}

static const RouteCacheEntry *findRouteCacheEntryIn(
    const RouteCacheEntry *entries,
    size_t entryCount,
    const char *callsign
) {
    for (size_t i = 0; i < entryCount; i++) {
        if (entries[i].active && strcmp(entries[i].callsign, callsign) == 0) {
            return &entries[i];
        }
    }
    return nullptr;
}

template <typename Gfx>
static bool routeLabelForCallsign(
    Gfx &g,
    const RouteCacheEntry *entries,
    size_t entryCount,
    const char *callsign,
    int maxWidth,
    char *out,
    size_t outLen
) {
    if (outLen == 0) return false;
    out[0] = '\0';
    char normalized[10];
    if (!normalizeCallsign(callsign, normalized, sizeof(normalized))) {
        return false;
    }

    const RouteCacheEntry *entry = findRouteCacheEntryIn(entries, entryCount, normalized);
    if (entry == nullptr || !entry->hasRoute) {
        return false;
    }

    const char *originCity = cityForIata(entry->originIata);
    const char *destinationCity = cityForIata(entry->destinationIata);
    snprintf(out,
             outLen,
             "%s - %s",
             originCity != nullptr ? originCity : entry->originIata,
             destinationCity != nullptr ? destinationCity : entry->destinationIata);

    if (g.textWidth(out) > maxWidth) {
        snprintf(out, outLen, "%s - %s", entry->originIata, entry->destinationIata);
    }
    return out[0] != '\0';
}

static void formatAltitude(const JsonObject &plane, char *out, size_t outLen) {
    if (outLen == 0) return;
    out[0] = '\0';
    if (plane["alt_baro"].is<const char *>()) {
        const char *s = plane["alt_baro"].as<const char *>();
        if (strcmp(s, "ground") == 0) {
            strlcpy(out, "GND", outLen);
            return;
        }
    }
    float alt = 0;
    if (readJsonFloat(plane, "alt_baro", alt) || readJsonFloat(plane, "alt_geom", alt)) {
        snprintf(out, outLen, "%d ft", static_cast<int>(lroundf(alt)));
    }
}

static void formatVerticalRate(float fpm, char *out, size_t outLen) {
    if (outLen == 0) return;
    out[0] = '\0';
    if (fabsf(fpm) < 64.0f) {
        strlcpy(out, "LVL", outLen);
        return;
    }
    snprintf(out, outLen, "%c%d", fpm > 0 ? '^' : 'v', static_cast<int>(lroundf(fabsf(fpm))));
}

static bool isGroundAircraft(const JsonObject &plane) {
    return plane["alt_baro"].is<const char *>() && strcmp(plane["alt_baro"].as<const char *>(), "ground") == 0;
}

static void setLastFetchText(const String &text) {
    lockState();
    lastFetchText = text;
    networkDataDirty = true;
    unlockState();
}

static bool fetchAdsb() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    double centerLat = 0;
    double centerLon = 0;
    float outerKm = 0;
    size_t activeRangeIndex = 0;
    lockState();
    centerLat = config.lat;
    centerLon = config.lon;
    outerKm = activeOuterKm();
    activeRangeIndex = rangeIndex;
    unlockState();

    float fetchScale = 1.25f;
    if (RadarMap::background.isReady(activeRangeIndex)) {
        float mapHalfWidth = static_cast<float>(
            std::max(RADAR_CX, PANEL_X - RADAR_CX)
        );
        float mapHalfHeight = static_cast<float>(
            std::max(RADAR_CY, SCREEN_H - RADAR_CY)
        );
        fetchScale = hypotf(mapHalfWidth, mapHalfHeight) / RADAR_RADIUS;
    }
    float fetchNm = (outerKm * fetchScale) / KM_PER_NM;
    String url = "https://opendata.adsb.fi/api/v3/lat/";
    url += String(centerLat, 6);
    url += "/lon/";
    url += String(centerLon, 6);
    url += "/dist/";
    url += String(fetchNm, 1);

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        setLastFetchText("HTTP BEGIN FAIL");
        return false;
    }
    http.setTimeout(10000);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        setLastFetchText("HTTP " + String(code));
        http.end();
        return false;
    }

    JsonDocument filter;
    const char *fields[] = {
        "lat", "lon", "track", "true_heading", "mag_heading", "dir",
        "gs", "tas", "ias", "baro_rate", "geom_rate", "flight", "hex",
        "t", "category", "squawk", "alt_baro", "alt_geom"
    };
    for (const char *field : fields) {
        filter["ac"][0][field] = true;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(
        doc,
        http.getStream(),
        DeserializationOption::Filter(filter)
    );
    http.end();
    if (err) {
        setLastFetchText("JSON ERROR");
        return false;
    }

    static Aircraft fetchedAircraft[MAX_AIRCRAFT];
    size_t fetchedCount = 0;
    JsonArray ac = doc["ac"].as<JsonArray>();
    uint32_t fetchNow = millis();

    if (!ac.isNull()) {
        for (JsonObject plane : ac) {
            if (fetchedCount >= MAX_AIRCRAFT) break;
            if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) continue;
            if (isGroundAircraft(plane)) continue;

            Aircraft &dst = fetchedAircraft[fetchedCount];
            dst = Aircraft();
            dst.lat = plane["lat"].as<float>();
            dst.lon = plane["lon"].as<float>();
            dst.renderLat = dst.lat;
            dst.renderLon = dst.lon;
            dst.positionMs = fetchNow;
            dst.noseDeg = pickHeading(plane, false);
            dst.trackDeg = pickHeading(plane, true);
            dst.gsKnots = pickSpeed(plane);
            dst.verticalRateFpm = pickVerticalRate(plane);
            copyJsonStringTrimmed(plane, "flight", dst.callsign, sizeof(dst.callsign));
            dst.hasFlight = dst.callsign[0] != '\0';
            if (!dst.hasFlight) {
                copyJsonStringTrimmed(plane, "hex", dst.callsign, sizeof(dst.callsign));
            }
            copyJsonStringTrimmed(plane, "t", dst.type, sizeof(dst.type));
            copyJsonStringTrimmed(plane, "category", dst.category, sizeof(dst.category));
            copySquawkCode(plane, dst.squawk, sizeof(dst.squawk));
            formatAltitude(plane, dst.alt, sizeof(dst.alt));
            formatVerticalRate(dst.verticalRateFpm, dst.vsi, sizeof(dst.vsi));
            fetchedCount++;
        }
    }

    char fetchStatus[24];
    snprintf(fetchStatus, sizeof(fetchStatus), "%u AIRCRAFT", static_cast<unsigned>(fetchedCount));
    lockState();
    if (fetchedCount > 0) {
        memcpy(aircraft, fetchedAircraft, fetchedCount * sizeof(Aircraft));
    }
    aircraftCount = fetchedCount;
    syncRouteCacheFromAircraft(fetchNow);
    lastFetchText = fetchStatus;
    networkDataDirty = true;
    unlockState();

    Serial.printf("[adsb] %s\n", fetchStatus);
    return true;
}

static void offsetKm(float lat, float lon, float &dxKm, float &dyKm, float &distKm) {
    float latRad = static_cast<float>(config.lat * DEG_TO_RAD);
    dxKm = (lon - static_cast<float>(config.lon)) * KM_PER_DEG * cosf(latRad);
    dyKm = (lat - static_cast<float>(config.lat)) * KM_PER_DEG;
    distKm = sqrtf(dxKm * dxKm + dyKm * dyKm);
}

static bool toRadarPoint(float lat, float lon, int &x, int &y, float &distKm) {
    float dxKm = 0;
    float dyKm = 0;
    offsetKm(lat, lon, dxKm, dyKm, distKm);
    float pxPerKm = static_cast<float>(RADAR_RADIUS) / activeOuterKm();
    x = RADAR_CX + static_cast<int>(lroundf(dxKm * pxPerKm));
    y = RADAR_CY - static_cast<int>(lroundf(dyKm * pxPerKm));
    return distKm <= activeOuterKm();
}

static void extrapolatedPosition(const Aircraft &item, uint32_t now, float &lat, float &lon) {
    lat = item.lat;
    lon = item.lon;
    if (item.positionMs == 0 || item.gsKnots < 1.0f) {
        return;
    }

    uint32_t ageMs = now - item.positionMs;
    if (ageMs > AIRCRAFT_EXTRAPOLATE_MAX_MS) {
        ageMs = AIRCRAFT_EXTRAPOLATE_MAX_MS;
    }

    float distanceKm = item.gsKnots * KM_PER_NM * (static_cast<float>(ageMs) / 3600000.0f);
    if (distanceKm < 0.001f) {
        return;
    }

    float trackRad = item.trackDeg * DEG_TO_RAD;
    float northKm = cosf(trackRad) * distanceKm;
    float eastKm = sinf(trackRad) * distanceKm;
    float lonScale = KM_PER_DEG * std::max(0.1f, fabsf(cosf(item.lat * DEG_TO_RAD)));

    lat = item.lat + northKm / KM_PER_DEG;
    lon = item.lon + eastKm / lonScale;
}

static bool isInsideMapViewport(int x, int y) {
    return x >= 0 && x < PANEL_X && y >= 0 && y < SCREEN_H;
}

static void projectToMapEdge(int projectedX, int projectedY, int &x, int &y) {
    float dx = static_cast<float>(projectedX - RADAR_CX);
    float dy = static_cast<float>(projectedY - RADAR_CY);
    float scale = 1.0f;
    float left = static_cast<float>(MAP_EDGE_MARKER_MARGIN);
    float right = static_cast<float>(PANEL_X - 1 - MAP_EDGE_MARKER_MARGIN);
    float top = static_cast<float>(MAP_EDGE_MARKER_MARGIN);
    float bottom = static_cast<float>(SCREEN_H - 1 - MAP_EDGE_MARKER_MARGIN);

    if (dx < 0.0f) scale = std::min(scale, (left - RADAR_CX) / dx);
    if (dx > 0.0f) scale = std::min(scale, (right - RADAR_CX) / dx);
    if (dy < 0.0f) scale = std::min(scale, (top - RADAR_CY) / dy);
    if (dy > 0.0f) scale = std::min(scale, (bottom - RADAR_CY) / dy);

    x = RADAR_CX + static_cast<int>(lroundf(dx * scale));
    y = RADAR_CY + static_cast<int>(lroundf(dy * scale));
}

static void prepareAircraftGeometry(
    Aircraft *items,
    size_t itemCount,
    bool useMapViewport
) {
    uint32_t now = millis();
    for (size_t i = 0; i < itemCount; i++) {
        extrapolatedPosition(items[i], now, items[i].renderLat, items[i].renderLon);
        bool insideRadar = toRadarPoint(
            items[i].renderLat,
            items[i].renderLon,
            items[i].screenX,
            items[i].screenY,
            items[i].distanceKm
        );
        items[i].inside = useMapViewport
            ? isInsideMapViewport(items[i].screenX, items[i].screenY)
            : insideRadar;
    }
    std::sort(items, items + itemCount, [](const Aircraft &a, const Aircraft &b) {
        return a.distanceKm > b.distanceKm;
    });
}

static bool isRotorcraft(const Aircraft &item) {
    return strcmp(item.category, "A7") == 0;
}

static uint8_t planeSizeClass(const Aircraft &item) {
    if (item.category[0] == 'A') {
        if (item.category[1] == '1' || item.category[1] == '2') {
            return 0;
        }
        if (item.category[1] == '4' || item.category[1] == '5') {
            return 2;
        }
    }
    if (item.category[0] == 'B') {
        return 0;
    }
    return 1;
}

template <typename Gfx>
static void drawPlane(Gfx &g, int cx, int cy, float headingDeg, uint8_t sizeClass) {
    int tipLen = 12;
    int tailLen = 8;
    int wingLen = 6;
    if (sizeClass == 0) {
        tipLen = 9;
        tailLen = 6;
        wingLen = 5;
    } else if (sizeClass >= 2) {
        tipLen = 15;
        tailLen = 10;
        wingLen = 8;
    }

    float rad = headingDeg * DEG_TO_RAD;
    float s = sinf(rad);
    float c = cosf(rad);
    int tipX = cx + lroundf(s * tipLen);
    int tipY = cy - lroundf(c * tipLen);
    int tailX = cx - lroundf(s * tailLen);
    int tailY = cy + lroundf(c * tailLen);
    int wingX = lroundf(c * wingLen);
    int wingY = lroundf(s * wingLen);
    g.fillTriangle(tipX, tipY, tailX + wingX, tailY + wingY, tailX - wingX, tailY - wingY, colorPlane);
}

template <typename Gfx>
static void drawAircraftSymbol(Gfx &g, const Aircraft &item, int cx, int cy) {
    if (isRotorcraft(item)) {
        g.drawWideLine(cx - 3, cy - 3, cx + 3, cy + 3, 2.0f, colorPlane);
        g.drawWideLine(cx - 3, cy + 3, cx + 3, cy - 3, 2.0f, colorPlane);

        float rad = item.noseDeg * DEG_TO_RAD;
        int tailX = cx - lroundf(sinf(rad) * 7);
        int tailY = cy + lroundf(cosf(rad) * 7);
        g.drawWideLine(cx, cy, tailX, tailY, 2.0f, colorPlane);
        return;
    }
    drawPlane(g, cx, cy, item.noseDeg, planeSizeClass(item));
}

template <typename Gfx>
static void drawRunways(Gfx &g) {
    if (!config.showRunways) return;
    g.setTextSize(1);
    g.setTextColor(colorRunway, colorBg);
    g.setTextDatum(textdatum_t::middle_center);
    for (size_t i = 0; i < kRunwayCount; i++) {
        int x = 0;
        int y = 0;
        float distKm = 0;
        if (!toRadarPoint(kRunways[i].lat, kRunways[i].lon, x, y, distKm)) continue;
        if (x < -20 || x > SCREEN_W + 20 || y < -20 || y > SCREEN_H + 20) continue;
        float pxPerKm = static_cast<float>(RADAR_RADIUS) / activeOuterKm();
        float half = std::max(8.0f, kRunways[i].lengthKm * pxPerKm * 0.5f);
        float rad = kRunways[i].headingDeg * DEG_TO_RAD;
        float latRad = kRunways[i].lat * DEG_TO_RAD;
        int dx = lroundf(sinf(rad) * half);
        int dy = lroundf(cosf(rad) * half * cosf(latRad));
        g.drawWideLine(x - dx, y + dy, x + dx, y - dy, 2.0f, colorRunway);
        g.drawString(kRunways[i].icao, x, y - 12);
    }
}

template <typename Gfx>
static void appendTokenIfFits(
    Gfx &g,
    char *line,
    size_t lineLen,
    const char *token,
    int maxWidth
) {
    if (lineLen == 0 || token == nullptr || token[0] == '\0') return;
    size_t originalLen = strlen(line);
    size_t separatorLen = originalLen > 0 ? 1 : 0;
    if (originalLen + separatorLen + strlen(token) >= lineLen) return;
    if (separatorLen > 0) {
        line[originalLen++] = ' ';
        line[originalLen] = '\0';
    }
    strlcat(line, token, lineLen);
    if (g.textWidth(line) > maxWidth) {
        line[originalLen - separatorLen] = '\0';
    }
}

template <typename Gfx>
static void drawAircraftList(
    Gfx &g,
    const Aircraft *items,
    size_t itemCount,
    const RouteCacheEntry *routes,
    size_t routeCount,
    const char *emptyStatus
) {
    g.fillRect(PANEL_X, 0, SCREEN_W - PANEL_X, SCREEN_H, colorBg);
    g.drawWideLine(PANEL_X - 8, 18, PANEL_X - 8, SCREEN_H - 18, 1.0f, colorGrid);

    g.setTextDatum(textdatum_t::top_right);
    g.setTextSize(2);
    g.setTextColor(colorDim, colorBg);
    char rangeTitle[24];
    snprintf(rangeTitle, sizeof(rangeTitle), "RANGE %s", rangeLabel());
    g.drawString(rangeTitle, PANEL_RIGHT, 10);

    int textWidth = PANEL_RIGHT - PANEL_TEXT_X;
    int maxRows = (SCREEN_H - PANEL_LIST_TOP - 4) / PANEL_ROW_H;
    int drawn = 0;
    for (int idx = static_cast<int>(itemCount) - 1; idx >= 0 && drawn < maxRows; idx--) {
        const Aircraft &item = items[idx];
        int rowY = PANEL_LIST_TOP + drawn * PANEL_ROW_H;
        int iconX = PANEL_X + 20;
        int iconY = rowY + 23;

        drawAircraftSymbol(g, item, iconX, iconY);

        g.setTextDatum(textdatum_t::top_left);
        g.setTextSize(2);
        g.setTextColor(colorText, colorBg);
        g.drawString(item.callsign[0] ? item.callsign : "????", PANEL_TEXT_X, rowY);

        g.setTextSize(1);
        char detail[96] = {};
        char distance[16];
        char speed[16];
        formatDistanceLabel(item.distanceKm, distance, sizeof(distance));
        formatSpeedLabel(item.gsKnots, speed, sizeof(speed));
        appendTokenIfFits(g, detail, sizeof(detail), item.type, textWidth);
        appendTokenIfFits(g, detail, sizeof(detail), distance, textWidth);
        appendTokenIfFits(g, detail, sizeof(detail), item.alt[0] ? item.alt : "ALT --", textWidth);
        appendTokenIfFits(g, detail, sizeof(detail), item.vsi, textWidth);
        appendTokenIfFits(g, detail, sizeof(detail), speed, textWidth);
        g.setTextColor(colorDim, colorBg);
        g.drawString(detail, PANEL_TEXT_X, rowY + 20);

        const char *squawkAlert = squawkAlertLabel(item.squawk);
        if (squawkAlert != nullptr) {
            char alert[32];
            snprintf(alert, sizeof(alert), "%s %s", item.squawk, squawkAlert);
            g.setTextColor(colorWarn, colorBg);
            g.drawString(alert, PANEL_TEXT_X, rowY + 32);
        } else {
            char route[64];
            if (routeLabelForCallsign(
                    g, routes, routeCount, item.callsign, textWidth, route, sizeof(route))) {
                g.setTextColor(colorRunway, colorBg);
                g.drawString(route, PANEL_TEXT_X, rowY + 32);
            }
        }

        g.drawWideLine(PANEL_X + PANEL_PAD, rowY + PANEL_ROW_H - 4, PANEL_RIGHT, rowY + PANEL_ROW_H - 4, 1.0f, colorGrid);
        drawn++;
    }

    if (drawn == 0) {
        g.setTextDatum(textdatum_t::top_left);
        g.setTextSize(1);
        g.setTextColor(colorDim, colorBg);
        g.drawString(WiFi.status() == WL_CONNECTED ? "NO AIRCRAFT" : emptyStatus, PANEL_TEXT_X, PANEL_LIST_TOP);
    }
}

static void drawRadar() {
    static uint32_t drawCounter = 0;
    drawCounter++;

    size_t renderCount = 0;
    char emptyStatus[64];
    size_t renderRangeIndex = 0;
    lockState();
    renderCount = aircraftCount;
    if (renderCount > 0) {
        memcpy(renderAircraft, aircraft, renderCount * sizeof(Aircraft));
    }
    memcpy(renderRouteCache, routeCache, sizeof(renderRouteCache));
    strlcpy(emptyStatus, statusText.c_str(), sizeof(emptyStatus));
    renderRangeIndex = rangeIndex;
    unlockState();

    bool logDraw = drawCounter <= 3 || drawCounter % 120 == 0;
    if (logDraw) {
        Serial.printf("[draw] #%lu begin aircraft=%u wifi=%d w=%d h=%d free_heap=%u free_psram=%u\n",
                      static_cast<unsigned long>(drawCounter),
                      static_cast<unsigned>(renderCount),
                      WiFi.status(),
                      screen.width(),
                      screen.height(),
                      static_cast<unsigned>(ESP.getFreeHeap()),
                      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    }
    auto &g = screen;
    g.startWrite();
    bool mapVisible = RadarMap::background.draw(g, renderRangeIndex);
    if (!mapVisible) {
        g.fillScreen(colorBg);
    }
    prepareAircraftGeometry(renderAircraft, renderCount, mapVisible);
    int cx = RADAR_CX;
    int cy = RADAR_CY;
    int radius = RADAR_RADIUS;

    for (int i = 1; i <= 4; i++) {
        int r = (radius * i) / 4;
        g.drawCircle(cx, cy, r, colorGrid);
        g.drawCircle(cx, cy, r - 1, colorGrid);
    }
    g.drawWideLine(cx - radius, cy, cx + radius, cy, 1.0f, colorGrid);
    g.drawWideLine(cx, cy - radius, cx, cy + radius, 1.0f, colorGrid);
    g.fillSmoothCircle(cx, cy, 4, colorText);

    g.setTextDatum(textdatum_t::middle_center);
    g.setTextSize(3);
    g.setTextColor(colorText, colorBg);
    g.drawString("N", cx, 18);
    g.drawString("S", cx, SCREEN_H - 18);
    g.drawString("W", cx - radius - 18, cy);
    g.drawString("E", cx + radius + 18, cy);

    g.setTextSize(2);
    g.setTextColor(colorGrid, colorBg);
    g.drawString(rangeLabel(), cx + radius - 22, cy - 14);

    drawRunways(g);

    for (size_t i = 0; i < renderCount; i++) {
        int x = renderAircraft[i].screenX;
        int y = renderAircraft[i].screenY;
        if (!renderAircraft[i].inside) {
            if (mapVisible) {
                projectToMapEdge(x, y, x, y);
            } else {
                float dxKm = 0;
                float dyKm = 0;
                float distKm = 0;
                offsetKm(renderAircraft[i].renderLat, renderAircraft[i].renderLon, dxKm, dyKm, distKm);
                if (distKm < 0.01f) continue;
                float ang = atan2f(dxKm, dyKm);
                x = cx + lroundf(sinf(ang) * (radius + 12));
                y = cy - lroundf(cosf(ang) * (radius + 12));
            }
            g.fillSmoothCircle(x, y, 4, colorPlane);
            continue;
        }
        if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) continue;
        drawAircraftSymbol(g, renderAircraft[i], x, y);
    }

    g.setTextSize(1);
    for (size_t i = 0; i < renderCount; i++) {
        if (!renderAircraft[i].inside) continue;
        int x = renderAircraft[i].screenX;
        int y = renderAircraft[i].screenY;
        if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) continue;
        bool labelRight = x < cx;
        int tx = labelRight ? x + 16 : x - 16;
        int ty = std::max(10, std::min(SCREEN_H - 28, y - 10));
        g.setTextDatum(labelRight ? textdatum_t::top_left : textdatum_t::top_right);
        g.setTextColor(colorText, colorBg);
        g.drawString(renderAircraft[i].callsign[0] ? renderAircraft[i].callsign : "????", tx, ty);
        g.setTextColor(colorDim, colorBg);
        g.drawString(renderAircraft[i].type, tx, ty + 9);
        g.setTextColor(colorWarn, colorBg);
        char altitudeLine[32];
        snprintf(altitudeLine,
                 sizeof(altitudeLine),
                 "%s%s%s",
                 renderAircraft[i].alt,
                 renderAircraft[i].vsi[0] != '\0' ? " " : "",
                 renderAircraft[i].vsi);
        g.drawString(altitudeLine, tx, ty + 18);
    }

    drawAircraftList(
        g,
        renderAircraft,
        renderCount,
        renderRouteCache,
        MAX_ROUTE_CACHE,
        emptyStatus
    );

    g.endWrite();
    presentScreenOrRestart();
    uint32_t completedAt = millis();
    lockState();
    lastDrawMs = completedAt;
    unlockState();
    if (logDraw) {
        Serial.printf("[draw] #%lu end at=%lu\n",
                      static_cast<unsigned long>(drawCounter),
                      static_cast<unsigned long>(completedAt));
    }
}

static void handleTouch() {
    uint16_t x = 0;
    uint16_t y = 0;
    bool down = screen.readTouch(&x, &y);
    uint32_t now = millis();
    if (down && !touchWasDown) {
        touchDownMs = now;
        longPressHandled = false;
    }
    if (down && !longPressHandled && now - touchDownMs >= TOUCH_LONG_PRESS_MS) {
        longPressHandled = true;
        startPortal();
    }
    if (down && !longPressHandled && !configNoticeShown && now - touchDownMs >= CONFIG_HOLD_NOTICE_MS) {
        configNoticeShown = true;
        setStatus("HOLD FOR SETUP");
    }
    if (!down && touchWasDown) {
        uint32_t held = now - touchDownMs;
        if (!longPressHandled && held >= TOUCH_TAP_MIN_MS && held < TOUCH_LONG_PRESS_MS) {
            lockState();
            rangeIndex = (rangeIndex + 1) % RANGE_COUNT;
            forceAdsbFetch = true;
            networkDataDirty = true;
            unlockState();
            saveRange();
        }
    }
    if (!down) {
        configNoticeShown = false;
    }
    touchWasDown = down;
}

static bool shouldDrawRadarFrame(uint32_t now) {
    lockState();
    bool dirty = networkDataDirty;
    if (dirty) {
        networkDataDirty = false;
    }
    bool hasAircraft = aircraftCount > 0;
    uint32_t previousDrawMs = lastDrawMs;
    unlockState();

    return dirty || (hasAircraft && now - previousDrawMs >= RADAR_DRAW_INTERVAL_MS);
}

static void networkTaskMain(void *) {
    Serial.printf("[task] network start core=%d\n", xPortGetCoreID());
    Serial.flush();

    while (true) {
        uint32_t now = millis();
        serviceWifiReconnect(now);
        if (WiFi.status() == WL_CONNECTED) {
            bool fetchNow = false;
            lockState();
            if (forceAdsbFetch) {
                forceAdsbFetch = false;
                fetchNow = true;
            }
            unlockState();
            if (fetchNow || now - lastFetchMs >= ADSB_FETCH_INTERVAL_MS) {
                fetchAdsb();
                lastFetchMs = millis();
            }

            serviceRouteLookup();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void startNetworkTask() {
    if (networkTaskHandle != nullptr) {
        return;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        networkTaskMain,
        "plane-net",
        12288,
        nullptr,
        1,
        &networkTaskHandle,
        0
    );
    if (ok == pdPASS) {
        Serial.println("[task] network task created on core 0");
    } else {
        Serial.println("[task] network task create failed");
    }
    Serial.flush();
}

static void initPalette() {
    colorBg = screen.color565(2, 8, 7);
    colorGrid = screen.color565(8, 46, 33);
    colorText = screen.color565(235, 255, 238);
    colorDim = screen.color565(110, 190, 145);
    colorPlane = screen.color565(255, 55, 80);
    colorRunway = screen.color565(66, 210, 210);
    colorWarn = screen.color565(255, 220, 70);
}

void setup() {
    Serial.begin(115200);
    uint32_t serialStart = millis();
    while (!Serial && millis() - serialStart < 5000) {
        delay(20);
    }
    delay(250);
    stateMutex = xSemaphoreCreateMutexStatic(&stateMutexStorage);
    if (stateMutex == nullptr) {
        logLine("[task] state mutex create failed");
        while (true) {
            delay(1000);
        }
    }
    logLine("\n=== Plane Radar Display DIAG ===");
    logStep("setup start");
    logStep("display begin");
    if (!screen.begin()) {
        logStep("display failed");
        while (true) {
            delay(1000);
        }
    }
    logStep("display end");
    resetBootScreen();
    setBootStage(BOOT_LCD, BootStatus::Ok);

    logStep("palette begin");
    setBootStage(BOOT_PALETTE, BootStatus::Running);
    initPalette();
    setBootStage(BOOT_PALETTE, BootStatus::Ok);
    logStep("palette end");

    logStep("loadConfig begin");
    setBootStage(BOOT_CONFIG, BootStatus::Running);
    loadConfig();
    mapRuntimeReady = config.mapProvider == MapProvider::Stadia &&
                      !config.stadiaApiKey.isEmpty() &&
                      RadarMap::background.begin(PANEL_X, SCREEN_H, RANGE_COUNT);
    setBootStage(BOOT_CONFIG, BootStatus::Ok);
    Serial.printf("[config] configured=%d ssid_len=%u lat=%.6f lon=%.6f range=%u runways=%d miles=%d map=%u map_key_len=%u\n",
                  config.configured,
                  static_cast<unsigned>(config.ssid.length()),
                  config.lat,
                  config.lon,
                  static_cast<unsigned>(rangeIndex),
                  config.showRunways,
                  config.miles,
                  static_cast<unsigned>(config.mapProvider),
                  static_cast<unsigned>(config.stadiaApiKey.length()));
    Serial.flush();

    if (!config.configured) {
        setBootStage(BOOT_WIFI, BootStatus::Skip);
        setUnavailableMapBootStatus();
        setBootStage(BOOT_SERVICES, BootStatus::Running);
        logStep("startPortal begin");
        startPortal();
        setBootStage(BOOT_SERVICES, BootStatus::Ok);
        setBootStage(BOOT_DATA, BootStatus::Skip);
        logStep("startPortal end");
    } else {
        setBootStage(BOOT_WIFI, BootStatus::Running);
        logStep("connectWifi begin");
        if (!connectWifiOnce(WIFI_CONNECT_ATTEMPT_MS)) {
            setBootStage(BOOT_WIFI, BootStatus::Fail);
            setUnavailableMapBootStatus();
            setStatus("WIFI RETRY");
            setBootStage(BOOT_SERVICES, BootStatus::Running);
            logStep("connect failed, startPortal begin");
            startPortal();
            setBootStage(BOOT_SERVICES, BootStatus::Ok);
            setBootStage(BOOT_DATA, BootStatus::Skip);
            logStep("connect failed, startPortal end");
        } else {
            setBootStage(BOOT_WIFI, BootStatus::Ok);
            setBootStage(BOOT_SERVICES, BootStatus::Ok);
            preloadMapCache();
            setBootStage(BOOT_DATA, BootStatus::Running);
            bool dataOk = fetchAdsb();
            lastFetchMs = millis();
            if (dataOk) {
                setBootStage(BOOT_DATA, BootStatus::Ok);
            } else {
                setBootStage(BOOT_DATA, BootStatus::Fail);
            }
        }
    }
    setBootStage(BOOT_INTERFACE, BootStatus::Running);
    setBootStage(BOOT_INTERFACE, BootStatus::Ok);

    bool setupMode = portalActive;
    if (!setupMode) {
        setupMode = waitForBootSetupHold(BOOT_SETUP_WINDOW_MS);
    }
    bootScreenActive = false;
    startNetworkTask();
    if (setupMode || portalActive) {
        logStep("setup portal active");
        drawStatusScreen("PLANE RADAR SETUP", "Connect to Wi-Fi AP: PlaneRadar-Setup\nOpen http://192.168.4.1\nSet Wi-Fi and radar location.");
        return;
    }

    logStep("drawRadar begin");
    drawRadar();
    logStep("drawRadar end");
}

void loop() {
    server.handleClient();
    handleTouch();

    uint32_t now = millis();
    if (shouldDrawRadarFrame(now)) {
        drawRadar();
    }
    delay(1);
}
