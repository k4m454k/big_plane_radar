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

#include "aircraft_icons.h"
#include "app_log.h"
#include "build_diagnostics.h"
#include "app_watchdog.h"
#include "airport_catalog.h"
#include "label_layout.h"
#include "map_background.h"
#include "ota_update.h"
#include "panel_display.h"
#include "weather_time.h"

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

static constexpr char kSetupApName[] = "BigPlaneRadar-Setup";
static constexpr char kMdnsHostname[] = "bigplane-radar";

enum class MapProvider : uint8_t {
    None = 0,
    Stadia = 1,
};

enum class AircraftSymbolStyle : uint8_t {
    DetailedIcons = 0,
    Classic = 1,
};

enum class AirportSelectionMode : uint8_t {
    Automatic = 0,
    Manual = 1,
};

static int SCREEN_W = 800;
static int SCREEN_H = 480;
static int RADAR_CX = 260;
static int RADAR_CY = 240;
static int RADAR_RADIUS = 218;
static int PANEL_X = 520;
static constexpr int MAP_EDGE_MARKER_MARGIN = 5;
static constexpr int PANEL_PAD = 10;
static int PANEL_TEXT_X = 562;
static int PANEL_RIGHT = 790;
static constexpr int PANEL_LIST_TOP = 32;
static constexpr int PANEL_ROW_H = 54;
static constexpr int PANEL_FOOTER_H = 96;
static constexpr int PANEL_DETAIL_H = 148;
static constexpr int RANGE_BUTTON_W = 60;
static constexpr int RANGE_BUTTON_H = 16;
static constexpr int RANGE_BUTTON_GAP = 6;
static constexpr size_t PANEL_MAX_ROWS = 12;
static size_t panelVisibleRows = 8;
static constexpr int AIRCRAFT_LABEL_PADDING = 1;
static constexpr uint8_t MAP_BRIGHTNESS_MIN = 20;
static constexpr uint8_t MAP_BRIGHTNESS_DEFAULT = 100;
static constexpr int LABEL_TEXT_SCALE_MIN = 100;
static constexpr int LABEL_TEXT_SCALE_MAX = 200;
static constexpr int LABEL_TEXT_SCALE_DEFAULT = 100;
static constexpr uint8_t AIRPORT_COUNT_DEFAULT = 1;
static constexpr uint8_t AIRPORT_COUNT_MAX = 3;
static constexpr uint16_t AIRPORT_RADIUS_DEFAULT_KM = 100;
static constexpr uint16_t AIRPORT_RADIUS_MIN_KM = 10;
static constexpr uint16_t AIRPORT_RADIUS_MAX_KM = 500;
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
static constexpr uint32_t TOUCH_RELEASE_DEBOUNCE_MS = 80;
static constexpr int TOUCH_TAP_MOVE_MAX_PX = 18;
static constexpr uint32_t CONFIG_HOLD_NOTICE_MS = 900;
static constexpr uint32_t TRACK_STALE_MS = 60000;
static constexpr uint32_t TRACK_BREAK_GAP_MS = 60000;
static constexpr uint32_t TRACK_SELECTION_MISSING_MS = 15000;
static constexpr size_t TRACK_POINTS_PER_AIRCRAFT = 120;
static constexpr float TRACK_MIN_POINT_DISTANCE_KM = 0.03f;
static constexpr float TRACK_MAX_PLAUSIBLE_SPEED_KNOTS = 1500.0f;
static constexpr float KM_PER_NM = 1.852f;
static constexpr float KM_PER_DEG = 111.0f;
static constexpr size_t MAX_AIRCRAFT = 64;
static constexpr size_t MAX_ROUTE_CACHE = 40;
static constexpr size_t MAX_TYPE_CACHE = 32;
static constexpr size_t MAX_AIRPORT_CITY_CACHE = 128;
static constexpr size_t ROUTE_CITY_MAX_LEN = 48;
static constexpr size_t TYPE_NAME_MAX_LEN = 40;
static constexpr size_t ROUTE_CITY_MIN_PREFIX = 3;

static auto &screen = PanelDisplay::screen;
static WebServer server(80);
static Preferences prefs;

static void configureDisplayLayout() {
    SCREEN_W = screen.width();
    SCREEN_H = screen.height();
    PANEL_X = screen.model() == PanelDisplay::Model::TouchLcd7B
        ? 680
        : 520;
    PANEL_X = std::min(PANEL_X, SCREEN_W - 240);
    RADAR_CX = PANEL_X / 2;
    RADAR_CY = SCREEN_H / 2;
    RADAR_RADIUS = std::min(RADAR_CY - 22, RADAR_CX - 42);
    PANEL_TEXT_X = PANEL_X + 42;
    PANEL_RIGHT = SCREEN_W - 10;
    panelVisibleRows = std::min(
        PANEL_MAX_ROWS,
        static_cast<size_t>(std::max(0, SCREEN_H - PANEL_LIST_TOP - PANEL_FOOTER_H) / PANEL_ROW_H)
    );
}

static int panelFooterTopY() {
    return SCREEN_H - PANEL_FOOTER_H;
}

static int panelDetailTopY() {
    return panelFooterTopY() - PANEL_DETAIL_H;
}

static size_t listRowCapacity(bool selected) {
    int bottom = panelFooterTopY() - (selected ? PANEL_DETAIL_H : 0);
    int available = bottom - PANEL_LIST_TOP;
    if (available < PANEL_ROW_H) {
        return 0;
    }
    return std::min(PANEL_MAX_ROWS, static_cast<size_t>(available / PANEL_ROW_H));
}

struct AppConfig {
    String ssid;
    String password;
    double lat = DEFAULT_LAT;
    double lon = DEFAULT_LON;
    bool miles = false;
    bool showRunways = true;
    AirportSelectionMode airportSelectionMode = AirportSelectionMode::Automatic;
    uint8_t airportCount = AIRPORT_COUNT_DEFAULT;
    uint16_t airportRadiusKm = AIRPORT_RADIUS_DEFAULT_KM;
    String manualAirportIcao;
    bool showLabelCallsign = true;
    bool showLabelType = true;
    bool showLabelAltitude = true;
    bool showLabelVerticalRate = true;
    uint8_t labelTextScalePercent = LABEL_TEXT_SCALE_DEFAULT;
    AircraftSymbolStyle aircraftSymbolStyle = AircraftSymbolStyle::DetailedIcons;
    MapProvider mapProvider = DEFAULT_MAP_PROVIDER == 1
        ? MapProvider::Stadia
        : MapProvider::None;
    String stadiaApiKey = DEFAULT_STADIA_API_KEY;
    uint8_t mapBrightness = MAP_BRIGHTNESS_DEFAULT;
    bool showWeather = true;
    bool tempFahrenheit = false;
    bool clock24 = true;
    String otaPassword = "plane-radar";
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
    char hex[7] = {};
    char type[8] = {};
    char registration[10] = {};
    char category[4] = {};
    char squawk[5] = {};
    char alt[14] = {};
    char vsi[12] = {};
    float distanceKm = 0;
    float altitudeFt = -1.0e9f;
    float renderLat = 0;
    float renderLon = 0;
    int screenX = 0;
    int screenY = 0;
    uint32_t positionMs = 0;
    bool inside = false;
    bool hasFlight = false;
    bool hasTrack = false;
};

struct RadarLabelLine {
    char text[40] = {};
    uint16_t color = 0;
    int width = 0;
    int height = 0;
    int advance = 0;
    uint8_t textSize = 1;
};

struct RadarLabelRender {
    size_t lineCount = 0;
    size_t aircraftIndex = MAX_AIRCRAFT;
    int width = 0;
    int height = 0;
    bool mustShow = false;
    RadarLabelLine lines[3];
};

struct SelectedAirport {
    const AirportCatalogEntry *airport = nullptr;
    float distanceKm = 0;
};

struct TrackPoint {
    float lat = 0;
    float lon = 0;
    uint32_t receivedMs = 0;
};

struct AircraftTrack {
    char hex[7] = {};
    uint16_t start = 0;
    uint16_t count = 0;
    uint32_t lastSeenMs = 0;
    bool active = false;
    TrackPoint points[TRACK_POINTS_PER_AIRCRAFT];
};

struct RouteCacheEntry {
    char callsign[10] = {};
    char originIata[4] = {};
    char destinationIata[4] = {};
    char originIcao[5] = {};
    char destinationIcao[5] = {};
    char originCity[ROUTE_CITY_MAX_LEN] = {};
    char destinationCity[ROUTE_CITY_MAX_LEN] = {};
    uint32_t lastSeenMs = 0;
    uint32_t lastLookupMs = 0;
    bool active = false;
    bool hasRoute = false;
    bool lookupDone = false;
};

struct TypeNameCacheEntry {
    char typeCode[8] = {};
    char fullName[TYPE_NAME_MAX_LEN] = {};
    uint32_t lastLookupMs = 0;
    bool active = false;
    bool hasName = false;
    bool lookupDone = false;
};

struct TouchRect {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
};

struct RadarHitTarget {
    char hex[7] = {};
    int x = 0;
    int y = 0;
    int radius = 0;
};

struct AirportCityCacheEntry {
    char iata[4] = {};
    char city[ROUTE_CITY_MAX_LEN] = {};
    uint32_t lastUsedMs = 0;
    bool active = false;
};

static AppConfig config;
static SelectedAirport selectedAirports[AIRPORT_COUNT_MAX];
static size_t selectedAirportCount = 0;
static Aircraft aircraft[MAX_AIRCRAFT];
static RouteCacheEntry routeCache[MAX_ROUTE_CACHE];
static TypeNameCacheEntry typeNameCache[MAX_TYPE_CACHE];
static AirportCityCacheEntry airportCityCache[MAX_AIRPORT_CITY_CACHE];
static Aircraft renderAircraft[MAX_AIRCRAFT];
static RouteCacheEntry renderRouteCache[MAX_ROUTE_CACHE];
static TypeNameCacheEntry renderTypeNameCache[MAX_TYPE_CACHE];
static AircraftTrack *aircraftTracks = nullptr;
static TrackPoint renderTrack[TRACK_POINTS_PER_AIRCRAFT];
static RadarLabels::LabelLayout aircraftLabelLayout;
static RadarLabels::LabelLayoutInput labelLayoutInputs[MAX_AIRCRAFT];
static RadarLabels::LabelLayoutOutput labelLayoutOutputs[MAX_AIRCRAFT];
static RadarLabels::AircraftObstacle labelAircraftObstacles[MAX_AIRCRAFT];
static RadarLabelRender radarLabels[MAX_AIRCRAFT];
static char selectedAircraftHex[7] = {};
static char visibleListAircraftHex[PANEL_MAX_ROWS][7] = {};
static size_t visibleListRowCount = 0;
static RadarHitTarget radarHitTargets[MAX_AIRCRAFT];
static size_t radarHitCount = 0;
static TouchRect rangeMinusHit;
static TouchRect rangePlusHit;
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
static uint32_t lastTypeLookupMs = 0;
static StaticSemaphore_t stateMutexStorage;
static SemaphoreHandle_t stateMutex = nullptr;
static TaskHandle_t networkTaskHandle = nullptr;
static volatile bool networkDataDirty = false;
static bool touchWasDown = false;
static uint32_t touchDownMs = 0;
static uint32_t touchLastContactMs = 0;
static uint16_t touchDownX = 0;
static uint16_t touchDownY = 0;
static uint16_t touchLastX = 0;
static uint16_t touchLastY = 0;
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

static bool initAircraftTrackCache() {
    size_t bytes = MAX_AIRCRAFT * sizeof(AircraftTrack);
    aircraftTracks = static_cast<AircraftTrack *>(heap_caps_calloc(
        1,
        bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    ));
    if (aircraftTracks == nullptr) {
        RADAR_LOGE("[track] PSRAM allocation failed bytes=%u; tracks disabled\n",
                   static_cast<unsigned>(bytes));
        return false;
    }

    RADAR_LOGD("[track] cache ready tracks=%u points=%u bytes=%u\n",
               static_cast<unsigned>(MAX_AIRCRAFT),
               static_cast<unsigned>(TRACK_POINTS_PER_AIRCRAFT),
               static_cast<unsigned>(bytes));
    return true;
}

static float trackDistanceKm(float latA, float lonA, float latB, float lonB) {
    float averageLat = (latA + latB) * 0.5f * DEG_TO_RAD;
    float dxKm = (lonB - lonA) * KM_PER_DEG * cosf(averageLat);
    float dyKm = (latB - latA) * KM_PER_DEG;
    return sqrtf(dxKm * dxKm + dyKm * dyKm);
}

static float airportCoordinate(int32_t valueE5) {
    return static_cast<float>(valueE5) / 100000.0f;
}

static String normalizeAirportIcao(String value) {
    value.trim();
    value.toUpperCase();
    String normalized;
    normalized.reserve(4);
    for (size_t i = 0; i < value.length() && normalized.length() < 4; i++) {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (isalnum(c)) {
            normalized += static_cast<char>(c);
        }
    }
    return normalized.length() >= 2 ? normalized : String();
}

static const AirportCatalogEntry *findAirportByIcao(const char *icao) {
    if (icao == nullptr || icao[0] == '\0') return nullptr;
    size_t low = 0;
    size_t high = kAirportCatalogCount;
    while (low < high) {
        size_t middle = low + (high - low) / 2;
        int comparison = strcmp(kAirportCatalog[middle].icao, icao);
        if (comparison < 0) {
            low = middle + 1;
        } else if (comparison > 0) {
            high = middle;
        } else {
            return &kAirportCatalog[middle];
        }
    }
    return nullptr;
}

static size_t selectAirportsFor(
    double latitude,
    double longitude,
    AirportSelectionMode mode,
    uint8_t requestedCount,
    uint16_t radiusKm,
    const String &manualIcao,
    SelectedAirport *out
) {
    if (out == nullptr) return 0;
    for (size_t i = 0; i < AIRPORT_COUNT_MAX; i++) {
        out[i] = SelectedAirport();
    }

    if (mode == AirportSelectionMode::Manual) {
        String normalized = normalizeAirportIcao(manualIcao);
        const AirportCatalogEntry *airport = findAirportByIcao(normalized.c_str());
        if (airport == nullptr) return 0;
        out[0].airport = airport;
        out[0].distanceKm = trackDistanceKm(
            static_cast<float>(latitude),
            static_cast<float>(longitude),
            airportCoordinate(airport->latE5),
            airportCoordinate(airport->lonE5)
        );
        return 1;
    }

    size_t limit = std::max<size_t>(1, std::min<size_t>(requestedCount, AIRPORT_COUNT_MAX));
    size_t count = 0;
    for (size_t i = 0; i < kAirportCatalogCount; i++) {
        const AirportCatalogEntry &airport = kAirportCatalog[i];
        float distanceKm = trackDistanceKm(
            static_cast<float>(latitude),
            static_cast<float>(longitude),
            airportCoordinate(airport.latE5),
            airportCoordinate(airport.lonE5)
        );
        if (distanceKm > radiusKm) continue;

        size_t insertAt = count;
        for (size_t j = 0; j < count; j++) {
            if (distanceKm < out[j].distanceKm) {
                insertAt = j;
                break;
            }
        }
        if (insertAt >= limit) continue;
        if (count < limit) count++;
        for (size_t j = count - 1; j > insertAt; j--) {
            out[j] = out[j - 1];
        }
        out[insertAt].airport = &airport;
        out[insertAt].distanceKm = distanceKm;
    }
    return count;
}

static uint8_t clampLabelTextScalePercent(int value) {
    return value >= 150 ? 200 : 100;
}

static uint8_t labelBaseTextSize(int percent) {
    return percent >= 150 ? 2 : 1;
}

static const AirportCatalogEntry *nearestAirport(double latitude, double longitude) {
    const AirportCatalogEntry *best = nullptr;
    float bestDistanceKm = 1.0e30f;
    for (size_t i = 0; i < kAirportCatalogCount; i++) {
        const AirportCatalogEntry &airport = kAirportCatalog[i];
        float distanceKm = trackDistanceKm(
            static_cast<float>(latitude),
            static_cast<float>(longitude),
            airportCoordinate(airport.latE5),
            airportCoordinate(airport.lonE5)
        );
        if (distanceKm < bestDistanceKm) {
            bestDistanceKm = distanceKm;
            best = &airport;
        }
    }
    return best;
}

static void selectConfiguredAirports() {
    selectedAirportCount = selectAirportsFor(
        config.lat,
        config.lon,
        config.airportSelectionMode,
        config.airportCount,
        config.airportRadiusKm,
        config.manualAirportIcao,
        selectedAirports
    );
    RADAR_LOGD("[airport] mode=%u selected=%u radius_km=%u",
               static_cast<unsigned>(config.airportSelectionMode),
               static_cast<unsigned>(selectedAirportCount),
               static_cast<unsigned>(config.airportRadiusKm));
    for (size_t i = 0; i < selectedAirportCount; i++) {
        RADAR_LOGD(" %s=%.1fkm", selectedAirports[i].airport->icao, selectedAirports[i].distanceKm);
    }
    RADAR_LOGD("\n");
}

static AircraftTrack *findAircraftTrackLocked(const char *hex) {
    if (aircraftTracks == nullptr || hex == nullptr || hex[0] == '\0') {
        return nullptr;
    }
    for (size_t i = 0; i < MAX_AIRCRAFT; i++) {
        if (aircraftTracks[i].active && strcmp(aircraftTracks[i].hex, hex) == 0) {
            return &aircraftTracks[i];
        }
    }
    return nullptr;
}

static AircraftTrack *allocateAircraftTrackLocked(const char *hex) {
    if (aircraftTracks == nullptr || hex == nullptr || hex[0] == '\0') {
        return nullptr;
    }

    AircraftTrack *oldest = &aircraftTracks[0];
    for (size_t i = 0; i < MAX_AIRCRAFT; i++) {
        if (!aircraftTracks[i].active) {
            oldest = &aircraftTracks[i];
            break;
        }
        if (aircraftTracks[i].lastSeenMs < oldest->lastSeenMs) {
            oldest = &aircraftTracks[i];
        }
    }

    *oldest = AircraftTrack();
    strlcpy(oldest->hex, hex, sizeof(oldest->hex));
    oldest->active = true;
    return oldest;
}

static TrackPoint &latestTrackPoint(AircraftTrack &track) {
    size_t index = (
        static_cast<size_t>(track.start) + track.count - 1
    ) % TRACK_POINTS_PER_AIRCRAFT;
    return track.points[index];
}

static void appendTrackPoint(
    AircraftTrack &track,
    float lat,
    float lon,
    uint32_t receivedMs
) {
    size_t index = 0;
    if (track.count < TRACK_POINTS_PER_AIRCRAFT) {
        index = (
            static_cast<size_t>(track.start) + track.count
        ) % TRACK_POINTS_PER_AIRCRAFT;
        track.count++;
    } else {
        track.start = static_cast<uint16_t>(
            (track.start + 1) % TRACK_POINTS_PER_AIRCRAFT
        );
        index = (
            static_cast<size_t>(track.start) + track.count - 1
        ) % TRACK_POINTS_PER_AIRCRAFT;
    }
    track.points[index] = TrackPoint{lat, lon, receivedMs};
}

static void updateAircraftTrackLocked(const Aircraft &item, uint32_t now) {
    if (aircraftTracks == nullptr || item.hex[0] == '\0') return;

    AircraftTrack *track = findAircraftTrackLocked(item.hex);
    if (track == nullptr) {
        track = allocateAircraftTrackLocked(item.hex);
    }
    if (track == nullptr) return;

    if (track->count > 0) {
        TrackPoint &latest = latestTrackPoint(*track);
        uint32_t elapsedMs = now - latest.receivedMs;
        float movedKm = trackDistanceKm(latest.lat, latest.lon, item.lat, item.lon);
        bool breakTrack = elapsedMs > TRACK_BREAK_GAP_MS;
        if (!breakTrack && elapsedMs > 0) {
            float elapsedHours = static_cast<float>(elapsedMs) / 3600000.0f;
            float impliedKnots = (movedKm / KM_PER_NM) / elapsedHours;
            breakTrack = impliedKnots > TRACK_MAX_PLAUSIBLE_SPEED_KNOTS;
        }

        if (breakTrack) {
            track->start = 0;
            track->count = 0;
        } else if (movedKm < TRACK_MIN_POINT_DISTANCE_KM) {
            track->lastSeenMs = now;
            return;
        }
    }

    appendTrackPoint(*track, item.lat, item.lon, now);
    track->lastSeenMs = now;
}

static void updateAircraftTracksLocked(
    const Aircraft *items,
    size_t itemCount,
    uint32_t now
) {
    if (aircraftTracks == nullptr) return;

    for (size_t i = 0; i < itemCount; i++) {
        updateAircraftTrackLocked(items[i], now);
    }

    for (size_t i = 0; i < MAX_AIRCRAFT; i++) {
        AircraftTrack &track = aircraftTracks[i];
        if (track.active && now - track.lastSeenMs > TRACK_STALE_MS) {
            track = AircraftTrack();
        }
    }

    if (selectedAircraftHex[0] != '\0') {
        AircraftTrack *selected = findAircraftTrackLocked(selectedAircraftHex);
        if (selected == nullptr ||
            now - selected->lastSeenMs > TRACK_SELECTION_MISSING_MS) {
            RADAR_LOGD("[track] selection cleared missing=%s\n", selectedAircraftHex);
            selectedAircraftHex[0] = '\0';
        }
    }
}

static size_t snapshotSelectedTrackLocked(TrackPoint *out, size_t outCapacity) {
    if (out == nullptr || outCapacity == 0 || selectedAircraftHex[0] == '\0') {
        return 0;
    }

    AircraftTrack *track = findAircraftTrackLocked(selectedAircraftHex);
    if (track == nullptr) return 0;
    size_t count = std::min<size_t>(track->count, outCapacity);
    for (size_t i = 0; i < count; i++) {
        size_t index = (
            static_cast<size_t>(track->start) + i
        ) % TRACK_POINTS_PER_AIRCRAFT;
        out[i] = track->points[index];
    }
    return count;
}

static void presentScreenOrRestart() {
    if (screen.present()) {
        return;
    }
    RADAR_LOGE("[display] unrecoverable framebuffer synchronization failure; restarting\n");
    RADAR_LOGE_FLUSH();
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
    {66.7f, "50km", "31mi"},
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
static uint16_t colorTrackDim;
static uint16_t colorTrackBright;
static uint16_t colorTrackForecast;
static uint16_t colorSelectedRow;
static uint16_t colorSelected;
static uint16_t colorTrajectory;

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
    BOOT_SERVICES,
    BOOT_MAP,
    BOOT_DATA,
    BOOT_INTERFACE,
    BOOT_STAGE_COUNT,
};

struct BootStage {
    const char *label;
    BootStatus status;
    char details[4][88];
    uint8_t detailCount;
    uint32_t startedMs;
    uint32_t elapsedMs;
    bool revealed;
};

static BootStage bootStages[BOOT_STAGE_COUNT] = {
    {"LCD INIT", BootStatus::Pending, {}, 0, 0, 0, false},
    {"PALETTE", BootStatus::Pending, {}, 0, 0, 0, false},
    {"CONFIG LOAD", BootStatus::Pending, {}, 0, 0, 0, false},
    {"WIFI CONNECTION", BootStatus::Pending, {}, 0, 0, 0, false},
    {"WEB SERVICES", BootStatus::Pending, {}, 0, 0, 0, false},
    {"TILE CACHE", BootStatus::Pending, {}, 0, 0, 0, false},
    {"ADSB DATA", BootStatus::Pending, {}, 0, 0, 0, false},
    {"INTERFACE", BootStatus::Pending, {}, 0, 0, 0, false},
};
static bool bootScreenActive = false;

static void logLine(const String &message) {
    RADAR_LOGD("%s\n", message.c_str());
}

static void logStep(const char *step) {
    RADAR_LOGD("[boot] %lu ms %s\n", static_cast<unsigned long>(millis()), step);
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
    RADAR_LOGD("[status] %s\n", text.c_str());
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

static bool bootStageShowsDetails(const BootStage &stage) {
    return stage.detailCount > 0 && (
        stage.status == BootStatus::Running ||
        stage.status == BootStatus::Fail ||
        stage.status == BootStatus::Skip ||
        stage.status == BootStatus::NoKey
    );
}

static int bootStageHeight(const BootStage &stage) {
    return 24 + (bootStageShowsDetails(stage) ? stage.detailCount * 12 : 0);
}

static void formatBootStageStatus(const BootStage &stage, char *out, size_t outLen) {
    if (outLen == 0) return;
    const char *label = bootStatusLabel(stage.status);
    if (stage.status == BootStatus::Running || stage.elapsedMs == 0) {
        strlcpy(out, label, outLen);
        return;
    }
    if (stage.elapsedMs < 1000) {
        snprintf(out, outLen, "%s %lums", label,
                 static_cast<unsigned long>(stage.elapsedMs));
    } else {
        snprintf(out, outLen, "%s %.1fs", label, stage.elapsedMs / 1000.0f);
    }
}

static void drawBootStageLine(const BootStage &stage, uint8_t index, int y) {
    const int numberX = 48;
    const int labelX = 78;
    const int statusRight = SCREEN_W - 52;
    uint16_t bootBg = screen.color565(1, 6, 5);
    uint16_t bootText = screen.color565(230, 255, 235);
    uint16_t bootDim = screen.color565(95, 165, 125);
    uint16_t bootDots = screen.color565(35, 75, 55);

    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(1);
    screen.setTextColor(bootDim, bootBg);
    char stageNumber[4];
    snprintf(stageNumber, sizeof(stageNumber), "%02u", static_cast<unsigned>(index + 1));
    screen.drawString(stageNumber, numberX, y + 2);

    screen.setTextColor(bootText, bootBg);
    screen.drawMediumString(stage.label, labelX, y);

    char status[24];
    formatBootStageStatus(stage, status, sizeof(status));
    int dotX = labelX + screen.mediumTextWidth(stage.label) + 16;
    int statusLeft = statusRight - screen.mediumTextWidth(status);
    for (int x = dotX; x < statusLeft - 14; x += 12) {
        screen.fillRect(x, y + 6, 3, 2, bootDots);
    }

    screen.setTextColor(bootStatusColor(stage.status), bootBg);
    screen.setTextDatum(textdatum_t::top_right);
    screen.drawMediumString(status, statusRight, y);

    if (!bootStageShowsDetails(stage)) return;

    uint16_t detailColor = bootDim;
    if (stage.status == BootStatus::Fail) {
        detailColor = screen.color565(255, 115, 125);
    } else if (stage.status == BootStatus::Skip || stage.status == BootStatus::NoKey) {
        detailColor = screen.color565(205, 165, 85);
    }
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(1);
    screen.setTextColor(detailColor, bootBg);
    for (uint8_t line = 0; line < stage.detailCount; line++) {
        int detailY = y + 17 + line * 12;
        screen.drawString(">", labelX + 8, detailY);
        screen.drawString(stage.details[line], labelX + 24, detailY);
    }
}

static void drawBootScreen() {
    uint16_t bootBg = screen.color565(1, 6, 5);
    uint16_t bootTitle = screen.color565(68, 255, 122);
    uint16_t bootDim = screen.color565(95, 165, 125);
    uint16_t bootLine = screen.color565(18, 48, 35);

    screen.fillScreen(bootBg);
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(3);
    screen.setTextColor(bootTitle, bootBg);
    screen.drawString("PLANE RADAR", 48, 24);
    screen.setTextSize(1);
    screen.setTextColor(bootDim, bootBg);
    screen.drawString("BOOT LOG / ESP32-S3 / 240MHZ", 52, 62);
    screen.drawWideLine(52, 82, SCREEN_W - 52, 82, 1.0f, bootLine);

    uint8_t visibleCount = 0;
    while (visibleCount < BOOT_STAGE_COUNT && bootStages[visibleCount].revealed) {
        visibleCount++;
    }

    const int contentTop = 100;
    const int contentBottom = SCREEN_H - 54;
    int totalHeight = 0;
    for (uint8_t i = 0; i < visibleCount; i++) {
        totalHeight += bootStageHeight(bootStages[i]);
    }

    uint8_t firstVisible = 0;
    while (firstVisible + 1 < visibleCount &&
           totalHeight > contentBottom - contentTop) {
        totalHeight -= bootStageHeight(bootStages[firstVisible]);
        firstVisible++;
    }
    while (firstVisible + 1 < visibleCount && firstVisible > 0 &&
           totalHeight > contentBottom - contentTop - 16) {
        totalHeight -= bootStageHeight(bootStages[firstVisible]);
        firstVisible++;
    }

    int y = contentTop;
    if (firstVisible > 0) {
        char hidden[32];
        snprintf(hidden, sizeof(hidden), "... %u EARLIER STAGES", static_cast<unsigned>(firstVisible));
        screen.setTextSize(1);
        screen.setTextColor(bootDim, bootBg);
        screen.drawString(hidden, 52, y);
        y += 16;
    }
    for (uint8_t i = firstVisible; i < visibleCount; i++) {
        drawBootStageLine(bootStages[i], i, y);
        y += bootStageHeight(bootStages[i]);
    }

    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(1);
    screen.setTextColor(bootDim, bootBg);
    screen.drawString("LONG PRESS SCREEN FOR SETUP", 54, SCREEN_H - 36);
    presentScreenOrRestart();
}

static void resetBootScreen() {
    for (uint8_t i = 0; i < BOOT_STAGE_COUNT; i++) {
        bootStages[i].status = BootStatus::Pending;
        memset(bootStages[i].details, 0, sizeof(bootStages[i].details));
        bootStages[i].detailCount = 0;
        bootStages[i].startedMs = 0;
        bootStages[i].elapsedMs = 0;
        bootStages[i].revealed = false;
    }
    bootStages[BOOT_LCD].status = BootStatus::Running;
    bootStages[BOOT_LCD].startedMs = millis();
    bootStages[BOOT_LCD].revealed = true;
    snprintf(
        bootStages[BOOT_LCD].details[0],
        sizeof(bootStages[BOOT_LCD].details[0]),
        "FRAMEBUFFER %dX%d RGB565",
        SCREEN_W,
        SCREEN_H
    );
    snprintf(
        bootStages[BOOT_LCD].details[1],
        sizeof(bootStages[BOOT_LCD].details[1]),
        "%s / %luMHZ PCLK / DOUBLE BUFFER",
        screen.model() == PanelDisplay::Model::TouchLcd7B ? "LCD 7B" : "LCD 7",
        static_cast<unsigned long>(screen.pixelClockHz() / 1000000UL)
    );
    bootStages[BOOT_LCD].detailCount = 2;
    bootScreenActive = true;
    drawBootScreen();
}

static void setBootStageDetails(
    BootStageId id,
    const char *line0 = nullptr,
    const char *line1 = nullptr,
    const char *line2 = nullptr,
    const char *line3 = nullptr
) {
    if (!bootScreenActive) return;
    BootStage &stage = bootStages[id];
    const char *lines[4] = {line0, line1, line2, line3};
    memset(stage.details, 0, sizeof(stage.details));
    stage.detailCount = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (lines[i] == nullptr || lines[i][0] == '\0') continue;
        strlcpy(stage.details[stage.detailCount], lines[i], sizeof(stage.details[0]));
        stage.detailCount++;
    }
    if (bootScreenActive) {
        drawBootScreen();
    }
}

static void setBootStage(BootStageId id, BootStatus status) {
    BootStage &stage = bootStages[id];
    BootStatus previous = stage.status;
    stage.status = status;
    stage.revealed = true;
    if (status == BootStatus::Running && previous != BootStatus::Running) {
        stage.startedMs = millis();
        stage.elapsedMs = 0;
    } else if (status != BootStatus::Pending && status != BootStatus::Running &&
               stage.startedMs != 0) {
        stage.elapsedMs = millis() - stage.startedMs;
    }
    if (status == BootStatus::Ok) {
        memset(stage.details, 0, sizeof(stage.details));
        stage.detailCount = 0;
    }
    if (bootScreenActive) {
        drawBootScreen();
    }
}

static void formatBootByteCount(size_t bytes, char *out, size_t outLen) {
    if (outLen == 0) return;
    if (bytes >= 1024 * 1024) {
        snprintf(out, outLen, "%.2fMB", bytes / (1024.0f * 1024.0f));
    } else if (bytes >= 1024) {
        snprintf(out, outLen, "%.1fKB", bytes / 1024.0f);
    } else {
        snprintf(out, outLen, "%uB", static_cast<unsigned>(bytes));
    }
}

struct BootMapLoadContext {
    size_t tileCount = 0;
    const char *range = nullptr;
    size_t failureCount = 0;
    size_t lastFailureTile = 0;
    int lastHttpStatus = 0;
    char lastError[48] = {};
};

static void updateMapBootProgress(
    const RadarMap::LoadProgress &progress,
    void *rawContext
) {
    auto *context = static_cast<BootMapLoadContext *>(rawContext);
    if (context == nullptr) return;

    char tileLine[88];
    char geometryLine[88];
    char targetLine[88];
    char activityLine[88];
    snprintf(
        tileLine,
        sizeof(tileLine),
        "VIEW %u/%u / RANGE %s / XYZ %u/%u",
        static_cast<unsigned>(progress.viewIndex + 1),
        static_cast<unsigned>(context->tileCount),
        context->range != nullptr ? context->range : "?",
        static_cast<unsigned>(std::min(progress.tileIndex + 1, progress.tileCount)),
        static_cast<unsigned>(progress.tileCount)
    );
    snprintf(
        geometryLine,
        sizeof(geometryLine),
        "ZOOM %d / GRID %dX%d / SOURCE %dX%d",
        progress.zoom,
        progress.tileColumns,
        progress.tileRows,
        progress.sourceWidth,
        progress.sourceHeight
    );
    snprintf(
        targetLine,
        sizeof(targetLine),
        "TARGET %dX%d / RGB565",
        progress.destinationWidth,
        progress.destinationHeight
    );

    switch (progress.phase) {
    case RadarMap::LoadPhase::Request:
        snprintf(
            activityLine,
            sizeof(activityLine),
            "REQUEST XYZ / %d/%d/%d.PNG",
            progress.zoom,
            progress.tileX,
            progress.tileY
        );
        break;
    case RadarMap::LoadPhase::Response:
        snprintf(
            activityLine,
            sizeof(activityLine),
            "HTTP %d / XYZ %d/%d/%d",
            progress.httpStatus,
            progress.zoom,
            progress.tileX,
            progress.tileY
        );
        break;
    case RadarMap::LoadPhase::Download: {
        char received[20];
        char total[20];
        formatBootByteCount(progress.receivedBytes, received, sizeof(received));
        formatBootByteCount(progress.totalBytes, total, sizeof(total));
        unsigned percent = progress.totalBytes > 0
            ? static_cast<unsigned>((progress.receivedBytes * 100) / progress.totalBytes)
            : 0;
        snprintf(
            activityLine,
            sizeof(activityLine),
            "DOWNLOAD %s/%s / %uPCT / VIEW %uKB",
            received,
            total,
            percent,
            static_cast<unsigned>(progress.viewReceivedBytes / 1024)
        );
        break;
    }
    case RadarMap::LoadPhase::Decode: {
        char pngSize[20];
        formatBootByteCount(progress.totalBytes, pngSize, sizeof(pngSize));
        snprintf(activityLine, sizeof(activityLine), "DECODING PNG / %s", pngSize);
        break;
    }
    case RadarMap::LoadPhase::Ready: {
        char viewSize[20];
        formatBootByteCount(progress.viewReceivedBytes, viewSize, sizeof(viewSize));
        snprintf(
            activityLine,
            sizeof(activityLine),
            "READY / %u XYZ / %s / DECODE %lums",
            static_cast<unsigned>(progress.tileCount),
            viewSize,
            static_cast<unsigned long>(progress.decodeMs)
        );
        break;
    }
    case RadarMap::LoadPhase::Error:
        context->failureCount++;
        context->lastFailureTile = progress.viewIndex;
        context->lastHttpStatus = progress.httpStatus;
        strlcpy(
            context->lastError,
            progress.error != nullptr ? progress.error : "UNKNOWN ERROR",
            sizeof(context->lastError)
        );
        snprintf(
            activityLine,
            sizeof(activityLine),
            "ERROR / %s / HTTP %d",
            context->lastError,
            progress.httpStatus
        );
        break;
    }

    setBootStageDetails(
        BOOT_MAP,
        tileLine,
        geometryLine,
        targetLine,
        activityLine
    );
}

static void setUnavailableMapBootStatus() {
    if (config.mapProvider == MapProvider::None) {
        setBootStageDetails(BOOT_MAP, "MAP BACKGROUND DISABLED", "PLAIN RADAR MODE SELECTED");
        setBootStage(BOOT_MAP, BootStatus::Skip);
    } else if (config.stadiaApiKey.isEmpty()) {
        setBootStageDetails(BOOT_MAP, "STADIA MAP SELECTED", "API KEY IS EMPTY");
        setBootStage(BOOT_MAP, BootStatus::NoKey);
    } else {
        setBootStageDetails(BOOT_MAP, "CACHE NOT AVAILABLE", "WIFI OR PSRAM IS NOT READY");
        setBootStage(BOOT_MAP, BootStatus::Skip);
    }
}

static bool preloadMapCache() {
    if (config.mapProvider == MapProvider::None) {
        setBootStageDetails(BOOT_MAP, "MAP BACKGROUND DISABLED", "PLAIN RADAR MODE SELECTED");
        setBootStage(BOOT_MAP, BootStatus::Skip);
        return true;
    }
    if (config.stadiaApiKey.isEmpty()) {
        setBootStageDetails(BOOT_MAP, "STADIA MAP SELECTED", "API KEY IS EMPTY");
        setBootStage(BOOT_MAP, BootStatus::NoKey);
        return true;
    }
    if (!mapRuntimeReady || WiFi.status() != WL_CONNECTED) {
        setBootStageDetails(
            BOOT_MAP,
            mapRuntimeReady ? "MAP CACHE ALLOCATED" : "MAP CACHE ALLOCATION FAILED",
            WiFi.status() == WL_CONNECTED ? "WIFI CONNECTED" : "WIFI NOT CONNECTED"
        );
        setBootStage(BOOT_MAP, BootStatus::Fail);
        return false;
    }

    setBootStage(BOOT_MAP, BootStatus::Running);
    BootMapLoadContext progressContext;
    progressContext.tileCount = RANGE_COUNT;
    bool allLoaded = true;
    for (size_t i = 0; i < RANGE_COUNT; i++) {
        progressContext.range = config.miles ? ranges[i].miLabel : ranges[i].kmLabel;
        bool loaded = RadarMap::background.fetchStadia(
            config.lat,
            config.lon,
            ranges[i].outerKm,
            RADAR_RADIUS,
            config.stadiaApiKey,
            config.mapBrightness,
            i,
            updateMapBootProgress,
            &progressContext
        );
        allLoaded = loaded && allLoaded;
    }
    if (allLoaded) {
        setBootStage(BOOT_MAP, BootStatus::Ok);
    } else {
        char failedLine[88];
        char tileLine[88];
        char reasonLine[88];
        snprintf(
            failedLine,
            sizeof(failedLine),
            "FAILED %u OF %u VIEWS",
            static_cast<unsigned>(progressContext.failureCount),
            static_cast<unsigned>(RANGE_COUNT)
        );
        snprintf(
            tileLine,
            sizeof(tileLine),
            "LAST FAILURE VIEW %u/%u / HTTP %d",
            static_cast<unsigned>(progressContext.lastFailureTile + 1),
            static_cast<unsigned>(RANGE_COUNT),
            progressContext.lastHttpStatus
        );
        snprintf(reasonLine, sizeof(reasonLine), "REASON / %s", progressContext.lastError);
        setBootStageDetails(BOOT_MAP, failedLine, tileLine, reasonLine);
        setBootStage(BOOT_MAP, BootStatus::Fail);
    }
    return allLoaded;
}

static void drawBootSetupHint(const char *text, uint16_t color) {
    uint16_t bootBg = screen.color565(1, 6, 5);
    screen.fillRect(54, SCREEN_H - 44, 420, 26, bootBg);
    screen.setTextDatum(textdatum_t::top_left);
    screen.setTextSize(1);
    screen.setTextColor(color, bootBg);
    screen.drawString(text, 54, SCREEN_H - 36);
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
    config.airportSelectionMode = prefs.getUChar("apMode", 0) ==
            static_cast<uint8_t>(AirportSelectionMode::Manual)
        ? AirportSelectionMode::Manual
        : AirportSelectionMode::Automatic;
    config.airportCount = static_cast<uint8_t>(std::max(
        1,
        std::min(
            static_cast<int>(AIRPORT_COUNT_MAX),
            static_cast<int>(prefs.getUChar("apCount", AIRPORT_COUNT_DEFAULT))
        )
    ));
    config.airportRadiusKm = static_cast<uint16_t>(std::max(
        static_cast<int>(AIRPORT_RADIUS_MIN_KM),
        std::min(
            static_cast<int>(AIRPORT_RADIUS_MAX_KM),
            static_cast<int>(prefs.getUShort("apRadius", AIRPORT_RADIUS_DEFAULT_KM))
        )
    ));
    config.manualAirportIcao = normalizeAirportIcao(prefs.getString("apIcao", ""));
    config.showLabelCallsign = prefs.getBool("lblCall", true);
    config.showLabelType = prefs.getBool("lblType", true);
    config.showLabelAltitude = prefs.getBool("lblAlt", true);
    config.showLabelVerticalRate = prefs.getBool("lblVsi", true);
    config.labelTextScalePercent = clampLabelTextScalePercent(
        prefs.getUChar("lblPct", LABEL_TEXT_SCALE_DEFAULT)
    );
    config.aircraftSymbolStyle = prefs.getUChar("symbols", 0) ==
            static_cast<uint8_t>(AircraftSymbolStyle::Classic)
        ? AircraftSymbolStyle::Classic
        : AircraftSymbolStyle::DetailedIcons;
    uint8_t storedMapProvider = prefs.getUChar("map", DEFAULT_MAP_PROVIDER);
    config.mapProvider = storedMapProvider == static_cast<uint8_t>(MapProvider::Stadia)
        ? MapProvider::Stadia
        : MapProvider::None;
    config.stadiaApiKey = prefs.getString("stadiaKey", DEFAULT_STADIA_API_KEY);
    config.mapBrightness = static_cast<uint8_t>(std::max(
        static_cast<int>(MAP_BRIGHTNESS_MIN),
        std::min(100, static_cast<int>(prefs.getUChar("mapBright", MAP_BRIGHTNESS_DEFAULT)))
    ));
    config.showWeather = prefs.getBool("showWx", true);
    config.tempFahrenheit = prefs.getBool("tempF", false);
    config.clock24 = prefs.getBool("clk24", true);
    config.otaPassword = prefs.getString("otaPass", "plane-radar");
    if (config.otaPassword.length() == 0) {
        config.otaPassword = "plane-radar";
    }
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
    prefs.putUChar("apMode", static_cast<uint8_t>(config.airportSelectionMode));
    prefs.putUChar("apCount", config.airportCount);
    prefs.putUShort("apRadius", config.airportRadiusKm);
    prefs.putString("apIcao", config.manualAirportIcao);
    prefs.putBool("lblCall", config.showLabelCallsign);
    prefs.putBool("lblType", config.showLabelType);
    prefs.putBool("lblAlt", config.showLabelAltitude);
    prefs.putBool("lblVsi", config.showLabelVerticalRate);
    prefs.putUChar("lblPct", config.labelTextScalePercent);
    prefs.putUChar("symbols", static_cast<uint8_t>(config.aircraftSymbolStyle));
    prefs.putUChar("map", static_cast<uint8_t>(config.mapProvider));
    prefs.putString("stadiaKey", config.stadiaApiKey);
    prefs.putUChar("mapBright", config.mapBrightness);
    prefs.putBool("showWx", config.showWeather);
    prefs.putBool("tempF", config.tempFahrenheit);
    prefs.putBool("clk24", config.clock24);
    prefs.putString("otaPass", config.otaPassword);
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
    RADAR_LOGD("[display] width=%d height=%d rotation=%d\n",
               screen.width(), screen.height(), screen.getRotation());

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
    double formLat = config.lat;
    double formLon = config.lon;
    bool browserLocationLoaded = false;
    if (server.hasArg("browser_lat") && server.hasArg("browser_lon")) {
        double candidateLat = server.arg("browser_lat").toDouble();
        double candidateLon = server.arg("browser_lon").toDouble();
        if (candidateLat >= -90.0 && candidateLat <= 90.0 &&
            candidateLon >= -180.0 && candidateLon <= 180.0) {
            formLat = candidateLat;
            formLon = candidateLon;
            browserLocationLoaded = true;
        }
    }
    SelectedAirport previewAirports[AIRPORT_COUNT_MAX];
    size_t previewAirportCount = selectAirportsFor(
        formLat,
        formLon,
        config.airportSelectionMode,
        config.airportCount,
        config.airportRadiusKm,
        config.manualAirportIcao,
        previewAirports
    );

    String body;
    body.reserve(13000);
    body += F("<!doctype html><html><head><meta charset='utf-8'>");
    body += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    body += F("<title>Plane Radar Setup</title>");
    body += F("<style>"
              "*{box-sizing:border-box}body{max-width:720px;font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;background:#050805;color:#e8ffe8;margin:24px auto;padding:0 18px}"
              "h1{font-size:28px}h2{font-size:17px;margin:0 0 14px;color:#e8ffe8}section{border-top:1px solid #173c2d;padding:20px 0}"
              ".field{display:block;margin:12px 0 6px;color:#73ff8a}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.check{display:flex;align-items:center;gap:9px;margin:10px 0;color:#d9f5df}"
              "input:not([type=checkbox]):not([type=range]):not([type=radio]),select{width:100%;padding:10px;background:#101512;border:1px solid #295;color:#fff}input[type=checkbox]{width:18px;height:18px;margin:0;accent-color:#19d45a}"
              "input[type=range]{width:100%;min-width:0;accent-color:#19d45a}.range-row{display:grid;grid-template-columns:minmax(0,1fr) 54px;gap:12px;align-items:center}output{color:#73ff8a;text-align:right}"
              ".symbol-picker{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.symbol-choice{position:relative;cursor:pointer}.symbol-choice>input{position:absolute;opacity:0;pointer-events:none}.symbol-card{display:block;border:1px solid #295;background:#101512;padding:10px;text-align:center}.symbol-choice>input:checked+.symbol-card{border-color:#19d45a;background:#0c2118;box-shadow:inset 0 0 0 1px #19d45a}.symbol-preview{height:62px;display:flex;align-items:center;justify-content:center;gap:20px;background:#020807;border:1px solid #173c2d;margin-bottom:9px}.symbol-preview img{width:40px;height:40px;object-fit:contain}.symbol-title{display:block;color:#dfffea;font-weight:700}.classic-plane{width:0;height:0;border-left:9px solid transparent;border-right:9px solid transparent;border-bottom:27px solid #ff3750}.classic-heli{position:relative;width:28px;height:28px}.classic-heli:before,.classic-heli:after{content:'';position:absolute;left:2px;top:12px;width:24px;height:3px;background:#ff3750;transform:rotate(45deg)}.classic-heli:after{transform:rotate(-45deg)}.classic-heli i{position:absolute;left:13px;top:13px;width:3px;height:14px;background:#ff3750}"
              "button{padding:11px 16px;background:#19d45a;border:0;color:#001b08;font-weight:700;cursor:pointer}.secondary{margin-top:12px;background:#163e2d;color:#dfffea}"
              ".save{margin-top:4px;width:100%}small{display:block;color:#8a9;margin-top:8px;line-height:1.4}a{color:#73ff8a}@media(max-width:520px){.grid,.symbol-picker{grid-template-columns:1fr}}"
              "</style>");
    body += F("</head><body><h1>Plane Radar Setup</h1>");
    body += F("<form method='POST' action='/save'>");
    body += F("<section><h2>Network</h2><label class='field'>Wi-Fi SSID</label><input name='ssid' value='");
    body += htmlEscape(config.ssid);
    body += F("'>");
    body += F("<label class='field'>Wi-Fi password</label><input name='pass' type='password' value='");
    body += htmlEscape(config.password);
    body += F("'></section>");

    body += F("<section><h2>Location</h2><div class='grid'><div><label class='field'>Latitude</label><input id='lat' name='lat' type='number' min='-90' max='90' step='0.000001' value='");
    body += String(formLat, 6);
    body += F("'></div><div><label class='field'>Longitude</label><input id='lon' name='lon' type='number' min='-180' max='180' step='0.000001' value='");
    body += String(formLon, 6);
    body += F("'></div></div><button class='secondary' type='button' onclick='useBrowserLocation()'>Use browser location</button><small id='location-status'>");
    if (browserLocationLoaded) {
        body += F("Browser location loaded. Save to apply.");
    }
    body += F("</small></section>");

    body += F("<section><h2>Radar</h2><label class='check'><input type='checkbox' name='miles' ");
    if (config.miles) body += F("checked");
    body += F(">Display distances in miles</label>");
    body += F("<label class='check'><input type='checkbox' name='runways' ");
    if (config.showRunways) body += F("checked");
    body += F(">Show airports and runways</label>");
    body += F("<label class='field'>Airport selection</label><select id='airport-mode' name='airport_mode' onchange='updateAirportMode()'><option value='0'");
    if (config.airportSelectionMode == AirportSelectionMode::Automatic) body += F(" selected");
    body += F(">Nearest to radar center</option><option value='1'");
    if (config.airportSelectionMode == AirportSelectionMode::Manual) body += F(" selected");
    body += F(">Manual ICAO code</option></select>");
    body += F("<div id='airport-auto'><div class='grid'><div><label class='field'>Airports shown</label><select name='airport_count'>");
    for (uint8_t i = 1; i <= AIRPORT_COUNT_MAX; i++) {
        body += F("<option value='");
        body += String(i);
        body += F("'");
        if (config.airportCount == i) body += F(" selected");
        body += F(">");
        body += String(i);
        body += F("</option>");
    }
    body += F("</select></div><div><label class='field'>Search radius, km</label><input name='airport_radius' type='number' min='10' max='500' step='10' value='");
    body += String(config.airportRadiusKm);
    body += F("'></div></div></div>");
    body += F("<div id='airport-manual'><label class='field'>Airport ICAO code</label><input name='airport_icao' maxlength='4' placeholder='LEVC' value='");
    body += htmlEscape(config.manualAirportIcao);
    body += F("'></div><small>Current selection: ");
    if (!config.showRunways) {
        body += F("overlay disabled");
    } else if (previewAirportCount == 0) {
        body += config.airportSelectionMode == AirportSelectionMode::Manual
            ? F("ICAO code not found")
            : F("no airport inside the search radius");
    } else {
        for (size_t i = 0; i < previewAirportCount; i++) {
            if (i > 0) body += F(", ");
            const AirportCatalogEntry *airport = previewAirports[i].airport;
            body += airport->icao;
            if (airport->iata[0] != '\0') {
                body += F(" / ");
                body += airport->iata;
            }
            body += F(" / ");
            body += String(previewAirports[i].distanceKm, 1);
            body += F(" km");
        }
    }
    body += F(". Selection is recalculated after saving the radar location.</small></section>");

    body += F("<section><h2>Aircraft symbols</h2><div class='symbol-picker'><label class='symbol-choice'><input type='radio' name='symbol_style' value='0' ");
    if (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons) body += F("checked");
    body += F("><span class='symbol-card'><span class='symbol-preview'><img alt='' src='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABQAAAAUCAYAAACNiR0NAAAACXBIWXMAAAO+AAAD2QDe9o6lAAABO0lEQVR4nO3TLUgDYRzH8Uc5RRFkYriiDBSDYcVisKhBEHzBgVgsYrdsbcU6Ed+iyWDzDXHJ4EsxajUYdE1W1hac0++f5zl8GOzuHg+bf/iw3e253/Psf8/T9vUyoyKqH0+oYBz1sMFeVBo1hEGjzwQnCnSq/8C/D+zCinW9igN8/iZwHntKb5ugdrCGDdzFDRzBPmZbTJTBLU6QR7lVYA8KyKEzZOVBLWMORWyhZgdKn7YxECPIrm5sKt0GWciZBN5gqmlgAw84x6P66dckxrCECbSb+2mcSpZnhX2YcAm5xLs1OKg33GMXPhaRxTQ65FMCL0xICVWHvysTHhoppfuZ9cwMSUsWcizinBS/6ftr2OCoQGn+kXV9jXWlX4Bz4LDSvbWrV+kNPYpn10D5TbbMlSFbSY7jQthz3ys1OKWfIyQTAAAAAElFTkSuQmCC'><img alt='' src='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABQAAAAUCAYAAACNiR0NAAAACXBIWXMAAAAAAAAAAQCEeRdzAAABiUlEQVR4nL3USyhEURzH8TseMSk7Sqxkb2Fn4ZGwt7ERRWFDeZSUJMqClEcWpIQsJsXCwoIpirIjYoOlyMirSIwZ33/33DqOe8eY5F+f7tz/Oed3Z073jC96XmX9ZaVon8vwiINfrC/AMhpxogf6MKImVGihqRhCDpKwiUU1loUNtfbJ/IZRVGMNQS20C+vYU/PkvhiHqh9GOa7dfvI9ZEPntNBMLUxqEr3ogR+luNH3QA+UekM9BrCFVWP8HU0qRB54a4x/C3RKAi8wg1pj7AyVeHBb6BUotWTZe1Zo9INeYT8FJlT/Gijvl9+ln5xIoIRNI99lrAXblv0exh04gRoUYQolqj+OHaygHbPxBI6h2bLP9hFacYordKg5DZZ9hvPQHytwFJ2ow77qhYyrVADpmEeuZW9D2AwcRDeG1dOdukPECJRaUFcJfUGbGXhp2Uetz1gYUaFmoBP6il2noQfKMZM/hqjLwpBHoFRAvzH38MNlQZrqhz0Cv1Q8J0VemWNkIwPPsSZ/AhWIV5DszxqhAAAAAElFTkSuQmCC'></span><span class='symbol-title'>Detailed icons</span></span></label><label class='symbol-choice'><input type='radio' name='symbol_style' value='1' ");
    if (config.aircraftSymbolStyle == AircraftSymbolStyle::Classic) body += F("checked");
    body += F("><span class='symbol-card'><span class='symbol-preview'><span class='classic-plane'></span><span class='classic-heli'><i></i></span></span><span class='symbol-title'>Classic symbols</span></span></label></div></section>");

    body += F("<section><h2>Aircraft labels</h2><label class='check'><input type='checkbox' name='label_callsign' ");
    if (config.showLabelCallsign) body += F("checked");
    body += F(">Callsign</label><label class='check'><input type='checkbox' name='label_type' ");
    if (config.showLabelType) body += F("checked");
    body += F(">Aircraft type</label><label class='check'><input type='checkbox' name='label_altitude' ");
    if (config.showLabelAltitude) body += F("checked");
    body += F(">Altitude</label><label class='check'><input type='checkbox' name='label_vrate' ");
    if (config.showLabelVerticalRate) body += F("checked");
    body += F(">Vertical rate</label>");
    body += F("<label class='field'>Aircraft tag size</label><div class='range-row'><input id='label-scale' name='label_scale' type='range' min='100' max='200' step='100' value='");
    body += String(config.labelTextScalePercent);
    body += F("' oninput=\"document.getElementById('label-scale-value').value=this.value==='200'?'Large':'Normal'\"><output id='label-scale-value'>");
    body += config.labelTextScalePercent >= 150 ? F("Large") : F("Normal");
    body += F("</output></div><small>Uses crisp 1x/2x bitmap sizes. Route pairs on tags stay one size larger. Intermediate scaling is not used because it blurs this font.</small></section>");

    body += F("<section><h2>Map</h2><label class='field'>Map background</label><select name='map'>");
    body += F("<option value='0'");
    if (config.mapProvider == MapProvider::None) body += F(" selected");
    body += F(">None</option><option value='1'");
    if (config.mapProvider == MapProvider::Stadia) body += F(" selected");
    body += F(">Stadia Alidade Smooth Dark</option></select>");
    body += F("<label class='field'>Map brightness</label><div class='range-row'><input id='map-brightness' name='map_brightness' type='range' min='20' max='100' step='5' value='");
    body += String(config.mapBrightness);
    body += F("' oninput=\"document.getElementById('map-brightness-value').value=this.value+'%'\"><output id='map-brightness-value'>");
    body += String(config.mapBrightness);
    body += F("%</output></div><label class='field'>Stadia Maps API key</label><input name='stadia_key' type='password' value='");
    body += htmlEscape(config.stadiaApiKey);
    body += F("'>");
    body += F("<small>The radar continues without a map if this is empty or the map request fails.</small></section>");
    body += F("<section><h2>Clock and METAR</h2>");
    body += F("<label class='check'><input type='checkbox' name='show_weather' ");
    if (config.showWeather) body += F("checked");
    body += F(">Show METAR in the right-hand footer</label>");
    body += F("<label class='check'><input type='checkbox' name='temp_f' ");
    if (config.tempFahrenheit) body += F("checked");
    body += F(">Temperature in Fahrenheit</label>");
    body += F("<label class='check'><input type='checkbox' name='clock_24' ");
    if (config.clock24) body += F("checked");
    body += F(">24-hour clock</label>");
    body += F("<small>METAR is fetched from aviationweather.gov for the airport selected above. Manual ICAO uses that station; nearest-airport mode uses the closest selected field.</small></section>");
    body += F("<section><h2>Firmware</h2>");
    body += F("<label class='field'>OTA password (user: admin)</label>");
    body += F("<input name='ota_password' type='password' autocomplete='new-password' placeholder='leave blank to keep current'>");
    body += F("<p><a href='/firmware'>Open firmware update</a></p>");
    body += F("<small>Upload the application .bin, not the merged flash image. The first dual-OTA install must be flashed over USB.</small></section>");
    body += F("<button class='save' type='submit'>Save and reboot</button></form>");
    body += F("<p><a href='/screenshot.bmp'>Download current screen BMP</a></p>");
    body += F("<p><small>Use + and - next to RANGE to change distance. Tap an aircraft on the map or in the list for details. Tap the detail card to clear. Long press: setup portal.</small></p>");
    body += F("<p><small>Current IP: ");
    body += WiFi.localIP().toString();
    body += F(" AP: 192.168.4.1 Host: bigplane-radar.local</small></p>");
    body += F("<script>function updateAirportMode(){const manual=document.getElementById('airport-mode').value==='1';document.getElementById('airport-auto').style.display=manual?'none':'block';document.getElementById('airport-manual').style.display=manual?'block':'none'}"
              "function useBrowserLocation(){const status=document.getElementById('location-status');"
              "if(window.isSecureContext&&navigator.geolocation){status.textContent='Locating...';navigator.geolocation.getCurrentPosition(function(p){document.getElementById('lat').value=p.coords.latitude.toFixed(6);document.getElementById('lon').value=p.coords.longitude.toFixed(6);status.textContent='Browser location loaded. Save to apply.'},function(e){status.textContent='Location unavailable: '+e.message},{enableHighAccuracy:true,timeout:15000,maximumAge:60000});return}"
              "const target=location.origin+'/';location.href='https://k4m454k.github.io/big_plane_radar/location.html?return='+encodeURIComponent(target)}updateAirportMode();</script>");
    body += F("</body></html>");
    server.send(200, "text/html", body);
}

static void handleScreenshot() {
    const uint16_t *fb = screen.displayedFrameBuffer();
    if (fb == nullptr) {
        server.send(503, "text/plain", "Framebuffer unavailable");
        return;
    }

    if (SCREEN_W > 1024) {
        server.send(503, "text/plain", "Unsupported framebuffer width");
        return;
    }
    const uint32_t rowBytes = static_cast<uint32_t>(SCREEN_W) * 3U;
    const uint32_t paddedRowBytes = (rowBytes + 3U) & ~3U;
    const uint32_t pixelBytes = paddedRowBytes * static_cast<uint32_t>(SCREEN_H);
    const uint32_t fileBytes = 54U + pixelBytes;

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

    uint8_t row[1024 * 3] = {};
    for (int y = SCREEN_H - 1; y >= 0 && client.connected(); y--) {
        const uint16_t *src = fb + static_cast<size_t>(y) * SCREEN_W;
        for (int x = 0; x < SCREEN_W; x++) {
            uint16_t px = src[x];
            row[x * 3] = expand5(px & 0x1F);
            row[x * 3 + 1] = expand6((px >> 5) & 0x3F);
            row[x * 3 + 2] = expand5((px >> 11) & 0x1F);
        }
        if (paddedRowBytes > rowBytes) {
            memset(row + rowBytes, 0, paddedRowBytes - rowBytes);
        }
        client.write(row, paddedRowBytes);
        if ((y & 0x0F) == 0) {
            AppWatchdog::feed();
        }
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
    AirportSelectionMode airportSelectionMode = server.arg("airport_mode").toInt() ==
            static_cast<int>(AirportSelectionMode::Manual)
        ? AirportSelectionMode::Manual
        : AirportSelectionMode::Automatic;
    long requestedAirportCount = server.hasArg("airport_count")
        ? server.arg("airport_count").toInt()
        : config.airportCount;
    uint8_t airportCount = static_cast<uint8_t>(std::max<long>(
        1,
        std::min<long>(AIRPORT_COUNT_MAX, requestedAirportCount)
    ));
    long requestedAirportRadius = server.hasArg("airport_radius")
        ? server.arg("airport_radius").toInt()
        : config.airportRadiusKm;
    uint16_t airportRadiusKm = static_cast<uint16_t>(std::max<long>(
        AIRPORT_RADIUS_MIN_KM,
        std::min<long>(AIRPORT_RADIUS_MAX_KM, requestedAirportRadius)
    ));
    String manualAirportIcao = normalizeAirportIcao(server.arg("airport_icao"));
    bool showLabelCallsign = server.hasArg("label_callsign");
    bool showLabelType = server.hasArg("label_type");
    bool showLabelAltitude = server.hasArg("label_altitude");
    bool showLabelVerticalRate = server.hasArg("label_vrate");
    uint8_t labelTextScalePercent = clampLabelTextScalePercent(
        server.hasArg("label_scale") ? server.arg("label_scale").toInt()
                                     : config.labelTextScalePercent
    );
    AircraftSymbolStyle aircraftSymbolStyle = server.arg("symbol_style").toInt() ==
            static_cast<int>(AircraftSymbolStyle::Classic)
        ? AircraftSymbolStyle::Classic
        : AircraftSymbolStyle::DetailedIcons;
    MapProvider mapProvider = server.arg("map").toInt() == 1
        ? MapProvider::Stadia
        : MapProvider::None;
    String stadiaApiKey = server.arg("stadia_key");
    int requestedMapBrightness = server.hasArg("map_brightness")
        ? server.arg("map_brightness").toInt()
        : config.mapBrightness;
    uint8_t mapBrightness = static_cast<uint8_t>(std::max(
        static_cast<int>(MAP_BRIGHTNESS_MIN),
        std::min(100, requestedMapBrightness)
    ));
    bool showWeather = server.hasArg("show_weather");
    bool tempFahrenheit = server.hasArg("temp_f");
    bool clock24 = server.hasArg("clock_24");
    String otaPassword = server.arg("ota_password");
    otaPassword.trim();
    if (otaPassword.length() == 0) {
        otaPassword = config.otaPassword;
    }

    lockState();
    config.ssid = ssid;
    config.password = password;
    config.lat = lat;
    config.lon = lon;
    config.miles = miles;
    config.showRunways = showRunways;
    config.airportSelectionMode = airportSelectionMode;
    config.airportCount = airportCount;
    config.airportRadiusKm = airportRadiusKm;
    config.manualAirportIcao = manualAirportIcao;
    config.showLabelCallsign = showLabelCallsign;
    config.showLabelType = showLabelType;
    config.showLabelAltitude = showLabelAltitude;
    config.showLabelVerticalRate = showLabelVerticalRate;
    config.labelTextScalePercent = labelTextScalePercent;
    config.aircraftSymbolStyle = aircraftSymbolStyle;
    config.mapProvider = mapProvider;
    config.stadiaApiKey = stadiaApiKey;
    config.mapBrightness = mapBrightness;
    config.showWeather = showWeather;
    config.tempFahrenheit = tempFahrenheit;
    config.clock24 = clock24;
    config.otaPassword = otaPassword;
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
    OtaUpdate::attach(
        server,
        []() -> const char * { return config.otaPassword.c_str(); },
        [](const char *title, const char *body) {
            drawStatusScreen(title, body);
        }
    );
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
    WiFi.softAP(kSetupApName);
    startWebServer();
    if (!mdnsStarted && MDNS.begin(kMdnsHostname)) {
        mdnsStarted = true;
    }
    drawStatusScreen("BIG PLANE RADAR SETUP", "Connect to Wi-Fi AP: BigPlaneRadar-Setup\nOpen http://192.168.4.1\nSet Wi-Fi and radar location.");
    setStatus("SETUP PORTAL");
}

static const char *wifiStatusLabel(int status) {
    switch (status) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "SSID NOT FOUND";
    case WL_SCAN_COMPLETED: return "SCAN COMPLETE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "AUTH FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
    }
}

static bool connectWifiOnce(uint32_t timeoutMs) {
    if (config.ssid.length() == 0) {
        setBootStageDetails(BOOT_WIFI, "SSID IS EMPTY", "OPENING SETUP PORTAL");
        return false;
    }
    char ssidLine[88];
    char modeLine[88];
    char waitLine[88];
    char statusLine[88];
    snprintf(ssidLine, sizeof(ssidLine), "SSID / %s", config.ssid.c_str());
    strlcpy(modeLine, "MODE / STATION / WIFI SLEEP OFF", sizeof(modeLine));
    snprintf(
        waitLine,
        sizeof(waitLine),
        "ASSOCIATING / TIMEOUT %.1fs",
        timeoutMs / 1000.0f
    );
    strlcpy(statusLine, "DHCP / WAITING FOR ADDRESS", sizeof(statusLine));
    setBootStageDetails(BOOT_WIFI, ssidLine, modeLine, waitLine, statusLine);

    WiFi.mode(portalActive ? WIFI_AP_STA : WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(config.ssid.c_str(), config.password.c_str());
    uint32_t start = millis();
    uint32_t lastDisplayedSecond = UINT32_MAX;
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        server.handleClient();
        uint32_t elapsedMs = millis() - start;
        uint32_t elapsedSecond = elapsedMs / 1000;
        if (bootScreenActive && elapsedSecond != lastDisplayedSecond) {
            lastDisplayedSecond = elapsedSecond;
            snprintf(
                waitLine,
                sizeof(waitLine),
                "WAIT %lus / %lus",
                static_cast<unsigned long>(elapsedSecond),
                static_cast<unsigned long>((timeoutMs + 999) / 1000)
            );
            snprintf(
                statusLine,
                sizeof(statusLine),
                "RADIO STATUS / %s",
                wifiStatusLabel(WiFi.status())
            );
            setBootStageDetails(BOOT_WIFI, ssidLine, modeLine, waitLine, statusLine);
        }
        delay(50);
    }
    if (WiFi.status() != WL_CONNECTED) {
        snprintf(
            waitLine,
            sizeof(waitLine),
            "TIMEOUT AFTER %.1fs",
            (millis() - start) / 1000.0f
        );
        snprintf(
            statusLine,
            sizeof(statusLine),
            "STATUS / %s / FALLBACK TO SETUP AP",
            wifiStatusLabel(WiFi.status())
        );
        setBootStageDetails(BOOT_WIFI, ssidLine, modeLine, waitLine, statusLine);
        return false;
    }
    if (!mdnsStarted && MDNS.begin(kMdnsHostname)) {
        mdnsStarted = true;
    }
    if (!portalActive) {
        startWebServer();
    }
    wifiWasConnected = true;
    wifiReconnectInProgress = false;
    snprintf(
        waitLine,
        sizeof(waitLine),
        "CONNECTED IN %.1fs / RSSI %dDBM",
        (millis() - start) / 1000.0f,
        WiFi.RSSI()
    );
    snprintf(statusLine, sizeof(statusLine), "IP / %s", WiFi.localIP().toString().c_str());
    setBootStageDetails(BOOT_WIFI, ssidLine, modeLine, waitLine, statusLine);
    RADAR_LOGI("[wifi] connected ip=%s rssi=%d\n",
               WiFi.localIP().toString().c_str(), WiFi.RSSI());
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
            RADAR_LOGI("[wifi] reconnected ip=%s rssi=%d\n",
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

static bool copyIcaoCode(const JsonObject &obj, const char *key, char *out, size_t outLen) {
    if (outLen < 5) return false;
    out[0] = '\0';
    if (!obj[key].is<const char *>()) return false;
    const char *value = obj[key].as<const char *>();
    size_t len = 0;
    for (size_t i = 0; value[i] != '\0' && len < 4; i++) {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (!isalnum(c)) continue;
        out[len++] = static_cast<char>(toupper(c));
    }
    out[len] = '\0';
    return len >= 3;
}

static bool copyAirportIcao(const JsonObject &airport, char *out, size_t outLen) {
    return copyIcaoCode(airport, "icao_code", out, outLen) ||
           copyIcaoCode(airport, "icao", out, outLen) ||
           copyIcaoCode(airport, "ident", out, outLen);
}

static char latinCodepointToAscii(uint32_t codepoint) {
    if (codepoint >= 0x00C0 && codepoint <= 0x00C5) return 'A';
    if (codepoint >= 0x00E0 && codepoint <= 0x00E5) return 'a';
    if (codepoint == 0x00C6) return 'A';
    if (codepoint == 0x00E6) return 'a';
    if (codepoint == 0x00C7) return 'C';
    if (codepoint == 0x00E7) return 'c';
    if (codepoint >= 0x00C8 && codepoint <= 0x00CB) return 'E';
    if (codepoint >= 0x00E8 && codepoint <= 0x00EB) return 'e';
    if (codepoint >= 0x00CC && codepoint <= 0x00CF) return 'I';
    if (codepoint >= 0x00EC && codepoint <= 0x00EF) return 'i';
    if (codepoint == 0x00D0) return 'D';
    if (codepoint == 0x00F0) return 'd';
    if (codepoint == 0x00D1) return 'N';
    if (codepoint == 0x00F1) return 'n';
    if ((codepoint >= 0x00D2 && codepoint <= 0x00D6) || codepoint == 0x00D8) return 'O';
    if ((codepoint >= 0x00F2 && codepoint <= 0x00F6) || codepoint == 0x00F8) return 'o';
    if (codepoint >= 0x00D9 && codepoint <= 0x00DC) return 'U';
    if (codepoint >= 0x00F9 && codepoint <= 0x00FC) return 'u';
    if (codepoint == 0x00DD || codepoint == 0x0178) return 'Y';
    if (codepoint == 0x00FD || codepoint == 0x00FF) return 'y';
    if (codepoint == 0x00DE) return 'T';
    if (codepoint == 0x00FE) return 't';
    if (codepoint == 0x00DF) return 's';
    return '\0';
}

static bool copyRouteCity(const JsonObject &airport, char *out, size_t outLen) {
    if (outLen == 0) return false;
    out[0] = '\0';
    const char *value = nullptr;
    if (airport["municipality"].is<const char *>()) {
        value = airport["municipality"].as<const char *>();
    }
    if ((value == nullptr || value[0] == '\0') && airport["name"].is<const char *>()) {
        value = airport["name"].as<const char *>();
    }
    if (value == nullptr) return false;

    size_t readIndex = 0;
    size_t writeIndex = 0;
    bool previousSpace = false;
    while (value[readIndex] != '\0' && writeIndex + 1 < outLen) {
        const uint8_t first = static_cast<uint8_t>(value[readIndex]);
        uint32_t codepoint = 0;
        size_t advance = 1;
        if (first < 0x80) {
            codepoint = first;
        } else if ((first & 0xE0) == 0xC0 && value[readIndex + 1] != '\0') {
            codepoint = ((first & 0x1F) << 6) |
                        (static_cast<uint8_t>(value[readIndex + 1]) & 0x3F);
            advance = 2;
        } else if ((first & 0xF0) == 0xE0 &&
                   value[readIndex + 1] != '\0' && value[readIndex + 2] != '\0') {
            codepoint = ((first & 0x0F) << 12) |
                        ((static_cast<uint8_t>(value[readIndex + 1]) & 0x3F) << 6) |
                        (static_cast<uint8_t>(value[readIndex + 2]) & 0x3F);
            advance = 3;
        }
        readIndex += advance;

        char ascii = '\0';
        if (codepoint >= 0x20 && codepoint <= 0x7E) {
            ascii = static_cast<char>(codepoint);
        } else {
            ascii = latinCodepointToAscii(codepoint);
        }
        if (ascii == '\0') continue;
        if (ascii == ' ') {
            if (writeIndex == 0 || previousSpace) continue;
            previousSpace = true;
        } else {
            previousSpace = false;
        }
        out[writeIndex++] = ascii;
    }
    while (writeIndex > 0 && out[writeIndex - 1] == ' ') {
        writeIndex--;
    }
    out[writeIndex] = '\0';
    return writeIndex > 0;
}

static AirportCityCacheEntry *findAirportCityCacheLocked(const char *iata) {
    if (iata == nullptr || strlen(iata) != 3) return nullptr;
    for (size_t i = 0; i < MAX_AIRPORT_CITY_CACHE; i++) {
        if (airportCityCache[i].active && strcmp(airportCityCache[i].iata, iata) == 0) {
            return &airportCityCache[i];
        }
    }
    return nullptr;
}

static void rememberAirportCityLocked(const char *iata, const char *city, uint32_t now) {
    if (iata == nullptr || strlen(iata) != 3 || city == nullptr || city[0] == '\0') return;
    AirportCityCacheEntry *entry = findAirportCityCacheLocked(iata);
    if (entry == nullptr) {
        entry = &airportCityCache[0];
        for (size_t i = 0; i < MAX_AIRPORT_CITY_CACHE; i++) {
            if (!airportCityCache[i].active) {
                entry = &airportCityCache[i];
                break;
            }
            if (airportCityCache[i].lastUsedMs < entry->lastUsedMs) {
                entry = &airportCityCache[i];
            }
        }
        *entry = AirportCityCacheEntry();
        strlcpy(entry->iata, iata, sizeof(entry->iata));
        entry->active = true;
    }
    strlcpy(entry->city, city, sizeof(entry->city));
    entry->lastUsedMs = now;
}

static void fillRouteCityFromCacheLocked(
    const char *iata,
    char *city,
    size_t cityLen,
    uint32_t now
) {
    if (cityLen == 0 || city[0] != '\0') return;
    AirportCityCacheEntry *entry = findAirportCityCacheLocked(iata);
    if (entry == nullptr) return;
    strlcpy(city, entry->city, cityLen);
    entry->lastUsedMs = now;
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

static TypeNameCacheEntry *findTypeCacheEntry(const char *typeCode) {
    if (typeCode == nullptr || typeCode[0] == '\0') {
        return nullptr;
    }
    for (size_t i = 0; i < MAX_TYPE_CACHE; i++) {
        if (typeNameCache[i].active && strcmp(typeNameCache[i].typeCode, typeCode) == 0) {
            return &typeNameCache[i];
        }
    }
    return nullptr;
}

static void touchTypeCacheEntry(const char *typeCode, uint32_t now) {
    if (typeCode == nullptr || typeCode[0] == '\0') {
        return;
    }
    TypeNameCacheEntry *entry = findTypeCacheEntry(typeCode);
    if (entry == nullptr) {
        for (size_t i = 0; i < MAX_TYPE_CACHE; i++) {
            if (!typeNameCache[i].active) {
                entry = &typeNameCache[i];
                *entry = TypeNameCacheEntry();
                strlcpy(entry->typeCode, typeCode, sizeof(entry->typeCode));
                entry->active = true;
                break;
            }
        }
    }
    if (entry == nullptr) {
        return;
    }
}

static void syncTypeCacheFromAircraft(uint32_t now) {
    for (size_t i = 0; i < aircraftCount; i++) {
        touchTypeCacheEntry(aircraft[i].type, now);
    }
}

static void copyPrintableName(const char *value, char *out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    if (value == nullptr) {
        return;
    }
    size_t written = 0;
    bool previousSpace = true;
    for (size_t i = 0; value[i] != '\0' && written + 1 < outLen; ++i) {
        unsigned char ch = static_cast<unsigned char>(value[i]);
        if (ch < 32 || ch > 126) {
            continue;
        }
        if (ch == ' ') {
            if (previousSpace) {
                continue;
            }
            previousSpace = true;
        } else {
            previousSpace = false;
        }
        out[written++] = static_cast<char>(ch);
    }
    while (written > 0 && out[written - 1] == ' ') {
        --written;
    }
    out[written] = '\0';
}

static bool readDecodedHttpBody(HTTPClient &http, String &payload);

static bool lookupTypeNameForHex(const char *hex, char *out, size_t outLen) {
    if (hex == nullptr || hex[0] == '\0' || outLen == 0) {
        return false;
    }
    out[0] = '\0';
    String url = "https://api.adsbdb.com/v0/aircraft/";
    url += hex;

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
    String payload;
    if (!readDecodedHttpBody(http, payload)) {
        http.end();
        return false;
    }
    http.end();

    JsonDocument filter;
    filter["response"]["aircraft"]["type"] = true;
    filter["response"]["aircraft"]["icao_type"] = true;
    JsonDocument doc;
    if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) {
        return false;
    }
    JsonObject aircraftObj = doc["response"]["aircraft"].as<JsonObject>();
    if (aircraftObj.isNull()) {
        return false;
    }
    const char *detailed = aircraftObj["type"].is<const char *>()
        ? aircraftObj["type"].as<const char *>()
        : nullptr;
    copyPrintableName(detailed, out, outLen);
    return out[0] != '\0';
}

static bool serviceTypeLookup() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    uint32_t now = millis();
    if (now - lastTypeLookupMs < ROUTE_LOOKUP_INTERVAL_MS) {
        return false;
    }

    char typeCode[8] = {};
    char hex[7] = {};
    lockState();
    auto pickAircraftForType = [&](const char *wantedType) -> bool {
        for (size_t i = 0; i < aircraftCount; i++) {
            if (strcmp(aircraft[i].type, wantedType) != 0) {
                continue;
            }
            if (aircraft[i].hex[0] == '\0') {
                continue;
            }
            strlcpy(hex, aircraft[i].hex, sizeof(hex));
            return true;
        }
        return false;
    };

    if (selectedAircraftHex[0] != '\0') {
        for (size_t i = 0; i < aircraftCount; i++) {
            if (strcmp(aircraft[i].hex, selectedAircraftHex) != 0) {
                continue;
            }
            TypeNameCacheEntry *selectedType = findTypeCacheEntry(aircraft[i].type);
            if (selectedType != nullptr && !selectedType->hasName &&
                (!selectedType->lookupDone ||
                 now - selectedType->lastLookupMs >= ROUTE_LOOKUP_RETRY_MS) &&
                aircraft[i].hex[0] != '\0') {
                strlcpy(typeCode, aircraft[i].type, sizeof(typeCode));
                strlcpy(hex, aircraft[i].hex, sizeof(hex));
            }
            break;
        }
    }
    if (typeCode[0] == '\0') {
        for (size_t i = 0; i < MAX_TYPE_CACHE; i++) {
            TypeNameCacheEntry &entry = typeNameCache[i];
            if (!entry.active || entry.hasName) {
                continue;
            }
            if (entry.lookupDone && now - entry.lastLookupMs < ROUTE_LOOKUP_RETRY_MS) {
                continue;
            }
            if (!pickAircraftForType(entry.typeCode)) {
                continue;
            }
            strlcpy(typeCode, entry.typeCode, sizeof(typeCode));
            break;
        }
    }
    if (typeCode[0] == '\0' || hex[0] == '\0') {
        unlockState();
        return false;
    }
    TypeNameCacheEntry *queued = findTypeCacheEntry(typeCode);
    if (queued != nullptr) {
        queued->lookupDone = true;
        queued->lastLookupMs = now;
    }
    lastTypeLookupMs = now;
    unlockState();

    char fullName[TYPE_NAME_MAX_LEN] = {};
    bool ok = lookupTypeNameForHex(hex, fullName, sizeof(fullName));
    lockState();
    TypeNameCacheEntry *entry = findTypeCacheEntry(typeCode);
    if (entry != nullptr && ok) {
        strlcpy(entry->fullName, fullName, sizeof(entry->fullName));
        entry->hasName = true;
        networkDataDirty = true;
    }
    unlockState();
    return ok;
}

/**
 * HTTPClient::writeToStream() unwraps Content-Length and Transfer-Encoding:
 * chunked. Reading getStream() directly does not, so Cloudflare chunk-size
 * lines (and a trailing "0") made ArduinoJson report InvalidInput.
 */
class DecodedBodySink : public Stream {
 public:
    explicit DecodedBodySink(String &dest) : dest_(dest) {}

    size_t write(uint8_t c) override {
        return dest_.concat(static_cast<char>(c)) ? 1 : 0;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        if (buffer == nullptr || size == 0) {
            return 0;
        }
        return dest_.concat(reinterpret_cast<const char *>(buffer),
                            static_cast<unsigned>(size))
                   ? size
                   : 0;
    }

    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}

 private:
    String &dest_;
};

static bool readDecodedHttpBody(HTTPClient &http, String &payload) {
    payload = "";
    const int contentLength = http.getSize();
    if (contentLength > 0) {
        payload.reserve(static_cast<unsigned>(contentLength + 1));
    }

    DecodedBodySink sink(payload);
    return http.writeToStream(&sink) > 0 && payload.length() > 0;
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

    String payload;
    if (!readDecodedHttpBody(http, payload)) {
        http.end();
        return false;
    }
    http.end();

    JsonDocument filter;
    filter["response"]["flightroute"]["origin"]["iata_code"] = true;
    filter["response"]["flightroute"]["origin"]["iata"] = true;
    filter["response"]["flightroute"]["origin"]["iataCode"] = true;
    filter["response"]["flightroute"]["origin"]["icao_code"] = true;
    filter["response"]["flightroute"]["origin"]["icao"] = true;
    filter["response"]["flightroute"]["origin"]["ident"] = true;
    filter["response"]["flightroute"]["origin"]["municipality"] = true;
    filter["response"]["flightroute"]["origin"]["name"] = true;
    filter["response"]["flightroute"]["destination"]["iata_code"] = true;
    filter["response"]["flightroute"]["destination"]["iata"] = true;
    filter["response"]["flightroute"]["destination"]["iataCode"] = true;
    filter["response"]["flightroute"]["destination"]["icao_code"] = true;
    filter["response"]["flightroute"]["destination"]["icao"] = true;
    filter["response"]["flightroute"]["destination"]["ident"] = true;
    filter["response"]["flightroute"]["destination"]["municipality"] = true;
    filter["response"]["flightroute"]["destination"]["name"] = true;
    JsonDocument doc;
    DeserializationError err = deserializeJson(
        doc,
        payload,
        DeserializationOption::Filter(filter)
    );
    if (err) {
        return false;
    }

    JsonObject route = doc["response"]["flightroute"].as<JsonObject>();
    if (route.isNull()) {
        return false;
    }

    char origin[4] = {};
    char destination[4] = {};
    char originIcao[5] = {};
    char destinationIcao[5] = {};
    JsonObject originAirport = route["origin"].as<JsonObject>();
    JsonObject destinationAirport = route["destination"].as<JsonObject>();
    copyAirportIata(originAirport, origin, sizeof(origin));
    copyAirportIata(destinationAirport, destination, sizeof(destination));
    copyAirportIcao(originAirport, originIcao, sizeof(originIcao));
    copyAirportIcao(destinationAirport, destinationIcao, sizeof(destinationIcao));
    if ((origin[0] == '\0' && originIcao[0] == '\0') ||
        (destination[0] == '\0' && destinationIcao[0] == '\0')) {
        return false;
    }

    strlcpy(entry.originIata, origin, sizeof(entry.originIata));
    strlcpy(entry.destinationIata, destination, sizeof(entry.destinationIata));
    strlcpy(entry.originIcao, originIcao, sizeof(entry.originIcao));
    strlcpy(entry.destinationIcao, destinationIcao, sizeof(entry.destinationIcao));
    copyRouteCity(originAirport, entry.originCity, sizeof(entry.originCity));
    copyRouteCity(destinationAirport, entry.destinationCity, sizeof(entry.destinationCity));
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
            strlcpy(entry->originIcao, lookupEntry.originIcao, sizeof(entry->originIcao));
            strlcpy(entry->destinationIcao, lookupEntry.destinationIcao, sizeof(entry->destinationIcao));
            strlcpy(entry->originCity, lookupEntry.originCity, sizeof(entry->originCity));
            strlcpy(entry->destinationCity, lookupEntry.destinationCity, sizeof(entry->destinationCity));
            rememberAirportCityLocked(entry->originIata, entry->originCity, now);
            rememberAirportCityLocked(entry->destinationIata, entry->destinationCity, now);
            fillRouteCityFromCacheLocked(
                entry->originIata,
                entry->originCity,
                sizeof(entry->originCity),
                now
            );
            fillRouteCityFromCacheLocked(
                entry->destinationIata,
                entry->destinationCity,
                sizeof(entry->destinationCity),
                now
            );
            entry->hasRoute = true;
            networkDataDirty = true;
        }
        RADAR_LOGD("[route] callsign=%s ok=%d origin=%s/%s destination=%s/%s\n",
                   entry->callsign,
                   ok ? 1 : 0,
                   entry->originIata,
                   entry->originCity,
                   entry->destinationIata,
                   entry->destinationCity);
    }
    unlockState();
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

static void formatRoutePlace(
    const char *city,
    const char *iata,
    const char *icao,
    size_t keepChars,
    char *out,
    size_t outLen
) {
    if (outLen == 0) return;
    out[0] = '\0';
    const char *code = (icao != nullptr && icao[0] != '\0') ? icao
        : ((iata != nullptr && iata[0] != '\0') ? iata : "");
    if (city == nullptr || city[0] == '\0') {
        strlcpy(out, code, outLen);
        return;
    }

    size_t cityLen = strlen(city);
    if (keepChars >= cityLen) {
        if (code[0] != '\0') {
            snprintf(out, outLen, "%s %s", city, code);
        } else {
            strlcpy(out, city, outLen);
        }
        return;
    }

    size_t copyLen = std::min(keepChars, outLen > 4 ? outLen - 4 : 0);
    while (copyLen > 0 && city[copyLen - 1] == ' ') {
        copyLen--;
    }
    if (copyLen == 0) {
        strlcpy(out, code, outLen);
        return;
    }
    memcpy(out, city, copyLen);
    out[copyLen] = '\0';
    strlcat(out, "...", outLen);
    if (code[0] != '\0') {
        strlcat(out, " ", outLen);
        strlcat(out, code, outLen);
    }
}

static bool canShortenRoutePlace(size_t cityLen, size_t keepChars) {
    if (cityLen <= ROUTE_CITY_MIN_PREFIX) return false;
    if (keepChars >= cityLen) {
        return cityLen > ROUTE_CITY_MIN_PREFIX + 3;
    }
    return keepChars > ROUTE_CITY_MIN_PREFIX;
}

static void shortenRoutePlace(size_t cityLen, size_t &keepChars) {
    if (keepChars >= cityLen) {
        keepChars = std::max(ROUTE_CITY_MIN_PREFIX, cityLen - 4);
    } else if (keepChars > ROUTE_CITY_MIN_PREFIX) {
        keepChars--;
    }
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

    size_t originLen = strlen(entry->originCity);
    size_t destinationLen = strlen(entry->destinationCity);
    size_t originKeep = originLen;
    size_t destinationKeep = destinationLen;
    char origin[ROUTE_CITY_MAX_LEN + 8];
    char destination[ROUTE_CITY_MAX_LEN + 8];

    for (size_t attempt = 0; attempt < ROUTE_CITY_MAX_LEN * 2; attempt++) {
        formatRoutePlace(
            entry->originCity,
            entry->originIata,
            entry->originIcao,
            originKeep,
            origin,
            sizeof(origin)
        );
        formatRoutePlace(
            entry->destinationCity,
            entry->destinationIata,
            entry->destinationIcao,
            destinationKeep,
            destination,
            sizeof(destination)
        );
        snprintf(out, outLen, "%s - %s", origin, destination);
        if (g.textWidth(out) <= maxWidth) {
            return out[0] != '\0';
        }

        bool canShortenOrigin = canShortenRoutePlace(originLen, originKeep);
        bool canShortenDestination = canShortenRoutePlace(destinationLen, destinationKeep);
        if (!canShortenOrigin && !canShortenDestination) break;
        if (canShortenOrigin &&
            (!canShortenDestination || g.textWidth(origin) >= g.textWidth(destination))) {
            shortenRoutePlace(originLen, originKeep);
        } else {
            shortenRoutePlace(destinationLen, destinationKeep);
        }
    }

    if (entry->originIata[0] != '\0' && entry->destinationIata[0] != '\0') {
        snprintf(out, outLen, "%s - %s", entry->originIata, entry->destinationIata);
    } else {
        snprintf(out, outLen, "%s - %s", entry->originIcao, entry->destinationIcao);
    }
    return out[0] != '\0';
}

static bool routePairLabel(
    const RouteCacheEntry *entries,
    size_t entryCount,
    const char *callsign,
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
    if (entry->originIata[0] != '\0' && entry->destinationIata[0] != '\0') {
        snprintf(out, outLen, "%s-%s", entry->originIata, entry->destinationIata);
        return true;
    }
    if (entry->originIcao[0] != '\0' && entry->destinationIcao[0] != '\0') {
        snprintf(out, outLen, "%s-%s", entry->originIcao, entry->destinationIcao);
        return true;
    }
    return false;
}

static const char *typeNameForCode(
    const TypeNameCacheEntry *entries,
    size_t entryCount,
    const char *typeCode
) {
    if (typeCode == nullptr || typeCode[0] == '\0') {
        return nullptr;
    }
    for (size_t i = 0; i < entryCount; i++) {
        if (entries[i].active && entries[i].hasName &&
            strcmp(entries[i].typeCode, typeCode) == 0) {
            return entries[i].fullName;
        }
    }
    return nullptr;
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
    snprintf(
        out,
        outLen,
        "%c%d",
        fpm > 0 ? '^' : PanelDisplay::GLYPH_ARROW_DOWN,
        static_cast<int>(lroundf(fabsf(fpm)))
    );
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
        setBootStageDetails(BOOT_DATA, "ADSB REQUEST NOT STARTED", "WIFI IS NOT CONNECTED");
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

    char endpointLine[88];
    char centerLine[88];
    char radiusLine[88];
    char responseLine[88];
    strlcpy(endpointLine, "ENDPOINT / OPENDATA.ADSB.FI API V3", sizeof(endpointLine));
    snprintf(
        centerLine,
        sizeof(centerLine),
        "CENTER / %.5f / %.5f",
        centerLat,
        centerLon
    );
    snprintf(
        radiusLine,
        sizeof(radiusLine),
        "QUERY RADIUS / %.1fNM / DISPLAY %s",
        fetchNm,
        rangeLabel()
    );
    strlcpy(responseLine, "HTTPS REQUEST / CONNECTING", sizeof(responseLine));
    setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        strlcpy(responseLine, "HTTP BEGIN FAILED", sizeof(responseLine));
        setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
        setLastFetchText("HTTP BEGIN FAIL");
        return false;
    }
    http.setTimeout(10000);
    int code = http.GET();
    int responseLength = http.getSize();
    if (responseLength > 0) {
        char responseSize[20];
        formatBootByteCount(static_cast<size_t>(responseLength), responseSize, sizeof(responseSize));
        snprintf(
            responseLine,
            sizeof(responseLine),
            "HTTP %d / RESPONSE %s",
            code,
            responseSize
        );
    } else {
        snprintf(responseLine, sizeof(responseLine), "HTTP %d / STREAMING RESPONSE", code);
    }
    setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
    if (code != HTTP_CODE_OK) {
        setLastFetchText("HTTP " + String(code));
        http.end();
        return false;
    }

    JsonDocument filter;
    const char *fields[] = {
        "lat", "lon", "track", "true_heading", "mag_heading", "dir",
        "gs", "tas", "ias", "baro_rate", "geom_rate", "flight", "hex",
        "t", "r", "category", "squawk", "alt_baro", "alt_geom"
    };
    for (const char *field : fields) {
        filter["ac"][0][field] = true;
    }
    if (responseLength > 0) {
        char responseSize[20];
        formatBootByteCount(static_cast<size_t>(responseLength), responseSize, sizeof(responseSize));
        snprintf(responseLine, sizeof(responseLine), "HTTP %d / PARSING JSON / %s", code, responseSize);
    } else {
        snprintf(responseLine, sizeof(responseLine), "HTTP %d / PARSING JSON STREAM", code);
    }
    setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
    String payload;
    if (!readDecodedHttpBody(http, payload)) {
        http.end();
        strlcpy(responseLine, "EMPTY OR CHUNKED BODY FAILED", sizeof(responseLine));
        setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
        setLastFetchText("EMPTY RESPONSE");
        return false;
    }
    http.end();
    JsonDocument doc;
    DeserializationError err = deserializeJson(
        doc,
        payload,
        DeserializationOption::Filter(filter)
    );
    if (err) {
        snprintf(responseLine, sizeof(responseLine), "JSON ERROR / %s", err.c_str());
        setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
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
            float rawTrack = 0;
            dst.hasTrack = readJsonFloat(plane, "track", rawTrack) ||
                readJsonFloat(plane, "dir", rawTrack);
            dst.gsKnots = pickSpeed(plane);
            dst.verticalRateFpm = pickVerticalRate(plane);
            copyJsonStringTrimmed(plane, "hex", dst.hex, sizeof(dst.hex));
            copyJsonStringTrimmed(plane, "flight", dst.callsign, sizeof(dst.callsign));
            dst.hasFlight = dst.callsign[0] != '\0';
            if (!dst.hasFlight) {
                strlcpy(dst.callsign, dst.hex, sizeof(dst.callsign));
            }
            copyJsonStringTrimmed(plane, "t", dst.type, sizeof(dst.type));
            copyJsonStringTrimmed(plane, "r", dst.registration, sizeof(dst.registration));
            copyJsonStringTrimmed(plane, "category", dst.category, sizeof(dst.category));
            copySquawkCode(plane, dst.squawk, sizeof(dst.squawk));
            dst.altitudeFt = -1.0e9f;
            float altitudeFt = 0;
            if (readJsonFloat(plane, "alt_baro", altitudeFt) ||
                readJsonFloat(plane, "alt_geom", altitudeFt)) {
                dst.altitudeFt = altitudeFt;
            }
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
    updateAircraftTracksLocked(fetchedAircraft, fetchedCount, fetchNow);
    syncRouteCacheFromAircraft(fetchNow);
    syncTypeCacheFromAircraft(fetchNow);
    lastFetchText = fetchStatus;
    networkDataDirty = true;
    unlockState();

    RADAR_LOGD("[adsb] %s\n", fetchStatus);
    snprintf(
        responseLine,
        sizeof(responseLine),
        "PARSED / %u AIRCRAFT / HTTP %d",
        static_cast<unsigned>(fetchedCount),
        code
    );
    setBootStageDetails(BOOT_DATA, endpointLine, centerLine, radiusLine, responseLine);
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

static bool clipTrackLineToMapViewport(int &x0, int &y0, int &x1, int &y1) {
    float startX = static_cast<float>(x0);
    float startY = static_cast<float>(y0);
    float dx = static_cast<float>(x1 - x0);
    float dy = static_cast<float>(y1 - y0);
    float enter = 0.0f;
    float leave = 1.0f;

    auto clip = [&](float direction, float distance) {
        if (fabsf(direction) < 0.0001f) {
            return distance >= 0.0f;
        }
        float ratio = distance / direction;
        if (direction < 0.0f) {
            if (ratio > leave) return false;
            enter = std::max(enter, ratio);
        } else {
            if (ratio < enter) return false;
            leave = std::min(leave, ratio);
        }
        return true;
    };

    if (!clip(-dx, startX) ||
        !clip(dx, static_cast<float>(PANEL_X - 1) - startX) ||
        !clip(-dy, startY) ||
        !clip(dy, static_cast<float>(SCREEN_H - 1) - startY)) {
        return false;
    }

    x0 = static_cast<int>(lroundf(startX + enter * dx));
    y0 = static_cast<int>(lroundf(startY + enter * dy));
    x1 = static_cast<int>(lroundf(startX + leave * dx));
    y1 = static_cast<int>(lroundf(startY + leave * dy));
    return true;
}

static bool prepareTrackSegment(
    bool useMapViewport,
    bool startInsideRadar,
    bool endInsideRadar,
    int &x0,
    int &y0,
    int &x1,
    int &y1
) {
    if (useMapViewport) {
        return clipTrackLineToMapViewport(x0, y0, x1, y1);
    }
    return startInsideRadar && endInsideRadar;
}

template <typename Gfx>
static void drawDashedTrackLine(
    Gfx &g,
    int x0,
    int y0,
    int x1,
    int y1,
    uint16_t color
) {
    float dx = static_cast<float>(x1 - x0);
    float dy = static_cast<float>(y1 - y0);
    float length = hypotf(dx, dy);
    if (length < 1.0f) return;

    static constexpr float DASH_LENGTH = 4.0f;
    static constexpr float DASH_PERIOD = 8.0f;
    for (float offset = 0.0f; offset < length; offset += DASH_PERIOD) {
        float from = offset / length;
        float to = std::min(length, offset + DASH_LENGTH) / length;
        int dashX0 = x0 + static_cast<int>(lroundf(dx * from));
        int dashY0 = y0 + static_cast<int>(lroundf(dy * from));
        int dashX1 = x0 + static_cast<int>(lroundf(dx * to));
        int dashY1 = y0 + static_cast<int>(lroundf(dy * to));
        g.drawLine(dashX0, dashY0, dashX1, dashY1, color);
    }
}

static int trajectoryLengthPx(float gsKnots) {
    if (gsKnots <= 1.0f) {
        return 0;
    }
    constexpr float kmPerKnotHorizon = KM_PER_NM * 60.0f / 3600.0f;
    constexpr float refOuterKm = 13.3f;
    constexpr float lengthScale = 1.5f / 5.0f;
    float px = gsKnots * kmPerKnotHorizon * static_cast<float>(RADAR_RADIUS) /
        refOuterKm * lengthScale;
    int len = static_cast<int>(lroundf(px));
    return len < 6 ? 6 : len;
}

template <typename Gfx>
static void drawAircraftTrajectory(
    Gfx &g,
    const Aircraft &item,
    int x,
    int y,
    bool selected
) {
    int len = trajectoryLengthPx(item.gsKnots);
    if (len <= 0) {
        return;
    }
    float heading = item.hasTrack ? item.trackDeg : item.noseDeg;
    float rad = heading * DEG_TO_RAD;
    float s = sinf(rad);
    float c = cosf(rad);
    int startX = x + static_cast<int>(lroundf(s * 10.0f));
    int startY = y - static_cast<int>(lroundf(c * 10.0f));
    int endX = startX + static_cast<int>(lroundf(s * static_cast<float>(len)));
    int endY = startY - static_cast<int>(lroundf(c * static_cast<float>(len)));
    if (!clipTrackLineToMapViewport(startX, startY, endX, endY)) {
        return;
    }
    if (abs(endX - startX) + abs(endY - startY) < 2) {
        return;
    }
    g.drawWideLine(
        startX,
        startY,
        endX,
        endY,
        selected ? 1.8f : 1.0f,
        colorTrajectory
    );
}

static const Aircraft *findAircraftByHex(
    const Aircraft *items,
    size_t itemCount,
    const char *hex
) {
    if (hex == nullptr || hex[0] == '\0') return nullptr;
    for (size_t i = 0; i < itemCount; i++) {
        if (strcmp(items[i].hex, hex) == 0) {
            return &items[i];
        }
    }
    return nullptr;
}

template <typename Gfx>
static void drawSelectedAircraftTrack(
    Gfx &g,
    const TrackPoint *points,
    size_t pointCount,
    const Aircraft *selectedAircraft,
    bool useMapViewport
) {
    if (points == nullptr || pointCount == 0) return;

    int previousX = 0;
    int previousY = 0;
    float previousDistanceKm = 0;
    bool previousInsideRadar = toRadarPoint(
        points[0].lat,
        points[0].lon,
        previousX,
        previousY,
        previousDistanceKm
    );
    size_t brightStart = pointCount > 18 ? pointCount - 18 : 1;

    for (size_t i = 1; i < pointCount; i++) {
        int x = 0;
        int y = 0;
        float distanceKm = 0;
        bool insideRadar = toRadarPoint(
            points[i].lat,
            points[i].lon,
            x,
            y,
            distanceKm
        );
        int lineX0 = previousX;
        int lineY0 = previousY;
        int lineX1 = x;
        int lineY1 = y;
        if (prepareTrackSegment(
                useMapViewport,
                previousInsideRadar,
                insideRadar,
                lineX0,
                lineY0,
                lineX1,
                lineY1)) {
            uint16_t color = i >= brightStart ? colorTrackBright : colorTrackDim;
            g.drawLine(lineX0, lineY0, lineX1, lineY1, color);
        }
        previousX = x;
        previousY = y;
        previousInsideRadar = insideRadar;
    }

    if (selectedAircraft == nullptr) return;
    int lineX0 = previousX;
    int lineY0 = previousY;
    int lineX1 = selectedAircraft->screenX;
    int lineY1 = selectedAircraft->screenY;
    bool forecastInsideRadar =
        selectedAircraft->distanceKm <= activeOuterKm();
    if (!prepareTrackSegment(
            useMapViewport,
            previousInsideRadar,
            forecastInsideRadar,
            lineX0,
            lineY0,
            lineX1,
            lineY1)) {
        return;
    }
    if (abs(lineX1 - lineX0) + abs(lineY1 - lineY0) < 2) return;
    drawDashedTrackLine(
        g,
        lineX0,
        lineY0,
        lineX1,
        lineY1,
        colorTrackForecast
    );
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
static void drawPlane(Gfx &g, int cx, int cy, float headingDeg, uint8_t sizeClass, uint16_t color) {
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
    g.fillTriangle(tipX, tipY, tailX + wingX, tailY + wingY, tailX - wingX, tailY - wingY, color);
}

template <typename Gfx>
static void drawAircraftSymbol(Gfx &g, const Aircraft &item, int cx, int cy, uint16_t color) {
    if (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons) {
        AircraftIcons::draw(
            g,
            isRotorcraft(item),
            planeSizeClass(item),
            item.noseDeg,
            cx,
            cy,
            color
        );
        return;
    }
    if (isRotorcraft(item)) {
        float rad = item.noseDeg * DEG_TO_RAD;
        float s = sinf(rad);
        float c = cosf(rad);
        int rotor1X = lroundf((s + c) * 3);
        int rotor1Y = lroundf((s - c) * 3);
        int rotor2X = lroundf((s - c) * 3);
        int rotor2Y = -lroundf((s + c) * 3);
        g.drawWideLine(
            cx - rotor1X, cy - rotor1Y,
            cx + rotor1X, cy + rotor1Y,
            2.0f, color
        );
        g.drawWideLine(
            cx - rotor2X, cy - rotor2Y,
            cx + rotor2X, cy + rotor2Y,
            2.0f, color
        );

        int tailX = cx - lroundf(s * 7);
        int tailY = cy + lroundf(c * 7);
        g.drawWideLine(cx, cy, tailX, tailY, 2.0f, color);
        return;
    }
    drawPlane(g, cx, cy, item.noseDeg, planeSizeClass(item), color);
}

static uint32_t aircraftLabelId(const Aircraft &item) {
    uint32_t hash = 2166136261U;
    const char *value = item.hex[0] != '\0' ? item.hex : item.callsign;
    for (size_t i = 0; value[i] != '\0'; i++) {
        hash ^= static_cast<uint8_t>(value[i]);
        hash *= 16777619U;
    }
    return hash == 0 ? 1 : hash;
}

static float aircraftSymbolRadius(const Aircraft &item) {
    if (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons) {
        return static_cast<float>(AircraftIcons::halfExtent(
            isRotorcraft(item),
            planeSizeClass(item)
        ));
    }
    if (isRotorcraft(item)) return 8.0f;
    uint8_t sizeClass = planeSizeClass(item);
    if (sizeClass == 0) return 9.0f;
    if (sizeClass >= 2) return 15.0f;
    return 12.0f;
}

static bool aircraftMapPoint(
    const Aircraft &item,
    bool mapVisible,
    int &x,
    int &y
) {
    x = item.screenX;
    y = item.screenY;
    if (item.inside) return true;

    if (mapVisible) {
        projectToMapEdge(x, y, x, y);
        return true;
    }

    float dxKm = 0;
    float dyKm = 0;
    float distKm = 0;
    offsetKm(item.renderLat, item.renderLon, dxKm, dyKm, distKm);
    if (distKm < 0.01f) return false;
    float angle = atan2f(dxKm, dyKm);
    x = RADAR_CX + lroundf(sinf(angle) * (RADAR_RADIUS + 12));
    y = RADAR_CY - lroundf(cosf(angle) * (RADAR_RADIUS + 12));
    return true;
}

template <typename Gfx>
static size_t prepareRadarLabels(
    Gfx &g,
    const Aircraft *items,
    size_t itemCount,
    const char *selectedHex,
    int labelScalePercent,
    const RouteCacheEntry *routes,
    size_t routeCount
) {
    size_t labelCount = 0;
    for (size_t aircraftIndex = 0;
         aircraftIndex < itemCount && labelCount < MAX_AIRCRAFT;
         aircraftIndex++) {
        const Aircraft &item = items[aircraftIndex];
        if (!item.inside || item.screenX < 0 || item.screenX >= PANEL_X ||
            item.screenY < 0 || item.screenY >= SCREEN_H) {
            continue;
        }

        RadarLabelRender &label = radarLabels[labelCount];
        label = RadarLabelRender();
        label.aircraftIndex = aircraftIndex;
        label.mustShow = squawkAlertLabel(item.squawk) != nullptr ||
            (selectedHex != nullptr && selectedHex[0] != '\0' &&
             strcmp(item.hex, selectedHex) == 0);

        auto appendLine = [&](const char *text, uint16_t color, uint8_t textSize) {
            if (text == nullptr || text[0] == '\0' || label.lineCount >= 3) return;
            g.setTextSize(textSize);
            RadarLabelLine &line = label.lines[label.lineCount++];
            strlcpy(line.text, text, sizeof(line.text));
            line.color = color;
            line.textSize = textSize;
            line.width = g.textWidth(line.text);
            line.height = g.textHeight();
            line.advance = g.textLineAdvance();
            label.width = std::max(label.width, line.width);
        };

        const uint8_t baseSize = labelBaseTextSize(labelScalePercent);
        const uint8_t routeSize = static_cast<uint8_t>(baseSize + 1);

        char routePair[16] = {};
        const bool hasRoute = routePairLabel(
            routes, routeCount, item.callsign, routePair, sizeof(routePair));
        const char *callsign = item.callsign[0] ? item.callsign : "????";

        if (hasRoute) {
            appendLine(routePair, colorText, routeSize);
        } else if (config.showLabelCallsign) {
            appendLine(callsign, colorText, routeSize);
        }

        if (hasRoute) {
            char identity[40] = {};
            if (config.showLabelCallsign && config.showLabelType && item.type[0] != '\0') {
                snprintf(identity, sizeof(identity), "%s %s", callsign, item.type);
                appendLine(identity, colorRunway, baseSize);
            } else if (config.showLabelCallsign) {
                appendLine(callsign, colorRunway, baseSize);
            } else if (config.showLabelType) {
                appendLine(item.type, colorRunway, baseSize);
            }
        } else if (config.showLabelType) {
            appendLine(item.type, colorRunway, baseSize);
        }

        char altitudeLine[32] = {};
        if (config.showLabelAltitude && item.alt[0] != '\0') {
            strlcpy(altitudeLine, item.alt, sizeof(altitudeLine));
        }
        if (config.showLabelVerticalRate && item.vsi[0] != '\0') {
            if (altitudeLine[0] != '\0') {
                strlcat(altitudeLine, " ", sizeof(altitudeLine));
            }
            strlcat(altitudeLine, item.vsi, sizeof(altitudeLine));
        }
        appendLine(altitudeLine, colorWarn, baseSize);
        if (label.lineCount == 0) continue;

        label.width += AIRCRAFT_LABEL_PADDING * 2;
        int height = AIRCRAFT_LABEL_PADDING * 2;
        for (size_t lineIndex = 0; lineIndex < label.lineCount; lineIndex++) {
            const RadarLabelLine &line = label.lines[lineIndex];
            height += (lineIndex + 1 == label.lineCount) ? line.height : line.advance;
        }
        label.height = height;

        RadarLabels::LabelLayoutInput &input = labelLayoutInputs[labelCount];
        input = RadarLabels::LabelLayoutInput();
        input.id = aircraftLabelId(item);
        input.anchorX = static_cast<float>(item.screenX);
        input.anchorY = static_cast<float>(item.screenY);
        input.width = static_cast<float>(label.width);
        input.height = static_cast<float>(label.height);
        input.symbolRadius = aircraftSymbolRadius(item);
        input.courseDeg = item.trackDeg;
        input.distanceKm = item.distanceKm;
        input.courseValid = item.hasTrack;
        input.mustShow = label.mustShow;
        labelCount++;
    }
    return labelCount;
}

template <typename Gfx>
static void drawRunways(Gfx &g) {
    if (!config.showRunways) return;
    g.setTextSize(1);
    g.setTextColor(colorRunway, colorBg);
    g.setTextDatum(textdatum_t::middle_center);
    bool rectangularMap = RadarMap::background.isReady(rangeIndex);
    float pxPerKm = static_cast<float>(RADAR_RADIUS) / activeOuterKm();
    for (size_t airportIndex = 0; airportIndex < selectedAirportCount; airportIndex++) {
        const AirportCatalogEntry &airport = *selectedAirports[airportIndex].airport;
        for (size_t runwayOffset = 0; runwayOffset < airport.runwayCount; runwayOffset++) {
            size_t runwayIndex = static_cast<size_t>(airport.firstRunway) + runwayOffset;
            if (runwayIndex >= kAirportRunwayCount) break;
            const AirportRunwayEntry &runway = kAirportRunways[runwayIndex];
            float latitude = airportCoordinate(runway.latE5);
            float longitude = airportCoordinate(runway.lonE5);
            int x = 0;
            int y = 0;
            float distKm = 0;
            bool insideRadar = toRadarPoint(latitude, longitude, x, y, distKm);
            if (!rectangularMap && !insideRadar) continue;
            if (x < -20 || x > PANEL_X + 20 || y < -20 || y > SCREEN_H + 20) continue;

            float half = std::max(
                8.0f,
                (static_cast<float>(runway.lengthMeters) / 1000.0f) * pxPerKm * 0.5f
            );
            float rad = (static_cast<float>(runway.headingDeciDeg) / 10.0f) * DEG_TO_RAD;
            int dx = lroundf(sinf(rad) * half);
            int dy = lroundf(cosf(rad) * half);
            g.drawWideLine(x - dx, y + dy, x + dx, y - dy, 2.0f, colorRunway);
        }

        int labelX = 0;
        int labelY = 0;
        float labelDistanceKm = 0;
        bool labelInsideRadar = toRadarPoint(
            airportCoordinate(airport.latE5),
            airportCoordinate(airport.lonE5),
            labelX,
            labelY,
            labelDistanceKm
        );
        if ((rectangularMap || labelInsideRadar) &&
            labelX >= -20 && labelX <= PANEL_X + 20 &&
            labelY >= -20 && labelY <= SCREEN_H + 20) {
            g.drawString(airport.icao, labelX, labelY - 12);
        }
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
static void truncateToWidth(Gfx &g, char *line, int maxWidth) {
    if (line == nullptr) return;
    while (line[0] != '\0' && g.textWidth(line) > maxWidth) {
        line[strlen(line) - 1] = '\0';
    }
}

template <typename Gfx>
static void drawAircraftList(
    Gfx &g,
    const Aircraft *items,
    size_t itemCount,
    const RouteCacheEntry *routes,
    size_t routeCount,
    const char *emptyStatus,
    const char *selectedHex
) {
    g.fillRect(PANEL_X, 0, SCREEN_W - PANEL_X, SCREEN_H, colorBg);
    g.drawWideLine(PANEL_X - 8, 18, PANEL_X - 8, SCREEN_H - 18, 1.0f, colorGrid);
    visibleListRowCount = 0;
    memset(visibleListAircraftHex, 0, sizeof(visibleListAircraftHex));

    rangeMinusHit = {
        PANEL_X + PANEL_PAD,
        10,
        RANGE_BUTTON_W,
        RANGE_BUTTON_H
    };
    rangePlusHit = {
        rangeMinusHit.x + RANGE_BUTTON_W + RANGE_BUTTON_GAP,
        10,
        RANGE_BUTTON_W,
        RANGE_BUTTON_H
    };
    auto drawRangeButton = [&](const TouchRect &rect, const char *label) {
        g.fillRect(rect.x, rect.y, rect.w, rect.h, colorSelectedRow);
        g.drawWideLine(rect.x, rect.y, rect.x + rect.w, rect.y, 1.0f, colorGrid);
        g.drawWideLine(rect.x, rect.y + rect.h - 1, rect.x + rect.w, rect.y + rect.h - 1, 1.0f, colorGrid);
        g.drawWideLine(rect.x, rect.y, rect.x, rect.y + rect.h, 1.0f, colorGrid);
        g.drawWideLine(rect.x + rect.w - 1, rect.y, rect.x + rect.w - 1, rect.y + rect.h, 1.0f, colorGrid);
        g.setTextDatum(textdatum_t::middle_center);
        g.setTextSize(2);
        g.setTextColor(colorText, colorSelectedRow);
        g.drawString(label, rect.x + rect.w / 2, rect.y + rect.h / 2);
    };
    drawRangeButton(rangeMinusHit, "-");
    drawRangeButton(rangePlusHit, "+");

    g.setTextDatum(textdatum_t::top_right);
    g.setTextSize(2);
    g.setTextColor(colorDim, colorBg);
    char rangeTitle[24];
    snprintf(rangeTitle, sizeof(rangeTitle), "RANGE %s", rangeLabel());
    g.drawString(rangeTitle, PANEL_RIGHT, 10);

    int textWidth = PANEL_RIGHT - PANEL_TEXT_X;
    bool selectedList = selectedHex != nullptr && selectedHex[0] != '\0';
    int maxRows = static_cast<int>(listRowCapacity(selectedList));
    int drawn = 0;
    for (int idx = static_cast<int>(itemCount) - 1; idx >= 0 && drawn < maxRows; idx--) {
        const Aircraft &item = items[idx];
        int rowY = PANEL_LIST_TOP + drawn * PANEL_ROW_H;
        int iconX = PANEL_X + 20;
        int iconY = rowY + 23;
        bool selected = selectedHex != nullptr &&
            selectedHex[0] != '\0' &&
            strcmp(item.hex, selectedHex) == 0;
        uint16_t rowBg = selected ? colorSelectedRow : colorBg;
        if (selected) {
            g.fillRect(
                PANEL_X + 1,
                rowY - 2,
                SCREEN_W - PANEL_X - 2,
                PANEL_ROW_H - 1,
                rowBg
            );
            g.fillRect(PANEL_X + 2, rowY - 2, 3, PANEL_ROW_H - 1, colorSelected);
        }

        uint16_t iconColor = selected
            ? colorSelected
            : (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons
                ? colorWarn
                : colorPlane);
        drawAircraftSymbol(g, item, iconX, iconY, iconColor);

        g.setTextDatum(textdatum_t::top_left);
        g.setTextSize(2);
        g.setTextColor(colorText, rowBg);
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
        g.setTextColor(colorDim, rowBg);
        g.drawString(detail, PANEL_TEXT_X, rowY + 20);

        const char *squawkAlert = squawkAlertLabel(item.squawk);
        if (squawkAlert != nullptr) {
            char alert[32];
            snprintf(alert, sizeof(alert), "%s %s", item.squawk, squawkAlert);
            g.setTextColor(colorWarn, rowBg);
            g.drawString(alert, PANEL_TEXT_X, rowY + 32);
        } else {
            char route[(ROUTE_CITY_MAX_LEN * 2) + 16];
            if (routeLabelForCallsign(
                    g, routes, routeCount, item.callsign, textWidth, route, sizeof(route))) {
                g.setTextColor(colorRunway, rowBg);
                g.drawString(route, PANEL_TEXT_X, rowY + 32);
            }
        }

        g.drawWideLine(PANEL_X + PANEL_PAD, rowY + PANEL_ROW_H - 4, PANEL_RIGHT, rowY + PANEL_ROW_H - 4, 1.0f, colorGrid);
        strlcpy(
            visibleListAircraftHex[drawn],
            item.hex,
            sizeof(visibleListAircraftHex[drawn])
        );
        visibleListRowCount = static_cast<size_t>(drawn + 1);
        drawn++;
    }

    if (drawn == 0) {
        g.setTextDatum(textdatum_t::top_left);
        g.setTextSize(1);
        g.setTextColor(colorDim, colorBg);
        g.drawString(WiFi.status() == WL_CONNECTED ? "NO AIRCRAFT" : emptyStatus, PANEL_TEXT_X, PANEL_LIST_TOP);
    }
}

template <typename Gfx>
static void drawWeatherClockFooter(Gfx &g, bool showMetar, bool fahrenheit, bool clock24) {
    int top = panelFooterTopY();
    g.fillRect(PANEL_X + 1, top, SCREEN_W - PANEL_X - 1, PANEL_FOOTER_H, colorBg);
    g.drawWideLine(PANEL_X + PANEL_PAD, top + 2, PANEL_RIGHT, top + 2, 1.0f, colorGrid);

    const int leftX = PANEL_X + PANEL_PAD;
    const int metarWidth = PANEL_RIGHT - leftX;
    char timeText[12];
    char dateText[12];
    char icaoText[8];
    WeatherTime::formatTime(timeText, sizeof(timeText), clock24);
    WeatherTime::stationIcao(icaoText, sizeof(icaoText));
    if (!(showMetar && WeatherTime::formatMetarObsTime(dateText, sizeof(dateText)))) {
        WeatherTime::formatDate(dateText, sizeof(dateText));
    }

    g.setTextSize(2);
    g.setTextColor(colorText, colorBg);
    g.setTextDatum(textdatum_t::top_left);
    g.drawString(timeText, leftX, top + 8);
    g.setTextDatum(textdatum_t::top_right);
    g.drawString(dateText, PANEL_RIGHT, top + 8);
    if (showMetar && icaoText[0] != '\0') {
        g.setTextDatum(textdatum_t::middle_center);
        g.drawString(icaoText, (leftX + PANEL_RIGHT) / 2, top + 15);
    }

    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorDim, colorBg);
    if (showMetar) {
        char metar[160];
        WeatherTime::formatMetarBody(metar, sizeof(metar), fahrenheit);
        int lineY = top + 32;
        int maxLines = 3;
        const char *cursor = metar;
        for (int line = 0; line < maxLines && cursor[0] != '\0'; line++) {
            char wrapped[96] = {};
            size_t used = 0;
            while (cursor[0] != '\0') {
                while (cursor[0] == ' ') {
                    ++cursor;
                }
                if (cursor[0] == '\0') {
                    break;
                }
                const char *token = cursor;
                while (cursor[0] != '\0' && cursor[0] != ' ') {
                    ++cursor;
                }
                size_t tokenLen = static_cast<size_t>(cursor - token);
                char candidate[96];
                strlcpy(candidate, wrapped, sizeof(candidate));
                if (used > 0) {
                    strlcat(candidate, " ", sizeof(candidate));
                }
                char tokenText[48];
                size_t copyLen = tokenLen < sizeof(tokenText) - 1 ? tokenLen : sizeof(tokenText) - 1;
                memcpy(tokenText, token, copyLen);
                tokenText[copyLen] = '\0';
                strlcat(candidate, tokenText, sizeof(candidate));
                if (g.textWidth(candidate) > metarWidth) {
                    cursor = token;
                    break;
                }
                strlcpy(wrapped, candidate, sizeof(wrapped));
                used = strlen(wrapped);
            }
            if (wrapped[0] == '\0') {
                break;
            }
            g.drawString(wrapped, leftX, lineY);
            lineY += g.textLineAdvance();
        }
    } else {
        g.setTextSize(1);
        g.drawString("LOCAL TIME", leftX, top + 36);
    }
    g.setTextSize(1);
}

template <typename Gfx>
static void drawSelectedAircraftDetail(
    Gfx &g,
    const Aircraft *item,
    const RouteCacheEntry *routes,
    size_t routeCount
) {
    int top = panelDetailTopY();
    int height = PANEL_DETAIL_H;
    g.fillRect(PANEL_X + 1, top, SCREEN_W - PANEL_X - 1, height, colorSelectedRow);
    g.fillRect(PANEL_X + 2, top + 6, 3, height - 12, colorSelected);
    g.drawWideLine(PANEL_X + PANEL_PAD, top + 2, PANEL_RIGHT, top + 2, 1.0f, colorGrid);

    if (item == nullptr) {
        return;
    }

    int textWidth = PANEL_RIGHT - PANEL_TEXT_X;
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(2);
    g.setTextColor(colorText, colorSelectedRow);
    g.drawString(item->callsign[0] ? item->callsign : "????", PANEL_TEXT_X, top + 6);

    const char *typeCode = item->type[0] ? item->type : "--";
    const char *fullType = typeNameForCode(
        renderTypeNameCache,
        MAX_TYPE_CACHE,
        item->type
    );
    g.setTextColor(colorSelected, colorSelectedRow);
    g.drawString(typeCode, PANEL_TEXT_X, top + 26);
    if (fullType != nullptr) {
        int nameX = PANEL_TEXT_X + g.textWidth(typeCode) + g.textWidth("  ");
        int nameWidth = PANEL_RIGHT - nameX;
        if (nameWidth > 8) {
            char typeLine[48];
            strlcpy(typeLine, fullType, sizeof(typeLine));
            truncateToWidth(g, typeLine, nameWidth);
            g.setTextColor(colorText, colorSelectedRow);
            g.drawString(typeLine, nameX, top + 26);
        }
    }

    g.setTextSize(1);
    g.setTextColor(colorDim, colorSelectedRow);
    char line[96] = {};
    appendTokenIfFits(g, line, sizeof(line), item->hex, textWidth);
    appendTokenIfFits(g, line, sizeof(line), item->registration, textWidth);
    appendTokenIfFits(g, line, sizeof(line), item->category, textWidth);
    g.drawString(line[0] ? line : "NO ID", PANEL_TEXT_X, top + 48);

    char distance[16];
    char speed[16];
    formatDistanceLabel(item->distanceKm, distance, sizeof(distance));
    formatSpeedLabel(item->gsKnots, speed, sizeof(speed));
    line[0] = '\0';
    appendTokenIfFits(g, line, sizeof(line), distance, textWidth);
    appendTokenIfFits(g, line, sizeof(line), item->alt[0] ? item->alt : "ALT --", textWidth);
    appendTokenIfFits(g, line, sizeof(line), item->vsi, textWidth);
    appendTokenIfFits(g, line, sizeof(line), speed, textWidth);
    g.drawString(line, PANEL_TEXT_X, top + 62);

    char heading[24];
    snprintf(heading, sizeof(heading), "TRK %03.0f", item->hasTrack ? item->trackDeg : item->noseDeg);
    line[0] = '\0';
    appendTokenIfFits(g, line, sizeof(line), heading, textWidth);
    if (item->squawk[0] != '\0') {
        appendTokenIfFits(g, line, sizeof(line), item->squawk, textWidth);
        const char *alert = squawkAlertLabel(item->squawk);
        if (alert != nullptr) {
            appendTokenIfFits(g, line, sizeof(line), alert, textWidth);
        }
    }
    g.setTextColor(colorWarn, colorSelectedRow);
    g.drawString(line, PANEL_TEXT_X, top + 76);

    char route[(ROUTE_CITY_MAX_LEN * 2) + 16];
    if (routeLabelForCallsign(
            g, routes, routeCount, item->callsign, textWidth, route, sizeof(route))) {
        g.setTextColor(colorRunway, colorSelectedRow);
        g.drawString(route, PANEL_TEXT_X, top + 90);
    }

    g.setTextColor(colorDim, colorSelectedRow);
    g.drawString("TAP HERE TO CLEAR", PANEL_TEXT_X, top + 124);
}

static void drawMapAttribution(PanelDisplay::Canvas &g) {
    char firstLine[40];
    char secondLine[24];
    snprintf(
        firstLine,
        sizeof(firstLine),
        "%c STADIA MAPS / %c OPENMAPTILES",
        PanelDisplay::GLYPH_COPYRIGHT,
        PanelDisplay::GLYPH_COPYRIGHT
    );
    snprintf(
        secondLine,
        sizeof(secondLine),
        "%c OPENSTREETMAP",
        PanelDisplay::GLYPH_COPYRIGHT
    );

    g.fillRect(2, SCREEN_H - 23, 186, 21, colorBg);
    g.setTextDatum(textdatum_t::top_left);
    g.setTextSize(1);
    g.setTextColor(colorDim, colorBg);
    g.drawString(firstLine, 4, SCREEN_H - 21);
    g.drawString(secondLine, 4, SCREEN_H - 12);
}

static void drawRadar() {
    static uint32_t drawCounter = 0;
    drawCounter++;

    size_t renderCount = 0;
    size_t renderTrackCount = 0;
    char emptyStatus[64];
    char renderSelectedHex[7] = {};
    size_t renderRangeIndex = 0;
    bool renderShowWeather = true;
    bool renderTempF = false;
    bool renderClock24 = true;
    int renderLabelScale = LABEL_TEXT_SCALE_DEFAULT;
    lockState();
    renderCount = aircraftCount;
    if (renderCount > 0) {
        memcpy(renderAircraft, aircraft, renderCount * sizeof(Aircraft));
    }
    memcpy(renderRouteCache, routeCache, sizeof(renderRouteCache));
    memcpy(renderTypeNameCache, typeNameCache, sizeof(renderTypeNameCache));
    strlcpy(
        renderSelectedHex,
        selectedAircraftHex,
        sizeof(renderSelectedHex)
    );
    renderTrackCount = snapshotSelectedTrackLocked(
        renderTrack,
        TRACK_POINTS_PER_AIRCRAFT
    );
    strlcpy(emptyStatus, statusText.c_str(), sizeof(emptyStatus));
    renderRangeIndex = rangeIndex;
    renderShowWeather = config.showWeather;
    renderTempF = config.tempFahrenheit;
    renderClock24 = config.clock24;
    renderLabelScale = config.labelTextScalePercent;
    unlockState();

    bool logDraw = drawCounter <= 3 || drawCounter % 120 == 0;
    if (logDraw) {
        RADAR_LOGD("[draw] #%lu begin aircraft=%u wifi=%d w=%d h=%d free_heap=%u free_psram=%u\n",
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
    const Aircraft *selectedAircraft = findAircraftByHex(
        renderAircraft,
        renderCount,
        renderSelectedHex
    );
    drawSelectedAircraftTrack(
        g,
        renderTrack,
        renderTrackCount,
        selectedAircraft,
        mapVisible
    );
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

    radarHitCount = 0;
    size_t aircraftObstacleCount = 0;
    for (size_t i = 0; i < renderCount; i++) {
        int x = 0;
        int y = 0;
        if (!aircraftMapPoint(renderAircraft[i], mapVisible, x, y)) continue;

        RadarLabels::AircraftObstacle &obstacle =
            labelAircraftObstacles[aircraftObstacleCount++];
        obstacle.x = static_cast<float>(x);
        obstacle.y = static_cast<float>(y);
        obstacle.radius = renderAircraft[i].inside
            ? aircraftSymbolRadius(renderAircraft[i])
            : 4.0f;

        if (radarHitCount < MAX_AIRCRAFT && renderAircraft[i].hex[0] != '\0') {
            RadarHitTarget &hit = radarHitTargets[radarHitCount++];
            strlcpy(hit.hex, renderAircraft[i].hex, sizeof(hit.hex));
            hit.x = x;
            hit.y = y;
            hit.radius = std::max(
                18,
                static_cast<int>(lroundf(obstacle.radius + 8.0f))
            );
        }
    }

    size_t labelCount = prepareRadarLabels(
        g,
        renderAircraft,
        renderCount,
        renderSelectedHex,
        renderLabelScale,
        renderRouteCache,
        MAX_ROUTE_CACHE
    );
    RadarLabels::LabelRectObstacle staticLabelObstacles[] = {
        {static_cast<float>(PANEL_X - 11), 0.0f, 11.0f, static_cast<float>(SCREEN_H)},
    };
    static uint32_t previousLabelLayoutMs = 0;
    uint32_t labelLayoutNowMs = millis();
    float labelLayoutDeltaSeconds = previousLabelLayoutMs == 0
        ? (1.0f / 30.0f)
        : static_cast<float>(labelLayoutNowMs - previousLabelLayoutMs) / 1000.0f;
    previousLabelLayoutMs = labelLayoutNowMs;
    uint32_t labelLayoutRevision = static_cast<uint32_t>(renderRangeIndex) |
        (mapVisible ? 0x100U : 0U) |
        (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons ? 0x200U : 0U) |
        (static_cast<uint32_t>(renderLabelScale) << 16);
    RadarLabels::LabelLayoutMetrics labelMetrics;
    uint32_t labelLayoutStartedUs = micros();
    aircraftLabelLayout.solve(
        labelLayoutInputs,
        labelCount,
        labelAircraftObstacles,
        aircraftObstacleCount,
        staticLabelObstacles,
        sizeof(staticLabelObstacles) / sizeof(staticLabelObstacles[0]),
        RadarLabels::LabelLayoutBounds{
            4.0f,
            4.0f,
            static_cast<float>(PANEL_X - 12),
            static_cast<float>(SCREEN_H - 4)
        },
        labelLayoutNowMs,
        labelLayoutRevision,
        labelLayoutDeltaSeconds,
        labelLayoutOutputs,
        &labelMetrics
    );
    uint32_t labelLayoutElapsedUs = micros() - labelLayoutStartedUs;

    g.setTextDatum(textdatum_t::top_left);
    auto drawRadarLabel = [&](size_t labelIndex) {
        const RadarLabelRender &label = radarLabels[labelIndex];
        const RadarLabels::LabelLayoutOutput &layout =
            labelLayoutOutputs[labelIndex];
        if (!layout.visible) return;

        int textX = lroundf(layout.x) + AIRCRAFT_LABEL_PADDING;
        int textY = lroundf(layout.y) + AIRCRAFT_LABEL_PADDING;
        int lineY = textY;
        for (size_t lineIndex = 0; lineIndex < label.lineCount; lineIndex++) {
            const RadarLabelLine &line = label.lines[lineIndex];
            g.setTextSize(line.textSize);
            g.fillRect(
                textX - AIRCRAFT_LABEL_PADDING,
                lineY - AIRCRAFT_LABEL_PADDING,
                line.width + AIRCRAFT_LABEL_PADDING * 2,
                line.height + AIRCRAFT_LABEL_PADDING * 2,
                colorBg
            );
            g.setTextColor(line.color, colorBg);
            g.drawString(line.text, textX, lineY);
            lineY += line.advance;
        }
    };

    size_t labelByAircraft[MAX_AIRCRAFT];
    for (size_t i = 0; i < renderCount; i++) {
        labelByAircraft[i] = MAX_AIRCRAFT;
    }
    for (size_t labelIndex = 0; labelIndex < labelCount; labelIndex++) {
        size_t aircraftIndex = radarLabels[labelIndex].aircraftIndex;
        if (aircraftIndex < renderCount) {
            labelByAircraft[aircraftIndex] = labelIndex;
        }
    }

    size_t aircraftDrawOrder[MAX_AIRCRAFT];
    for (size_t i = 0; i < renderCount; i++) {
        aircraftDrawOrder[i] = i;
    }
    auto aircraftDrawsBefore = [&](size_t leftIndex, size_t rightIndex) {
        const Aircraft &left = renderAircraft[leftIndex];
        const Aircraft &right = renderAircraft[rightIndex];
        bool leftSelected = renderSelectedHex[0] != '\0' &&
            strcmp(left.hex, renderSelectedHex) == 0;
        bool rightSelected = renderSelectedHex[0] != '\0' &&
            strcmp(right.hex, renderSelectedHex) == 0;
        if (leftSelected != rightSelected) {
            return !leftSelected;
        }
        if (left.altitudeFt != right.altitudeFt) {
            return left.altitudeFt < right.altitudeFt;
        }
        return leftIndex < rightIndex;
    };
    for (size_t i = 1; i < renderCount; i++) {
        size_t value = aircraftDrawOrder[i];
        size_t j = i;
        while (j > 0 && aircraftDrawsBefore(value, aircraftDrawOrder[j - 1])) {
            aircraftDrawOrder[j] = aircraftDrawOrder[j - 1];
            j--;
        }
        aircraftDrawOrder[j] = value;
    }

    for (size_t orderIndex = 0; orderIndex < renderCount; orderIndex++) {
        size_t i = aircraftDrawOrder[orderIndex];
        int x = 0;
        int y = 0;
        if (!aircraftMapPoint(renderAircraft[i], mapVisible, x, y)) continue;

        bool selected = renderSelectedHex[0] != '\0' &&
            strcmp(renderAircraft[i].hex, renderSelectedHex) == 0;
        uint16_t symbolColor = selected
            ? colorSelected
            : (config.aircraftSymbolStyle == AircraftSymbolStyle::DetailedIcons
                ? colorWarn
                : colorPlane);

        if (!renderAircraft[i].inside) {
            g.fillSmoothCircle(x, y, selected ? 6 : 4, symbolColor);
        } else if (x >= 0 && x < PANEL_X && y >= 0 && y < SCREEN_H) {
            drawAircraftTrajectory(g, renderAircraft[i], x, y, selected);
            drawAircraftSymbol(g, renderAircraft[i], x, y, symbolColor);
        }

        size_t labelIndex = labelByAircraft[i];
        if (labelIndex < labelCount) {
            drawRadarLabel(labelIndex);
        }
    }

    if (mapVisible) {
        drawMapAttribution(g);
    }

    if (logDraw) {
        RADAR_LOGD(
            "[labels] visible=%u hidden=%u max_overlap=%.2f solver_us=%lu\n",
            static_cast<unsigned>(labelMetrics.visibleCount),
            static_cast<unsigned>(labelMetrics.hiddenCount),
            labelMetrics.maxOverlapPx,
            static_cast<unsigned long>(labelLayoutElapsedUs)
        );
    }

    drawAircraftList(
        g,
        renderAircraft,
        renderCount,
        renderRouteCache,
        MAX_ROUTE_CACHE,
        emptyStatus,
        renderSelectedHex
    );
    if (renderSelectedHex[0] != '\0') {
        drawSelectedAircraftDetail(
            g,
            selectedAircraft,
            renderRouteCache,
            MAX_ROUTE_CACHE
        );
    }
    drawWeatherClockFooter(g, renderShowWeather, renderTempF, renderClock24);

    g.endWrite();
    presentScreenOrRestart();
    uint32_t completedAt = millis();
    lockState();
    lastDrawMs = completedAt;
    unlockState();
    if (logDraw) {
        RADAR_LOGD("[draw] #%lu end at=%lu\n",
                   static_cast<unsigned long>(drawCounter),
                   static_cast<unsigned long>(completedAt));
    }
}

static bool pointInRect(uint16_t x, uint16_t y, const TouchRect &rect) {
    return static_cast<int>(x) >= rect.x &&
        static_cast<int>(x) < rect.x + rect.w &&
        static_cast<int>(y) >= rect.y &&
        static_cast<int>(y) < rect.y + rect.h;
}

static void applyRangeDelta(int delta) {
    bool changed = false;
    lockState();
    int next = static_cast<int>(rangeIndex) + delta;
    if (next < 0) {
        next = 0;
    }
    if (next >= static_cast<int>(RANGE_COUNT)) {
        next = static_cast<int>(RANGE_COUNT) - 1;
    }
    if (static_cast<size_t>(next) != rangeIndex) {
        rangeIndex = static_cast<size_t>(next);
        forceAdsbFetch = true;
        networkDataDirty = true;
        changed = true;
    }
    unlockState();
    if (changed) {
        saveRange();
    }
}

static bool handleRangeButtonTap(uint16_t x, uint16_t y) {
    if (pointInRect(x, y, rangeMinusHit)) {
        applyRangeDelta(-1);
        return true;
    }
    if (pointInRect(x, y, rangePlusHit)) {
        applyRangeDelta(1);
        return true;
    }
    return false;
}

static bool selectOrToggleAircraftHex(const char *hex) {
    if (hex == nullptr || hex[0] == '\0') {
        return false;
    }
    bool selected = false;
    lockState();
    if (strcmp(selectedAircraftHex, hex) == 0) {
        selectedAircraftHex[0] = '\0';
    } else {
        strlcpy(selectedAircraftHex, hex, sizeof(selectedAircraftHex));
        selected = true;
    }
    networkDataDirty = true;
    unlockState();
    RADAR_LOGD("[track] %s hex=%s\n", selected ? "selected" : "cleared", hex);
    return true;
}

static bool handleRadarAircraftTap(uint16_t x, uint16_t y) {
    if (x >= PANEL_X) {
        return false;
    }
    int bestIndex = -1;
    int bestDist = 0;
    for (size_t i = 0; i < radarHitCount; i++) {
        int dx = static_cast<int>(x) - radarHitTargets[i].x;
        int dy = static_cast<int>(y) - radarHitTargets[i].y;
        int dist2 = dx * dx + dy * dy;
        int limit = radarHitTargets[i].radius;
        if (dist2 > limit * limit) {
            continue;
        }
        if (bestIndex < 0 || dist2 < bestDist) {
            bestIndex = static_cast<int>(i);
            bestDist = dist2;
        }
    }
    if (bestIndex < 0) {
        return true;
    }
    selectOrToggleAircraftHex(radarHitTargets[bestIndex].hex);
    return true;
}

static bool handleAircraftListTap(uint16_t x, uint16_t y) {
    if (x < PANEL_X) {
        return false;
    }
    if (y >= panelFooterTopY()) {
        return true;
    }

    bool hadSelection = false;
    lockState();
    hadSelection = selectedAircraftHex[0] != '\0';
    unlockState();

    if (hadSelection && y >= panelDetailTopY() && y < panelFooterTopY()) {
        lockState();
        selectedAircraftHex[0] = '\0';
        networkDataDirty = true;
        unlockState();
        RADAR_LOGD("[track] cleared from detail card\n");
        return true;
    }

    if (y < PANEL_LIST_TOP) {
        return true;
    }

    size_t row = static_cast<size_t>((y - PANEL_LIST_TOP) / PANEL_ROW_H);
    size_t maxRows = listRowCapacity(hadSelection);
    if (row >= visibleListRowCount || row >= maxRows) {
        return true;
    }

    char tappedHex[7] = {};
    strlcpy(tappedHex, visibleListAircraftHex[row], sizeof(tappedHex));
    if (tappedHex[0] == '\0') {
        return true;
    }

    bool selected = false;
    bool changed = false;
    lockState();
    if (strcmp(selectedAircraftHex, tappedHex) == 0) {
        selectedAircraftHex[0] = '\0';
        changed = true;
    } else {
        strlcpy(
            selectedAircraftHex,
            tappedHex,
            sizeof(selectedAircraftHex)
        );
        selected = true;
        changed = true;
    }
    if (changed) {
        networkDataDirty = true;
    }
    unlockState();

    if (changed) {
        RADAR_LOGD(
            "[track] %s hex=%s row=%u\n",
            selected ? "selected" : "cleared",
            tappedHex,
            static_cast<unsigned>(row)
        );
    }
    return true;
}

static void handleTouch() {
    if (OtaUpdate::inProgress()) {
        return;
    }
    uint16_t x = 0;
    uint16_t y = 0;
    uint32_t now = millis();
    bool rawDown = screen.readTouch(&x, &y);
    if (rawDown) {
        touchLastContactMs = now;
    }
    bool down = rawDown ||
        (touchWasDown && now - touchLastContactMs < TOUCH_RELEASE_DEBOUNCE_MS);
    if (down && !touchWasDown) {
        touchDownMs = now;
        touchDownX = x;
        touchDownY = y;
        touchLastX = x;
        touchLastY = y;
        longPressHandled = false;
    }
    if (rawDown) {
        touchLastX = x;
        touchLastY = y;
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
        int movedX = abs(static_cast<int>(touchLastX) - touchDownX);
        int movedY = abs(static_cast<int>(touchLastY) - touchDownY);
        bool tap = !longPressHandled &&
            held < TOUCH_LONG_PRESS_MS &&
            movedX <= TOUCH_TAP_MOVE_MAX_PX &&
            movedY <= TOUCH_TAP_MOVE_MAX_PX;
        if (tap) {
            if (!handleRangeButtonTap(touchDownX, touchDownY) &&
                !handleAircraftListTap(touchDownX, touchDownY)) {
                handleRadarAircraftTap(touchDownX, touchDownY);
            }
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

    return dirty ||
        (hasAircraft && now - previousDrawMs >= RADAR_DRAW_INTERVAL_MS) ||
        (now - previousDrawMs >= 1000);
}

static void networkTaskMain(void *) {
    RADAR_LOGD("[task] network start core=%d\n", xPortGetCoreID());
    AppWatchdog::subscribeCurrentTask("plane-net");

    while (true) {
        AppWatchdog::feed();
        if (OtaUpdate::inProgress()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
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
            serviceTypeLookup();

            static char cachedNearestIcao[8] = {};
            static double cachedNearestLat = 999.0;
            static double cachedNearestLon = 999.0;
            char metarIcao[8] = {};
            char manualIcao[8] = {};
            char selectedIcao[8] = {};
            double weatherLat = 0;
            double weatherLon = 0;
            bool fetchMetar = false;
            AirportSelectionMode airportMode = AirportSelectionMode::Automatic;
            lockState();
            weatherLat = config.lat;
            weatherLon = config.lon;
            fetchMetar = config.showWeather;
            airportMode = config.airportSelectionMode;
            strlcpy(manualIcao, config.manualAirportIcao.c_str(), sizeof(manualIcao));
            if (selectedAirportCount > 0 && selectedAirports[0].airport != nullptr) {
                strlcpy(
                    selectedIcao,
                    selectedAirports[0].airport->icao,
                    sizeof(selectedIcao)
                );
            }
            unlockState();
            if (fetchMetar) {
                if (airportMode == AirportSelectionMode::Manual && manualIcao[0] != '\0') {
                    strlcpy(metarIcao, manualIcao, sizeof(metarIcao));
                } else if (selectedIcao[0] != '\0') {
                    strlcpy(metarIcao, selectedIcao, sizeof(metarIcao));
                } else {
                    const bool locationChanged =
                        fabs(weatherLat - cachedNearestLat) > 0.0001 ||
                        fabs(weatherLon - cachedNearestLon) > 0.0001;
                    if (cachedNearestIcao[0] == '\0' || locationChanged) {
                        const AirportCatalogEntry *airport =
                            nearestAirport(weatherLat, weatherLon);
                        cachedNearestIcao[0] = '\0';
                        if (airport != nullptr) {
                            strlcpy(cachedNearestIcao, airport->icao,
                                    sizeof(cachedNearestIcao));
                        }
                        cachedNearestLat = weatherLat;
                        cachedNearestLon = weatherLon;
                    }
                    strlcpy(metarIcao, cachedNearestIcao, sizeof(metarIcao));
                }
            }
            if (WeatherTime::refreshIfDue(
                    weatherLat, weatherLon, fetchMetar ? metarIcao : "")) {
                lockState();
                networkDataDirty = true;
                unlockState();
            }
        }

        AppWatchdog::feed();
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
        RADAR_LOGI("[task] network task created on core 0\n");
    } else {
        RADAR_LOGE("[task] network task create failed\n");
    }
}

static void initPalette() {
    colorBg = screen.color565(2, 8, 7);
    colorGrid = screen.color565(8, 46, 33);
    colorText = screen.color565(235, 255, 238);
    colorDim = screen.color565(110, 190, 145);
    colorPlane = screen.color565(255, 55, 80);
    colorRunway = screen.color565(66, 210, 210);
    colorWarn = screen.color565(255, 220, 70);
    colorTrackDim = screen.color565(84, 78, 30);
    colorTrackBright = screen.color565(210, 176, 42);
    colorTrackForecast = screen.color565(125, 108, 42);
    colorSelectedRow = screen.color565(5, 28, 19);
    colorSelected = screen.color565(46, 214, 92);
    colorTrajectory = screen.color565(255, 0, 255);
}

#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_DEBUG
static void benchmarkAircraftLabelLayout() {
    for (size_t i = 0; i < MAX_AIRCRAFT; i++) {
        RadarLabels::LabelLayoutInput &input = labelLayoutInputs[i];
        input = RadarLabels::LabelLayoutInput();
        input.id = static_cast<uint32_t>(0x100000U + i);
        input.anchorX = 245.0f + static_cast<float>(i % 8) * 4.0f;
        input.anchorY = 225.0f + static_cast<float>(i / 8) * 4.0f;
        input.width = 62.0f;
        input.height = 27.0f;
        input.symbolRadius = 12.0f;
        input.courseDeg = static_cast<float>((i * 37U) % 360U);
        input.distanceKm = static_cast<float>(i + 1);
        input.courseValid = true;
        input.mustShow = i < 2;

        labelAircraftObstacles[i].x = input.anchorX;
        labelAircraftObstacles[i].y = input.anchorY;
        labelAircraftObstacles[i].radius = input.symbolRadius;
    }

    RadarLabels::LabelLayoutMetrics metrics;
    aircraftLabelLayout.reset();
    uint32_t startedUs = micros();
    aircraftLabelLayout.solve(
        labelLayoutInputs,
        MAX_AIRCRAFT,
        labelAircraftObstacles,
        MAX_AIRCRAFT,
        nullptr,
        0,
        RadarLabels::LabelLayoutBounds{
            4.0f,
            4.0f,
            static_cast<float>(PANEL_X - 12),
            static_cast<float>(SCREEN_H - 4)
        },
        millis(),
        1,
        1.0f / 30.0f,
        labelLayoutOutputs,
        &metrics
    );
    uint32_t initialUs = micros() - startedUs;

    startedUs = micros();
    aircraftLabelLayout.solve(
        labelLayoutInputs,
        MAX_AIRCRAFT,
        labelAircraftObstacles,
        MAX_AIRCRAFT,
        nullptr,
        0,
        RadarLabels::LabelLayoutBounds{
            4.0f,
            4.0f,
            static_cast<float>(PANEL_X - 12),
            static_cast<float>(SCREEN_H - 4)
        },
        millis(),
        1,
        1.0f / 30.0f,
        labelLayoutOutputs,
        &metrics
    );
    uint32_t steadyUs = micros() - startedUs;
    RADAR_LOGD(
        "[labels] benchmark64 initial_us=%lu steady_us=%lu visible=%u hidden=%u\n",
        static_cast<unsigned long>(initialUs),
        static_cast<unsigned long>(steadyUs),
        static_cast<unsigned>(metrics.visibleCount),
        static_cast<unsigned>(metrics.hiddenCount)
    );
    aircraftLabelLayout.reset();
}
#endif

void setup() {
    Serial.begin(115200);
    uint32_t serialStart = millis();
    while (!Serial && millis() - serialStart < 5000) {
        delay(20);
    }
    delay(250);
    BuildDiagnostics::logBuildConfiguration();
    BuildDiagnostics::logMemory("startup");
    AppWatchdog::logResetReason();
    stateMutex = xSemaphoreCreateMutexStatic(&stateMutexStorage);
    if (stateMutex == nullptr) {
        RADAR_LOGE("[task] state mutex create failed\n");
        RADAR_LOGE_FLUSH();
        while (true) {
            delay(1000);
        }
    }
    logLine("\n=== Plane Radar Display DIAG ===");
    logStep("setup start");
    logStep("display begin");
    if (!screen.begin()) {
        RADAR_LOGE("[display] initialization failed\n");
        RADAR_LOGE_FLUSH();
        while (true) {
            delay(1000);
        }
    }
    configureDisplayLayout();
    RADAR_LOGI(
        "[display] model=%s resolution=%dx%d pclk=%lu layout=%d+%d\n",
        screen.modelName(),
        SCREEN_W,
        SCREEN_H,
        static_cast<unsigned long>(screen.pixelClockHz()),
        PANEL_X,
        SCREEN_W - PANEL_X
    );
    BuildDiagnostics::logMemory("display-ready");
    logStep("display end");
    initAircraftTrackCache();
    resetBootScreen();
    setBootStage(BOOT_LCD, BootStatus::Ok);

    logStep("palette begin");
    setBootStage(BOOT_PALETTE, BootStatus::Running);
    setBootStageDetails(
        BOOT_PALETTE,
        "BUILDING RGB565 COLOR TABLE",
        "GRID / TEXT / AIRCRAFT / RUNWAY / TRACK"
    );
    initPalette();
    setBootStage(BOOT_PALETTE, BootStatus::Ok);
    logStep("palette end");

    logStep("loadConfig begin");
    setBootStage(BOOT_CONFIG, BootStatus::Running);
    setBootStageDetails(
        BOOT_CONFIG,
        "NVS NAMESPACE / PLANE-RADAR",
        "READING SAVED SETTINGS"
    );
    loadConfig();
    selectConfiguredAirports();
    mapRuntimeReady = config.mapProvider == MapProvider::Stadia &&
                      !config.stadiaApiKey.isEmpty() &&
                      RadarMap::background.begin(PANEL_X, SCREEN_H, RANGE_COUNT);
    char configLine[88];
    char centerLine[88];
    char mapLine[88];
    char airportLine[88];
    snprintf(
        configLine,
        sizeof(configLine),
        "SAVED CONFIG / %s / RANGE %s",
        config.configured ? "YES" : "NO",
        rangeLabel()
    );
    snprintf(centerLine, sizeof(centerLine), "CENTER / %.5f / %.5f", config.lat, config.lon);
    snprintf(
        mapLine,
        sizeof(mapLine),
        "MAP / %s / BRIGHTNESS %uPCT / CACHE %s",
        config.mapProvider == MapProvider::Stadia ? "STADIA" : "NONE",
        static_cast<unsigned>(config.mapBrightness),
        mapRuntimeReady ? "READY" : "OFF"
    );
    snprintf(
        airportLine,
        sizeof(airportLine),
        "AIRPORTS / %u SELECTED / RUNWAYS %s",
        static_cast<unsigned>(selectedAirportCount),
        config.showRunways ? "ON" : "OFF"
    );
    setBootStageDetails(BOOT_CONFIG, configLine, centerLine, mapLine, airportLine);
    setBootStage(BOOT_CONFIG, BootStatus::Ok);
    RADAR_LOGD("[config] configured=%d ssid_len=%u lat=%.6f lon=%.6f range=%u runways=%d airport_mode=%u airport_count=%u airport_radius=%u airport_icao=%s miles=%d map=%u map_brightness=%u map_key_len=%u labels=%d%d%d%d symbols=%u\n",
               config.configured,
               static_cast<unsigned>(config.ssid.length()),
               config.lat,
               config.lon,
               static_cast<unsigned>(rangeIndex),
               config.showRunways,
               static_cast<unsigned>(config.airportSelectionMode),
               static_cast<unsigned>(config.airportCount),
               static_cast<unsigned>(config.airportRadiusKm),
               config.manualAirportIcao.c_str(),
               config.miles,
               static_cast<unsigned>(config.mapProvider),
               static_cast<unsigned>(config.mapBrightness),
               static_cast<unsigned>(config.stadiaApiKey.length()),
               config.showLabelCallsign,
               config.showLabelType,
               config.showLabelAltitude,
               config.showLabelVerticalRate,
               static_cast<unsigned>(config.aircraftSymbolStyle));

    if (!config.configured) {
        setBootStageDetails(BOOT_WIFI, "NO SAVED WIFI CONFIGURATION", "SETUP PORTAL REQUIRED");
        setBootStage(BOOT_WIFI, BootStatus::Skip);
        setBootStage(BOOT_SERVICES, BootStatus::Running);
        setBootStageDetails(
            BOOT_SERVICES,
            "HTTP SERVER / PORT 80",
            "SETUP AP / BIGPLANERADAR-SETUP",
            "AP ADDRESS / 192.168.4.1"
        );
        logStep("startPortal begin");
        startPortal();
        setBootStage(BOOT_SERVICES, BootStatus::Ok);
        setUnavailableMapBootStatus();
        setBootStageDetails(BOOT_DATA, "ADSB REQUEST SKIPPED", "WIFI SETUP IS ACTIVE");
        setBootStage(BOOT_DATA, BootStatus::Skip);
        logStep("startPortal end");
    } else {
        setBootStage(BOOT_WIFI, BootStatus::Running);
        logStep("connectWifi begin");
        if (!connectWifiOnce(WIFI_CONNECT_ATTEMPT_MS)) {
            setBootStage(BOOT_WIFI, BootStatus::Fail);
            setStatus("WIFI RETRY");
            setBootStage(BOOT_SERVICES, BootStatus::Running);
            setBootStageDetails(
                BOOT_SERVICES,
                "HTTP SERVER / PORT 80",
                "FALLBACK AP / BIGPLANERADAR-SETUP",
                "AP ADDRESS / 192.168.4.1"
            );
            logStep("connect failed, startPortal begin");
            startPortal();
            setBootStage(BOOT_SERVICES, BootStatus::Ok);
            setUnavailableMapBootStatus();
            setBootStageDetails(BOOT_DATA, "ADSB REQUEST SKIPPED", "WIFI CONNECTION FAILED");
            setBootStage(BOOT_DATA, BootStatus::Skip);
            logStep("connect failed, startPortal end");
        } else {
            setBootStage(BOOT_WIFI, BootStatus::Ok);
            setBootStage(BOOT_SERVICES, BootStatus::Running);
            char serviceIpLine[88];
            snprintf(serviceIpLine, sizeof(serviceIpLine), "HTTP SERVER / %s:80", WiFi.localIP().toString().c_str());
            setBootStageDetails(
                BOOT_SERVICES,
                serviceIpLine,
                "MDNS / BIGPLANE-RADAR.LOCAL",
                "BACKGROUND NETWORK TASK / CORE 0"
            );
            setBootStage(BOOT_SERVICES, BootStatus::Ok);
            preloadMapCache();
            BuildDiagnostics::logMemory("map-cache-ready");
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
    char radarViewLine[48];
    char aircraftListLine[48];
    snprintf(
        radarViewLine,
        sizeof(radarViewLine),
        "RADAR VIEW / %dX%d",
        PANEL_X,
        SCREEN_H
    );
    snprintf(
        aircraftListLine,
        sizeof(aircraftListLine),
        "AIRCRAFT LIST / %dX%d / %u ROWS",
        SCREEN_W - PANEL_X,
        SCREEN_H,
        static_cast<unsigned>(panelVisibleRows)
    );
    setBootStageDetails(
        BOOT_INTERFACE,
        radarViewLine,
        aircraftListLine,
        "TOUCH / GT911 / FRAME LOOP UNLOCKED",
        "PREPARING FIRST FRAME"
    );
    setBootStage(BOOT_INTERFACE, BootStatus::Ok);

    bool setupMode = portalActive;
    if (!setupMode) {
        setupMode = waitForBootSetupHold(BOOT_SETUP_WINDOW_MS);
    }
#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_DEBUG
    benchmarkAircraftLabelLayout();
#endif
    bootScreenActive = false;
    AppWatchdog::begin();
    AppWatchdog::subscribeCurrentTask("loop");
    startNetworkTask();
    if (setupMode || portalActive) {
        logStep("setup portal active");
        drawStatusScreen("BIG PLANE RADAR SETUP", "Connect to Wi-Fi AP: BigPlaneRadar-Setup\nOpen http://192.168.4.1\nSet Wi-Fi and radar location.");
        return;
    }

    logStep("drawRadar begin");
    drawRadar();
    logStep("drawRadar end");
    AppWatchdog::feed();
}

void loop() {
    AppWatchdog::feed();
    server.handleClient();
    if (OtaUpdate::inProgress()) {
        delay(1);
        return;
    }
    handleTouch();

    uint32_t now = millis();
    if (shouldDrawRadarFrame(now)) {
        drawRadar();
    }
    delay(1);
}
