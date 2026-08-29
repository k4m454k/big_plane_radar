#include "weather_time.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/time.h>

#include "app_log.h"

namespace WeatherTime {
namespace {

constexpr char kMetarApi[] = "https://aviationweather.gov/api/data/metar";
constexpr char kTimezoneApi[] = "https://api.open-meteo.com/v1/forecast";
constexpr unsigned long kMetarIntervalMs = 10UL * 60UL * 1000UL;
constexpr unsigned long kTimezoneIntervalMs = 12UL * 60UL * 60UL * 1000UL;
constexpr unsigned long kRequestTimeoutMs = 8000UL;
constexpr time_t kMinimumValidEpoch = 1609459200;

bool s_started = false;
bool s_valid = false;
char s_stationIcao[8] = {};
char s_requestedIcao[8] = {};
char s_rawOb[160] = {};
char s_wind[16] = {};
char s_visib[12] = {};
char s_wx[24] = {};
char s_cover[8] = {};
char s_fltCat[8] = {};
float s_temperatureC = 0.0f;
bool s_hasTemperature = false;
float s_altimHpa = 0.0f;
bool s_hasAltim = false;
int32_t s_utcOffsetSeconds = 0;
unsigned long s_lastMetarAttemptMs = 0;
unsigned long s_lastTimezoneAttemptMs = 0;
double s_lastLatitude = 999.0;
double s_lastLongitude = 999.0;

bool clockValid() { return time(nullptr) >= kMinimumValidEpoch; }

void copyToken(char *out, size_t outLen, const char *value, size_t maxCopy) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    if (value == nullptr || value[0] == '\0' || maxCopy == 0) {
        return;
    }
    size_t written = 0;
    for (size_t i = 0; value[i] != '\0' && written + 1 < outLen && written < maxCopy;
         ++i) {
        const unsigned char ch = static_cast<unsigned char>(value[i]);
        if (ch < 32 || ch > 126) {
            continue;
        }
        out[written++] = static_cast<char>(ch);
    }
    out[written] = '\0';
}

void appendToken(char *out, size_t outLen, const char *token) {
    if (outLen == 0 || token == nullptr || token[0] == '\0') {
        return;
    }
    const size_t used = strlen(out);
    if (used + 1 >= outLen) {
        return;
    }
    if (used > 0) {
        out[used] = ' ';
        out[used + 1] = '\0';
    }
    strlcat(out, token, outLen);
}

void normalizeIcao(const char *icao, char *out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    if (icao == nullptr) {
        return;
    }
    size_t written = 0;
    for (size_t i = 0; icao[i] != '\0' && written + 1 < outLen && written < 4; ++i) {
        const unsigned char ch = static_cast<unsigned char>(icao[i]);
        if ((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')) {
            out[written++] = static_cast<char>(ch);
        } else if (ch >= 'a' && ch <= 'z') {
            out[written++] = static_cast<char>(ch - 'a' + 'A');
        }
    }
    out[written] = '\0';
}

bool fetchTimezoneOffset(double latitude, double longitude) {
    String url = kTimezoneApi;
    url += "?latitude=";
    url += String(latitude, 6);
    url += "&longitude=";
    url += String(longitude, 6);
    url += "&timezone=auto&forecast_days=1";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        RADAR_LOGE("timezone: http.begin failed\n");
        return false;
    }
    http.setUserAgent("BigPlaneRadar/1.0");
    http.setTimeout(kRequestTimeoutMs);
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        RADAR_LOGE("timezone: HTTP %d\n", code);
        http.end();
        return false;
    }

    const String payload = http.getString();
    http.end();

    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        RADAR_LOGE("timezone: JSON parse error: %s\n", error.c_str());
        return false;
    }
    if (!(doc["utc_offset_seconds"].is<int>() ||
          doc["utc_offset_seconds"].is<long>() ||
          doc["utc_offset_seconds"].is<float>())) {
        return false;
    }
    s_utcOffsetSeconds = doc["utc_offset_seconds"].as<int32_t>();
    RADAR_LOGI("timezone: utc offset %d s\n", static_cast<int>(s_utcOffsetSeconds));
    return true;
}

void formatWind(JsonObject ob, char *out, size_t outLen) {
    out[0] = '\0';
    int speed = -1;
    if (ob["wspd"].is<int>() || ob["wspd"].is<float>()) {
        speed = ob["wspd"].as<int>();
    }
    if (speed < 0) {
        return;
    }

    int gust = -1;
    if (ob["wgst"].is<int>() || ob["wgst"].is<float>()) {
        gust = ob["wgst"].as<int>();
    }

    bool variable = false;
    int direction = -1;
    if (ob["wdir"].is<const char *>()) {
        const char *dir = ob["wdir"].as<const char *>();
        if (dir != nullptr && (strcmp(dir, "VRB") == 0 || strcmp(dir, "vrb") == 0)) {
            variable = true;
        }
    } else if (ob["wdir"].is<int>() || ob["wdir"].is<float>()) {
        direction = ob["wdir"].as<int>();
    }

    if (speed == 0 && (direction <= 0 || variable)) {
        snprintf(out, outLen, "CALM");
        return;
    }
    if (variable || direction < 0) {
        if (gust > speed) {
            snprintf(out, outLen, "VRB%02dG%02dKT", speed, gust);
        } else {
            snprintf(out, outLen, "VRB%02dKT", speed);
        }
        return;
    }
    if (gust > speed) {
        snprintf(out, outLen, "%03d%02dG%02dKT", direction, speed, gust);
        return;
    }
    snprintf(out, outLen, "%03d%02dKT", direction, speed);
}

void formatVisibility(JsonObject ob, char *out, size_t outLen) {
    out[0] = '\0';
    if (ob["visib"].is<const char *>()) {
        copyToken(out, outLen, ob["visib"].as<const char *>(), 8);
    } else if (ob["visib"].is<int>() || ob["visib"].is<float>()) {
        snprintf(out, outLen, "%g", ob["visib"].as<float>());
    }
    if (out[0] == '\0') {
        return;
    }
    if (strstr(out, "SM") == nullptr && strchr(out, 'M') == nullptr) {
        strlcat(out, "SM", outLen);
    }
}

bool fetchMetar(const char *icao) {
    String url = kMetarApi;
    url += "?ids=";
    url += icao;
    url += "&format=json&hours=2&taf=false";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        RADAR_LOGE("metar: http.begin failed\n");
        return false;
    }
    http.setUserAgent("BigPlaneRadar/1.0");
    http.addHeader("Accept", "application/json");
    http.setTimeout(kRequestTimeoutMs);
    const int code = http.GET();
    if (code == HTTP_CODE_NO_CONTENT) {
        RADAR_LOGI("metar: no observation for %s\n", icao);
        http.end();
        return false;
    }
    if (code != HTTP_CODE_OK) {
        RADAR_LOGE("metar: HTTP %d\n", code);
        http.end();
        return false;
    }

    const String payload = http.getString();
    http.end();
    if (payload.length() == 0) {
        return false;
    }

    JsonDocument filter;
    filter[0]["icaoId"] = true;
    filter[0]["rawOb"] = true;
    filter[0]["temp"] = true;
    filter[0]["wdir"] = true;
    filter[0]["wspd"] = true;
    filter[0]["wgst"] = true;
    filter[0]["visib"] = true;
    filter[0]["wxString"] = true;
    filter[0]["cover"] = true;
    filter[0]["fltCat"] = true;
    filter[0]["altim"] = true;

    JsonDocument doc;
    const DeserializationError error =
        deserializeJson(doc, payload, DeserializationOption::Filter(filter));
    if (error) {
        RADAR_LOGE("metar: JSON parse error: %s\n", error.c_str());
        return false;
    }

    JsonObject ob = doc[0].as<JsonObject>();
    if (ob.isNull() || !ob["rawOb"].is<const char *>()) {
        RADAR_LOGE("metar: missing observation for %s\n", icao);
        return false;
    }

    copyToken(s_stationIcao, sizeof(s_stationIcao),
              ob["icaoId"].is<const char *>() ? ob["icaoId"].as<const char *>() : icao,
              4);
    copyToken(s_rawOb, sizeof(s_rawOb), ob["rawOb"].as<const char *>(),
              sizeof(s_rawOb) - 1);
    copyToken(s_wx, sizeof(s_wx),
              ob["wxString"].is<const char *>() ? ob["wxString"].as<const char *>()
                                                : "",
              sizeof(s_wx) - 1);
    copyToken(s_cover, sizeof(s_cover),
              ob["cover"].is<const char *>() ? ob["cover"].as<const char *>() : "",
              sizeof(s_cover) - 1);
    copyToken(s_fltCat, sizeof(s_fltCat),
              ob["fltCat"].is<const char *>() ? ob["fltCat"].as<const char *>() : "",
              sizeof(s_fltCat) - 1);
    formatWind(ob, s_wind, sizeof(s_wind));
    formatVisibility(ob, s_visib, sizeof(s_visib));

    s_hasTemperature = ob["temp"].is<int>() || ob["temp"].is<float>();
    s_temperatureC = s_hasTemperature ? ob["temp"].as<float>() : 0.0f;
    s_hasAltim = ob["altim"].is<int>() || ob["altim"].is<float>();
    s_altimHpa = s_hasAltim ? ob["altim"].as<float>() : 0.0f;
    s_valid = true;
    RADAR_LOGI("metar: %s\n", s_rawOb);
    return true;
}

}  // namespace

void begin() {
    if (!s_started) {
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        s_started = true;
    }
}

bool refreshIfDue(double latitude, double longitude, const char *icao, bool force) {
    begin();
    const unsigned long now = millis();
    const bool locationChanged =
        fabs(latitude - s_lastLatitude) > 0.0001 ||
        fabs(longitude - s_lastLongitude) > 0.0001;
    bool changed = false;

    if (force || locationChanged || s_lastTimezoneAttemptMs == 0 ||
        now - s_lastTimezoneAttemptMs >= kTimezoneIntervalMs) {
        s_lastTimezoneAttemptMs = now;
        s_lastLatitude = latitude;
        s_lastLongitude = longitude;
        if (fetchTimezoneOffset(latitude, longitude)) {
            changed = true;
        }
    }

    char normalized[8] = {};
    normalizeIcao(icao, normalized, sizeof(normalized));
    const bool icaoChanged = strcmp(normalized, s_requestedIcao) != 0;
    if (icaoChanged) {
        strlcpy(s_requestedIcao, normalized, sizeof(s_requestedIcao));
        s_valid = false;
        s_stationIcao[0] = '\0';
        s_rawOb[0] = '\0';
        changed = true;
    }
    if (normalized[0] == '\0') {
        return changed;
    }
    if (!force && !icaoChanged && s_lastMetarAttemptMs != 0 &&
        now - s_lastMetarAttemptMs < kMetarIntervalMs) {
        return changed;
    }
    s_lastMetarAttemptMs = now;
    if (fetchMetar(normalized)) {
        changed = true;
    }
    return changed;
}

bool valid() { return s_valid; }

void stationIcao(char *out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    const char *icao = s_stationIcao[0] ? s_stationIcao : s_requestedIcao;
    strlcpy(out, icao, outLen);
}

static bool tokenEquals(const char *token, size_t tokenLen, const char *literal) {
    const size_t literalLen = strlen(literal);
    return tokenLen == literalLen && strncmp(token, literal, tokenLen) == 0;
}

static bool isMetarTypeToken(const char *token, size_t tokenLen) {
    return tokenEquals(token, tokenLen, "METAR") ||
        tokenEquals(token, tokenLen, "SPECI") ||
        tokenEquals(token, tokenLen, "COR") ||
        tokenEquals(token, tokenLen, "AMD");
}

static bool isMetarIcaoToken(const char *token, size_t tokenLen) {
    if (tokenLen != 4) {
        return false;
    }
    if (s_stationIcao[0] != '\0' && tokenEquals(token, tokenLen, s_stationIcao)) {
        return true;
    }
    if (s_requestedIcao[0] != '\0' && tokenEquals(token, tokenLen, s_requestedIcao)) {
        return true;
    }
    return false;
}

static bool isMetarTimeToken(const char *token, size_t tokenLen) {
    if (tokenLen != 7 || (token[6] != 'Z' && token[6] != 'z')) {
        return false;
    }
    for (size_t i = 0; i < 6; i++) {
        if (token[i] < '0' || token[i] > '9') {
            return false;
        }
    }
    return true;
}

// Skip METAR/SPECI, station ICAO, and the observation time (ddHHMMZ).
static const char *metarBodyStart(char *obsTime, size_t obsTimeLen) {
    if (obsTime != nullptr && obsTimeLen > 0) {
        obsTime[0] = '\0';
    }
    const char *p = s_rawOb;
    bool skippedIcao = false;
    bool skippedTime = false;
    while (p[0] != '\0') {
        while (p[0] == ' ') {
            ++p;
        }
        if (p[0] == '\0') {
            break;
        }
        const char *token = p;
        while (p[0] != '\0' && p[0] != ' ') {
            ++p;
        }
        const size_t tokenLen = static_cast<size_t>(p - token);
        if (!skippedTime &&
            (isMetarTypeToken(token, tokenLen) ||
             tokenEquals(token, tokenLen, "AUTO"))) {
            continue;
        }
        if (!skippedIcao && !skippedTime && isMetarIcaoToken(token, tokenLen)) {
            skippedIcao = true;
            continue;
        }
        if (!skippedTime && isMetarTimeToken(token, tokenLen)) {
            skippedTime = true;
            if (obsTime != nullptr && obsTimeLen > 0) {
                const size_t copyLen = tokenLen < obsTimeLen - 1 ? tokenLen : obsTimeLen - 1;
                memcpy(obsTime, token, copyLen);
                obsTime[copyLen] = '\0';
                if (copyLen > 0 && obsTime[copyLen - 1] == 'z') {
                    obsTime[copyLen - 1] = 'Z';
                }
            }
            continue;
        }
        return token;
    }
    return "";
}

void formatMetarBody(char *out, size_t outLen, bool fahrenheit) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    if (!s_valid) {
        snprintf(out, outLen, "--");
        return;
    }

    if (s_hasTemperature && fahrenheit) {
        char temp[12];
        const float temperature = s_temperatureC * 9.0f / 5.0f + 32.0f;
        snprintf(temp, sizeof(temp), "%dF", static_cast<int>(lroundf(temperature)));
        appendToken(out, outLen, temp);
    }

    char unusedTime[12];
    const char *raw = metarBodyStart(unusedTime, sizeof(unusedTime));
    if (raw[0] == '\0') {
        appendToken(out, outLen, s_wind);
        appendToken(out, outLen, s_visib);
        appendToken(out, outLen, s_wx);
        appendToken(out, outLen, s_cover);
        appendToken(out, outLen, s_fltCat);
        return;
    }
    appendToken(out, outLen, raw);
}

bool formatMetarObsTime(char *out, size_t outLen) {
    if (outLen == 0) {
        return false;
    }
    out[0] = '\0';
    if (!s_valid) {
        return false;
    }
    metarBodyStart(out, outLen);
    return out[0] != '\0';
}

static void formatLocalClock(const tm &local, char *out, size_t outLen, bool clock24) {
    if (clock24) {
        snprintf(out, outLen, "%02d:%02d", local.tm_hour, local.tm_min);
        return;
    }
    int hour = local.tm_hour % 12;
    if (hour == 0) {
        hour = 12;
    }
    snprintf(out, outLen, "%d:%02d%c", hour, local.tm_min,
             local.tm_hour >= 12 ? 'P' : 'A');
}

static bool localTimeNow(tm *local) {
    const time_t utcNow = time(nullptr);
    if (utcNow < kMinimumValidEpoch || local == nullptr) {
        return false;
    }
    const time_t localNow = utcNow + s_utcOffsetSeconds;
    gmtime_r(&localNow, local);
    return true;
}

void formatTime(char *out, size_t outLen, bool clock24) {
    if (outLen == 0) {
        return;
    }
    tm local = {};
    if (!localTimeNow(&local)) {
        snprintf(out, outLen, "--:--");
        return;
    }
    formatLocalClock(local, out, outLen, clock24);
}

void formatDate(char *out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    tm local = {};
    if (!localTimeNow(&local)) {
        snprintf(out, outLen, "-- ---");
        return;
    }
    constexpr const char *kMonths[] = {"JAN", "FEB", "MAR", "APR",
                                       "MAY", "JUN", "JUL", "AUG",
                                       "SEP", "OCT", "NOV", "DEC"};
    const char *month =
        local.tm_mon >= 0 && local.tm_mon < 12 ? kMonths[local.tm_mon] : "---";
    snprintf(out, outLen, "%02d %s", local.tm_mday, month);
}

}  // namespace WeatherTime
