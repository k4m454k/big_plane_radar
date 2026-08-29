#pragma once

#include <cstddef>

namespace WeatherTime {

void begin();
bool refreshIfDue(double latitude, double longitude, const char *icao,
                  bool force = false);
bool valid();
void stationIcao(char *out, size_t outLen);
void formatMetarBody(char *out, size_t outLen, bool fahrenheit);
bool formatMetarObsTime(char *out, size_t outLen);
void formatTime(char *out, size_t outLen, bool clock24);
void formatDate(char *out, size_t outLen);

}  // namespace WeatherTime
