#pragma once

#include <Arduino.h>

#include "panel_display.h"

namespace RadarMap {

class Background {
public:
    bool begin(int width, int height, size_t viewCount);
    bool fetchStadia(
        double centerLat,
        double centerLon,
        float outerKm,
        int radarRadius,
        const String &apiKey,
        size_t viewIndex
    );
    bool draw(PanelDisplay::Canvas &canvas, size_t viewIndex);
    bool isReady(size_t viewIndex);
    void clear();

private:
    static constexpr size_t MAX_VIEWS = 4;
    uint16_t *_buffers[MAX_VIEWS] = {};
    bool _ready[MAX_VIEWS] = {};
    size_t _viewCount = 0;
    int _width = 0;
    int _height = 0;
    StaticSemaphore_t _mutexStorage = {};
    SemaphoreHandle_t _mutex = nullptr;
};

extern Background background;

} // namespace RadarMap
