#pragma once

#include <Arduino.h>

#include "panel_display.h"

namespace RadarMap {

enum class LoadPhase : uint8_t {
    Request,
    Response,
    Download,
    Decode,
    Ready,
    Error,
};

struct LoadProgress {
    LoadPhase phase = LoadPhase::Request;
    size_t viewIndex = 0;
    int zoom = 0;
    int sourceWidth = 0;
    int sourceHeight = 0;
    int destinationWidth = 0;
    int destinationHeight = 0;
    size_t tileIndex = 0;
    size_t tileCount = 0;
    int tileColumns = 0;
    int tileRows = 0;
    int tileX = 0;
    int tileY = 0;
    int httpStatus = 0;
    size_t receivedBytes = 0;
    size_t totalBytes = 0;
    size_t viewReceivedBytes = 0;
    uint32_t decodeMs = 0;
    const char *error = nullptr;
};

using LoadProgressCallback = void (*)(const LoadProgress &progress, void *context);

class Background {
public:
    bool begin(int width, int height, size_t viewCount);
    bool fetchStadia(
        double centerLat,
        double centerLon,
        float outerKm,
        int radarRadius,
        const String &apiKey,
        uint8_t brightnessPercent,
        size_t viewIndex,
        LoadProgressCallback progressCallback = nullptr,
        void *progressContext = nullptr
    );
    bool draw(PanelDisplay::Canvas &canvas, size_t viewIndex);
    bool isReady(size_t viewIndex);
    void clear();

private:
    static constexpr size_t MAX_VIEWS = 5;
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
