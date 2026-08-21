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
        // When non-empty, tiles are fetched from a pi-feed proxy over plain
        // HTTP instead of Stadia over TLS. The proxy holds the API key and
        // caches tiles, so the device needs neither a key nor a TLS stack.
        const String &feedHost,
        uint8_t brightnessPercent,
        size_t viewIndex,
        LoadProgressCallback progressCallback = nullptr,
        void *progressContext = nullptr
    );
    bool draw(PanelDisplay::Canvas &canvas, size_t viewIndex);
    bool isReady(size_t viewIndex);
    // Whether a buffer is currently assigned to this view. False means the view
    // has to be fetched before it can be drawn, which is the caller's cue.
    bool hasSlot(size_t viewIndex);
    void clear();

private:
    // Buffers are assigned to range presets on demand rather than one each.
    // A 1024x600 panel only has room for three at 816 KB apiece, so fixing a
    // buffer per range index meant the outermost ranges could never show a map
    // however long you sat on them -- while an inner range nobody was looking
    // at held one. Slots are claimed by whichever range is actually being
    // viewed, evicting the least recently used.
    static constexpr size_t MAX_VIEWS = 8;
    static constexpr size_t kNoView = static_cast<size_t>(-1);
    int slotFor(size_t viewIndex) const;
    int claimSlot(size_t viewIndex);
    uint16_t *_buffers[MAX_VIEWS] = {};
    bool _ready[MAX_VIEWS] = {};
    size_t _slotView[MAX_VIEWS] = {};
    uint32_t _slotUse[MAX_VIEWS] = {};
    uint32_t _useCounter = 0;
    // Allocated buffers, which may be fewer than the range presets.
    size_t _slotCount = 0;
    // Range presets, i.e. the range of valid viewIndex values.
    size_t _viewCount = 0;
    int _width = 0;
    int _height = 0;
    StaticSemaphore_t _mutexStorage = {};
    SemaphoreHandle_t _mutex = nullptr;
};

extern Background background;

} // namespace RadarMap
