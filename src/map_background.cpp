#include "map_background.h"
#include "app_log.h"

#include <HTTPClient.h>
#include <PNGdec.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <math.h>
#include <memory>
#include <new>

namespace RadarMap {

static constexpr char STADIA_RASTER_TILE_URL[] =
    "https://tiles-eu.stadiamaps.com/tiles/alidade_smooth_dark/%d/%d/%d.png";
static constexpr char PROXY_RASTER_TILE_URL[] = "http://%s/tiles/%d/%d/%d.png";
static constexpr uint32_t MAP_HTTP_TIMEOUT_MS = 20000;
static constexpr size_t MAP_MAX_PNG_BYTES = 256 * 1024;
static constexpr int MAP_TILE_SIZE = 256;
static constexpr int MAP_MAX_TILE_COLUMNS = 6;
static constexpr int MAP_MAX_TILE_ROWS = 6;
static constexpr int MAP_MAX_SOURCE_WIDTH = MAP_MAX_TILE_COLUMNS * MAP_TILE_SIZE;
static constexpr int MAP_MAX_SOURCE_HEIGHT = MAP_MAX_TILE_ROWS * MAP_TILE_SIZE;
static constexpr unsigned MAP_DOWNLOAD_PROGRESS_STEPS = 5;
static constexpr double WEB_MERCATOR_MAX_LATITUDE = 85.05112878;
static constexpr double WEB_MERCATOR_METERS_PER_PIXEL_Z0 = 156543.03392804097;

Background background;

struct MapGeometry {
    int zoom = 0;
    double centerPixelX = 0;
    double centerPixelY = 0;
    double sourcePixelsPerDestinationPixel = 1;
    double worldPixelSize = 0;
    int64_t tileMinX = 0;
    int64_t tileMaxX = 0;
    int tileMinY = 0;
    int tileMaxY = 0;
    int tileColumns = 0;
    int tileRows = 0;
    int sourceWidth = 0;
    int sourceHeight = 0;
};

struct PixelSample {
    int64_t first = 0;
    int64_t second = 0;
    uint16_t weight = 0;
};

struct BilinearSample {
    uint16_t first = 0;
    uint16_t second = 0;
    uint16_t weight = 0;
};

struct TileDecodeContext {
    PNG *decoder = nullptr;
    uint16_t *strip = nullptr;
    uint16_t *line = nullptr;
    int stripWidth = 0;
    int destinationX = 0;
};

static int64_t floorDiv(int64_t value, int64_t divisor) {
    if (value >= 0) return value / divisor;
    return -((-value + divisor - 1) / divisor);
}

static int wrapTileX(int64_t tileX, int zoom) {
    int64_t tileCount = 1LL << zoom;
    int64_t wrapped = tileX % tileCount;
    if (wrapped < 0) wrapped += tileCount;
    return static_cast<int>(wrapped);
}

static PixelSample pixelSample(double coordinate) {
    double firstValue = floor(coordinate);
    double fraction = coordinate - firstValue;
    int weight = static_cast<int>(lround(fraction * 256.0));
    int64_t first = static_cast<int64_t>(firstValue);
    if (weight >= 256) {
        first++;
        weight = 0;
    }

    PixelSample result;
    result.first = first;
    result.second = weight == 0 ? first : first + 1;
    result.weight = static_cast<uint16_t>(weight);
    return result;
}

static double sourceCoordinate(
    double centerPixel,
    int destinationIndex,
    int destinationSize,
    double scale
) {
    return centerPixel +
        (destinationIndex + 0.5 - destinationSize / 2.0) * scale - 0.5;
}

static double clampSourceY(double coordinate, double worldPixelSize) {
    return std::max(0.0, std::min(worldPixelSize - 1.0, coordinate));
}

static MapGeometry mapGeometry(
    double centerLat,
    double centerLon,
    float outerKm,
    int radarRadius,
    int destinationWidth,
    int destinationHeight
) {
    MapGeometry result;
    double latitude = std::max(
        -WEB_MERCATOR_MAX_LATITUDE,
        std::min(WEB_MERCATOR_MAX_LATITUDE, centerLat)
    );
    double longitude = fmod(centerLon + 180.0, 360.0);
    if (longitude < 0) longitude += 360.0;
    longitude -= 180.0;

    double metersPerDestinationPixel = (outerKm * 1000.0) / radarRadius;
    double latitudeScale = std::max(0.01, cos(latitude * DEG_TO_RAD));
    double rawZoom = log2(
        (WEB_MERCATOR_METERS_PER_PIXEL_Z0 * latitudeScale) /
        metersPerDestinationPixel
    );
    result.zoom = std::max(0, std::min(20, static_cast<int>(ceil(rawZoom))));
    result.worldPixelSize = ldexp(static_cast<double>(MAP_TILE_SIZE), result.zoom);

    double latitudeRadians = latitude * DEG_TO_RAD;
    double sinLatitude = sin(latitudeRadians);
    result.centerPixelX = ((longitude + 180.0) / 360.0) * result.worldPixelSize;
    result.centerPixelY =
        (0.5 - log((1.0 + sinLatitude) / (1.0 - sinLatitude)) / (4.0 * PI)) *
        result.worldPixelSize;

    double metersPerSourcePixel =
        (WEB_MERCATOR_METERS_PER_PIXEL_Z0 * latitudeScale) /
        static_cast<double>(1UL << result.zoom);
    result.sourcePixelsPerDestinationPixel =
        metersPerDestinationPixel / metersPerSourcePixel;
    result.sourceWidth = static_cast<int>(ceil(
        destinationWidth * result.sourcePixelsPerDestinationPixel
    ));
    result.sourceHeight = static_cast<int>(ceil(
        destinationHeight * result.sourcePixelsPerDestinationPixel
    ));

    PixelSample firstX = pixelSample(sourceCoordinate(
        result.centerPixelX,
        0,
        destinationWidth,
        result.sourcePixelsPerDestinationPixel
    ));
    PixelSample lastX = pixelSample(sourceCoordinate(
        result.centerPixelX,
        destinationWidth - 1,
        destinationWidth,
        result.sourcePixelsPerDestinationPixel
    ));
    PixelSample firstY = pixelSample(clampSourceY(sourceCoordinate(
        result.centerPixelY,
        0,
        destinationHeight,
        result.sourcePixelsPerDestinationPixel
    ), result.worldPixelSize));
    PixelSample lastY = pixelSample(clampSourceY(sourceCoordinate(
        result.centerPixelY,
        destinationHeight - 1,
        destinationHeight,
        result.sourcePixelsPerDestinationPixel
    ), result.worldPixelSize));

    result.tileMinX = floorDiv(firstX.first, MAP_TILE_SIZE);
    result.tileMaxX = floorDiv(lastX.second, MAP_TILE_SIZE);
    result.tileMinY = static_cast<int>(floorDiv(firstY.first, MAP_TILE_SIZE));
    result.tileMaxY = static_cast<int>(floorDiv(lastY.second, MAP_TILE_SIZE));
    int worldTileCount = 1 << result.zoom;
    result.tileMinY = std::max(0, std::min(worldTileCount - 1, result.tileMinY));
    result.tileMaxY = std::max(0, std::min(worldTileCount - 1, result.tileMaxY));
    result.tileColumns = static_cast<int>(result.tileMaxX - result.tileMinX + 1);
    result.tileRows = result.tileMaxY - result.tileMinY + 1;
    return result;
}

static uint16_t interpolateRgb565(uint16_t first, uint16_t second, uint16_t weight) {
    if (weight == 0 || first == second) return first;

    uint32_t inverseWeight = 256 - weight;
    uint32_t red =
        (((first >> 11) & 0x1f) * inverseWeight +
         ((second >> 11) & 0x1f) * weight + 128) >> 8;
    uint32_t green =
        (((first >> 5) & 0x3f) * inverseWeight +
         ((second >> 5) & 0x3f) * weight + 128) >> 8;
    uint32_t blue =
        ((first & 0x1f) * inverseWeight +
         (second & 0x1f) * weight + 128) >> 8;
    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
}

static uint16_t adjustBrightnessRgb565(uint16_t pixel, uint8_t brightnessPercent) {
    if (brightnessPercent >= 100) return pixel;

    uint32_t red = (((pixel >> 11) & 0x1f) * brightnessPercent + 50) / 100;
    uint32_t green = (((pixel >> 5) & 0x3f) * brightnessPercent + 50) / 100;
    uint32_t blue = ((pixel & 0x1f) * brightnessPercent + 50) / 100;
    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
}

static void emitProgress(
    LoadProgress &progress,
    LoadPhase phase,
    LoadProgressCallback callback,
    void *context,
    const char *error = nullptr
) {
    progress.phase = phase;
    progress.error = error;
    if (callback != nullptr) {
        callback(progress, context);
    }
}

static bool readHttpBody(
    HTTPClient &http,
    uint8_t *&data,
    size_t &size,
    LoadProgress &progress,
    LoadProgressCallback callback,
    void *context
) {
    int contentLength = http.getSize();
    if (contentLength <= 0 || static_cast<size_t>(contentLength) > MAP_MAX_PNG_BYTES) {
        RADAR_LOGE("[map] invalid tile content length=%d\n", contentLength);
        emitProgress(progress, LoadPhase::Error, callback, context, "INVALID CONTENT LENGTH");
        return false;
    }

    size_t completedBytes = progress.viewReceivedBytes;
    progress.totalBytes = static_cast<size_t>(contentLength);
    progress.receivedBytes = 0;
    emitProgress(progress, LoadPhase::Download, callback, context);

    data = static_cast<uint8_t *>(heap_caps_malloc(
        static_cast<size_t>(contentLength),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    ));
    if (data == nullptr) {
        RADAR_LOGE("[map] tile PNG allocation failed bytes=%d\n", contentLength);
        emitProgress(progress, LoadPhase::Error, callback, context, "PNG ALLOCATION FAILED");
        return false;
    }

    auto *stream = http.getStreamPtr();
    size_t received = 0;
    uint32_t lastProgressMs = millis();
    unsigned lastProgressStep = 0;
    while (received < static_cast<size_t>(contentLength)) {
        int available = stream->available();
        if (available > 0) {
            size_t requested = std::min(
                static_cast<size_t>(available),
                static_cast<size_t>(contentLength) - received
            );
            int count = stream->read(data + received, requested);
            if (count > 0) {
                received += static_cast<size_t>(count);
                lastProgressMs = millis();
                unsigned progressStep = static_cast<unsigned>(
                    (received * MAP_DOWNLOAD_PROGRESS_STEPS) /
                    static_cast<size_t>(contentLength)
                );
                if (progressStep > lastProgressStep) {
                    lastProgressStep = progressStep;
                    progress.receivedBytes = received;
                    progress.viewReceivedBytes = completedBytes + received;
                    emitProgress(progress, LoadPhase::Download, callback, context);
                }
                continue;
            }
        }
        if ((!http.connected() && stream->available() == 0) ||
            millis() - lastProgressMs >= MAP_HTTP_TIMEOUT_MS) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (received != static_cast<size_t>(contentLength)) {
        RADAR_LOGE("[map] short tile response bytes=%u/%d\n",
                   static_cast<unsigned>(received), contentLength);
        progress.receivedBytes = received;
        progress.viewReceivedBytes = completedBytes + received;
        emitProgress(progress, LoadPhase::Error, callback, context, "INCOMPLETE DOWNLOAD");
        heap_caps_free(data);
        data = nullptr;
        return false;
    }

    size = received;
    progress.receivedBytes = received;
    progress.viewReceivedBytes = completedBytes + received;
    emitProgress(progress, LoadPhase::Download, callback, context);
    return true;
}

static int drawTilePngLine(PNGDRAW *draw) {
    auto *context = static_cast<TileDecodeContext *>(draw->pUser);
    if (context == nullptr || draw->iWidth != MAP_TILE_SIZE ||
        draw->y < 0 || draw->y >= MAP_TILE_SIZE) {
        return 0;
    }

    context->decoder->getLineAsRGB565(
        draw,
        context->line,
        PNG_RGB565_LITTLE_ENDIAN,
        0xffffffff
    );
    memcpy(
        context->strip +
            static_cast<size_t>(draw->y + 1) * context->stripWidth +
            context->destinationX,
        context->line,
        MAP_TILE_SIZE * sizeof(uint16_t)
    );
    return 1;
}

static bool decodeTilePng(
    uint8_t *pngData,
    size_t pngSize,
    uint16_t *strip,
    int stripWidth,
    int destinationX
) {
    void *decoderStorage = heap_caps_calloc(
        1,
        sizeof(PNG),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    auto *line = static_cast<uint16_t *>(heap_caps_malloc(
        MAP_TILE_SIZE * sizeof(uint16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    if (decoderStorage == nullptr || line == nullptr) {
        RADAR_LOGE("[map] tile decoder allocation failed\n");
        if (line != nullptr) heap_caps_free(line);
        if (decoderStorage != nullptr) heap_caps_free(decoderStorage);
        return false;
    }

    auto *decoder = new (decoderStorage) PNG();
    TileDecodeContext context;
    context.decoder = decoder;
    context.strip = strip;
    context.line = line;
    context.stripWidth = stripWidth;
    context.destinationX = destinationX;

    int result = decoder->openRAM(pngData, static_cast<int>(pngSize), drawTilePngLine);
    if (result == PNG_SUCCESS &&
        decoder->getWidth() == MAP_TILE_SIZE &&
        decoder->getHeight() == MAP_TILE_SIZE &&
        !decoder->isInterlaced()) {
        result = decoder->decode(&context, PNG_FAST_PALETTE);
    } else if (result == PNG_SUCCESS) {
        RADAR_LOGE("[map] unexpected tile PNG %dx%d interlaced=%d\n",
                   decoder->getWidth(), decoder->getHeight(), decoder->isInterlaced());
        result = PNG_INVALID_FILE;
    }
    decoder->close();
    decoder->~PNG();
    heap_caps_free(line);
    heap_caps_free(decoderStorage);

    if (result != PNG_SUCCESS) {
        RADAR_LOGE("[map] tile PNG decode failed code=%d\n", result);
        return false;
    }
    return true;
}

static bool downloadTile(
    WiFiClient &client,
    HTTPClient &http,
    const MapGeometry &geometry,
    int64_t unwrappedTileX,
    int tileY,
    const String &apiKey,
    const String &feedHost,
    uint16_t *strip,
    int stripWidth,
    int destinationX,
    LoadProgress &progress,
    LoadProgressCallback callback,
    void *callbackContext
) {
    int tileX = wrapTileX(unwrappedTileX, geometry.zoom);
    progress.tileX = tileX;
    progress.tileY = tileY;
    progress.httpStatus = 0;
    progress.receivedBytes = 0;
    progress.totalBytes = 0;
    emitProgress(progress, LoadPhase::Request, callback, callbackContext);

    char url[192];
    const bool viaProxy = feedHost.length() > 0;
    if (viaProxy) {
        snprintf(url, sizeof(url), PROXY_RASTER_TILE_URL,
                 feedHost.c_str(), geometry.zoom, tileX, tileY);
    } else {
        snprintf(url, sizeof(url), STADIA_RASTER_TILE_URL,
                 geometry.zoom, tileX, tileY);
    }
    if (!http.begin(client, url)) {
        RADAR_LOGE("[map] tile HTTP begin failed z=%d x=%d y=%d\n",
                   geometry.zoom, tileX, tileY);
        emitProgress(progress, LoadPhase::Error, callback, callbackContext, "HTTP BEGIN FAILED");
        return false;
    }
    if (!viaProxy) {
        String authorization = F("Stadia-Auth ");
        authorization += apiKey;
        http.addHeader(F("Authorization"), authorization);
    }
    int status = http.GET();
    progress.httpStatus = status;
    emitProgress(progress, LoadPhase::Response, callback, callbackContext);
    if (status != HTTP_CODE_OK) {
        RADAR_LOGE("[map] tile HTTP status=%d z=%d x=%d y=%d\n",
                   status, geometry.zoom, tileX, tileY);
        emitProgress(progress, LoadPhase::Error, callback, callbackContext, "HTTP REQUEST FAILED");
        http.end();
        return false;
    }

    uint8_t *pngData = nullptr;
    size_t pngSize = 0;
    bool downloaded = readHttpBody(
        http,
        pngData,
        pngSize,
        progress,
        callback,
        callbackContext
    );
    http.end();
    if (!downloaded) return false;

    uint32_t startedMs = millis();
    emitProgress(progress, LoadPhase::Decode, callback, callbackContext);
    bool decoded = decodeTilePng(
        pngData,
        pngSize,
        strip,
        stripWidth,
        destinationX
    );
    heap_caps_free(pngData);
    progress.decodeMs += millis() - startedMs;
    if (!decoded) {
        emitProgress(progress, LoadPhase::Error, callback, callbackContext, "PNG DECODE FAILED");
        return false;
    }
    return true;
}

static bool renderAvailableRows(
    const MapGeometry &geometry,
    const BilinearSample *sourceX,
    const uint16_t *strip,
    int stripWidth,
    int64_t rowBase,
    uint16_t *destination,
    int destinationWidth,
    int destinationHeight,
    uint8_t brightnessPercent,
    int &nextDestinationY
) {
    int64_t availableFirstY = rowBase - 1;
    int64_t availableLastY = rowBase + MAP_TILE_SIZE - 1;
    while (nextDestinationY < destinationHeight) {
        double coordinate = clampSourceY(sourceCoordinate(
            geometry.centerPixelY,
            nextDestinationY,
            destinationHeight,
            geometry.sourcePixelsPerDestinationPixel
        ), geometry.worldPixelSize);
        PixelSample sourceY = pixelSample(coordinate);
        if (sourceY.second > availableLastY) break;
        if (sourceY.first < availableFirstY) {
            RADAR_LOGE("[map] source row unavailable y=%d source=%lld..%lld available=%lld..%lld\n",
                       nextDestinationY,
                       static_cast<long long>(sourceY.first),
                       static_cast<long long>(sourceY.second),
                       static_cast<long long>(availableFirstY),
                       static_cast<long long>(availableLastY));
            return false;
        }

        size_t firstRow = static_cast<size_t>(sourceY.first - availableFirstY) * stripWidth;
        size_t secondRow = static_cast<size_t>(sourceY.second - availableFirstY) * stripWidth;
        uint16_t *destinationLine = destination +
            static_cast<size_t>(nextDestinationY) * destinationWidth;
        for (int x = 0; x < destinationWidth; x++) {
            const BilinearSample &sampleX = sourceX[x];
            uint16_t firstRowPixel = interpolateRgb565(
                strip[firstRow + sampleX.first],
                strip[firstRow + sampleX.second],
                sampleX.weight
            );
            uint16_t pixel = firstRowPixel;
            if (sourceY.weight != 0) {
                uint16_t secondRowPixel = interpolateRgb565(
                    strip[secondRow + sampleX.first],
                    strip[secondRow + sampleX.second],
                    sampleX.weight
                );
                pixel = interpolateRgb565(
                    firstRowPixel,
                    secondRowPixel,
                    sourceY.weight
                );
            }
            destinationLine[x] = adjustBrightnessRgb565(pixel, brightnessPercent);
        }
        nextDestinationY++;
    }
    return true;
}

bool Background::begin(int width, int height, size_t viewCount) {
    if (_mutex == nullptr) {
        _mutex = xSemaphoreCreateMutexStatic(&_mutexStorage);
    }
    if (_mutex == nullptr || width <= 0 || height <= 0 || viewCount == 0) {
        return false;
    }
    if (viewCount > MAX_VIEWS) {
        // Silently returning here disables the map with no clue why; adding a
        // range preset without raising MAX_VIEWS looks exactly like a missing
        // API key.
        RADAR_LOGE("[map] viewCount=%u exceeds MAX_VIEWS=%u; map disabled\n",
                   static_cast<unsigned>(viewCount),
                   static_cast<unsigned>(MAX_VIEWS));
        return false;
    }
    if (_viewCount > 0) {
        return width == _width && height == _height && viewCount == _viewCount;
    }

    size_t bytes = static_cast<size_t>(width) * height * sizeof(uint16_t);
    for (size_t i = 0; i < viewCount; i++) {
        _buffers[i] = static_cast<uint16_t *>(heap_caps_malloc(
            bytes,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        ));
        if (_buffers[i] == nullptr) {
            for (size_t allocated = 0; allocated < i; allocated++) {
                heap_caps_free(_buffers[allocated]);
                _buffers[allocated] = nullptr;
            }
            RADAR_LOGE("[map] framebuffer allocation failed views=%u bytes=%u\n",
                       static_cast<unsigned>(viewCount),
                       static_cast<unsigned>(bytes * viewCount));
            return false;
        }
    }
    _width = width;
    _height = height;
    _viewCount = viewCount;
    memset(_ready, 0, sizeof(_ready));
    RADAR_LOGI("[map] cache ready size=%dx%d views=%u bytes=%u free_psram=%u\n",
               width,
               height,
               static_cast<unsigned>(viewCount),
               static_cast<unsigned>(bytes * viewCount),
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    return true;
}

bool Background::fetchStadia(
    double centerLat,
    double centerLon,
    float outerKm,
    int radarRadius,
    const String &apiKey,
    const String &feedHost,
    uint8_t brightnessPercent,
    size_t viewIndex,
    LoadProgressCallback progressCallback,
    void *progressContext
) {
    // The proxy supplies the key, so only a direct fetch needs one on-device.
    if (viewIndex >= _viewCount || _buffers[viewIndex] == nullptr ||
        (feedHost.isEmpty() && apiKey.isEmpty())) {
        return false;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _ready[viewIndex] = false;
    xSemaphoreGive(_mutex);

    MapGeometry geometry = mapGeometry(
        centerLat,
        centerLon,
        outerKm,
        radarRadius,
        _width,
        _height
    );
    LoadProgress progress;
    progress.viewIndex = viewIndex;
    progress.zoom = geometry.zoom;
    progress.sourceWidth = geometry.sourceWidth;
    progress.sourceHeight = geometry.sourceHeight;
    progress.destinationWidth = _width;
    progress.destinationHeight = _height;
    progress.tileCount = static_cast<size_t>(geometry.tileColumns * geometry.tileRows);
    progress.tileColumns = geometry.tileColumns;
    progress.tileRows = geometry.tileRows;

    if (geometry.sourceWidth > MAP_MAX_SOURCE_WIDTH ||
        geometry.sourceHeight > MAP_MAX_SOURCE_HEIGHT ||
        geometry.tileColumns <= 0 || geometry.tileRows <= 0 ||
        geometry.tileColumns > MAP_MAX_TILE_COLUMNS ||
        geometry.tileRows > MAP_MAX_TILE_ROWS) {
        RADAR_LOGE("[map] invalid XYZ geometry source=%dx%d tiles=%dx%d\n",
                   geometry.sourceWidth,
                   geometry.sourceHeight,
                   geometry.tileColumns,
                   geometry.tileRows);
        emitProgress(
            progress,
            LoadPhase::Error,
            progressCallback,
            progressContext,
            "XYZ GEOMETRY TOO LARGE"
        );
        return false;
    }

    int stripWidth = geometry.tileColumns * MAP_TILE_SIZE;
    size_t stripPixels = static_cast<size_t>(stripWidth) * (MAP_TILE_SIZE + 1);
    auto *strip = static_cast<uint16_t *>(heap_caps_malloc(
        stripPixels * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    ));
    auto *previousLine = static_cast<uint16_t *>(heap_caps_malloc(
        static_cast<size_t>(stripWidth) * sizeof(uint16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    auto *sourceX = static_cast<BilinearSample *>(heap_caps_malloc(
        static_cast<size_t>(_width) * sizeof(BilinearSample),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    if (strip == nullptr || previousLine == nullptr || sourceX == nullptr) {
        RADAR_LOGE("[map] XYZ working buffer allocation failed strip=%u\n",
                   static_cast<unsigned>(stripPixels * sizeof(uint16_t)));
        if (sourceX != nullptr) heap_caps_free(sourceX);
        if (previousLine != nullptr) heap_caps_free(previousLine);
        if (strip != nullptr) heap_caps_free(strip);
        emitProgress(
            progress,
            LoadPhase::Error,
            progressCallback,
            progressContext,
            "XYZ BUFFER ALLOCATION FAILED"
        );
        return false;
    }

    int64_t stripOriginX = geometry.tileMinX * MAP_TILE_SIZE;
    bool samplesValid = true;
    for (int x = 0; x < _width; x++) {
        PixelSample worldX = pixelSample(sourceCoordinate(
            geometry.centerPixelX,
            x,
            _width,
            geometry.sourcePixelsPerDestinationPixel
        ));
        int64_t first = worldX.first - stripOriginX;
        int64_t second = worldX.second - stripOriginX;
        if (first < 0 || second >= stripWidth) {
            samplesValid = false;
            break;
        }
        sourceX[x].first = static_cast<uint16_t>(first);
        sourceX[x].second = static_cast<uint16_t>(second);
        sourceX[x].weight = worldX.weight;
    }
    if (!samplesValid) {
        RADAR_LOGE("[map] horizontal XYZ samples exceed strip\n");
        heap_caps_free(sourceX);
        heap_caps_free(previousLine);
        heap_caps_free(strip);
        emitProgress(
            progress,
            LoadPhase::Error,
            progressCallback,
            progressContext,
            "XYZ SAMPLE RANGE ERROR"
        );
        return false;
    }

    RADAR_LOGI(
        "[map] XYZ view=%u zoom=%d source=%dx%d grid=%dx%d working=%u\n",
        static_cast<unsigned>(viewIndex),
        geometry.zoom,
        geometry.sourceWidth,
        geometry.sourceHeight,
        geometry.tileColumns,
        geometry.tileRows,
        static_cast<unsigned>(stripPixels * sizeof(uint16_t))
    );

    // Only build a TLS client when actually talking to Stadia.
    std::unique_ptr<WiFiClient> clientHolder;
    if (feedHost.length() > 0) {
        clientHolder.reset(new WiFiClient());
    } else {
        auto *secure = new WiFiClientSecure();
        secure->setInsecure();
        clientHolder.reset(secure);
    }
    WiFiClient &client = *clientHolder;
    HTTPClient http;
    http.setTimeout(MAP_HTTP_TIMEOUT_MS);
    http.setReuse(true);
    bool loaded = true;
    int nextDestinationY = 0;
    uint8_t brightness = static_cast<uint8_t>(std::min(100, static_cast<int>(brightnessPercent)));
    for (int tileRow = 0; tileRow < geometry.tileRows && loaded; tileRow++) {
        if (tileRow > 0) {
            memcpy(strip, previousLine, static_cast<size_t>(stripWidth) * sizeof(uint16_t));
        }
        int tileY = geometry.tileMinY + tileRow;
        for (int tileColumn = 0; tileColumn < geometry.tileColumns; tileColumn++) {
            progress.tileIndex = static_cast<size_t>(
                tileRow * geometry.tileColumns + tileColumn
            );
            loaded = downloadTile(
                client,
                http,
                geometry,
                geometry.tileMinX + tileColumn,
                tileY,
                apiKey,
                feedHost,
                strip,
                stripWidth,
                tileColumn * MAP_TILE_SIZE,
                progress,
                progressCallback,
                progressContext
            );
            if (!loaded) break;
        }
        if (!loaded) break;

        int64_t rowBase = static_cast<int64_t>(tileY) * MAP_TILE_SIZE;
        loaded = renderAvailableRows(
            geometry,
            sourceX,
            strip,
            stripWidth,
            rowBase,
            _buffers[viewIndex],
            _width,
            _height,
            brightness,
            nextDestinationY
        );
        memcpy(
            previousLine,
            strip + static_cast<size_t>(MAP_TILE_SIZE) * stripWidth,
            static_cast<size_t>(stripWidth) * sizeof(uint16_t)
        );
    }

    heap_caps_free(sourceX);
    heap_caps_free(previousLine);
    heap_caps_free(strip);

    if (!loaded || nextDestinationY != _height) {
        if (loaded) {
            RADAR_LOGE("[map] XYZ render incomplete rows=%d/%d\n", nextDestinationY, _height);
            emitProgress(
                progress,
                LoadPhase::Error,
                progressCallback,
                progressContext,
                "XYZ RENDER INCOMPLETE"
            );
        }
        return false;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _ready[viewIndex] = true;
    xSemaphoreGive(_mutex);
    progress.tileIndex = progress.tileCount;
    progress.receivedBytes = 0;
    progress.totalBytes = 0;
    emitProgress(progress, LoadPhase::Ready, progressCallback, progressContext);
    RADAR_LOGI(
        "[map] XYZ ready view=%u tiles=%u bytes=%u decode_ms=%lu free_psram=%u\n",
        static_cast<unsigned>(viewIndex),
        static_cast<unsigned>(progress.tileCount),
        static_cast<unsigned>(progress.viewReceivedBytes),
        static_cast<unsigned long>(progress.decodeMs),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM))
    );
    return true;
}

bool Background::draw(PanelDisplay::Canvas &canvas, size_t viewIndex) {
    if (_mutex == nullptr || viewIndex >= _viewCount) return false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    bool ready = _ready[viewIndex] && _buffers[viewIndex] != nullptr;
    if (ready) {
        canvas.blitRGB565(0, 0, _width, _height, _buffers[viewIndex], _width);
    }
    xSemaphoreGive(_mutex);
    return ready;
}

bool Background::isReady(size_t viewIndex) {
    if (_mutex == nullptr || viewIndex >= _viewCount) return false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    bool ready = _ready[viewIndex];
    xSemaphoreGive(_mutex);
    return ready;
}

void Background::clear() {
    if (_mutex == nullptr) return;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    memset(_ready, 0, sizeof(_ready));
    xSemaphoreGive(_mutex);
}

} // namespace RadarMap
