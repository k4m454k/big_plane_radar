#include "map_background.h"

#include <HTTPClient.h>
#include <PNGdec.h>
#include <WiFiClientSecure.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <math.h>
#include <new>

namespace RadarMap {

static constexpr char STADIA_STATIC_MAP_URL[] =
    "https://tiles-eu.stadiamaps.com/static/alidade_smooth_dark.png";
static constexpr uint32_t MAP_HTTP_TIMEOUT_MS = 20000;
static constexpr size_t MAP_MAX_PNG_BYTES = 1024 * 1024;
static constexpr int MAP_MAX_SOURCE_WIDTH = 1040;
// Stadia Static Maps uses a 512-point world at zoom 0, unlike classic 256px XYZ tiles.
static constexpr double STADIA_METERS_PER_POINT_Z0 = 78271.51696402048;

Background background;

struct MapGeometry {
    int zoom = 0;
    int sourceWidth = 0;
    int sourceHeight = 0;
};

struct BilinearSample {
    uint16_t first = 0;
    uint16_t second = 0;
    uint16_t weight = 0;
};

struct DecodeContext {
    PNG *decoder = nullptr;
    uint16_t *destination = nullptr;
    uint16_t *sourceLines[2] = {nullptr, nullptr};
    BilinearSample *sourceX = nullptr;
    int sourceWidth = 0;
    int sourceHeight = 0;
    int destinationWidth = 0;
    int destinationHeight = 0;
    int nextDestinationY = 0;
};

static BilinearSample bilinearSample(
    int destinationIndex,
    int destinationSize,
    int sourceSize
) {
    constexpr int64_t SUBPIXEL_SCALE = 256;
    int64_t sourcePosition =
        ((static_cast<int64_t>(destinationIndex * 2 + 1) * sourceSize * SUBPIXEL_SCALE) /
         (destinationSize * 2)) -
        (SUBPIXEL_SCALE / 2);
    int64_t lastSourcePosition =
        static_cast<int64_t>(sourceSize - 1) * SUBPIXEL_SCALE;

    BilinearSample result;
    if (sourcePosition <= 0) return result;
    if (sourcePosition >= lastSourcePosition) {
        result.first = static_cast<uint16_t>(sourceSize - 1);
        result.second = result.first;
        return result;
    }

    result.first = static_cast<uint16_t>(sourcePosition / SUBPIXEL_SCALE);
    result.weight = static_cast<uint16_t>(sourcePosition % SUBPIXEL_SCALE);
    result.second = result.weight == 0 ? result.first : result.first + 1;
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

static MapGeometry mapGeometry(
    double centerLat,
    float outerKm,
    int radarRadius,
    int destinationWidth,
    int destinationHeight
) {
    MapGeometry result;
    double latitude = std::max(-85.0, std::min(85.0, centerLat));
    double metersPerDestinationPixel = (outerKm * 1000.0) / radarRadius;
    double latitudeScale = std::max(0.01, cos(latitude * DEG_TO_RAD));
    double rawZoom = log2(
        (STADIA_METERS_PER_POINT_Z0 * latitudeScale) /
        metersPerDestinationPixel
    );
    result.zoom = std::max(0, std::min(18, static_cast<int>(ceil(rawZoom))));

    double metersPerSourcePixel =
        (STADIA_METERS_PER_POINT_Z0 * latitudeScale) /
        static_cast<double>(1UL << result.zoom);
    double scale = metersPerDestinationPixel / metersPerSourcePixel;
    result.sourceWidth = std::max(
        destinationWidth,
        static_cast<int>(ceil(destinationWidth * scale))
    );
    result.sourceHeight = std::max(
        destinationHeight,
        static_cast<int>(ceil(destinationHeight * scale))
    );
    return result;
}

static int drawPngLine(PNGDRAW *draw) {
    auto *context = static_cast<DecodeContext *>(draw->pUser);
    if (context == nullptr || draw->iWidth != context->sourceWidth) return 0;

    uint16_t *currentSourceLine = context->sourceLines[draw->y & 1];
    context->decoder->getLineAsRGB565(
        draw,
        currentSourceLine,
        PNG_RGB565_LITTLE_ENDIAN,
        0xffffffff
    );

    while (context->nextDestinationY < context->destinationHeight) {
        BilinearSample sourceY = bilinearSample(
            context->nextDestinationY,
            context->destinationHeight,
            context->sourceHeight
        );
        if (sourceY.second > draw->y) return 1;

        const uint16_t *firstSourceLine =
            context->sourceLines[sourceY.first & 1];
        const uint16_t *secondSourceLine =
            context->sourceLines[sourceY.second & 1];
        uint16_t *destinationLine =
            context->destination +
            static_cast<size_t>(context->nextDestinationY) * context->destinationWidth;

        for (int x = 0; x < context->destinationWidth; x++) {
            const BilinearSample &sourceX = context->sourceX[x];
            uint16_t firstRowPixel = interpolateRgb565(
                firstSourceLine[sourceX.first],
                firstSourceLine[sourceX.second],
                sourceX.weight
            );
            if (sourceY.weight == 0) {
                destinationLine[x] = firstRowPixel;
                continue;
            }
            uint16_t secondRowPixel = interpolateRgb565(
                secondSourceLine[sourceX.first],
                secondSourceLine[sourceX.second],
                sourceX.weight
            );
            destinationLine[x] = interpolateRgb565(
                firstRowPixel,
                secondRowPixel,
                sourceY.weight
            );
        }
        context->nextDestinationY++;
    }
    return 1;
}

static bool readHttpBody(HTTPClient &http, uint8_t *&data, size_t &size) {
    int contentLength = http.getSize();
    if (contentLength <= 0 || static_cast<size_t>(contentLength) > MAP_MAX_PNG_BYTES) {
        Serial.printf("[map] invalid content length=%d\n", contentLength);
        return false;
    }

    data = static_cast<uint8_t *>(heap_caps_malloc(
        static_cast<size_t>(contentLength),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    ));
    if (data == nullptr) {
        Serial.printf("[map] PNG allocation failed bytes=%d\n", contentLength);
        return false;
    }

    auto *stream = http.getStreamPtr();
    size_t received = 0;
    uint32_t lastProgressMs = millis();
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
        Serial.printf("[map] short response bytes=%u/%d\n",
                      static_cast<unsigned>(received), contentLength);
        heap_caps_free(data);
        data = nullptr;
        return false;
    }
    size = received;
    return true;
}

static bool decodePng(
    uint8_t *pngData,
    size_t pngSize,
    uint16_t *destination,
    int destinationWidth,
    int destinationHeight,
    int expectedSourceWidth,
    int expectedSourceHeight
) {
    void *decoderStorage = heap_caps_calloc(
        1,
        sizeof(PNG),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    auto *sourceLine0 = static_cast<uint16_t *>(heap_caps_malloc(
        static_cast<size_t>(expectedSourceWidth) * sizeof(uint16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    auto *sourceLine1 = static_cast<uint16_t *>(heap_caps_malloc(
        static_cast<size_t>(expectedSourceWidth) * sizeof(uint16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    auto *sourceX = static_cast<BilinearSample *>(heap_caps_malloc(
        static_cast<size_t>(destinationWidth) * sizeof(BilinearSample),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    ));
    if (decoderStorage == nullptr || sourceLine0 == nullptr ||
        sourceLine1 == nullptr || sourceX == nullptr) {
        Serial.println("[map] decoder allocation failed");
        if (sourceX != nullptr) heap_caps_free(sourceX);
        if (sourceLine1 != nullptr) heap_caps_free(sourceLine1);
        if (sourceLine0 != nullptr) heap_caps_free(sourceLine0);
        if (decoderStorage != nullptr) heap_caps_free(decoderStorage);
        return false;
    }

    auto *decoder = new (decoderStorage) PNG();
    DecodeContext context;
    context.decoder = decoder;
    context.destination = destination;
    context.sourceLines[0] = sourceLine0;
    context.sourceLines[1] = sourceLine1;
    context.sourceX = sourceX;
    context.sourceWidth = expectedSourceWidth;
    context.sourceHeight = expectedSourceHeight;
    context.destinationWidth = destinationWidth;
    context.destinationHeight = destinationHeight;
    for (int x = 0; x < destinationWidth; x++) {
        context.sourceX[x] = bilinearSample(
            x,
            destinationWidth,
            expectedSourceWidth
        );
    }

    int result = decoder->openRAM(pngData, static_cast<int>(pngSize), drawPngLine);
    if (result == PNG_SUCCESS &&
        decoder->getWidth() == expectedSourceWidth &&
        decoder->getHeight() == expectedSourceHeight &&
        !decoder->isInterlaced()) {
        result = decoder->decode(&context, PNG_FAST_PALETTE);
    } else if (result == PNG_SUCCESS) {
        Serial.printf("[map] unexpected PNG %dx%d interlaced=%d\n",
                      decoder->getWidth(), decoder->getHeight(), decoder->isInterlaced());
        result = PNG_INVALID_FILE;
    }
    decoder->close();
    decoder->~PNG();
    heap_caps_free(sourceX);
    heap_caps_free(sourceLine1);
    heap_caps_free(sourceLine0);
    heap_caps_free(decoderStorage);

    if (result != PNG_SUCCESS || context.nextDestinationY != destinationHeight) {
        Serial.printf("[map] PNG decode failed code=%d rows=%d/%d\n",
                      result, context.nextDestinationY, destinationHeight);
        return false;
    }
    return true;
}

bool Background::begin(int width, int height, size_t viewCount) {
    if (_mutex == nullptr) {
        _mutex = xSemaphoreCreateMutexStatic(&_mutexStorage);
    }
    if (_mutex == nullptr || width <= 0 || height <= 0 ||
        viewCount == 0 || viewCount > MAX_VIEWS) {
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
            Serial.printf("[map] framebuffer allocation failed views=%u bytes=%u\n",
                          static_cast<unsigned>(viewCount),
                          static_cast<unsigned>(bytes * viewCount));
            return false;
        }
    }
    _width = width;
    _height = height;
    _viewCount = viewCount;
    memset(_ready, 0, sizeof(_ready));
    Serial.printf("[map] cache ready size=%dx%d views=%u bytes=%u free_psram=%u\n",
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
    size_t viewIndex
) {
    if (viewIndex >= _viewCount || _buffers[viewIndex] == nullptr || apiKey.isEmpty()) {
        return false;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _ready[viewIndex] = false;
    xSemaphoreGive(_mutex);

    MapGeometry geometry = mapGeometry(
        centerLat,
        outerKm,
        radarRadius,
        _width,
        _height
    );
    if (geometry.sourceWidth > MAP_MAX_SOURCE_WIDTH ||
        geometry.sourceHeight > MAP_MAX_SOURCE_WIDTH) {
        Serial.printf("[map] source dimensions too large %dx%d\n",
                      geometry.sourceWidth, geometry.sourceHeight);
        return false;
    }

    char url[256];
    snprintf(url,
             sizeof(url),
             "%s?center=%.6f,%.6f&zoom=%d&size=%dx%d",
             STADIA_STATIC_MAP_URL,
             centerLat,
             centerLon,
             geometry.zoom,
             geometry.sourceWidth,
             geometry.sourceHeight);

    Serial.printf("[map] fetch begin view=%u zoom=%d source=%dx%d\n",
                  static_cast<unsigned>(viewIndex),
                  geometry.zoom,
                  geometry.sourceWidth,
                  geometry.sourceHeight);
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setTimeout(MAP_HTTP_TIMEOUT_MS);
    if (!http.begin(client, url)) {
        Serial.println("[map] HTTP begin failed");
        return false;
    }
    String authorization = F("Stadia-Auth ");
    authorization += apiKey;
    http.addHeader(F("Authorization"), authorization);
    int status = http.GET();
    if (status != HTTP_CODE_OK) {
        Serial.printf("[map] HTTP status=%d\n", status);
        http.end();
        return false;
    }

    uint8_t *pngData = nullptr;
    size_t pngSize = 0;
    bool downloaded = readHttpBody(http, pngData, pngSize);
    http.end();
    if (!downloaded) return false;

    uint32_t decodeStartedMs = millis();
    bool decoded = decodePng(
        pngData,
        pngSize,
        _buffers[viewIndex],
        _width,
        _height,
        geometry.sourceWidth,
        geometry.sourceHeight
    );
    heap_caps_free(pngData);
    if (!decoded) return false;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _ready[viewIndex] = true;
    xSemaphoreGive(_mutex);
    Serial.printf("[map] ready view=%u png=%u decode_ms=%lu\n",
                  static_cast<unsigned>(viewIndex),
                  static_cast<unsigned>(pngSize),
                  static_cast<unsigned long>(millis() - decodeStartedMs));
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
