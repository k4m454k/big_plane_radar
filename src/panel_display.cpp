#include "panel_display.h"
#include "panel_font.h"
#include "app_log.h"
#include "display_tuning.h"

#include <algorithm>
#include <ctype.h>
#include <cstring>
#include <driver/i2c.h>
#include <esp_display_panel.hpp>
#include <esp_heap_caps.h>
#include <math.h>
#include <pgmspace.h>
#include <board/esp_panel_board_default_config.hpp>

using namespace esp_panel::board;
using namespace esp_panel::drivers;

namespace PanelDisplay {

static Board *board = nullptr;
static LCD *lcd = nullptr;
static Touch *touch = nullptr;
static uint32_t presentCounter = 0;
static StaticSemaphore_t refreshFinishedSemaphoreStorage;
static SemaphoreHandle_t refreshFinishedSemaphore = nullptr;
static uint8_t lcd7bOutputState = 0xFF;

Canvas screen;

static bool IRAM_ATTR onRefreshFinished(void *) {
    if (refreshFinishedSemaphore == nullptr) {
        return false;
    }
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(refreshFinishedSemaphore, &higherPriorityTaskWoken);
    return higherPriorityTaskWoken == pdTRUE;
}

static void drainRefreshSemaphore() {
    while (refreshFinishedSemaphore != nullptr &&
           xSemaphoreTake(refreshFinishedSemaphore, 0) == pdTRUE) {}
}


// Probe bus pins. The Waveshare boards expose the CH422G expander and GT911 on
// GPIO 8/9; on the CrowPanel those two pins are RGB data lines (G3 and G0), and
// its GT911 lives on GPIO 19/20 instead. The two pin sets are mutually
// destructive, which is why the CrowPanel cannot be autodetected.
#if PLANE_RADAR_BOARD_CROWPANEL7
static constexpr int PROBE_I2C_SDA = 19;
static constexpr int PROBE_I2C_SCL = 20;
#else
static constexpr int PROBE_I2C_SDA = 8;
static constexpr int PROBE_I2C_SCL = 9;
#endif

static bool beginProbeI2c() {
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = static_cast<gpio_num_t>(PROBE_I2C_SDA);
    config.scl_io_num = static_cast<gpio_num_t>(PROBE_I2C_SCL);
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;
    config.clk_flags = 0;
    return i2c_param_config(I2C_NUM_0, &config) == ESP_OK &&
           i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK;
}

static bool readI2cRegister16(uint8_t address, uint16_t reg, uint8_t *data, size_t size) {
    uint8_t command[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg),
    };
    return i2c_master_write_read_device(
        I2C_NUM_0,
        address,
        command,
        sizeof(command),
        data,
        size,
        pdMS_TO_TICKS(100)
    ) == ESP_OK;
}

static bool readI2cRegister8(uint8_t address, uint8_t reg, uint8_t &value) {
    return i2c_master_write_read_device(
        I2C_NUM_0,
        address,
        &reg,
        sizeof(reg),
        &value,
        sizeof(value),
        pdMS_TO_TICKS(100)
    ) == ESP_OK;
}

static bool has7BControllerSignature() {
    uint8_t id0 = 0;
    uint8_t id1 = 0;
    uint8_t mode = 0;
    return readI2cRegister8(0x24, 0x00, id0) &&
           readI2cRegister8(0x24, 0x01, id1) &&
           readI2cRegister8(0x24, 0x02, mode) &&
           id0 == 0xFF && id1 == 0xAA && mode == 0xFF;
}

static bool write7BRegisterDriver(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(
        I2C_NUM_0,
        0x24,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    ) == ESP_OK;
}

static bool readGt911Dimensions(uint8_t address, uint16_t &width, uint16_t &height) {
    uint8_t product[4] = {};
    uint8_t limits[4] = {};
    if (!readI2cRegister16(address, 0x8140, product, sizeof(product)) ||
        !readI2cRegister16(address, 0x8048, limits, sizeof(limits))) {
        return false;
    }
    if (product[0] != '9' || product[1] != '1' || product[2] != '1') {
        return false;
    }
    width = static_cast<uint16_t>(limits[0] | (limits[1] << 8));
    height = static_cast<uint16_t>(limits[2] | (limits[3] << 8));
    return true;
}

static bool prepare7BHardware() {
    constexpr uint8_t TOUCH_RESET = 1U << 1;
    constexpr uint8_t BACKLIGHT = 1U << 2;
    constexpr uint8_t LCD_RESET = 1U << 3;
    constexpr uint8_t LCD_VDD = 1U << 6;

    if (!write7BRegisterDriver(0x02, 0xFF)) return false;
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    lcd7bOutputState = static_cast<uint8_t>(0xFFU & ~(TOUCH_RESET | BACKLIGHT | LCD_RESET));
    lcd7bOutputState |= LCD_VDD;
    if (!write7BRegisterDriver(0x03, lcd7bOutputState)) return false;
    delay(20);

    lcd7bOutputState |= LCD_RESET;
    if (!write7BRegisterDriver(0x03, lcd7bOutputState)) return false;
    delay(100);

    lcd7bOutputState |= TOUCH_RESET;
    if (!write7BRegisterDriver(0x03, lcd7bOutputState)) return false;
    delay(200);
    pinMode(4, INPUT);
    return true;
}

#if PLANE_RADAR_BOARD_CROWPANEL7
// CrowPanel V3.0 adds a PCA9557 that gates the GT911 reset line; V2.0 wires the
// panel directly and has no expander. Probing lets one image serve both, so the
// board revision never has to be known at build time.
// PCA9557 registers: 0x00 input, 0x01 output, 0x02 polarity, 0x03 direction
// (bit set = input).
// IO0 is the GT911 reset line and IO1 is its interrupt line. INT must be held
// LOW across the reset rising edge -- that level is what the GT911 latches to
// choose its I2C address (low -> 0x5D, high -> 0x14) -- and must then be handed
// back as an input so the controller can actually signal contacts. Sequence
// taken from Elecrow's own V3.0 demo.
static constexpr uint8_t PCA9557_TOUCH_RESET = 1U << 0;
static constexpr uint8_t PCA9557_TOUCH_INT = 1U << 1;
static uint8_t crowPanelExpanderAddress = 0;
// The GT911 answers on 0x5D or 0x14 depending on the INT pin level latched at
// reset. INT is not broken out here, so the level is whatever the board leaves
// it at; the probe records which address replied and the bus is pinned to it
// rather than trusting the driver's compiled-in default.
static uint8_t crowPanelTouchAddress = 0;

static bool writePca9557(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(
        I2C_NUM_0,
        crowPanelExpanderAddress,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    ) == ESP_OK;
}

// Returns true when a V3.0 expander was found and the touch reset pulse was
// issued. A false return is the normal, healthy V2.0 path, not an error.
static bool prepareCrowPanelExpander() {
    crowPanelExpanderAddress = 0;
    for (uint8_t candidate : {0x19, 0x18}) {
        uint8_t scratch = 0;
        if (readI2cRegister8(candidate, 0x00, scratch)) {
            crowPanelExpanderAddress = candidate;
            break;
        }
    }
    if (crowPanelExpanderAddress == 0) {
        return false;
    }

    // Drive both RST and INT as outputs, and hold both low.
    const uint8_t driven = static_cast<uint8_t>(PCA9557_TOUCH_RESET | PCA9557_TOUCH_INT);
    if (!writePca9557(0x03, static_cast<uint8_t>(~driven))) return false;
    if (!writePca9557(0x01, 0x00)) return false;
    delay(20);

    // Release reset while INT is still low, latching address 0x5D.
    if (!writePca9557(0x01, PCA9557_TOUCH_RESET)) return false;
    delay(100);

    // Hand INT back to the controller (config bit set = input).
    if (!writePca9557(0x03, static_cast<uint8_t>(~PCA9557_TOUCH_RESET))) return false;
    delay(50);
    return true;
}

static BoardConfig makeCrowPanelBoardConfig() {
    BoardConfig config = ESP_PANEL_BOARD_DEFAULT_CONFIG;
    config.name = "Elecrow:CrowPanel-7.0-DIS08070H";
    config.stage_callbacks.fill(nullptr);
    // No CH422G on this board; the compile-time default would otherwise try to
    // drive an expander bus over two of the RGB data lines.
    config.io_expander.reset();

    auto *rgb = std::get_if<BusRGB::Config>(&config.lcd->bus_config);
    auto *refresh = rgb == nullptr
        ? nullptr
        : std::get_if<BusRGB::RefreshPanelPartialConfig>(&rgb->refresh_panel);
    if (refresh != nullptr) {
        refresh->pclk_hz = PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ;
        refresh->h_res = 800;
        refresh->v_res = 480;
        // Elecrow reference timing for the LI0704122Z panel.
        refresh->hsync_pulse_width = 48;
        refresh->hsync_back_porch = 40;
        refresh->hsync_front_porch = 40;
        refresh->vsync_pulse_width = 31;
        refresh->vsync_back_porch = 13;
        refresh->vsync_front_porch = 1;
        refresh->bounce_buffer_size_px = 800 * PLANE_RADAR_RGB_BOUNCE_LINES;
        refresh->flags_pclk_active_neg = true;
        refresh->hsync_gpio_num = 39;
        refresh->vsync_gpio_num = 40;
        refresh->de_gpio_num = 41;
        refresh->pclk_gpio_num = 0;
        refresh->disp_gpio_num = -1;
        // Data lines are ordered B0-B4, G0-G5, R0-R4 for RGB565.
        const int dataPins[16] = {
            15, 7, 6, 5, 4,          // B0-B4
            9, 46, 3, 8, 16, 1,      // G0-G5
            14, 21, 47, 48, 45,      // R0-R4
        };
        std::copy(std::begin(dataPins), std::end(dataPins), std::begin(refresh->data_gpio_nums));
    }

    auto *lcdVendor = std::get_if<LCD::VendorPartialConfig>(
        &config.lcd->device_config.vendor
    );
    if (lcdVendor != nullptr) {
        lcdVendor->hor_res = 800;
        lcdVendor->ver_res = 480;
    }

    auto *touchBus = std::get_if<BusI2C::Config>(&config.touch->bus_config);
    if (touchBus != nullptr) {
        if (touchBus->host.has_value()) {
            auto *touchHost = std::get_if<BusI2C::HostPartialConfig>(&touchBus->host.value());
            if (touchHost != nullptr) {
                touchHost->sda_io_num = PROBE_I2C_SDA;
                touchHost->scl_io_num = PROBE_I2C_SCL;
            }
        }
        if (crowPanelTouchAddress != 0) {
            touchBus->control_panel.dev_addr = crowPanelTouchAddress;
        }
    }

    auto *touchDevice = std::get_if<Touch::DevicePartialConfig>(
        &config.touch->device_config.device
    );
    if (touchDevice != nullptr) {
        touchDevice->x_max = 800;
        touchDevice->y_max = 480;
        // Reset is either hard-wired (V2.0) or already pulsed via the PCA9557
        // above, and no interrupt line is broken out.
        touchDevice->rst_gpio_num = -1;
        touchDevice->int_gpio_num = -1;
    }

    // Backlight is a plain GPIO here, so LEDC gives real brightness control
    // rather than the expander on/off switch the Waveshare boards use.
    config.backlight = BoardConfig::BacklightConfig{
        .config = BacklightPWM_LEDC::Config{
            .ledc_channel = BacklightPWM_LEDC::LEDC_ChannelPartialConfig{
                .io_num = 2,
                .on_level = 1,
            },
        },
        .pre_process = {.idle_off = 0},
    };
    return config;
}
#endif // PLANE_RADAR_BOARD_CROWPANEL7

static Model detectAndPrepareModel() {
#if PLANE_RADAR_BOARD_CROWPANEL7
    uint16_t crowTouchWidth = 0;
    uint16_t crowTouchHeight = 0;
    bool crowProbeOk = false;
    bool crowExpander = false;

    if (beginProbeI2c()) {
        crowExpander = prepareCrowPanelExpander();
        for (uint8_t attempt = 0; attempt < 3 && !crowProbeOk; attempt++) {
            for (uint8_t address : {0x5D, 0x14}) {
                if (readGt911Dimensions(address, crowTouchWidth, crowTouchHeight)) {
                    crowProbeOk = true;
                    crowPanelTouchAddress = address;
                    break;
                }
            }
            if (!crowProbeOk) delay(30);
        }
        i2c_driver_delete(I2C_NUM_0);
    }

    RADAR_LOGI(
        "[display] probe gt911=%d addr=0x%02X limits=%ux%u pca9557=%d rev=%s selected=CrowPanel-7.0\n",
        crowProbeOk ? 1 : 0,
        static_cast<unsigned>(crowPanelTouchAddress),
        static_cast<unsigned>(crowTouchWidth),
        static_cast<unsigned>(crowTouchHeight),
        crowExpander ? 1 : 0,
        crowExpander ? "V3.0" : "V2.0"
    );
    return Model::CrowPanel7;
#else
    Model detected = Model::TouchLcd7;
    uint16_t touchWidth = 0;
    uint16_t touchHeight = 0;
    bool probeOk = false;
    bool controller7B = false;

    bool i2cReady = beginProbeI2c();
    if (i2cReady) {
        controller7B = has7BControllerSignature();
        for (uint8_t attempt = 0; attempt < 3 && !probeOk; attempt++) {
            probeOk = readGt911Dimensions(0x5D, touchWidth, touchHeight) ||
                      readGt911Dimensions(0x14, touchWidth, touchHeight);
            if (!probeOk) delay(30);
        }
    }

#if PLANE_RADAR_DISPLAY_PROFILE == 8
    detected = Model::TouchLcd7B;
#elif PLANE_RADAR_DISPLAY_PROFILE == 7
    detected = Model::TouchLcd7;
#else
    if ((probeOk && touchWidth == 1024 && touchHeight == 600) || controller7B) {
        detected = Model::TouchLcd7B;
    }
#endif

    RADAR_LOGI(
        "[display] probe gt911=%d limits=%ux%u controller_7b=%d selected=%s\n",
        probeOk ? 1 : 0,
        static_cast<unsigned>(touchWidth),
        static_cast<unsigned>(touchHeight),
        controller7B ? 1 : 0,
        detected == Model::TouchLcd7B ? "7B" : "7"
    );

    if (detected == Model::TouchLcd7B && (!i2cReady || !prepare7BHardware())) {
        RADAR_LOGE("[display] 7B power/reset controller setup failed\n");
    }
    if (i2cReady) {
        i2c_driver_delete(I2C_NUM_0);
    }
    return detected;
#endif // PLANE_RADAR_BOARD_CROWPANEL7
}

static BoardConfig make7BBoardConfig() {
    BoardConfig config = ESP_PANEL_BOARD_DEFAULT_CONFIG;
    config.name = "Waveshare:ESP32-S3-Touch-LCD-7B";
    config.stage_callbacks.fill(nullptr);
    config.io_expander.reset();
    config.backlight.reset();

    auto *rgb = std::get_if<BusRGB::Config>(&config.lcd->bus_config);
    auto *refresh = rgb == nullptr
        ? nullptr
        : std::get_if<BusRGB::RefreshPanelPartialConfig>(&rgb->refresh_panel);
    if (refresh != nullptr) {
        refresh->pclk_hz = PLANE_RADAR_RGB_7B_PCLK_HZ;
        refresh->h_res = 1024;
        refresh->v_res = 600;
        refresh->hsync_pulse_width = 162;
        refresh->hsync_back_porch = 152;
        refresh->hsync_front_porch = 48;
        refresh->vsync_pulse_width = 45;
        refresh->vsync_back_porch = 13;
        refresh->vsync_front_porch = 3;
        refresh->bounce_buffer_size_px = 1024 * PLANE_RADAR_RGB_BOUNCE_LINES;
        refresh->flags_pclk_active_neg = true;
    }

    auto *lcdVendor = std::get_if<LCD::VendorPartialConfig>(
        &config.lcd->device_config.vendor
    );
    if (lcdVendor != nullptr) {
        lcdVendor->hor_res = 1024;
        lcdVendor->ver_res = 600;
    }

    auto *touchDevice = std::get_if<Touch::DevicePartialConfig>(
        &config.touch->device_config.device
    );
    if (touchDevice != nullptr) {
        touchDevice->x_max = 1024;
        touchDevice->y_max = 600;
        touchDevice->rst_gpio_num = -1;
        touchDevice->int_gpio_num = 4;
    }
    return config;
}


bool Canvas::begin() {
    RADAR_LOGD("[display] ESP32_Display_Panel backend begin\n");
    _model = detectAndPrepareModel();
#if PLANE_RADAR_BOARD_CROWPANEL7
    BoardConfig config = makeCrowPanelBoardConfig();
    board = new Board(config);
#else
    if (_model == Model::TouchLcd7B) {
        BoardConfig config = make7BBoardConfig();
        board = new Board(config);
    } else {
        board = new Board();
    }
#endif
    if (board == nullptr) {
        RADAR_LOGE("[display] Board allocation failed\n");
        return false;
    }

    RADAR_LOGD("[display] board.init begin\n");
    if (!board->init()) {
        RADAR_LOGE("[display] board.init failed\n");
        return false;
    }

    lcd = board->getLCD();
    if (lcd == nullptr) {
        RADAR_LOGE("[display] LCD is null after init\n");
        return false;
    }
    lcd->configFrameBufferNumber(2);

    RADAR_LOGD("[display] board.begin begin\n");
    if (!board->begin()) {
        RADAR_LOGE("[display] board.begin failed\n");
        return false;
    }

    lcd = board->getLCD();
    touch = board->getTouch();
    if (lcd == nullptr) {
        RADAR_LOGE("[display] LCD is null after begin\n");
        return false;
    }

    _width = lcd->getFrameWidth();
    _height = lcd->getFrameHeight();
    if (_model == Model::TouchLcd7B) {
        lcd7bOutputState |= 1U << 2;
        if (!write7BRegisterDriver(0x03, lcd7bOutputState)) {
            RADAR_LOGE("[display] 7B backlight enable failed\n");
        }
    }

    size_t pixels = static_cast<size_t>(_width) * _height;
    _driverFb[0] = static_cast<uint16_t *>(lcd->getFrameBufferByIndex(0));
    _driverFb[1] = static_cast<uint16_t *>(lcd->getFrameBufferByIndex(1));
    if (_driverFb[0] != nullptr && _driverFb[1] != nullptr) {
        _usingDriverFrameBuffers = true;
        _drawFbIndex = 1;
        _fb = _driverFb[_drawFbIndex];
        std::fill(_driverFb[0], _driverFb[0] + pixels, TFT_BLACK);
        std::fill(_driverFb[1], _driverFb[1] + pixels, TFT_BLACK);
        refreshFinishedSemaphore = xSemaphoreCreateBinaryStatic(&refreshFinishedSemaphoreStorage);
        // Pass real internal-RAM user data rather than letting it default to
        // nullptr. When XIP-on-PSRAM is off the driver asserts the ISR's user
        // data lives in SRAM, and esp_ptr_internal(nullptr) is false; with XIP
        // on, that check is compiled out entirely, which is the only reason a
        // null pointer ever worked here. The semaphore storage is a plain
        // static, so it is in .bss and satisfies the check for both SDKs.
        if (refreshFinishedSemaphore == nullptr ||
            !lcd->attachRefreshFinishCallback(
                onRefreshFinished, &refreshFinishedSemaphoreStorage
            )) {
            RADAR_LOGE("[display] refresh synchronization setup failed\n");
            return false;
        }
        drainRefreshSemaphore();
        if (!lcd->switchFrameBufferTo(_driverFb[0])) {
            RADAR_LOGE("[display] initial framebuffer switch failed\n");
            return false;
        }
        drainRefreshSemaphore();
        if (xSemaphoreTake(refreshFinishedSemaphore, pdMS_TO_TICKS(150)) != pdTRUE) {
            RADAR_LOGE("[display] initial framebuffer synchronization failed\n");
            return false;
        }
    } else {
        RADAR_LOGI("[display] driver framebuffers unavailable, using copy framebuffer\n");
        size_t bytes = pixels * sizeof(uint16_t);
        _fb = static_cast<uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (_fb == nullptr) {
            RADAR_LOGI("[display] PSRAM framebuffer unavailable, trying internal heap\n");
            _fb = static_cast<uint16_t *>(malloc(bytes));
        }
        if (_fb == nullptr) {
            RADAR_LOGE("[display] framebuffer allocation failed\n");
            return false;
        }
    }

    RADAR_LOGI("[display] ready lcd=%dx%d touch=%d double=%d free_heap=%u free_psram=%u\n",
               lcd->getFrameWidth(),
               lcd->getFrameHeight(),
               touch != nullptr ? 1 : 0,
               _usingDriverFrameBuffers ? 1 : 0,
               static_cast<unsigned>(ESP.getFreeHeap()),
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    RADAR_LOGD("[display] color_bits=%d fb=%p fb0=%p fb1=%p\n",
               lcd->getFrameColorBits(), _fb, _driverFb[0], _driverFb[1]);
    if (_usingDriverFrameBuffers) {
        return true;
    }
    fillScreen(TFT_BLACK);
    return present();
}

bool Canvas::present() {
    if (lcd == nullptr || _fb == nullptr) {
        RADAR_LOGE("[display] present skipped: lcd/fb null\n");
        return false;
    }
    uint32_t start = millis();
    if (_usingDriverFrameBuffers) {
        if (refreshFinishedSemaphore == nullptr) {
            RADAR_LOGE("[display] refresh semaphore unavailable\n");
            return false;
        }

        bool synchronized = false;
        for (uint8_t attempt = 0; attempt < 2 && !synchronized; attempt++) {
            drainRefreshSemaphore();
            if (!lcd->switchFrameBufferTo(_fb)) {
                RADAR_LOGE("[display] framebuffer switch failed attempt=%u\n", attempt + 1);
                continue;
            }
            drainRefreshSemaphore();
            synchronized = xSemaphoreTake(refreshFinishedSemaphore, pdMS_TO_TICKS(150)) == pdTRUE;
            if (!synchronized) {
                RADAR_LOGE("[display] refresh wait timeout attempt=%u\n", attempt + 1);
            }
        }
        if (!synchronized) {
            return false;
        }
        _drawFbIndex ^= 1;
        _fb = _driverFb[_drawFbIndex];
    } else {
        if (!lcd->drawBitmap(0, 0, _width, _height, reinterpret_cast<const uint8_t *>(_fb), -1)) {
            RADAR_LOGE("[display] drawBitmap failed\n");
            return false;
        }
    }
    presentCounter++;
    if (presentCounter <= 3 || presentCounter % 120 == 0) {
        RADAR_LOGD("[display] present #%lu dt=%lu double=%d\n",
                   static_cast<unsigned long>(presentCounter),
                   static_cast<unsigned long>(millis() - start),
                   _usingDriverFrameBuffers ? 1 : 0);
    }
    return true;
}

bool Canvas::readTouch(uint16_t *x, uint16_t *y) {
    if (touch == nullptr) {
        return false;
    }
    TouchPoint points[1];
    int count = touch->readPoints(points, 1, 0);
    if (count <= 0) {
        return false;
    }
    _lastRawTouchX = points[0].x;
    _lastRawTouchY = points[0].y;
    _touchReadCount++;
    if (x != nullptr) *x = static_cast<uint16_t>(std::max(0, std::min(_width - 1, points[0].x)));
    if (y != nullptr) *y = static_cast<uint16_t>(std::max(0, std::min(_height - 1, points[0].y)));
    return true;
}

const uint16_t *Canvas::displayedFrameBuffer() const {
    if (_usingDriverFrameBuffers) {
        return _driverFb[_drawFbIndex ^ 1];
    }
    return _fb;
}

const char *Canvas::modelName() const {
    switch (_model) {
    case Model::TouchLcd7B: return "ESP32-S3-Touch-LCD-7B";
    case Model::CrowPanel7: return "CrowPanel-7.0-DIS08070H";
    default:                return "ESP32-S3-Touch-LCD-7";
    }
}

uint32_t Canvas::pixelClockHz() const {
    switch (_model) {
    case Model::TouchLcd7B: return PLANE_RADAR_RGB_7B_PCLK_HZ;
    case Model::CrowPanel7: return PLANE_RADAR_RGB_CROWPANEL_PCLK_HZ;
    default:                return PLANE_RADAR_RGB_PCLK_HZ;
    }
}

uint16_t Canvas::color565(uint8_t r, uint8_t g, uint8_t b) const {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Canvas::fillScreen(uint16_t color) {
    if (_fb == nullptr) return;
    std::fill(_fb, _fb + static_cast<size_t>(_width) * _height, color);
}

void Canvas::fillRect(int x, int y, int w, int h, uint16_t color) {
    if (_fb == nullptr || w <= 0 || h <= 0) return;
    int x0 = std::max(0, x);
    int y0 = std::max(0, y);
    int x1 = std::min(_width, x + w);
    int y1 = std::min(_height, y + h);
    if (x0 >= x1 || y0 >= y1) return;
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = _fb + static_cast<size_t>(yy) * _width;
        std::fill(row + x0, row + x1, color);
    }
}

void Canvas::drawPixel(int x, int y, uint16_t color) {
    if (_fb == nullptr || x < 0 || x >= _width || y < 0 || y >= _height) return;
    _fb[static_cast<size_t>(y) * _width + x] = color;
}

void Canvas::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        drawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Canvas::drawWideLine(int x0, int y0, int x1, int y1, float width, uint16_t color) {
    if (width <= 1.1f) {
        drawLine(x0, y0, x1, y1, color);
        return;
    }
    int radius = std::max(1, static_cast<int>(lroundf(width * 0.5f)));
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        fillCircle(x0, y0, radius, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Canvas::drawCircle(int x0, int y0, int r, uint16_t color) {
    if (r <= 0) return;
    int x = -r;
    int y = 0;
    int err = 2 - 2 * r;
    do {
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 - y, y0 - x, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        int e2 = err;
        if (e2 <= y) err += ++y * 2 + 1;
        if (e2 > x || err > y) err += ++x * 2 + 1;
    } while (x < 0);
}

void Canvas::fillCircle(int x0, int y0, int r, uint16_t color) {
    if (r <= 0) {
        drawPixel(x0, y0, color);
        return;
    }
    for (int y = -r; y <= r; y++) {
        int span = static_cast<int>(sqrtf(static_cast<float>(r * r - y * y)));
        fillRect(x0 - span, y0 + y, span * 2 + 1, 1, color);
    }
}

void Canvas::fillSmoothCircle(int x0, int y0, int r, uint16_t color) {
    fillCircle(x0, y0, r, color);
}

static int edgeValue(int ax, int ay, int bx, int by, int px, int py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

void Canvas::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
    int minX = std::max(0, std::min({x0, x1, x2}));
    int maxX = std::min(_width - 1, std::max({x0, x1, x2}));
    int minY = std::max(0, std::min({y0, y1, y2}));
    int maxY = std::min(_height - 1, std::max({y0, y1, y2}));
    int area = edgeValue(x0, y0, x1, y1, x2, y2);
    if (area == 0) return;
    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            int w0 = edgeValue(x1, y1, x2, y2, x, y);
            int w1 = edgeValue(x2, y2, x0, y0, x, y);
            int w2 = edgeValue(x0, y0, x1, y1, x, y);
            if ((area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                drawPixel(x, y, color);
            }
        }
    }
}

void Canvas::blitRGB565(int x, int y, int w, int h, const uint16_t *pixels, int stride) {
    if (_fb == nullptr || pixels == nullptr || w <= 0 || h <= 0 || stride < w) return;

    int srcX = 0;
    int srcY = 0;
    if (x < 0) {
        srcX = -x;
        w += x;
        x = 0;
    }
    if (y < 0) {
        srcY = -y;
        h += y;
        y = 0;
    }
    w = std::min(w, _width - x);
    h = std::min(h, _height - y);
    if (w <= 0 || h <= 0) return;

    for (int row = 0; row < h; row++) {
        const uint16_t *src = pixels + static_cast<size_t>(srcY + row) * stride + srcX;
        uint16_t *dst = _fb + static_cast<size_t>(y + row) * _width + x;
        memcpy(dst, src, static_cast<size_t>(w) * sizeof(uint16_t));
    }
}

void Canvas::blendAlphaMask4(
    int x,
    int y,
    int w,
    int h,
    const uint8_t *packedAlpha,
    uint16_t color
) {
    if (_fb == nullptr || packedAlpha == nullptr || w <= 0 || h <= 0) return;

    int sourceX = std::max(0, -x);
    int sourceY = std::max(0, -y);
    int destinationX = std::max(0, x);
    int destinationY = std::max(0, y);
    int drawWidth = std::min(w - sourceX, _width - destinationX);
    int drawHeight = std::min(h - sourceY, _height - destinationY);
    if (drawWidth <= 0 || drawHeight <= 0) return;

    uint16_t sourceRed = (color >> 11) & 0x1f;
    uint16_t sourceGreen = (color >> 5) & 0x3f;
    uint16_t sourceBlue = color & 0x1f;
    for (int row = 0; row < drawHeight; row++) {
        uint16_t *destination = _fb +
            static_cast<size_t>(destinationY + row) * _width + destinationX;
        size_t sourcePixel = static_cast<size_t>(sourceY + row) * w + sourceX;
        for (int column = 0; column < drawWidth; column++, sourcePixel++) {
            uint8_t packed = pgm_read_byte(packedAlpha + (sourcePixel >> 1));
            uint8_t alpha = (sourcePixel & 1) ? (packed & 0x0f) : (packed >> 4);
            if (alpha == 0) continue;
            if (alpha == 15) {
                destination[column] = color;
                continue;
            }

            uint16_t background = destination[column];
            uint16_t inverseAlpha = 15 - alpha;
            uint16_t red = (
                sourceRed * alpha + ((background >> 11) & 0x1f) * inverseAlpha + 7
            ) / 15;
            uint16_t green = (
                sourceGreen * alpha + ((background >> 5) & 0x3f) * inverseAlpha + 7
            ) / 15;
            uint16_t blue = (
                sourceBlue * alpha + (background & 0x1f) * inverseAlpha + 7
            ) / 15;
            destination[column] = static_cast<uint16_t>((red << 11) | (green << 5) | blue);
        }
    }
}

void Canvas::setTextSize(uint8_t size) {
    _textSize = std::max<uint8_t>(1, size);
}

void Canvas::setTextColor(uint16_t fg) {
    _textFg = fg;
}

void Canvas::setTextColor(uint16_t fg, uint16_t bg) {
    _textFg = fg;
    _textBg = bg;
}

void Canvas::setTextDatum(textdatum_t datum) {
    _datum = datum;
}

int Canvas::textWidth(const char *text) const {
    if (text == nullptr || text[0] == '\0') return 0;
    return static_cast<int>(strlen(text)) * FONT_ADVANCE * _textSize - _textSize;
}

int Canvas::textWidth(const String &text) const {
    return textWidth(text.c_str());
}

int Canvas::mediumTextWidth(const char *text) const {
    if (text == nullptr || text[0] == '\0') return 0;
    return static_cast<int>(strlen(text)) * MEDIUM_FONT_ADVANCE -
        (MEDIUM_FONT_ADVANCE - MEDIUM_FONT_W);
}

int Canvas::mediumTextWidth(const String &text) const {
    return mediumTextWidth(text.c_str());
}

void Canvas::drawChar(char ch, int x, int y) {
    const uint8_t *rows = glyphFor(ch);
    int s = _textSize;
    fillRect(x, y, FONT_W * s, FONT_H * s, _textBg);
    for (int yy = 0; yy < FONT_H; yy++) {
        for (int xx = 0; xx < FONT_W; xx++) {
            if (rows[yy] & (1 << (FONT_W - 1 - xx))) {
                fillRect(x + xx * s, y + yy * s, s, s, _textFg);
            }
        }
    }
}

void Canvas::drawString(const char *text, int x, int y) {
    if (text == nullptr) return;
    int w = textWidth(text);
    int h = FONT_H * _textSize;
    int startX = x;
    int startY = y;
    if (_datum == textdatum_t::top_right) {
        startX = x - w;
    } else if (_datum == textdatum_t::middle_center) {
        startX = x - w / 2;
        startY = y - h / 2;
    }
    for (size_t i = 0; text[i] != '\0'; i++) {
        drawChar(text[i], startX + static_cast<int>(i) * FONT_ADVANCE * _textSize, startY);
    }
}

void Canvas::drawString(const String &text, int x, int y) {
    int newline = text.indexOf('\n');
    if (newline >= 0) {
        int lineY = y;
        int start = 0;
        while (start <= static_cast<int>(text.length())) {
            int next = text.indexOf('\n', start);
            String line = (next >= 0) ? text.substring(start, next) : text.substring(start);
            drawString(line, x, lineY);
            if (next < 0) break;
            start = next + 1;
            lineY += LINE_ADVANCE * _textSize;
        }
        return;
    }
    drawString(text.c_str(), x, y);
}

void Canvas::drawMediumChar(char ch, int x, int y) {
    const uint8_t *rows = glyphFor(ch);
    fillRect(x, y, MEDIUM_FONT_W, MEDIUM_FONT_H, _textBg);

    int destinationY = y;
    for (int sourceY = 0; sourceY < FONT_H; sourceY++) {
        int destinationX = x;
        for (int sourceX = 0; sourceX < FONT_W; sourceX++) {
            if (rows[sourceY] & (1 << (FONT_W - 1 - sourceX))) {
                fillRect(
                    destinationX,
                    destinationY,
                    MEDIUM_COLUMN_WIDTHS[sourceX],
                    MEDIUM_ROW_HEIGHTS[sourceY],
                    _textFg
                );
            }
            destinationX += MEDIUM_COLUMN_WIDTHS[sourceX];
        }
        destinationY += MEDIUM_ROW_HEIGHTS[sourceY];
    }
}

void Canvas::drawMediumString(const char *text, int x, int y) {
    if (text == nullptr) return;
    int width = mediumTextWidth(text);
    int startX = x;
    int startY = y;
    if (_datum == textdatum_t::top_right) {
        startX = x - width;
    } else if (_datum == textdatum_t::middle_center) {
        startX = x - width / 2;
        startY = y - MEDIUM_FONT_H / 2;
    }
    for (size_t i = 0; text[i] != '\0'; i++) {
        drawMediumChar(
            text[i],
            startX + static_cast<int>(i) * MEDIUM_FONT_ADVANCE,
            startY
        );
    }
}

void Canvas::drawMediumString(const String &text, int x, int y) {
    drawMediumString(text.c_str(), x, y);
}

} // namespace PanelDisplay
