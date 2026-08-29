#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$PROJECT_DIR/.." && pwd)"
ARDUINO_CLI_BIN="${ARDUINO_CLI_BIN:-arduino-cli}"
ARDUINO_CLI_CONFIG_FILE="${ARDUINO_CLI_CONFIG_FILE:-}"

if [[ -n "${WAVESHARE_LIB_DIR:-}" ]]; then
  :
elif [[ -d "$PROJECT_DIR/vendor/waveshare-libraries" ]]; then
  WAVESHARE_LIB_DIR="$PROJECT_DIR/vendor/waveshare-libraries"
elif [[ -d "$ROOT_DIR/ESP32-S3-Touch-LCD-7-Demo/Arduino/libraries" ]]; then
  WAVESHARE_LIB_DIR="$ROOT_DIR/ESP32-S3-Touch-LCD-7-Demo/Arduino/libraries"
else
  echo "Waveshare Arduino libraries not found." >&2
  echo "Set WAVESHARE_LIB_DIR=/path/to/ESP32-S3-Touch-LCD-7-Demo/Arduino/libraries" >&2
  exit 1
fi

PORT="${PORT:-/dev/cu.usbmodem5AE71132621}"
UPLOAD="${UPLOAD:-0}"
CLEAN="${CLEAN:-0}"
BUILD_PATH="${BUILD_PATH:-$PROJECT_DIR/build/arduino}"
DEFAULT_WIFI_SSID="${DEFAULT_WIFI_SSID:-}"
DEFAULT_WIFI_PASSWORD="${DEFAULT_WIFI_PASSWORD:-}"
DEFAULT_LAT="${DEFAULT_LAT:-51.507400}"
DEFAULT_LON="${DEFAULT_LON:--0.127800}"
DEFAULT_MAP_PROVIDER="${DEFAULT_MAP_PROVIDER:-none}"
DEFAULT_STADIA_API_KEY="${DEFAULT_STADIA_API_KEY:-}"
LOG_LEVEL="${LOG_LEVEL:-info}"
RGB_BOUNCE_LINES="${RGB_BOUNCE_LINES:-10}"
REQUIRE_HIGH_PERF="${REQUIRE_HIGH_PERF:-0}"

case "$RGB_BOUNCE_LINES" in
  10|20) ;;
  *)
    echo "RGB_BOUNCE_LINES must be 10 or 20." >&2
    exit 1
    ;;
esac

case "$REQUIRE_HIGH_PERF" in
  0|1) ;;
  *)
    echo "REQUIRE_HIGH_PERF must be 0 or 1." >&2
    exit 1
    ;;
esac

case "$LOG_LEVEL" in
  off|0)
    APP_LOG_LEVEL=0
    CORE_DEBUG_LEVEL=none
    ;;
  error|1)
    APP_LOG_LEVEL=1
    CORE_DEBUG_LEVEL=error
    ;;
  info|2)
    APP_LOG_LEVEL=2
    CORE_DEBUG_LEVEL=error
    ;;
  debug|3)
    APP_LOG_LEVEL=3
    CORE_DEBUG_LEVEL=debug
    ;;
  *)
    echo "LOG_LEVEL must be 'off', 'error', 'info', or 'debug'." >&2
    exit 1
    ;;
esac

case "$DEFAULT_MAP_PROVIDER" in
  none|0) DEFAULT_MAP_PROVIDER_CODE=0 ;;
  stadia|1) DEFAULT_MAP_PROVIDER_CODE=1 ;;
  *)
    echo "DEFAULT_MAP_PROVIDER must be 'none' or 'stadia'." >&2
    exit 1
    ;;
esac

FQBN="esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=default,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=$CORE_DEBUG_LEVEL,PSRAM=opi,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default"

c_define_string() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  printf '"%s"' "$value"
}

COMMON_FLAGS="-I$PROJECT_DIR -I$PROJECT_DIR/src -DPNG_MAX_BUFFERED_PIXELS=8322"
CPP_FLAGS="$COMMON_FLAGS"
CPP_FLAGS+=" -DPLANE_RADAR_LOG_LEVEL=$APP_LOG_LEVEL"
CPP_FLAGS+=" -DPLANE_RADAR_RGB_BOUNCE_LINES=$RGB_BOUNCE_LINES"
CPP_FLAGS+=" -DPLANE_RADAR_REQUIRE_HIGH_PERF=$REQUIRE_HIGH_PERF"
CPP_FLAGS+=" -DDEFAULT_WIFI_SSID=$(c_define_string "$DEFAULT_WIFI_SSID")"
CPP_FLAGS+=" -DDEFAULT_WIFI_PASSWORD=$(c_define_string "$DEFAULT_WIFI_PASSWORD")"
CPP_FLAGS+=" -DDEFAULT_LAT=$DEFAULT_LAT"
CPP_FLAGS+=" -DDEFAULT_LON=$DEFAULT_LON"
CPP_FLAGS+=" -DDEFAULT_MAP_PROVIDER=$DEFAULT_MAP_PROVIDER_CODE"
CPP_FLAGS+=" -DDEFAULT_STADIA_API_KEY=$(c_define_string "$DEFAULT_STADIA_API_KEY")"

mkdir -p "$BUILD_PATH"

args=(
  compile
  -b "$FQBN"
  --libraries "$WAVESHARE_LIB_DIR"
  --libraries "$PROJECT_DIR/lib"
  --build-path "$BUILD_PATH"
  --build-property "compiler.cpp.extra_flags=$CPP_FLAGS"
  --build-property "compiler.c.extra_flags=$COMMON_FLAGS"
)

cli=("$ARDUINO_CLI_BIN")
if [[ -n "$ARDUINO_CLI_CONFIG_FILE" ]]; then
  cli+=(--config-file "$ARDUINO_CLI_CONFIG_FILE")
fi

if [[ "$CLEAN" == "1" ]]; then
  args+=(--clean)
fi

if [[ "$UPLOAD" == "1" ]]; then
  args+=(--upload -p "$PORT")
fi

args+=("$PROJECT_DIR")

"${cli[@]}" "${args[@]}"
