#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$PROJECT_DIR/.." && pwd)"

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

case "$DEFAULT_MAP_PROVIDER" in
  none|0) DEFAULT_MAP_PROVIDER_CODE=0 ;;
  stadia|1) DEFAULT_MAP_PROVIDER_CODE=1 ;;
  *)
    echo "DEFAULT_MAP_PROVIDER must be 'none' or 'stadia'." >&2
    exit 1
    ;;
esac

FQBN="esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=default,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=info,PSRAM=opi,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default"

c_define_string() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  printf '"%s"' "$value"
}

COMMON_FLAGS="-I$PROJECT_DIR -I$PROJECT_DIR/src -DPNG_MAX_BUFFERED_PIXELS=8322"
CPP_FLAGS="$COMMON_FLAGS"
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

if [[ "$CLEAN" == "1" ]]; then
  args+=(--clean)
fi

if [[ "$UPLOAD" == "1" ]]; then
  args+=(--upload -p "$PORT")
fi

args+=("$PROJECT_DIR")

arduino-cli "${args[@]}"
