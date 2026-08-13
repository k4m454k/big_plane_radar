#!/usr/bin/env bash
# Bootstrap a Raspberry Pi into a dual-band ADS-B receiver serving Big Plane Radar.
#
# Idempotent: safe to re-run after fixing something or changing position. Each
# phase checks whether it is already satisfied and says so rather than redoing
# work.
#
# The one thing that cannot be automated is telling two identical RTL-SDR
# dongles apart, because that requires knowing which physical stick is which.
# The script detects the situation and walks you through it rather than
# guessing, since mis-serialising means 978 quietly decodes 1090.
#
#   ./bootstrap.sh serialise 1090     # with ONLY the 1090 receiver connected
#   ./bootstrap.sh serialise 978      # with ONLY the 978 receiver connected
#   ./bootstrap.sh install --lat 38.677024 --lon -90.506763 --alt 160
#   ./bootstrap.sh status
set -euo pipefail

SERIAL_1090="00001090"
SERIAL_978="00000978"
ADSB_DIR="${ADSB_DIR:-$HOME/adsb}"
FEED_DIR=/opt/pi-feed
FEED_PORT="${FEED_PORT:-8081}"
FEED_LIMIT="${FEED_LIMIT:-64}"
TAR1090_PORT="${TAR1090_PORT:-8080}"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

HAVE_1090=0; HAVE_978=0
LAT=""; LON=""; ALT="160"; TZONE="$(cat /etc/timezone 2>/dev/null || echo UTC)"
SKIP_978=0

c_ok()   { printf '\033[32m  ok\033[0m   %s\n' "$*"; }
c_do()   { printf '\033[36m  ..\033[0m   %s\n' "$*"; }
c_warn() { printf '\033[33m  !!\033[0m   %s\n' "$*"; }
c_err()  { printf '\033[31m  xx\033[0m   %s\n' "$*" >&2; }
phase()  { printf '\n\033[1m%s\033[0m\n' "$*"; }

need_root() {
  if [[ $EUID -ne 0 ]] && ! sudo -n true 2>/dev/null; then
    c_warn "sudo password will be requested"
  fi
}

# ---------------------------------------------------------------- dongles ----

rtl_serials() {
  # rtl_test writes its device list to stderr, one line per device ending in
  # "SN: <serial>".
  rtl_test -t 2>&1 | sed -n 's/.*SN: *\([^ ,]*\).*/\1/p' || true
}

rtl_count() { rtl_serials | grep -c . || true; }

# Match on band rather than an exact string. Sticks sold pre-serialised for
# ADS-B (ADSBexchange ships "1090" and "978") are already distinguishable, and
# demanding an exact serial would send the user through a serialise step they do
# not need. Anything containing the band number counts.
detect_serials() {
  local sns; sns="$(rtl_serials)"
  local found_1090 found_978
  found_1090="$(printf '%s\n' "$sns" | grep -m1 '1090' || true)"
  found_978="$(printf '%s\n' "$sns" | grep '978' | grep -v '1090' | head -1 || true)"
  [[ -n "$found_1090" ]] && SERIAL_1090="$found_1090"
  [[ -n "$found_978" ]] && SERIAL_978="$found_978"
  [[ -n "$found_1090" ]] && HAVE_1090=1 || HAVE_1090=0
  [[ -n "$found_978" ]] && HAVE_978=1 || HAVE_978=0
}

cmd_serialise() {
  local which="${1:-}" serial
  case "$which" in
    1090) serial="00001090" ;;
    978)  serial="00000978" ;;
    *) c_err "usage: $0 serialise <1090|978>"; exit 2 ;;
  esac

  command -v rtl_eeprom >/dev/null || { c_do "installing rtl-sdr"; sudo apt-get install -y rtl-sdr >/dev/null; }

  local n; n="$(rtl_count)"
  if [[ "$n" -eq 0 ]]; then
    c_err "no RTL-SDR found. Connect ONLY the $which receiver and retry."
    exit 1
  fi
  if [[ "$n" -gt 1 ]]; then
    c_err "$n RTL-SDRs connected. Serialising writes to device 0 and cannot tell"
    c_err "them apart -- disconnect all but the $which receiver and retry."
    exit 1
  fi

  c_do "writing serial $serial to the connected device"
  # rtl_eeprom prompts for confirmation on stdin.
  echo y | sudo rtl_eeprom -d 0 -s "$serial"
  c_ok "written. Unplug and reconnect it for the change to take effect."
}

check_serials() {
  local n; n="$(rtl_count)"
  detect_serials

  if [[ $HAVE_1090 -eq 1 && ( $HAVE_978 -eq 1 || $SKIP_978 -eq 1 ) ]]; then
    c_ok "1090 MHz receiver: serial '$SERIAL_1090'"
    [[ $SKIP_978 -eq 0 ]] && c_ok "978 MHz receiver:  serial '$SERIAL_978'"
    return 0
  fi

  c_warn "receivers are not serialised yet (found $n device(s))"
  rtl_serials | sed 's/^/         /'
  cat <<EOF

  Two identical dongles cannot be told apart by USB path across reboots, so
  each must carry its own serial. Do this one stick at a time:

      # connect ONLY the 1090 MHz receiver (the one with the filter)
      $0 serialise 1090
      # power-cycle it, then connect ONLY the 978 MHz receiver
      $0 serialise 978
      # reconnect both, then re-run:
      $0 install --lat <lat> --lon <lon>

EOF
  exit 1
}

# ------------------------------------------------------------------ phases ---

phase_blacklist() {
  phase "1. DVB-T driver"
  local f=/etc/modprobe.d/blacklist-rtlsdr.conf
  if [[ -f $f ]] && grep -q dvb_usb_rtl28xxu "$f"; then
    c_ok "already blacklisted"
  else
    c_do "blacklisting dvb_usb_rtl28xxu"
    sudo tee "$f" >/dev/null <<'EOF'
blacklist dvb_usb_rtl28xxu
blacklist rtl2832
blacklist rtl2830
EOF
    c_ok "written to $f"
  fi
  if lsmod | grep -q dvb_usb_rtl28xxu; then
    c_do "unloading the running module"
    sudo modprobe -r dvb_usb_rtl28xxu 2>/dev/null || \
      c_warn "could not unload; a reboot will clear it"
  fi
}

phase_tools() {
  phase "2. SDR tools"
  if command -v rtl_test >/dev/null; then
    c_ok "rtl-sdr present"
  else
    c_do "installing rtl-sdr"
    sudo apt-get update -qq
    sudo apt-get install -y rtl-sdr >/dev/null
    c_ok "installed"
  fi
}

phase_docker() {
  phase "3. Docker"
  if command -v docker >/dev/null; then
    c_ok "docker present"
  else
    c_do "installing docker (this takes a few minutes)"
    curl -fsSL https://get.docker.com | sudo sh >/dev/null
    sudo usermod -aG docker "$USER"
    c_ok "installed -- you may need to log out and back in for group membership"
  fi
  docker compose version >/dev/null 2>&1 || {
    c_err "docker compose plugin missing; install docker-compose-plugin"; exit 1; }
}

phase_stack() {
  phase "4. Receiver stack"
  mkdir -p "$ADSB_DIR"
  local f="$ADSB_DIR/docker-compose.yml"

  local dump978_service="" connector=""
  if [[ $SKIP_978 -eq 0 ]]; then
    connector='      - READSB_NET_CONNECTOR=dump978,30978,raw_in'
    dump978_service="
  dump978:
    image: ghcr.io/sdr-enthusiasts/docker-dump978:latest
    container_name: dump978
    hostname: dump978
    restart: unless-stopped
    device_cgroup_rules:
      - 'c 189:* rwm'
    environment:
      - TZ=$TZONE
      - LAT=$LAT
      - LON=$LON
      - DUMP978_RTLSDR_DEVICE=$SERIAL_978
    volumes:
      - /proc/diskstats:/proc/diskstats:ro
    devices:
      - /dev/bus/usb:/dev/bus/usb
    tmpfs:
      - /run:exec,size=64M"
  fi

  c_do "writing $f"
  cat > "$f" <<EOF
# Generated by pi-feed/bootstrap.sh -- re-run it to regenerate.
services:
  ultrafeeder:
    image: ghcr.io/sdr-enthusiasts/docker-adsb-ultrafeeder:latest
    container_name: ultrafeeder
    hostname: ultrafeeder
    restart: unless-stopped
    device_cgroup_rules:
      - 'c 189:* rwm'
    ports:
      - $TAR1090_PORT:80
    environment:
      - TZ=$TZONE
      - READSB_LAT=$LAT
      - READSB_LON=$LON
      - READSB_ALT=${ALT}m
      - READSB_DEVICE_TYPE=rtlsdr
      - READSB_RTLSDR_DEVICE=$SERIAL_1090
      - READSB_GAIN=autogain
      - READSB_NET_ENABLE=true
$connector
    volumes:
      - ./globe_history:/var/globe_history
      - /proc/diskstats:/proc/diskstats:ro
    devices:
      - /dev/bus/usb:/dev/bus/usb
    tmpfs:
      - /run:exec,size=64M
      - /var/log
$dump978_service
EOF

  c_do "starting containers"
  (cd "$ADSB_DIR" && docker compose up -d)
  c_ok "stack up -- map at http://$(hostname -I | awk '{print $1}'):$TAR1090_PORT/"
}

phase_feed() {
  phase "5. pi-feed"
  sudo mkdir -p "$FEED_DIR"
  sudo install -m 0755 "$SRC_DIR/radar_feed.py" "$FEED_DIR/radar_feed.py"

  # Render the unit with this run's ports rather than shipping defaults that
  # silently disagree with the compose file.
  sudo tee /etc/systemd/system/radar-feed.service >/dev/null <<EOF
[Unit]
Description=Big Plane Radar local ADS-B feed adapter
After=network-online.target docker.service
Wants=network-online.target

[Service]
Type=simple
Environment=RADAR_FEED_SOURCE=http://127.0.0.1:$TAR1090_PORT/data/aircraft.json
Environment=RADAR_FEED_PORT=$FEED_PORT
Environment=RADAR_FEED_LIMIT=$FEED_LIMIT
ExecStart=/usr/bin/python3 $FEED_DIR/radar_feed.py
Restart=always
RestartSec=5
DynamicUser=yes
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes
RestrictAddressFamilies=AF_INET AF_INET6
MemoryMax=128M

[Install]
WantedBy=multi-user.target
EOF

  sudo systemctl daemon-reload
  sudo systemctl enable --now radar-feed >/dev/null
  sudo systemctl restart radar-feed
  c_ok "radar-feed running on port $FEED_PORT"
}

phase_verify() {
  phase "6. Verify"
  local ip; ip="$(hostname -I | awk '{print $1}')"
  local ok=1

  c_do "waiting for readsb to produce aircraft.json"
  for _ in $(seq 1 30); do
    curl -fsS -m 3 "http://127.0.0.1:$TAR1090_PORT/data/aircraft.json" >/dev/null 2>&1 && break
    sleep 2
  done

  if curl -fsS -m 5 "http://127.0.0.1:$TAR1090_PORT/data/aircraft.json" -o /tmp/ac.json 2>/dev/null; then
    local n; n="$(grep -o '"hex"' /tmp/ac.json | wc -l | tr -d ' ')"
    c_ok "readsb serving aircraft.json ($n aircraft seen)"
    [[ "$n" -eq 0 ]] && c_warn "zero aircraft -- almost always the antenna, not the software"
  else
    c_err "readsb is not serving aircraft.json; check: docker compose logs"
    ok=0
  fi

  if curl -fsS -m 5 "http://127.0.0.1:$FEED_PORT/health" -o /tmp/h.json 2>/dev/null; then
    c_ok "radar-feed healthy: $(tr -d '\n ' < /tmp/h.json)"
  else
    c_err "radar-feed not responding; check: journalctl -u radar-feed -n 50"
    ok=0
  fi

  printf '\n'
  if [[ $ok -eq 1 ]]; then
    printf '\033[1mNext:\033[0m open the radar web portal, set\n'
    printf '  Local ADS-B feed host = \033[1m%s:%s\033[0m\n' "$ip" "$FEED_PORT"
    printf 'then on the device: long-press -> ADS-B SOURCE -> LOCAL\n\n'
  else
    c_err "finish the failures above, then re-run: $0 status"
    exit 1
  fi
}

cmd_status() {
  phase "Receivers"; rtl_serials | sed 's/^/         /' || c_warn "none found"
  phase "Containers"; docker ps --format '  {{.Names}}\t{{.Status}}' 2>/dev/null || c_warn "docker unavailable"
  phase "Services"
  systemctl is-active --quiet radar-feed && c_ok "radar-feed active" || c_err "radar-feed inactive"
  phase "Endpoints"
  curl -fsS -m 5 "http://127.0.0.1:$TAR1090_PORT/data/aircraft.json" >/dev/null 2>&1 \
    && c_ok "readsb aircraft.json" || c_err "readsb aircraft.json unreachable"
  curl -fsS -m 5 "http://127.0.0.1:$FEED_PORT/health" 2>/dev/null | sed 's/^/         /' \
    || c_err "radar-feed /health unreachable"
}

cmd_install() {
  [[ -n "$LAT" && -n "$LON" ]] || { c_err "--lat and --lon are required"; exit 2; }
  need_root
  phase_blacklist
  phase_tools
  check_serials
  phase_docker
  phase_stack
  phase_feed
  phase_verify
}

usage() {
  # Print the header comment only -- stop at the first line that is not one,
  # rather than trusting a fixed line range to stay accurate as this grows.
  awk 'NR==1 {next} /^#/ {sub(/^# ?/, ""); print; next} {exit}' "${BASH_SOURCE[0]}"
}

main() {
  local cmd="${1:-}"; shift || true
  case "$cmd" in
    serialise|serialize) cmd_serialise "${1:-}" ;;
    status) cmd_status ;;
    install)
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --lat) LAT="$2"; shift 2 ;;
          --lon) LON="$2"; shift 2 ;;
          --alt) ALT="$2"; shift 2 ;;
          --port) FEED_PORT="$2"; shift 2 ;;
          --tar1090-port) TAR1090_PORT="$2"; shift 2 ;;
          --no-978) SKIP_978=1; shift ;;
          *) c_err "unknown option: $1"; exit 2 ;;
        esac
      done
      cmd_install
      ;;
    ""|-h|--help|help) usage ;;
    *) c_err "unknown command: $cmd"; usage; exit 2 ;;
  esac
}

main "$@"
