# pi-feed — local ADS-B source for Big Plane Radar

Serves a Raspberry Pi's own ADS-B reception to the radar instead of the public
API, in the exact shape the firmware already parses.

## Why bother

| | public API | local receiver |
|---|---|---|
| Position updates | one fix per aircraft every **15–20 s** (measured) | **~1 Hz** |
| Transport | HTTPS — the largest transient allocation the firmware makes | plain HTTP |
| Coverage | 1090 MHz only | 1090 MHz **and** 978 MHz UAT |
| Internet required | yes | no |

The update rate is the important one. The firmware dead-reckons between fixes
and eases the correction over 600 ms; with a local receiver there is barely
anything left to correct. Dropping TLS also frees the internal DRAM that
currently limits `PLANE_RADAR_RGB_BOUNCE_LINES` to 20.

## Hardware

- Raspberry Pi 4 or 5. A Zero 2 W will not comfortably run two SDRs.
- 5.1 V 3 A supply. Undervoltage shows up as dropped USB devices, not as a
  warning you will notice.
- 8 GB+ card.
- Your two receivers: R820T2 + 1090 MHz filter for ADS-B, R860 for 978 UAT.
- **Two antennas.** This matters more than anything else here — a modest
  receiver with an outdoor antenna and clear sky view beats a good one indoors.
  Quarter-wave is ~6.9 cm for 1090 MHz and ~7.7 cm for 978 MHz, so they are not
  interchangeable.

## Quick start

Flash Raspberry Pi OS Lite (64-bit), enable SSH, then:

```sh
git clone https://github.com/k4m454k/big_plane_radar
cd big_plane_radar/pi-feed

# one dongle connected at a time -- see "Serialising" below for why
./bootstrap.sh serialise 1090     # ONLY the 1090 MHz receiver connected
./bootstrap.sh serialise 978      # ONLY the 978 MHz receiver connected

# both reconnected
./bootstrap.sh install --lat 38.677024 --lon -90.506763 --alt 160
```

That handles the DVB-T blacklist, SDR tools, Docker, the receiver stack, the
`radar-feed` service, and verification. It is idempotent — re-run it after
changing position or fixing a failure. `./bootstrap.sh status` reports on
receivers, containers, the service and both endpoints.

Add `--no-978` for a 1090-only start; re-run without it when the second antenna
is up.

The rest of this document is what the script does, for when something needs
fixing by hand.

## 1. Base OS

Raspberry Pi OS Lite (64-bit). Enable SSH, set a hostname you will remember —
the radar will point at it.

## 2. Stop the kernel claiming the dongles

The DVB-T driver binds RTL2832U devices on sight and SDR software then finds
nothing:

```sh
sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf <<'EOF'
blacklist dvb_usb_rtl28xxu
blacklist rtl2832
blacklist rtl2830
EOF
sudo reboot
```

## 3. Serialising the dongles

**Do this before anything else, and with only one dongle plugged in at a time.**
Two identical RTL-SDRs cannot be told apart by USB path reliably across reboots,
so each container must select by serial. This is the step people skip and then
spend an evening debugging why 978 is decoding 1090.

```sh
sudo apt install -y rtl-sdr
# with ONLY the 1090 unit connected:
rtl_eeprom -d 0 -s 00001090
# power-cycle it, then with ONLY the 978 unit connected:
rtl_eeprom -d 0 -s 00000978
```

Confirm both are visible afterwards:

```sh
rtl_test -t     # should list two devices with those serials
```

## 4. Docker

```sh
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
newgrp docker
```

## 5. Receiver stack

`docker-adsb-ultrafeeder` runs readsb, tar1090 and every aggregator feeder in
one container. Put this in `~/adsb/docker-compose.yml`, substituting your
position (decimal degrees) and antenna altitude in metres:

```yaml
services:
  ultrafeeder:
    image: ghcr.io/sdr-enthusiasts/docker-adsb-ultrafeeder:latest
    container_name: ultrafeeder
    hostname: ultrafeeder
    restart: unless-stopped
    device_cgroup_rules:
      - 'c 189:* rwm'
    ports:
      - 8080:80          # tar1090 map + /data/aircraft.json
    environment:
      - TZ=America/Chicago
      - READSB_LAT=38.677024
      - READSB_LON=-90.506763
      - READSB_ALT=160m
      - READSB_RTLSDR_DEVICE=00001090
      - READSB_DEVICE_TYPE=rtlsdr
      - READSB_GAIN=autogain
      - READSB_NET_ENABLE=true
      # 978 UAT arrives here from the dump978 container below
      - READSB_NET_CONNECTOR=dump978,30978,raw_in
    volumes:
      - ./globe_history:/var/globe_history
      - /proc/diskstats:/proc/diskstats:ro
    devices:
      - /dev/bus/usb:/dev/bus/usb
    tmpfs:
      - /run:exec,size=64M
      - /var/log

  dump978:
    image: ghcr.io/sdr-enthusiasts/docker-dump978:latest
    container_name: dump978
    hostname: dump978
    restart: unless-stopped
    device_cgroup_rules:
      - 'c 189:* rwm'
    environment:
      - TZ=America/Chicago
      - LAT=38.677024
      - LON=-90.506763
      - DUMP978_RTLSDR_DEVICE=00000978
    volumes:
      - /proc/diskstats:/proc/diskstats:ro
    devices:
      - /dev/bus/usb:/dev/bus/usb
    tmpfs:
      - /run:exec,size=64M
```

```sh
cd ~/adsb && docker compose up -d
docker compose logs -f      # watch for decoded messages
```

Then open `http://<pi>:8080/` for the tar1090 map, and check the file the radar
will actually read:

```sh
curl -s http://localhost:8080/data/aircraft.json | head -c 400
```

## 6. Aggregator feeding (optional, free subscriptions)

One receiver can feed several networks at once, and most give contributors a
paid tier free — Flightradar24 Business, FlightAware Enterprise. Register for
keys, then add the relevant `ULTRAFEEDER_CONFIG` entries per the
[ultrafeeder docs](https://github.com/sdr-enthusiasts/docker-adsb-ultrafeeder).
This is entirely optional; the radar does not care.

## 7. pi-feed

`bootstrap.sh install` does all of this; the manual equivalent is:

`readsb` serves *everything the antennas hear*, unsorted. The firmware caps at
`MAX_AIRCRAFT` (64), so handing it a raw busy feed would silently drop nearby
traffic in favour of whatever happened to be listed first. `radar_feed.py`
filters by radius and picks the nearest N.

```sh
sudo mkdir -p /opt/pi-feed
sudo cp radar_feed.py /opt/pi-feed/
sudo cp radar-feed.service /etc/systemd/system/
sudo systemctl enable --now radar-feed
systemctl status radar-feed
```

Check it:

```sh
curl -s http://localhost:8081/health
curl -s "http://localhost:8081/api/v3/lat/38.677024/lon/-90.506763/dist/29.2" | head -c 300
```

Standard library only — no `pip install`, nothing to break on OS upgrades.

### Testing before the receiver exists

`RADAR_FEED_SOURCE=upstream` proxies the public API instead of readsb, so the
whole path can be verified before any hardware arrives:

```sh
RADAR_FEED_SOURCE=upstream ./radar_feed.py --port 8081
```

## 8. Point the radar at it

Build the firmware with the Pi's address baked in and the display needs nothing
but Wi-Fi credentials:

```sh
DEFAULT_FEED_HOST=<pi>:8081 DISPLAY_PROFILE=5 ./build_arduino_cli.sh
```

Otherwise, open the device's web portal and set **Local ADS-B feed host** to
`<pi>:8081`. The host and the enable flag are stored separately, so you can
switch back to `PUBLIC` on-device without retyping the address — useful if the
Pi is down or you take the display elsewhere.

### The Pi owns the settings

The display stores no position. On boot it fetches `/config`, which reports the
receiver's own coordinates straight out of readsb's `receiver.json`:

```sh
curl -s http://localhost:8081/config
```

```json
{"lat":38.677024,"lon":-90.506763,"range_index":1,"runways":true,
 "position_known":true,"position_source":"receiver"}
```

That is what makes a panel disposable. Reflash it, swap it for a different
board, or add a second one, and it centres itself with no input — where
previously an empty NVS meant the display silently fell back to a compiled-in
position and drew a sky nobody was standing under. When the position is genuinely
unknown the display says `NO SITE POSITION` on screen rather than guessing.

Settings changed on a panel's settings screen are POSTed back here and persisted
to `/var/lib/radar-feed/config.json`, so they outlive the board and reach every
other panel. Position is the exception: a receiver that publishes its own
location cannot be moved by a display, so one bad request can't relocate the
site. If readsb has no location set, `POST /config` with `lat`/`lon` (or set
`RADAR_FEED_LAT`/`RADAR_FEED_LON`) supplies one.

Range is deliberately per-display: two panels on one receiver can sit at
different zooms, and the Pi's `range_index` is only the default a panel uses
until someone changes it on that panel.

## Troubleshooting

| Symptom | Cause |
|---|---|
| No devices found | DVB-T driver still bound — check step 2 and reboot |
| One dongle works, the other decodes nothing | Serials not set, or both containers grabbed the same device |
| Few aircraft, all close | Antenna. Almost always the antenna |
| `radar-feed` 503 | readsb not up, or `RADAR_FEED_SOURCE` points at the wrong URL |
| Radar shows nothing on LOCAL | Firewall on the Pi, or wrong port in the portal — `curl` the URL from another machine first |
