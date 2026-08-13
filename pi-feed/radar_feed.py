#!/usr/bin/env python3
"""Serve a local readsb/dump1090 feed in the shape Big Plane Radar already parses.

The firmware polls opendata.adsb.fi's v3 endpoint, which returns aircraft that
are already filtered to a radius and small enough to parse on an ESP32. A local
readsb instance serves everything its antennas hear, unsorted, which is both far
larger and in a slightly different envelope. This bridges the two.

The request path deliberately mirrors the upstream API exactly:

    /api/v3/lat/<lat>/lon/<lon>/dist/<nm>

so the only firmware change is the scheme and host. Pointing the device back at
the public API remains a working fallback.

Standard library only, so it runs on a stock Raspberry Pi OS image with no pip
install. Reads either a URL or a file path, so it works against readsb over HTTP,
against a tmpfs file directly, or against the public API for testing before any
receiver hardware exists.

Usage:
    RADAR_FEED_SOURCE=http://127.0.0.1/data/aircraft.json ./radar_feed.py
    RADAR_FEED_SOURCE=/run/readsb/aircraft.json ./radar_feed.py --port 8080
    RADAR_FEED_SOURCE=upstream ./radar_feed.py        # proxy the public API
"""

import argparse
import json
import math
import os
import ssl
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Fields the firmware's ArduinoJson filter asks for. Anything else is dropped so
# the response stays small enough to parse comfortably on the device.
PASSTHROUGH_FIELDS = (
    "lat", "lon", "track", "true_heading", "mag_heading", "dir",
    "gs", "tas", "ias", "baro_rate", "geom_rate", "flight", "hex",
    "t", "category", "squawk", "alt_baro", "alt_geom",
    # Age of the position fix in seconds. Without it the firmware treats every
    # fix as current and dead-reckons forward from a position that is already
    # stale, so it runs permanently ahead and each new fix drags it back.
    "seen_pos",
)

UPSTREAM_TEMPLATE = "https://opendata.adsb.fi/api/v3/lat/{lat}/lon/{lon}/dist/{dist}"
ROUTE_TEMPLATE = "https://api.adsbdb.com/v0/callsign/{callsign}"
TILE_TEMPLATE = ("https://tiles-eu.stadiamaps.com/tiles/alidade_smooth_dark/"
                 "{z}/{x}/{y}.png?api_key={key}")

# Routes and tiles are both effectively immutable: a flight number's endpoints do
# not change, and a map tile at a given z/x/y is fixed. Caching them here means
# the display never opens a TLS connection -- the single largest transient
# allocation it makes, and what caps its RGB bounce buffers.
ROUTE_TTL_S = 7 * 24 * 3600
CACHE_DIR = os.environ.get("RADAR_FEED_CACHE", "/var/cache/radar-feed")

KM_PER_DEG = 111.195
KM_PER_NM = 1.852

# readsb rewrites aircraft.json about once a second; re-reading faster than that
# only burns I/O. Several devices polling at 5 s share one cached read.
CACHE_TTL_S = 0.8

# Drop positions older than this. Local reception updates about once a second,
# so anything older is an aircraft going out of range. Kept well below the
# firmware's extrapolation cap: stale age plus the poll interval must stay
# inside it, or the display freezes the aircraft until the next fix.
STALE_POSITION_S = 5.0


def urlopen(url, timeout):
    """Fetch a URL, tolerating hosts with no usable CA bundle.

    The real deployment talks plain HTTP to readsb on localhost, so this only
    matters for the `upstream` testing mode. Some Python installs (notably
    python.org builds on macOS) ship without a CA store and fail every HTTPS
    request; fall back to an unverified context rather than making the fallback
    data path unusable there.
    """
    # opendata.adsb.fi rejects urllib's default agent with 403.
    req = urllib.request.Request(url, headers={"User-Agent": "big-plane-radar-feed/1.0"})
    try:
        return urllib.request.urlopen(req, timeout=timeout)
    except urllib.error.URLError as exc:
        if not isinstance(getattr(exc, "reason", None), ssl.SSLError):
            raise
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        return urllib.request.urlopen(req, timeout=timeout, context=ctx)


def cache_path(kind, name):
    d = os.path.join(CACHE_DIR, kind)
    os.makedirs(d, exist_ok=True)
    safe = "".join(c for c in name if c.isalnum() or c in "-_.")
    return os.path.join(d, safe)


class FeedCache:
    """Reads the upstream aircraft list at most once per CACHE_TTL_S."""

    def __init__(self, source):
        self.source = source
        self._lock = threading.Lock()
        self._at = 0.0
        self._aircraft = []
        self._error = None
        self._reads = 0

    def _load(self):
        if self.source.startswith(("http://", "https://")):
            with urlopen(self.source, timeout=4) as r:
                return json.loads(r.read().decode("utf-8", "replace"))
        with open(self.source, "r", encoding="utf-8", errors="replace") as f:
            return json.load(f)

    def aircraft(self):
        """Returns (aircraft, error, age_of_this_read_in_seconds)."""
        with self._lock:
            if time.time() - self._at < CACHE_TTL_S:
                return self._aircraft, self._error, time.time() - self._at
            try:
                doc = self._load()
                # readsb/dump1090 use "aircraft"; adsb.fi uses "ac".
                self._aircraft = doc.get("aircraft") or doc.get("ac") or []
                self._error = None
                self._reads += 1
            except Exception as exc:  # noqa: BLE001 - report, never crash the server
                self._error = f"{type(exc).__name__}: {exc}"
            self._at = time.time()
            return self._aircraft, self._error, 0.0


def distance_km(lat_a, lon_a, lat_b, lon_b):
    dlat = (lat_a - lat_b) * KM_PER_DEG
    dlon = (lon_a - lon_b) * KM_PER_DEG * math.cos(math.radians(lat_a))
    return math.hypot(dlat, dlon)


def select(aircraft, lat, lon, dist_nm, limit, cache_age_s=0.0):
    """Nearest `limit` aircraft within `dist_nm`.

    Selection is the part that matters, not order: the firmware re-sorts by
    distance itself, but it also caps at MAX_AIRCRAFT, so handing it an
    arbitrary slice of a busy local feed would silently drop nearby traffic in
    favour of whatever happened to be listed first. Emitting farthest-first
    matches the order the firmware ends up in anyway.
    """
    radius_km = dist_nm * KM_PER_NM
    picked = []
    for plane in aircraft:
        plat = plane.get("lat")
        plon = plane.get("lon")
        if not isinstance(plat, (int, float)) or not isinstance(plon, (int, float)):
            continue
        # readsb keeps aircraft listed for up to a minute after their last
        # position. The firmware caps dead reckoning at 8 s, so anything older
        # than that arrives already beyond what it can project and lands as a
        # visible jump when the next real fix appears. Better to omit it.
        age = plane.get("seen_pos")
        if isinstance(age, (int, float)) and age > STALE_POSITION_S:
            continue
        d = distance_km(float(plat), float(plon), lat, lon)
        if d > radius_km:
            continue
        picked.append((d, plane))

    picked.sort(key=lambda item: item[0], reverse=True)
    del picked[:max(0, len(picked) - limit)]

    out = []
    for _, p in picked:
        item = {k: p[k] for k in PASSTHROUGH_FIELDS if k in p}
        # seen_pos was measured when this snapshot was read, not when the client
        # asked. Without adding the cache age the device back-dates by too
        # little, dead reckons slightly ahead, and every poll ends in a visible
        # correction.
        if isinstance(item.get("seen_pos"), (int, float)):
            item["seen_pos"] = round(item["seen_pos"] + cache_age_s, 3)
        out.append(item)
    return out


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "radar-feed"

    def log_message(self, fmt, *args):
        if self.server.verbose:
            sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    def _send(self, code, payload, content_type="application/json"):
        body = payload if isinstance(payload, bytes) else payload.encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802 - http.server API
        path = self.path.split("?", 1)[0].strip("/")
        parts = path.split("/")

        if path in ("health", "healthz"):
            if self.server.cache.source == "upstream":
                aircraft, error = [], None
            else:
                aircraft, error, _ = self.server.cache.aircraft()
            self._send(200, json.dumps({
                "ok": error is None,
                "source": self.server.cache.source,
                "aircraft_seen": len(aircraft),
                "upstream_reads": self.server.cache._reads,
                "error": error,
            }, indent=1))
            return

        # /route/<callsign> -- cached flight route lookup
        if len(parts) == 2 and parts[0] == "route" and parts[1]:
            callsign = parts[1].strip().upper()[:12]
            path_c = cache_path("route", callsign + ".json")
            try:
                age = time.time() - os.path.getmtime(path_c)
                if age < ROUTE_TTL_S:
                    with open(path_c, "rb") as f:
                        self._send(200, f.read())
                        return
            except OSError:
                pass
            try:
                with urlopen(ROUTE_TEMPLATE.format(callsign=callsign), timeout=6) as r:
                    body = r.read()
            except Exception as exc:  # noqa: BLE001
                # Negative results are worth caching too: an unknown callsign
                # would otherwise be re-queried on every poll forever.
                body = json.dumps({"response": "unknown callsign"}).encode()
                if "404" not in str(exc):
                    self._send(502, json.dumps({"error": str(exc)}))
                    return
            with open(path_c, "wb") as f:
                f.write(body)
            self._send(200, body)
            return

        # /tiles/<z>/<x>/<y>.png -- cached map tile
        if len(parts) == 4 and parts[0] == "tiles":
            try:
                z, x = int(parts[1]), int(parts[2])
                y = int(parts[3].split(".")[0])
            except ValueError:
                self._send(400, b"bad tile", "text/plain")
                return
            name = "%d_%d_%d.png" % (z, x, y)
            path_c = cache_path("tiles", name)
            if os.path.exists(path_c):
                with open(path_c, "rb") as f:
                    self._send(200, f.read(), "image/png")
                    return
            key = self.server.stadia_key
            if not key:
                self._send(503, b"no stadia key configured", "text/plain")
                return
            try:
                with urlopen(TILE_TEMPLATE.format(z=z, x=x, y=y, key=key), timeout=10) as r:
                    body = r.read()
            except Exception as exc:  # noqa: BLE001
                self._send(502, str(exc).encode(), "text/plain")
                return
            with open(path_c, "wb") as f:
                f.write(body)
            self._send(200, body, "image/png")
            return

        # /api/v3/lat/<lat>/lon/<lon>/dist/<nm>
        if len(parts) == 8 and parts[:2] == ["api", "v3"] and \
                parts[2] == "lat" and parts[4] == "lon" and parts[6] == "dist":
            try:
                lat = float(parts[3])
                lon = float(parts[5])
                dist = float(parts[7])
            except ValueError:
                self._send(400, json.dumps({"ac": [], "error": "bad coordinates"}))
                return

            if self.server.cache.source == "upstream":
                try:
                    url = UPSTREAM_TEMPLATE.format(lat=lat, lon=lon, dist=dist)
                    with urlopen(url, timeout=6) as r:
                        doc = json.loads(r.read().decode("utf-8", "replace"))
                    ac = doc.get("ac") or []
                except Exception as exc:  # noqa: BLE001
                    self._send(502, json.dumps({"ac": [], "error": str(exc)}))
                    return
                # Same nearest-N selection as the local path, so switching
                # sources cannot change which aircraft the device sees.
                ac = select(ac, lat, lon, dist, self.server.limit)
            else:
                aircraft, error, cache_age = self.server.cache.aircraft()
                if error is not None and not aircraft:
                    self._send(503, json.dumps({"ac": [], "error": error}))
                    return
                ac = select(aircraft, lat, lon, dist, self.server.limit, cache_age)

            self._send(200, json.dumps({"ac": ac}, separators=(",", ":")))
            return

        self._send(404, json.dumps({"ac": [], "error": "not found"}))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=int(os.environ.get("RADAR_FEED_PORT", 8080)))
    ap.add_argument("--bind", default=os.environ.get("RADAR_FEED_BIND", "0.0.0.0"))
    ap.add_argument("--source", default=os.environ.get(
        "RADAR_FEED_SOURCE", "http://127.0.0.1/data/aircraft.json"))
    ap.add_argument("--limit", type=int, default=int(os.environ.get("RADAR_FEED_LIMIT", 64)),
                    help="max aircraft returned; must not exceed the firmware's MAX_AIRCRAFT")
    ap.add_argument("--stadia-key", default=os.environ.get("RADAR_FEED_STADIA_KEY", ""),
                    help="Stadia Maps key, so the display need not hold one")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.cache = FeedCache(args.source)
    server.limit = args.limit
    server.verbose = args.verbose
    server.stadia_key = args.stadia_key
    print("radar-feed: source=%s bind=%s:%d limit=%d" %
          (args.source, args.bind, args.port, args.limit), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
