"""
mission3_waypoint.py — Real-time flight path waypoint system for fixed-wing UAV.
Mission 3: Small Loiter Pattern
"""

import os
import sys
import json
import math
import time
#import subprocess
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.mission_state_utils import load_state, update_state
from state.nav_state_utils import load_nav_state as load_state_nav, update_nav_state as update_state_nav

###############################################################################
# CONFIGs
###############################################################################
LOITER_RADIUS_FT    = 100.0  # Loiter circle radius written into active_plan.loiter_radius_ft (ft)
BORDER_CLEARANCE_M  = 50.0   # Minimum distance circle edge must keep from search boundary (m)
LOITER_SAMPLE_PTS   = 72     # Number of points sampled around circle circumference for checks
UPDATE_INTERVAL_S   = 0.5    # Polling interval when waiting for target_location.valid (s)
GENERATE_IMAGE      = True   # Toggle PNG generation 

EARTH_RADIUS_M = 6371000

STANDBY_INTERVAL_S = 1.0
DEFAULT_ALT_FT = 150.0
MISSION3_ALT_FT = 150.0

###############################################################################
# Geodetic helper
###############################################################################

def haversine(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def destination(lat, lon, brng_deg, dist_m):
    br    = math.radians(brng_deg)
    rlat  = math.radians(lat)
    rlon  = math.radians(lon)
    dr    = dist_m / EARTH_RADIUS_M
    nlat  = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(br))
    nlon  = rlon + math.atan2(math.sin(br)*math.sin(dr)*math.cos(rlat),
                              math.cos(dr) - math.sin(rlat)*math.sin(nlat))
    return math.degrees(nlat), math.degrees(nlon)


def to_local(lat, lon, o_lat, o_lon):
    x = haversine(o_lat, o_lon, o_lat, lon) * (1 if lon >= o_lon else -1)
    y = haversine(o_lat, o_lon, lat,   o_lon) * (1 if lat >= o_lat else -1)
    return x, y


def from_local(x, y, o_lat, o_lon):
    lat, _ = destination(o_lat, o_lon,  0, y)
    _, lon  = destination(o_lat, o_lon, 90, x)
    return lat, lon


def centroid(pts):
    n = len(pts)
    return sum(p[0] for p in pts)/n, sum(p[1] for p in pts)/n


def sort_clockwise(pts):
    cx, cy = centroid(pts)
    return sorted(pts, key=lambda p: -math.atan2(p[1]-cy, p[0]-cx))


###############################################################################
# 2-D geometry helpers (local metre frame)
###############################################################################

def pt_to_seg_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    sq = dx*dx + dy*dy
    if sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / sq))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def min_dist_point_to_poly(px, py, poly):
    """Minimum distance from a point to any edge of a closed polygon."""
    n = len(poly)
    return min(
        pt_to_seg_dist(px, py,
                       poly[i][0],        poly[i][1],
                       poly[(i+1)%n][0],  poly[(i+1)%n][1])
        for i in range(n)
    )


def point_in_poly(px, py, poly):
    """Ray-casting point-in-polygon test."""
    n      = len(poly)
    inside = False
    j      = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-12)+xi):
            inside = not inside
        j = i
    return inside


def circle_clearance(cx, cy, radius_m, poly, n_samples=72):
    """
    Minimum clearance between a circle and all polygon edges.
    Samples n_samples points evenly around the circumference and returns
    the minimum distance from any sample point to any polygon edge.
    Positive → all samples inside with that margin; negative → breach.
    """
    min_d = float("inf")
    for i in range(n_samples):
        angle = 2 * math.pi * i / n_samples
        sx = cx + radius_m * math.cos(angle)
        sy = cy + radius_m * math.sin(angle)
        min_d = min(min_d, min_dist_point_to_poly(sx, sy, poly))
    return min_d


###############################################################################
# Circle containment / nudging
###############################################################################

def ft_to_m(ft):
    return ft * 0.3048


def clamp_circle_to_poly(cx, cy, radius_m, poly, clearance_m):
    """
    If the circle (cx,cy,radius_m) already satisfies clearance_m from every
    poly edge, return it unchanged.

    Otherwise nudge the centre toward the polygon centroid along the ray
    centroid→(cx,cy) until clearance is satisfied, using binary search.
    The result minimises displacement from the original centre while
    guaranteeing the constraint.

    Returns
    -------
    (new_cx, new_cy, was_moved)
    """
    required = radius_m + clearance_m

    # Fast path — already compliant (all circumference samples clear the boundary)
    if circle_clearance(cx, cy, radius_m, poly, LOITER_SAMPLE_PTS) >= clearance_m:
        return cx, cy, False

    pcx, pcy = centroid(poly)

    # Direction: from original centre toward polygon centroid
    dx = pcx - cx
    dy = pcy - cy
    dist_to_centroid = math.hypot(dx, dy)

    if dist_to_centroid < 1e-6:
        # Already at centroid — cannot move further; return centroid
        return pcx, pcy, True

    # Binary-search along the segment  original_centre → poly_centroid
    # for the smallest t that satisfies the clearance constraint.
    # t=0 → original (fails), t=1 → centroid (safe by design if radius < inradius)
    lo, hi = 0.0, 1.0

    for _ in range(64):
        mid  = (lo + hi) * 0.5
        qx   = cx + mid * dx
        qy   = cy + mid * dy
        if circle_clearance(qx, qy, radius_m, poly, LOITER_SAMPLE_PTS) >= clearance_m:
            hi = mid   # good — try pulling back toward original
        else:
            lo = mid   # still bad — push more toward centroid

    new_cx = cx + hi * dx
    new_cy = cy + hi * dy
    return new_cx, new_cy, True


###############################################################################
# Telemetry reader (tail-seek approach)
###############################################################################

def read_latest_telemetry(fusion_log_path):
    path = Path(fusion_log_path)
    if not path.exists():
        print(f"[WARN] Fusion log not found: {path}")
        return None
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            if size == 0:
                return None
            buf, pos = b"", size - 1
            while pos >= 0:
                f.seek(pos)
                ch = f.read(1)
                if ch == b"\n" and buf.strip():
                    break
                buf = ch + buf
                pos -= 1
            line = buf.decode("utf-8", errors="replace").strip()
            if line:
                return json.loads(line)
    except Exception as e:
        print(f"[WARN] Could not read fusion log: {e}")
    return None


###############################################################################
# PNG generation 
###############################################################################

def _png_write(path, width, height, pixels):
    """Write an RGB bytearray (width*height*3) as a PNG using stdlib only."""
    import zlib, struct

    def chunk(tag, data):
        c = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", c)

    raw = bytearray()
    for y in range(height):
        raw.append(0)  # filter byte
        raw.extend(pixels[y * width * 3:(y + 1) * width * 3])

    ihdr  = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    idat  = zlib.compress(bytes(raw), 9)
    data  = (b"\x89PNG\r\n\x1a\n"
             + chunk(b"IHDR", ihdr)
             + chunk(b"IDAT", idat)
             + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(data)


def _px(pixels, width, x, y, r, g, b):
    """Set one pixel (clipped)."""
    if 0 <= x < width and 0 <= y < len(pixels) // (width * 3):
        i = (y * width + x) * 3
        pixels[i], pixels[i+1], pixels[i+2] = r, g, b


def _line(pixels, width, height, x0, y0, x1, y1, r, g, b):
    """Bresenham line."""
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        _px(pixels, width, x0, y0, r, g, b)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 <  dx: err += dx; y0 += sy


def _circle_outline(pixels, width, height, cx, cy, rad, r, g, b, dashed=False):
    """Draw a circle outline, optionally dashed."""
    n = max(360, int(2 * math.pi * rad))
    for i in range(n):
        if dashed and (i // 6) % 2 == 1:
            continue
        a = 2 * math.pi * i / n
        _px(pixels, width, int(cx + rad*math.cos(a)), int(cy + rad*math.sin(a)), r, g, b)


def _filled_circle(pixels, width, height, cx, cy, rad, r, g, b):
    cx, cy, rad = int(cx), int(cy), int(rad)
    for dy in range(-rad, rad+1):
        for dx in range(-rad, rad+1):
            if dx*dx + dy*dy <= rad*rad:
                _px(pixels, width, cx+dx, cy+dy, r, g, b)


def _arrow(pixels, width, height, x0, y0, x1, y1, r, g, b):
    """Line with a small arrowhead at (x1,y1)."""
    _line(pixels, width, height, x0, y0, x1, y1, r, g, b)
    ang = math.atan2(y1-y0, x1-x0)
    for da in (2.5, -2.5):
        ax = x1 - 10 * math.cos(ang + da)
        ay = y1 - 10 * math.sin(ang + da)
        _line(pixels, width, height, x1, y1, int(ax), int(ay), r, g, b)


def generate_image(search_xy, orig_centre, new_centre, radius_m,
                   was_moved, loiter_radius_ft, alt_ft, o_lat, o_lon,
                   plane_lat=None, plane_lon=None,
                   target_lat=None, target_lon=None):
    W = H = 800
    PAD = 60

    # ---- world → screen transform ----------------------------------------
    all_x = [p[0] for p in search_xy]
    all_y = [p[1] for p in search_xy]
    # Include plane and raw target positions in extent so they're never clipped
    for glat, glon in [(plane_lat, plane_lon), (target_lat, target_lon)]:
        if glat is not None and glon is not None:
            gx, gy = to_local(glat, glon, o_lat, o_lon)
            all_x.append(gx); all_y.append(gy)
    for cx, cy in [(orig_centre[0], orig_centre[1]),
                   (new_centre[0],  new_centre[1])]:
        all_x += [cx - radius_m, cx + radius_m]
        all_y += [cy - radius_m, cy + radius_m]

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    scale = (W - 2*PAD) / span

    def s(x, y):   # local metres → screen pixels (y flipped)
        return (int(PAD + (x - min_x) * scale),
                int(H - PAD - (y - min_y) * scale))

    pixels = bytearray(W * H * 3)  # black background

    # ---- search area boundary (yellow) -----------------------------------
    n = len(search_xy)
    for i in range(n):
        x0, y0 = s(*search_xy[i])
        x1, y1 = s(*search_xy[(i+1) % n])
        _line(pixels, W, H, x0, y0, x1, y1, 255, 220, 0)

    # ---- original circle — solid white (always drawn) --------------------
    ocx, ocy = s(*orig_centre)
    r_px = int(radius_m * scale)
    _circle_outline(pixels, W, H, ocx, ocy, r_px, 255, 255, 255)
    _filled_circle(pixels, W, H, ocx, ocy, 5, 255, 255, 255)        # white centre dot

    # ---- final nudged circle — solid cyan (only when moved) --------------
    ncx, ncy = s(*new_centre)
    r_px = int(radius_m * scale)
    if was_moved:
        _circle_outline(pixels, W, H, ncx, ncy, r_px, 0, 220, 220)
        _filled_circle(pixels, W, H, ncx, ncy, 5, 0, 220, 220)      # cyan centre dot

    # ---- mra_refined_loiter_target dot — orange (small, under plane dot) -
    if target_lat is not None and target_lon is not None:
        tx, ty = to_local(target_lat, target_lon, o_lat, o_lon)
        tsx, tsy = s(tx, ty)
        _filled_circle(pixels, W, H, tsx, tsy, 4, 255, 140, 0)      # orange, radius 4

    # ---- plane position dot — green (stacked on top of orange) -----------
    if plane_lat is not None and plane_lon is not None:
        gx, gy = to_local(plane_lat, plane_lon, o_lat, o_lon)
        gsx, gsy = s(gx, gy)
        _filled_circle(pixels, W, H, gsx, gsy, 6, 0, 255, 80)       # green, radius 6

    # ---- info text (burn pixels manually — keep stdlib only) -------------
    # Skip font rendering; embed metadata in filename comment instead.
    # A simple scale-bar is drawn at bottom-left.
    bar_m   = 100                            # 100 m scale bar
    bar_px  = int(bar_m * scale)
    bx0, by = PAD, H - 20
    _line(pixels, W, H, bx0, by, bx0 + bar_px, by, 200, 200, 200)
    _line(pixels, W, H, bx0, by-4, bx0, by+4, 200, 200, 200)
    _line(pixels, W, H, bx0+bar_px, by-4, bx0+bar_px, by+4, 200, 200, 200)

    out = Path(__file__).resolve().parent / "mission3_map.png"
    _png_write(str(out), W, H, pixels)
    print(f"[IMAGE] Saved → {out}")

# ── Mission teardown ──────────────────────────────────────────────────────────

def _teardown(nav: dict, mis: dict):
    """Clear loiter-related fields on autonomy deactivation."""

    # navigation_state.json → mra_refined_loiter_target
    rlt = nav.setdefault("mra_refined_loiter_target", {})
    rlt["valid"]      = False
    rlt["lat"]        = None
    rlt["lon"]        = None
    rlt["confidence"] = None
    rlt["timestamp"]  = None
    rlt["fix_id"]     = None

    # navigation_state.json → target_location
    tl = nav.setdefault("target_location", {})
    tl["valid"]      = False
    tl["source"]     = None
    tl["lat"]        = None
    tl["lon"]        = None
    tl["confidence"] = None
    tl["timestamp"]  = None
    tl["id"]         = None

    # navigation_state.json → active_plan
    ap = nav.setdefault("active_plan", {})
    ap["plan_id"]          = None
    ap["plan_type"]        = None
    ap["label"]        = None
    ap["status"]           = None
    ap["loiter_radius_ft"] = None
    ap["alt_ft"]           = None
    ap["waypoints"]        = []
    ap["next_plan"]        = None


    update_state_nav("mra_refined_loiter_target", rlt)
    update_state_nav("target_location",           tl)
    update_state_nav("active_plan",               ap)
    update_state_nav("next_plan",                 None)
    print("[MISSION 3] Teardown complete. All loiter fields cleared.")

###############################################################################
# Main
###############################################################################

def run_mission_3():
    print("[MISSION 3] Starting …")

    # ------------------------------------------------------------------
    # Load state files
    # ------------------------------------------------------------------
    mission_state = load_state()
    nav_state     = load_state_nav()

    # ------------------------------------------------------------------
    # Startup: write plan metadata
    # ------------------------------------------------------------------
    nav = nav_state.get("navigation", {})
    nav["mission_phase"] = 3
    update_state_nav("navigation", nav)

    active_plan = {
        "plan_id": 3,
        "plan_type": "single_loiter",
        "label": "Small Loiter Pattern",
        "status": "building",
        "waypoints": [],
        "loiter_radius_ft": LOITER_RADIUS_FT,
        "alt_ft": None,
    }
    update_state_nav("active_plan", active_plan)

    update_state_nav("next_plan", "Manual Takeover")

    print("[STATE] mission_phase=3, plan_id=3, plan_type='single_loiter', "
      "label='Small Loiter Pattern', status='building', next_plan=Manual Takeover")

    # ------------------------------------------------------------------
    # Read search area
    # ------------------------------------------------------------------
    while True:
        nav_state = load_state_nav()
        raw_search = nav_state.get("search_area", [])

        if isinstance(raw_search, list) and 3 <= len(raw_search) <= 6:
            break

        if isinstance(raw_search, list) and len(raw_search) == 0:
            print("[MISSION] Waiting for search_area from GCS...")
        else:
            print(f"[MISSION] Invalid search_area. Need 3–6 points, got: {raw_search}")

        time.sleep(STANDBY_INTERVAL_S)

    search_coords = []
    for entry in raw_search:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            raise ValueError(f"Invalid search_area entry: {entry}")
        search_coords.append((float(entry[0]), float(entry[1])))

    n_coords = len(search_coords)
    o_lat = sum(p[0] for p in search_coords) / n_coords
    o_lon = sum(p[1] for p in search_coords) / n_coords

    local_pts = [to_local(lat, lon, o_lat, o_lon) for lat, lon in search_coords]
    search_xy = sort_clockwise(local_pts)   # closed convex quad in local metres

    # ------------------------------------------------------------------
    # Read loiter target
    # ------------------------------------------------------------------
    print("[MISSION 3] Waiting for valid target_location...")

    while True:
        nav_state = load_state_nav()
        loiter_target = nav_state.get("target_location", {})

        target_lat = loiter_target.get("lat")
        target_lon = loiter_target.get("lon")

        if loiter_target.get("valid", False) and target_lat is not None and target_lon is not None:
            print(f"[MISSION 3] Got final target: ({target_lat:.7f}, {target_lon:.7f})")
            break

        print("[MISSION 3] No valid target_location yet. Waiting...")
        time.sleep(STANDBY_INTERVAL_S)
    # ------------------------------------------------------------------
    # Read loiter radius (feet — kept as-is, no unit conversion)
    # ------------------------------------------------------------------
    active_plan = nav_state.get("active_plan", {})
    active_plan["loiter_radius_ft"] = LOITER_RADIUS_FT
    update_state_nav("active_plan", active_plan)
    print(f"[STATE] active_plan.loiter_radius_ft → {LOITER_RADIUS_FT} ft")


    loiter_radius_ft = LOITER_RADIUS_FT

    # Radius in metres for geometric calculations
    loiter_radius_m = ft_to_m(float(loiter_radius_ft))

    # ------------------------------------------------------------------
    # Read altitude (feet — kept as-is)
    # ------------------------------------------------------------------
    alt_ft = MISSION3_ALT_FT
    print(f"[STATE] Mission 3 planned altitude → {alt_ft} ft")

    # ------------------------------------------------------------------
    # Convert loiter centre to local frame and clamp to search area
    # ------------------------------------------------------------------
    orig_cx, orig_cy = to_local(target_lat, target_lon, o_lat, o_lon)

    new_cx, new_cy, was_moved = clamp_circle_to_poly(
        orig_cx, orig_cy, loiter_radius_m, search_xy, BORDER_CLEARANCE_M
    )

    if was_moved:
        new_lat, new_lon = from_local(new_cx, new_cy, o_lat, o_lon)
        move_m = math.hypot(new_cx - orig_cx, new_cy - orig_cy)
        print(f"[LOITER] Circle nudged {move_m:.1f} m to stay within boundary clearance.")
        print(f"[LOITER] Original centre: ({target_lat:.7f}, {target_lon:.7f})")
        print(f"[LOITER] Adjusted centre: ({new_lat:.7f}, {new_lon:.7f})")
    else:
        new_lat, new_lon = target_lat, target_lon
        print(f"[LOITER] Circle already within boundary — centre unchanged "
              f"({new_lat:.7f}, {new_lon:.7f})")

    print(f"[LOITER] Radius: {loiter_radius_ft} ft  |  alt: {alt_ft} ft")

    mission_state = load_state()
    fusion_log_path = mission_state.get("fusion_log")

    if not fusion_log_path:
        print("[WARN] No fusion_log path in mission_state.json. Continuing without plane telemetry.")
        telem = None
    else:
        telem = read_latest_telemetry(fusion_log_path)

    plane_lat = telem.get("lat_deg") if telem else None
    plane_lon = telem.get("lon_deg") if telem else None

    if GENERATE_IMAGE:
        try:
            generate_image(
                search_xy    = search_xy,
                orig_centre  = (orig_cx, orig_cy),
                new_centre   = (new_cx,  new_cy),
                radius_m     = loiter_radius_m,
                was_moved    = was_moved,
                loiter_radius_ft = loiter_radius_ft,
                alt_ft       = alt_ft,
                o_lat        = o_lat,
                o_lon        = o_lon,
                plane_lat        = plane_lat,
                plane_lon        = plane_lon,
                target_lat       = target_lat,
                target_lon       = target_lon,
            )
        except Exception:
            import traceback
            traceback.print_exc()
            print("[WARN] PNG generation failed — continuing.")

    # ------------------------------------------------------------------
    # Write waypoint (single centre point) to active_plan.waypoints
    # ------------------------------------------------------------------
    active_plan = {
        "plan_id": 3,
        "plan_type": "single_loiter",
        "label": "Small Loiter Pattern",
        "status": "ready",
        "waypoints": [
            {
                "lat": new_lat,
                "lon": new_lon,
                "alt_ft": alt_ft,
            }
        ],
        "loiter_radius_ft": loiter_radius_ft,
        "alt_ft": alt_ft,
    }

    update_state_nav("active_plan", active_plan)
    print("[STATE] active_plan written as single_loiter and status → 'ready'")

    print("[MISSION 3] Plan is ready. Waiting for autonomy_active before manual-takeover monitoring...")

    while True:
        mission_state = load_state()
        controller_status = mission_state.get("controller_status", {})
        nav_state = load_state_nav()
        active_plan = nav_state.get("active_plan", {})

        if (
            controller_status.get("autonomy_active", False)
            and active_plan.get("plan_id") == 3
            and active_plan.get("status") in ("ready", "uploaded", "running")
        ):
            print("[MISSION 3] Mission 3 is active/running. Waiting for manual takeover.")
            break

        time.sleep(STANDBY_INTERVAL_S)

    # ------------------------------------------------------------------
    # Guidance loop — poll until autonomy becomes False
    # ------------------------------------------------------------------
    print("[MISSION 3] Loitering … waiting for manual takeover …")

    if telem:
        print(f"[TELEM] Plane at ({plane_lat:.6f}, {plane_lon:.6f}), "
              f"yaw={telem.get('yaw_deg', 0):.1f}°")

    print("[MISSION 3] Entering polling loop (polling autonomy_active) …")
    while True:
        time.sleep(UPDATE_INTERVAL_S)

        mis = load_state()
        controller_status = mis.get("controller_status", {})

        if not controller_status.get("autonomy_active", True):
            print("[MISSION 3] controller_status.autonomy_active → False. Initiating teardown …")
            nav = load_state_nav()
            _teardown(nav, mis)
            break
        

    print("\n[MISSION 3] Exited.")
    os._exit(0)
    

if __name__ == "__main__":
    run_mission_3()