import sys
import json
import math
import time
import subprocess
import logging
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.mission_state_utils import load_state, update_state
from state.nav_state_utils import load_nav_state, update_nav_state


STANDBY_INTERVAL_S = 1.0    # Polling interval while waiting for autonomy_active (s)

# MISSION 1 CONFIGURATION
MIN_TURN_ANGLE_DEG      = 15.0   # Minimum heading change per waypoint segment (deg)
WAYPOINT_RADIUS_FT      = 20.0   # Waypoint capture radius (ft)  ~20 m (# Capture radius: advance to next WP within this distance (ft))
BORDER_CLEARANCE_FT     = 50.0  # Minimum clearance from boundary (ft)     ###################### Adjust this as necessary)
STRAIGHT_SPACING_FACTOR = 5.0    # Waypoint spacing on straights = factor × R
CURVATURE_SPACING_FACTOR= 0.5    # Waypoint spacing on tight curves = factor × R
CURVATURE_SENSITIVITY   = 2.0    # Higher = denser waypoints near turns
MAXIMUM_WAYPOINTS       = 30     # Hard cap on waypoints written to JSON (0 = no limit)

FT_TO_M              = 0.3048
BORDER_CLEARANCE_M   = BORDER_CLEARANCE_FT   * FT_TO_M
WAYPOINT_RADIUS_M    = WAYPOINT_RADIUS_FT    * FT_TO_M
# Min chord length from turning angle: chord = 2R·sin(θ/2), but without explicit R
# we derive a minimum spacing floor from the ellipse semi-minor at runtime (see sample_ellipse_waypoints)

L1_DISTANCE_M      = 30.0   # L1 guidance lookahead distance (m)
UPDATE_INTERVAL_S  = 0.1    # Guidance loop polling interval (s)
GENERATE_IMAGE     = True   # Toggle PNG map generation (True / False)
#MISSION_TIMEOUT_S  = 240.0  # Maximum mission runtime in seconds (0 = no limit)
CRUISE_ALT_FT      = 150.0  # Cruise altitude written into active_plan.alt_ft (ft)

EARTH_RADIUS_M = 6_371_000.0



log = logging.getLogger("mission_waypoint")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)

# Geodetic helpers

def haversine(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def bearing_to(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1)*math.sin(rlat2) - math.sin(rlat1)*math.cos(rlat2)*math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination(lat, lon, brng_deg, dist_m):
    br = math.radians(brng_deg)
    rlat, rlon = math.radians(lat), math.radians(lon)
    dr = dist_m / EARTH_RADIUS_M
    nlat = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(br))
    nlon = rlon + math.atan2(math.sin(br)*math.sin(dr)*math.cos(rlat),
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


def normalize_angle(a):
    while a >  180: a -= 360
    while a <= -180: a += 360
    return a


# -------------------------------------------------------
# 2-D geometry helpers (all units: metres in local frame)
# -------------------------------------------------------

def pt_to_seg_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    sq = dx*dx + dy*dy
    if sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / sq))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def min_dist_to_poly_edges(px, py, poly):
    """Minimum distance from point to any edge of a closed polygon."""
    n = len(poly)
    return min(
        pt_to_seg_dist(px, py,
                       poly[i][0], poly[i][1],
                       poly[(i+1)%n][0], poly[(i+1)%n][1])
        for i in range(n)
    )


def sort_clockwise(pts):
    cx, cy = centroid(pts)
    return sorted(pts, key=lambda p: -math.atan2(p[1]-cy, p[0]-cx))


def shrink_vertex_to_clearance(cx, cy, vx, vy, poly, clearance_m):
    """
    The raw diamond vertex (vx,vy) is a midpoint of a search-area edge and
    therefore lies ON the boundary (dist ≈ 0).  We need to move it inward
    along the ray  centroid→vertex  until it sits exactly CLEARANCE_M inside
    the polygon boundary.

    Strategy
    --------
    The centroid is deep inside the polygon so dist(centroid, boundary) >> 0.
    We binary-search along the segment  centroid→vertex  for the point whose
    distance to the nearest edge equals exactly clearance_m.

    lo  = parameter where dist >= clearance  (start: 0 = centroid, safe)
    hi  = parameter where dist <  clearance  (start: 1 = vertex,   on border)

    We maximise t (go as far outward as possible) while keeping dist >= clearance.
    """
    dx, dy = vx - cx, vy - cy

    # Parameterise the ray: P(t) = centroid + t*(vertex - centroid), t in [0,1]
    # t=0 → centroid (safe), t=1 → vertex (on border, unsafe)
    lo, hi = 0.0, 1.0

    for _ in range(64):          # 64 iterations → sub-micrometre precision
        mid = (lo + hi) * 0.5
        qx, qy = cx + mid*dx, cy + mid*dy
        if min_dist_to_poly_edges(qx, qy, poly) >= clearance_m:
            lo = mid             # safe here — try further outward
        else:
            hi = mid             # too close to edge — pull inward

    # lo is the furthest safe position
    return cx + lo*dx, cy + lo*dy

# ---------------------
# Ellipse path pipeline
# ---------------------

def convex_hull(pts):
    """Graham scan → clockwise-ordered convex hull."""
    pts = sorted(set(pts))
    if len(pts) <= 1:
        return pts
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def polygon_area_centroid(poly):
    """
    Area-weighted centroid of a polygon.
    Better than averaging vertices for weird trapezoid / triangle shapes.
    """
    a = 0.0
    cx = 0.0
    cy = 0.0
    n = len(poly)

    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        a += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross

    a *= 0.5

    if abs(a) < 1e-9:
        return centroid(poly)

    cx /= (6.0 * a)
    cy /= (6.0 * a)

    return cx, cy


def fit_oriented_ellipse(hull_xy):
    cx, cy = polygon_area_centroid(hull_xy)
    dx = [p[0]-cx for p in hull_xy]
    dy = [p[1]-cy for p in hull_xy]
    n  = len(hull_xy)
    cxx = sum(x*x for x in dx) / n
    cxy = sum(dx[i]*dy[i] for i in range(n)) / n
    cyy = sum(y*y for y in dy) / n
    trace = cxx + cyy
    det   = cxx*cyy - cxy*cxy
    l1    = trace/2 + math.sqrt(max(0.0, (trace/2)**2 - det))
    angle = math.atan2(l1 - cxx, cxy) if abs(cxy) > 1e-9 else (0.0 if cxx >= cyy else math.pi/2)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    # Bounding extent in PCA frame — ellipse starts larger than hull, then gets shrunk
    u_coords = [abs( (p[0]-cx)*cos_a + (p[1]-cy)*sin_a) for p in hull_xy]
    v_coords = [abs(-(p[0]-cx)*sin_a + (p[1]-cy)*cos_a) for p in hull_xy]
    a = max(u_coords) if u_coords else 1.0
    b = max(v_coords) if v_coords else 1.0
    if a < b:
        a, b = b, a
        angle = angle + math.pi/2

    # # Scale down uniformly until ellipse is fully inside hull (all 360 sample pts inside)
    # def ellipse_max_overshoot(s):
    #     """Returns how far outside the hull the scaled ellipse reaches (0 = fully inside)."""
    #     outside = 0.0
    #     for i in range(360):
    #         t  = 2*math.pi*i/360
    #         lx = s*a*math.cos(t); ly = s*b*math.sin(t)
    #         px = cx + lx*cos_a - ly*sin_a
    #         py = cy + lx*sin_a + ly*cos_a
    #         # Point is outside hull if it is beyond every hull edge (negative inward dist)
    #         d = min_dist_to_poly_edges(px, py, hull_xy)
    #         # Check if point is inside hull using winding — approximate via signed clearance
    #         # We use: if the point's nearest-edge distance is positive and point is outside, flag it.
    #         # Simpler: check if point projects outside any hull edge (cross product sign).
    #         nh = len(hull_xy)
    #         inside = True
    #         for j in range(nh):
    #             ax, ay = hull_xy[j]; bx, by = hull_xy[(j+1)%nh]
    #             cross = (bx-ax)*(py-ay) - (by-ay)*(px-ax)
    #             if cross > 0:   # CW hull: outside if cross > 0
    #                 inside = False
    #                 outside = max(outside, cross / (math.hypot(bx-ax, by-ay) + 1e-12))
    #                 break
    #     return outside
    # lo, hi = 0.0, 1.0
    # for _ in range(52):
    #     mid = (lo + hi) / 2
    #     if ellipse_max_overshoot(mid) == 0.0: lo = mid
    #     else:                                  hi = mid
    # a, b = lo*a, lo*b
    return cx, cy, a, b, angle


def shrink_ellipse_to_clearance(cx, cy, a, b, angle, hull_xy):
    """
    Binary-search scale s so the ellipse is fully inside the hull
    AND every point is >= BORDER_CLEARANCE_M from every hull edge.
    """
    print(f"[ELLIPSE] Raw inscribed axes: a={a:.1f} m, b={b:.1f} m, clearance={BORDER_CLEARANCE_M:.1f} m")
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    
    nh = len(hull_xy)

    def point_inside_hull(px, py):
        """True if point is inside or on the convex hull (works for CW or CCW)."""
        signs = []
        for j in range(nh):
            ax, ay = hull_xy[j]
            bx, by = hull_xy[(j+1) % nh]
            cross = (bx-ax)*(py-ay) - (by-ay)*(px-ax)
            signs.append(cross)
        return all(s <= 0 for s in signs) or all(s >= 0 for s in signs)

    def min_clearance(s):
        min_d = float('inf')
        for i in range(720):
            t  = 2*math.pi*i/720
            lx = s*a*math.cos(t); ly = s*b*math.sin(t)
            px = cx + lx*cos_a - ly*sin_a
            py = cy + lx*sin_a + ly*cos_a
            if not point_inside_hull(px, py):
                return -1.0          # outside hull — fail immediately
            d  = min_dist_to_poly_edges(px, py, hull_xy)
            if d < min_d: min_d = d
        return min_d
    
    if min_clearance(1.0) >= BORDER_CLEARANCE_M:
        return a, b
    lo, hi = 0.0, 1.0
    for _ in range(52):
        mid = (lo + hi) / 2
        if min_clearance(mid) >= BORDER_CLEARANCE_M: lo = mid
        else:                                         hi = mid
    print(f"[ELLIPSE] Shrunk axes: a={lo*a:.1f} m, b={lo*b:.1f} m")
    return lo*a, lo*b


def ellipse_curvature(a, b, t):
    """κ(t) of ellipse at parameter t."""
    denom = (b*b*math.cos(t)**2 + a*a*math.sin(t)**2) ** 1.5
    return (a*b) / denom if denom > 1e-12 else 0.0


def sample_ellipse_waypoints(cx, cy, a, b, angle):
    """
    Two-pass curvature-aware sampling.
    Pass 1: compute total arc length.
    Pass 2: adaptive spacing, base = arc_len / MAXIMUM_WAYPOINTS,
            floor = chord subtending MIN_TURN_ANGLE_DEG at local curvature.
    """
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    min_angle_rad = math.radians(MIN_TURN_ANGLE_DEG)
    dt = 2*math.pi / 3600
    arc_len = sum(math.hypot(-a*math.sin(t), b*math.cos(t)) * dt
                  for t in (i*dt for i in range(3600)))
    max_count = MAXIMUM_WAYPOINTS if MAXIMUM_WAYPOINTS > 0 else 50
    base_sp = arc_len / max_count
    print(f"[ELLIPSE] arc_len={arc_len:.1f} m, base_sp={base_sp:.1f} m, targeting {max_count} WPs")
    pts, accum, t = [], 0.0, 0.0
    while t < 2*math.pi:
        kappa   = ellipse_curvature(a, b, t)
        R_local = 1.0 / kappa if kappa > 1e-6 else 1e6
        min_sp  = 2.0 * R_local * math.sin(min_angle_rad / 2.0)
        curved_sp = base_sp / (1.0 + CURVATURE_SENSITIVITY * kappa * R_local)
        spacing = max(min_sp, min(base_sp, curved_sp))
        accum  += math.hypot(-a*math.sin(t), b*math.cos(t)) * dt
        if accum >= spacing:
            lx = a*math.cos(t); ly = b*math.sin(t)
            pts.append((cx + lx*cos_a - ly*sin_a, cy + lx*sin_a + ly*cos_a))
            accum = 0.0
        t += dt
    return pts


def downsample_waypoints(wp_xy, max_count):
    """Evenly subsample wp_xy to at most max_count points, preserving order."""
    n = len(wp_xy)
    if max_count <= 0 or n <= max_count:
        return wp_xy
    indices = [round(i * n / max_count) % n for i in range(max_count)]
    return [wp_xy[i] for i in indices]


def build_ellipse_path(search_coords):
    n = len(search_coords)
    if not (3 <= n <= 6):
        raise ValueError(f"search_area must have 3-6 entries, got {n}")
    o_lat = sum(p[0] for p in search_coords) / n
    o_lon = sum(p[1] for p in search_coords) / n
    local_pts  = [to_local(lat, lon, o_lat, o_lon) for lat, lon in search_coords]
    hull_xy    = convex_hull(local_pts)
    cx, cy, a, b, angle = fit_oriented_ellipse(hull_xy)
    a_s, b_s   = shrink_ellipse_to_clearance(cx, cy, a, b, angle, hull_xy)
    wp_xy      = sample_ellipse_waypoints(cx, cy, a_s, b_s, angle)
    wp_xy      = downsample_waypoints(wp_xy, MAXIMUM_WAYPOINTS)
    waypoints_ll = [from_local(x, y, o_lat, o_lon) for x, y in wp_xy]
    ellipse_params = (cx, cy, a, b, a_s, b_s, angle)
    return waypoints_ll, hull_xy, ellipse_params, wp_xy, o_lat, o_lon

# -------------------------------------------------
# Fusion-log reader — reads last line of JSONL file
# -------------------------------------------------

def read_latest_telemetry(fusion_log_path: str):
    if not fusion_log_path:
        print("[WARN] No fusion_log path in mission_state")
        return None

    path = Path(fusion_log_path)

    if not path.exists():
        print(f"[WARN] Fusion log does not exist: {path}")
        return None

    try:
        lines = path.read_text().splitlines()
    except Exception as exc:
        print(f"[WARN] Could not read fusion log file: {exc}")
        return None

    # Read from newest to oldest, skipping corrupted / partial lines
    for line in reversed(lines):
        line = line.strip()

        if not line:
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        if "lat_deg" in data and "lon_deg" in data:
            return data

    print("[WARN] No valid telemetry line with lat_deg/lon_deg found in fusion log")
    return None

# --------------------------
# Closest waypoint selection
# --------------------------

def closest_wp_index(plane_lat, plane_lon, waypoints_ll):
    best_i, best_d = 0, float("inf")
    for i, (wlat, wlon) in enumerate(waypoints_ll):
        d = haversine(plane_lat, plane_lon, wlat, wlon)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


# -----------
# L1 Guidance ######################################## ( might not be necessary, will probably remove)
# -----------

def l1_guidance(plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon):
    dist_m   = haversine(plane_lat, plane_lon, wp_lat, wp_lon)
    des_brng = bearing_to(plane_lat, plane_lon, wp_lat, wp_lon)
    eta      = math.radians(normalize_angle(des_brng - plane_yaw))
    corr     = math.degrees(math.asin(
        max(-1.0, min(1.0, 2.0 * math.sin(eta) * dist_m / max(dist_m, L1_DISTANCE_M)))
    ))
    return (des_brng + corr) % 360, des_brng, dist_m


# --------------------
# PNG image generation
# --------------------

def _write_png(filename, width, height, pixels):
    import zlib, struct

    def chunk(tag, data):
        payload = tag + data
        return (struct.pack(">I", len(data))
                + payload
                + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF))

    stride = 1 + width * 3
    raw = bytearray(height * stride)
    for row in range(height):
        rs = row * stride
        raw[rs] = 0                          # PNG filter: None
        for col in range(width):
            r, g, b = pixels[row * width + col]
            off = rs + 1 + col * 3
            raw[off] = r; raw[off+1] = g; raw[off+2] = b

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    data = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + chunk(b"IEND", b""))
    with open(filename, "wb") as f:
        f.write(data)


def generate_map_image(hull_xy, ellipse_params, wp_xy,
                       plane_lat, plane_lon, start_wp_idx,
                       o_lat, o_lon, filename="mission1_map.png"):
    cx, cy, a_raw, b_raw, a_s, b_s, angle = ellipse_params
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    """
    Renders:
      • Search boundary border                  (yellow)
      • Original fitted ellipse                 (white)
      • Shrunk/clearance ellipse + flight path  (cyan)
      • Plane position                          (green filled dot)
      • Arrow: plane → nearest waypoint         (green)
    """
    W, H, PAD = 800, 800, 70

    plane_x, plane_y = to_local(plane_lat, plane_lon, o_lat, o_lon)

    all_xy = list(hull_xy) + list(wp_xy) + [(plane_x, plane_y)]
    min_x = min(p[0] for p in all_xy)
    max_x = max(p[0] for p in all_xy)
    min_y = min(p[1] for p in all_xy)
    max_y = max(p[1] for p in all_xy)
    span_x = max_x - min_x or 1.0
    span_y = max_y - min_y or 1.0
    # Keep aspect ratio square — use the larger span for both axes
    span   = max(span_x, span_y)
    cx_w   = (min_x + max_x) / 2
    cy_w   = (min_y + max_y) / 2

    draw_w = W - 2*PAD
    draw_h = H - 2*PAD

    def to_px(lx, ly):
        px = int(PAD + (lx - (cx_w - span/2)) / span * draw_w)
        py = int(PAD + (1.0 - (ly - (cy_w - span/2)) / span) * draw_h)
        return px, py

    pixels = [(15, 17, 26)] * (W * H)

    def set_px(px, py, c):
        if 0 <= px < W and 0 <= py < H:
            pixels[py * W + px] = c

    def draw_line(x0, y0, x1, y1, c, t=1):
        dx, dy = abs(x1-x0), abs(y1-y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        half = t // 2
        while True:
            for tx in range(-half, half+1):
                for ty in range(-half, half+1):
                    set_px(x0+tx, y0+ty, c)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2*err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 <  dx: err += dx; y0 += sy

    def draw_circle(cx, cy, r, c, fill=False):
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                d = math.hypot(dx, dy)
                if fill:
                    if d <= r: set_px(cx+dx, cy+dy, c)
                else:
                    if abs(d - r) < 1.2: set_px(cx+dx, cy+dy, c)

    def draw_arrow(x0, y0, x1, y1, c, head=12, t=2):
        draw_line(x0, y0, x1, y1, c, t)
        ang = math.atan2(y1-y0, x1-x0)
        for side in (0.42, -0.42):
            ax = int(x1 - head * math.cos(ang - side))
            ay = int(y1 - head * math.sin(ang - side))
            draw_line(x1, y1, ax, ay, c, t)

    def draw_dashed_poly(pts, c, dash=8, gap=6):
        n = len(pts)
        for i in range(n):
            ax, ay = to_px(*pts[i])
            bx, by = to_px(*pts[(i+1)%n])
            dx, dy = bx-ax, by-ay
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            ux, uy = dx/length, dy/length
            pos = 0.0
            drawing = True
            while pos < length:
                seg = dash if drawing else gap
                end = min(pos + seg, length)
                if drawing:
                    x0 = int(ax + pos*ux); y0 = int(ay + pos*uy)
                    x1 = int(ax + end*ux); y1 = int(ay + end*uy)
                    draw_line(x0, y0, x1, y1, c, 1)
                pos = end
                drawing = not drawing

    # --- draw layers (back to front) ---

    def draw_ellipse_outline(aa, bb, color, dashed=False, steps=180):
        pts = []
        for i in range(steps):
            t  = 2*math.pi*i/steps
            lx = aa*math.cos(t); ly = bb*math.sin(t)
            pts.append(to_px(cx + lx*cos_a - ly*sin_a,
                              cy + lx*sin_a + ly*cos_a))
        if dashed:
            dash, gap = 8, 6
            for i in range(len(pts)):
                x0,y0 = pts[i]; x1,y1 = pts[(i+1)%len(pts)]
                if i % (dash+gap) < dash:
                    draw_line(x0, y0, x1, y1, color, 1)
        else:
            for i in range(len(pts)):
                draw_line(*pts[i], *pts[(i+1)%len(pts)], color, 1)

    # Search boundary border (yellow)
    nh = len(hull_xy)
    for i in range(nh):
        draw_line(*to_px(*hull_xy[i]), *to_px(*hull_xy[(i+1)%nh]), (220, 200, 50), 2)

    # Original fitted ellipse (white)
    draw_ellipse_outline(a_raw, b_raw, (210, 210, 210))

    # Shrunk ellipse + flight path arrows (cyan)
    draw_ellipse_outline(a_s, b_s, (0, 190, 215))
    nw = len(wp_xy)
    for i in range(nw):
        ax, ay = to_px(*wp_xy[i])
        bx, by = to_px(*wp_xy[(i+1)%nw])
        draw_arrow(ax, ay, bx, by, (0, 190, 215), 10, 2)

    # Plane (green filled dot)
    ppx, ppy = to_px(plane_x, plane_y)
    draw_circle(ppx, ppy, 7, (40, 230, 90), fill=True)

    # Green arrow: plane → nearest waypoint (first in ordered list plane will go to)
    start_wp_idx = max(0, min(start_wp_idx, len(wp_xy) - 1))
    sx, sy = to_px(*wp_xy[start_wp_idx])
    draw_arrow(ppx, ppy, sx, sy, (40, 230, 90), 12, 2)

    _write_png(filename, W, H, pixels)
    print(f"[IMAGE] Saved → {filename}")


# ---------------------------------------------------------------------------
# Main mission loop

def run_mission_1():
    print("[MISSION 1] Starting …")

    # ------------------------------------------------------------------
    # Standby — wait until mission_state.json autonomy_active is True
    # ------------------------------------------------------------------
    # print("[MISSION 1] Standby … waiting for autonomy_active …")
    # while True:
    #     mission_state = load_state()
    #     controller_status = mission_state.get("controller_status", {})
    #     if controller_status.get("autonomy_active", False):
    #         print("[MISSION 1] autonomy_active = True — proceeding.")
    #         break
    #     time.sleep(STANDBY_INTERVAL_S)

    mission_state = load_state()
    while True:
        nav = load_nav_state()
        raw_search = nav.get("search_area", [])

        if isinstance(raw_search, list) and 3 <= len(raw_search) <= 6:
            break

        if isinstance(raw_search, list) and len(raw_search) == 0:
            print("[MISSION 1] Waiting for search_area from GCS...")
        else:
            print(f"[MISSION 1] Invalid search_area. Need 3–6 points, got: {raw_search}")

        time.sleep(1.0)

    search_coords = []
    for entry in raw_search:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            raise ValueError(f"Invalid search_area entry: {entry}")
        search_coords.append((float(entry[0]), float(entry[1])))


    # ------------------------------------------------------------------
    # Startup: write plan metadata & mission phase before anything else
    # ------------------------------------------------------------------
    nav_state = load_nav_state()          # fresh read

    nav = nav_state.get("navigation", {})
    nav["mission_phase"] = 1
    update_nav_state("navigation", nav)
    print("[STATE] mission_phase → 1")

    active_plan = {
        "plan_id": 1,
        "plan_type": "waypoint_pattern",
        "label": "Ellipse Pattern",
        "status": "building",
        "waypoints": [],
        "alt_ft": CRUISE_ALT_FT,
    }
    update_nav_state("active_plan", active_plan)
    

    update_nav_state("next_plan", 2)

    print("[STATE] active_plan → plan_id=1, plan_type='Ellipse Pattern', status='building'")
    print("[STATE] next_plan   → 2")

    # Write cruise altitude into the nav state so mission 2+ can read it
    update_nav_state("alt_ft", CRUISE_ALT_FT)
    print(f"[STATE] alt_ft → {CRUISE_ALT_FT} ft")

    # Build diamond and waypoints
    (waypoints_ll, hull_xy, ellipse_params,
      wp_xy, o_lat, o_lon) = build_ellipse_path(search_coords)
    
    # ------------------------------------------------------------------
    # Serialise the 12 waypoints into active_plan.waypoints
    # alt_m is read from active_plan.alt_m (already in the JSON)
    # ------------------------------------------------------------------
    nav_state   = load_nav_state()

    nav = nav_state.get("navigation", {})
    nav["mission_phase"] = 1
    update_nav_state("navigation", nav)
    print("[STATE] mission_phase → 1")

    # Clear stale Mission 2 trigger
    mra_refined = nav_state.get("mra_refined_loiter_target", {})
    mra_refined.update({
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False,
    })
    update_nav_state("mra_refined_loiter_target", mra_refined)
    print("[STATE] mra_refined_loiter_target.valid → False")
    active_plan = nav_state.get("active_plan", {})

    active_plan = {
        "plan_id": 1,
        "plan_type": "waypoint_pattern",
        "label": "Ellipse Pattern",
        "status": "ready",
        "waypoints": [
            {"lat": lat, "lon": lon, "alt_ft": CRUISE_ALT_FT}
            for lat, lon in waypoints_ll
        ],
        "alt_ft": CRUISE_ALT_FT,
    }
    update_nav_state("active_plan", active_plan)
    print(f"[STATE] active_plan.waypoints → {len(waypoints_ll)} entries written")

    print(f"[WAYPOINTS] {len(waypoints_ll)} waypoints generated:")
    for i, (lat, lon) in enumerate(waypoints_ll):
        print(f"  WP{i}  lat={lat:.7f}  lon={lon:.7f}")

    print("[MISSION 1] Plan is ready. Waiting for autonomy_active before live guidance …")
    while True:
        mission_state = load_state()
        controller_status = mission_state.get("controller_status", {})

        if controller_status.get("autonomy_active", False):
            print("[MISSION 1] autonomy_active = True — entering live guidance.")
            break

        time.sleep(STANDBY_INTERVAL_S)

    

    # Get initial telemetry
    print("[MISSION 1] Waiting for initial valid fusion telemetry...")
    while True:
        mission_state = load_state()
        fusion_log_path = mission_state.get("fusion_log", "")

        telem = read_latest_telemetry(fusion_log_path)

        if telem is not None:
            plane_lat = telem.get("lat_deg")
            plane_lon = telem.get("lon_deg")

            if plane_lat is not None and plane_lon is not None:
                break

        print("[MISSION 1] No valid telemetry yet. Waiting...")
        time.sleep(STANDBY_INTERVAL_S)

    plane_lat = telem["lat_deg"]
    plane_lon = telem["lon_deg"]
    plane_yaw = telem.get("yaw_deg", 0.0)

    if not waypoints_ll:
       raise RuntimeError("[MISSION 1] No waypoints generated — check build_ellipse_path output.")
    current_wp_idx = closest_wp_index(plane_lat, plane_lon, waypoints_ll)
    print(f"[NAV] Plane at ({plane_lat:.6f}, {plane_lon:.6f}) "
          f"→ starting on WP{current_wp_idx}")

    # Generate image
    if GENERATE_IMAGE:
        try:
            img_path = Path(__file__).resolve().parent / "mission1_map.png"
            generate_map_image(
                hull_xy, ellipse_params, wp_xy,
                plane_lat, plane_lon, current_wp_idx,
                o_lat, o_lon, filename=str(img_path),
            )
        except Exception:
            import traceback
            print("[ERROR] Image generation failed:")
            traceback.print_exc()

    # Write initial state
    nav = load_nav_state().get("navigation", {})
    nav["current_waypoint"]  = list(waypoints_ll[current_wp_idx])
    nav["guidance_waypoint"] = list(waypoints_ll[current_wp_idx])
    update_nav_state("navigation", nav)

    print("[MISSION 1] Entering guidance loop (Ctrl-C to stop) …")

    #mission_start = time.monotonic()

    mission1_live_start = time.time()
    MIN_MISSION1_RUN_TIME_S = 10.0

    try:
        while True:
            #---- timeout check ----------------------------------------
            #if MISSION_TIMEOUT_S > 0:
            #    elapsed = time.monotonic() - mission_start
            #    if elapsed >= MISSION_TIMEOUT_S:
            #        print(f"[MISSION 1] Timeout reached ({MISSION_TIMEOUT_S:.0f}s). Ending mission.")
            #        break
            #------------------------------------------------------------
            mission_state = load_state()
            fusion_log_path = mission_state.get("fusion_log", "")

            telem = read_latest_telemetry(fusion_log_path)
            if telem is None:
                print("[MISSION 1] Waiting for valid fusion telemetry...")
                time.sleep(UPDATE_INTERVAL_S)
                continue

            plane_lat = telem["lat_deg"]
            plane_lon = telem["lon_deg"]
            plane_yaw = telem.get("yaw_deg", 0.0)

            if plane_lat is None or plane_lon is None:
                print("[MISSION 1] Fusion telemetry missing lat_deg/lon_deg. Waiting...")
                time.sleep(UPDATE_INTERVAL_S)
                continue

            wp_lat, wp_lon = waypoints_ll[current_wp_idx]
            dist_to_wp = haversine(plane_lat, plane_lon, wp_lat, wp_lon)

            # Waypoint capture
            if dist_to_wp < WAYPOINT_RADIUS_M:
                current_wp_idx = (current_wp_idx + 1) % len(waypoints_ll)
                wp_lat, wp_lon = waypoints_ll[current_wp_idx]
                print(f"[NAV] Captured → WP{current_wp_idx} "
                      f"({wp_lat:.7f}, {wp_lon:.7f})")
                nav = load_nav_state().get("navigation", {})
                nav["current_waypoint"]  = [wp_lat, wp_lon]
                nav["guidance_waypoint"] = [wp_lat, wp_lon]
                update_nav_state("navigation", nav)

            # L1 guidance — compute lookahead point and write to state
            _, des_brng, dist = l1_guidance(
                plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon
            )
            guide_lat, guide_lon = destination(
                plane_lat, plane_lon, des_brng, min(L1_DISTANCE_M, dist)
            )
            nav = load_nav_state().get("navigation", {})
            nav["guidance_waypoint"] = [guide_lat, guide_lon]
            update_nav_state("navigation", nav)

            
            # Check exit condition — mra_refined_loiter_target.valid
            fresh = load_nav_state()
            


            target = fresh.get("mra_refined_loiter_target", {})

            mission1_runtime = time.time() - mission1_live_start

            if (
                mission1_runtime >= MIN_MISSION1_RUN_TIME_S
                and target.get("valid", False)
            ):
                print("[MISSION 1] Big Loiter coordinates received. Ending mission 1. Proceeding to Mission 2.")
                break
             

            time.sleep(UPDATE_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n[MISSION 1] Stopped.")

    # ------------------------------------------------------------------
    # Mission end: mark status complete, then hand off to mission 2
    # ------------------------------------------------------------------
    nav_state   = load_nav_state()
    active_plan = nav_state.get("active_plan", {})
    active_plan["status"] = "complete"
    update_nav_state("active_plan", active_plan)
    print("[STATE] active_plan.status → 'complete'")

    print("[MISSION 1] Done. Launching mission2_waypoint.py …")
    mission2_path = Path(__file__).resolve().parent / "mission2_waypoint.py"
    if mission2_path.exists():
        proc = subprocess.Popen([sys.executable, str(mission2_path)])
        print(f"[HANDOFF] Launched {mission2_path}")
        proc.wait()
        sys.exit(0)
    else:
        print(f"[WARN] mission2_waypoint.py not found at {mission2_path} — skipping handoff.")


if __name__ == "__main__":
    run_mission_1()