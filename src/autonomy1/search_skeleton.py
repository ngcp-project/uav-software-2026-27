"""
search_skeleton.py (v7) - 3-stage UAV search on ArduPlane SITL via MAVSDK-Python
(pip install "mavsdk<4").

Stage 1  ellipse + lawnmower: location unknown. Last year's method: fit an ellipse
                              along the area's long side, then sweep lawnmower lanes
                              inside it (--pattern lawnmower sweeps the raw polygon).
Stage 2  inward spiral/square: after RSSI_DEBOUNCE_N readings in a row above
                              RSSI_DETECT_DBM, keep flying the lawnmower until
                              the signal peaks and falls PEAK_DROP_DB, then
                              estimate the range from the peak RSSI and fly an
                              inward spiral (or square) onto the strongest reading.
                              If the estimate is tighter than the plane can turn
                              into, skip straight to stage 3. (--stage2 chooses;
                              expanding_square is kept for comparison.)
Stage 3  circle the target  : (default stage 2 = orbit) as soon as the lawnmower pass
                              peaks, loiter in a circle on the best position.
                              With --stage2 inward_*: after STRONG_DEBOUNCE_N readings
                              above RSSI_STRONG_DBM, loiter on the best position.
                              Real gradient homing is still a TODO.

RSSI comes from a SIMULATED hidden target (SimRssi). Swap in the Gazebo sensor
by replacing SimRssi with a class that has the same .read(x, y, alt) method.

Run order: start SITL, start QGroundControl, then
    python search_skeleton.py                                  built-in test box
    python search_skeleton.py --boundary uav_area_example.json area from the GCS file
    python search_skeleton.py --boundary area.json --true-coords   real lat/lon (not SITL)
    python search_skeleton.py --target -200,150                hidden test target (metres
                                                               from the area centre)
Area file (JSON, lat/lon):
    {"search_area": [[lat, lon], ...],        UAV search area (required)
     "keep_in":     [[lat, lon], ...],        optional
     "keep_out":    [[[lat, lon], ...], ...]} optional list of polygons
In the simulator the area is recentered on SITL home, because SITL starts in
Canberra; use --true-coords for a real flight.
"""

import argparse
import asyncio
import math
import random
import time

try:
    from mavsdk import System
    from mavsdk.mission_raw import MissionItem
except ImportError:          # plan generation (autonomy_plans.py) does not need MAVSDK
    System = MissionItem = None

# ---------------------------------------------------------------- config
CONNECTION_URL = "udpin://0.0.0.0:14540"   # SITL --out=udp:127.0.0.1:14540
CRUISE_ALT_M = 60.0                        # relative to home
TAKEOFF_PITCH_DEG = 15.0
ARM_RETRIES = 12                           # the sim may refuse to arm for a while after start
ARM_RETRY_DELAY_S = 5.0
ADD_RTL_AT_END = True

# Turn radius (see ArduPlane defaults: ~22 m/s cruise, 45 deg bank limit).
# Check yours with `param show TRIM_ARSPD_CM` / `param show LIM_ROLL_CD`.
TURN_RADIUS_M = 50.0
TURN_DIAMETER_FACTOR = 2.0     # consecutive lanes must be at least this many turn radii apart
TAKEOFF_HEADING_DEG = 0.0      # SITL start heading (sim_vehicle --custom-location=...,0)
TAKEOFF_XY = (0.0, 0.0)        # takeoff point in local metres (area is centred on home)
CLIMBOUT_DIST_M = 300.0        # where the plane is after climbing out
LANE_SPACING_M = 100.0   # >= the turn diameter (2 x TURN_RADIUS_M): plain back-and-forth lanes, no doubling back
                                           # the plane makes wide loops past the lane ends
SQUARE_SPACING_M = 120.0                   # expanding-square ring spacing (> 2 * R)
EDGE_CLEARANCE_M = 60.0                    # lane ENDS keep this far inside (>= R, room to turn)
ELLIPSE_MIN_TURN_DEG = 15.0                # last year: MIN_TURN_ANGLE_DEG
ELLIPSE_MAX_WAYPOINTS = 30                 # last year: MAXIMUM_WAYPOINTS
# Stage 2 (RF estimate is decent): fly INWARD toward the estimate instead of outward
STAGE2_DEFAULT = "orbit"                   # orbit | inward_square | inward_spiral | expanding_square
LOITER_RADIUS_M = 60.0                     # stage-3 circle radius (>= the turn radius)
ORBIT_REFIT_S = 28.0                       # circle this long, then re-fit the target position
ORBIT_REFITS = 2
INWARD_OUTER_MARGIN = 1.3                  # outer radius = margin x RSSI-estimated range
INWARD_MAX_OUTER_M = 300.0
INWARD_MIN_OUTER_M = 2.6 * TURN_RADIUS_M   # smaller than this is not flyable as a spiral
SIDE_CLEARANCE_M = 25.0                    # lanes run this far from the long sides

# Search boundary as (east_m, north_m) offsets from HOME. Replace with the
# GCS-provided boundary later (convert lat/lon -> offsets with to_local()).
BOUNDARY_OFFSETS_M = [(-400, -330), (400, -330), (400, 330), (-400, 330)]   # built-in test box, 800 x 660 m

# Stage thresholds
RSSI_DETECT_DBM = -78.0
RSSI_DEBOUNCE_N = 3
RSSI_STRONG_DBM = -65.0
STRONG_DEBOUNCE_N = 2
SMOOTH_N = 3          # average the last N readings when looking for the peak
PEAK_DROP_DB = 6.0    # stage 1 hands off once RSSI falls this far below its peak
PEAK_MIN_DBM = -74.0  # weaker peaks are treated as noise at the fringe: keep sweeping

# Simulated RF source: RSSI = REF - 10*n*log10(d / REF_DIST) + noise
# 3D distance (altitude included): -78 dBm at ~210 m, -65 dBm at 100 m.
SIM_TARGET_DEFAULT = (250.0, -120.0)       # (east_m, north_m) from home; farther than the
                                           # ~210 m detection range from home
SIM_RSSI_REF_DBM = -65.0
SIM_RSSI_REF_DIST_M = 100.0
SIM_PATHLOSS_EXP = 4.0
SIM_NOISE_SIGMA_DB = 2.0

EARTH_R = 6_371_000.0

# MAVLink constants used by the raw mission
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_TAKEOFF = 22


# ------------------------------------------------------------ geo helpers
def to_local(lat, lon, o_lat, o_lon):
    """lat/lon -> (east_m, north_m) relative to origin (flat-earth)."""
    x = math.radians(lon - o_lon) * EARTH_R * math.cos(math.radians(o_lat))
    y = math.radians(lat - o_lat) * EARTH_R
    return x, y


def from_local(x, y, o_lat, o_lon):
    lat = o_lat + math.degrees(y / EARTH_R)
    lon = o_lon + math.degrees(x / (EARTH_R * math.cos(math.radians(o_lat))))
    return lat, lon


# ------------------------------------------------------- polygon clipping
def _ccw(poly):
    n = len(poly)
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1]
               for i in range(n))
    return list(poly) if area > 0 else list(poly)[::-1]


def _clip_line(poly, px, py, dx, dy, clearance, lo, hi, side_clearance=None):
    """Clip the line P + t*D (t in [lo, hi]) to the convex CCW polygon shrunk by
    `clearance`. Returns (t_lo, t_hi), or None if nothing is left."""
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        L = math.hypot(ex, ey)
        # inward (left of a->b) distance at t=0 is c0, changing at rate c1
        c1 = (ex * dy - ey * dx) / L
        # edges running roughly ALONG the lane are "sides": they need less room
        # than the edges the plane has to turn around at
        cl = side_clearance if (side_clearance is not None and abs(c1) < 0.5) else clearance
        c0 = (ex * (py - ay) - ey * (px - ax)) / L - cl
        if abs(c1) < 1e-9:
            if c0 < -1e-6:
                return None
        else:
            t = -c0 / c1
            if c1 > 0:
                lo = max(lo, t)
            else:
                hi = min(hi, t)
    return (lo, hi) if hi >= lo else None


# ------------------------------------------------ general polygon helpers
def polygon_centroid(pts):
    """Plain vertex average (good enough as a local-frame origin)."""
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def convex_hull(pts):
    pts = sorted(set((float(x), float(y)) for x, y in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]          # CCW


def is_convex(poly):
    n = len(poly)
    sign = 0
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        cx, cy = poly[(i + 2) % n]
        c = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if abs(c) < 1e-9:
            continue
        if sign and (c > 0) != (sign > 0):
            return False
        sign = 1 if c > 0 else -1
    return True


def point_in_poly(p, poly):
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _dist_pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _segments_cross(p1, p2, p3, p4):
    def o(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2, d3, d4 = o(p3, p4, p1), o(p3, p4, p2), o(p1, p2, p3), o(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def seg_poly_distance(p, q, poly):
    """Distance from segment p-q to a polygon (0 if they touch/overlap)."""
    if point_in_poly(p, poly) or point_in_poly(q, poly):
        return 0.0
    n = len(poly)
    best = 1e18
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if _segments_cross(p, q, a, b):
            return 0.0
        best = min(best, _dist_pt_seg(p, a, b), _dist_pt_seg(q, a, b),
                   _dist_pt_seg(a, p, q), _dist_pt_seg(b, p, q))
    return best


def poly_distance(a, b):
    """Smallest distance between two polygons (0 if they touch/overlap)."""
    best = 1e18
    for i in range(len(a)):
        best = min(best, seg_poly_distance(a[i], a[(i + 1) % len(a)], b))
    if point_in_poly(b[0], a):
        return 0.0
    return best


def load_boundary(path):
    """Read the GCS area file.  JSON:
        {"search_area": [[lat, lon], ...],          # required (UAV search area)
         "keep_in":     [[lat, lon], ...],          # optional
         "keep_out":    [[[lat, lon], ...], ...]}   # optional, list of polygons"""
    import json
    with open(path) as f:
        d = json.load(f)
    pts = lambda poly: [(float(a), float(b)) for a, b in poly]
    return {"search": pts(d["search_area"]),
            "keep_in": pts(d["keep_in"]) if d.get("keep_in") else None,
            "keep_out": [pts(k) for k in d.get("keep_out", [])]}


def prepare_area(search_xy, keep_in_xy=None, keep_out_xys=(), margin_m=0.0):
    """Turn the GCS polygons (local metres) into the convex search polygon the
    builders use.  Returns (polygon, messages, fatal).  Concave areas are
    replaced by their convex hull, like last year's code."""
    msgs, fatal = [], False
    poly = _ccw(search_xy)
    if not is_convex(poly):
        poly = _ccw(convex_hull(poly))
        msgs.append("search area is not convex -> using its convex hull")
    if keep_in_xy:
        kin = _ccw(keep_in_xy)
        bad = [v for v in poly if not point_in_poly(v, kin)]
        if bad:
            msgs.append(f"{len(bad)} search-area corner(s) lie outside the keep-in zone")
            fatal = True
    for i, ko in enumerate(keep_out_xys):
        d = poly_distance(poly, _ccw(ko))
        if d < 1.0:
            msgs.append(f"search area touches/overlaps keep-out zone #{i + 1}: "
                        "legs stay inset by the edge clearance, path is re-checked")
        elif d < margin_m:
            msgs.append(f"search area is only {d:.0f} m from keep-out zone #{i + 1}")
    return poly, msgs, fatal


def check_path(wps_xy, keep_out_xys, margin_m):
    """Messages for any leg that passes within margin_m of a keep-out zone."""
    out = []
    for k, ko in enumerate(keep_out_xys):
        ko = _ccw(ko)
        worst = min((seg_poly_distance(wps_xy[i], wps_xy[i + 1], ko)
                     for i in range(len(wps_xy) - 1)), default=1e18)
        if worst < margin_m:
            out.append(f"path comes within {worst:.0f} m of keep-out zone #{k + 1} "
                       f"(margin {margin_m:.0f} m)")
    return out


# -------------------------------------------------------- lawnmower builder
def _long_axis_angle(poly):
    """Principal (PCA) axis of the vertex cloud; used by the ellipse fit."""
    n = len(poly)
    mx = sum(p[0] for p in poly) / n
    my = sum(p[1] for p in poly) / n
    cxx = sum((p[0] - mx) ** 2 for p in poly)
    cyy = sum((p[1] - my) ** 2 for p in poly)
    cxy = sum((p[0] - mx) * (p[1] - my) for p in poly)
    return 0.5 * math.atan2(2 * cxy, cxx - cyy)


def _sweep_angle(poly):
    """Lane direction that gives the narrowest cross-section (fewest lanes and
    turns). Tries the direction of every polygon edge."""
    best_ang, best_w = 0.0, 1e18
    n = len(poly)
    for i in range(n):
        ex, ey = poly[(i + 1) % n][0] - poly[i][0], poly[(i + 1) % n][1] - poly[i][1]
        if math.hypot(ex, ey) < 1e-6:
            continue
        ang = math.atan2(ey, ex)
        vx, vy = -math.sin(ang), math.cos(ang)
        s = [p[0] * vx + p[1] * vy for p in poly]
        w = max(s) - min(s)
        if w < best_w:
            best_w, best_ang = w, ang
    return best_ang


def lane_offsets(smin, smax, spacing_m, clearance_m, min_spacing_m):
    """Evenly spread lane positions between the clearance lines. Spacing never
    exceeds `spacing_m`, and never drops below the turn diameter (`min_spacing_m`).
    Small areas get 1 centered lane (with a warning from the caller)."""
    span = (smax - smin) - 2 * clearance_m
    if span <= 0:
        return [0.5 * (smin + smax)]
    gaps = max(1, math.ceil(span / spacing_m - 1e-9))
    if span / gaps < min_spacing_m:
        gaps = int(span // min_spacing_m)
    if gaps < 1:
        return [0.5 * (smin + smax)]
    step = span / gaps
    return [smin + clearance_m + k * step for k in range(gaps + 1)]


def build_lawnmower(poly_xy, spacing_m=LANE_SPACING_M, clearance_m=EDGE_CLEARANCE_M,
                    side_clearance_m=None, **_):
    """Boustrophedon waypoints (x, y in metres) for a convex polygon.

    Lanes are spread evenly across the part of the area where a lane is long
    enough to be worth flying, so skewed quadrilaterals work as well as boxes."""
    poly = _ccw(poly_xy)
    ang = _sweep_angle(poly)
    ux, uy = math.cos(ang), math.sin(ang)      # along each leg
    vx, vy = -uy, ux                            # across legs
    s_vals = [p[0] * vx + p[1] * vy for p in poly]
    smin, smax = min(s_vals), max(s_vals)
    side = min(clearance_m, SIDE_CLEARANCE_M if side_clearance_m is None else side_clearance_m)

    def clip(s, end_clear):
        return _clip_line(poly, s * vx, s * vy, ux, uy, end_clear, -1e9, 1e9, side)

    # relax the demands step by step if the area is small
    for end_clear, min_len in ((clearance_m, 2 * TURN_RADIUS_M), (clearance_m, 50.0),
                               (0.5 * clearance_m, 50.0), (0.5 * clearance_m, 20.0)):
        valid, s = [], smin + side
        while s <= smax - side:
            iv = clip(s, end_clear)
            if iv and iv[1] - iv[0] >= min_len:
                valid.append(s)
            s += 2.0
        if valid:
            break
    if not valid:
        return []
    # an explicit tighter spacing is honoured; the plane then turns in wide loops past the lane ends
    lanes = lane_offsets(valid[0], valid[-1], spacing_m, 0.0, min(2 * TURN_RADIUS_M, spacing_m))

    segs = []
    for s in lanes:
        iv = clip(s, end_clear)
        if iv is None or iv[1] - iv[0] <= 1.0:
            continue
        segs.append(((s * vx + iv[0] * ux, s * vy + iv[0] * uy),
                     (s * vx + iv[1] * ux, s * vy + iv[1] * uy), s))
    if not segs:
        return []

    # Lanes closer together than the plane can U-turn are flown with a stride
    # (1,3,5,..then 2,4,6..) so every turn is wide enough; coverage is unchanged.
    gap = abs(segs[1][2] - segs[0][2]) if len(segs) > 1 else spacing_m
    k = max(1, math.ceil(TURN_DIAMETER_FACTOR * TURN_RADIUS_M / max(gap, 1.0)))
    order = [i for r in range(k) for i in range(r, len(segs), k)] if k > 1 else list(range(len(segs)))

    def assemble(idx_order, flip):
        out = []
        for i in idx_order:
            a_, b_, _s = segs[i]
            out += [b_, a_] if flip else [a_, b_]
            flip = not flip
        return out

    # pick the start (which end, which direction) nearest the point the plane
    # reaches after climbing out along the runway heading, to avoid a big U-turn
    hd = math.radians(TAKEOFF_HEADING_DEG)
    px = TAKEOFF_XY[0] + CLIMBOUT_DIST_M * math.sin(hd)
    py = TAKEOFF_XY[1] + CLIMBOUT_DIST_M * math.cos(hd)
    best = None
    for idx_order in (order, order[::-1]):
        for flip in (False, True):
            w = assemble(idx_order, flip)
            c = math.hypot(w[0][0] - px, w[0][1] - py)
            if best is None or c < best[0]:
                best = (c, w)
    return best[1]


# ----------------------------------------------- expanding square builder
def build_expanding_square(poly_xy, center_xy=(0.0, 0.0), spacing_m=SQUARE_SPACING_M,
                           clearance_m=EDGE_CLEARANCE_M, max_legs=80, **_):
    """Expanding square (legs 1,1,2,2,3,3,... x spacing, 90 deg turns, clockwise,
    first leg north) centered on `center_xy`, clipped to the convex polygon.

    Each leg is clipped to the boundary (shrunk by `clearance_m`). Because the
    polygon is convex, a straight line between any two kept points is inside,
    so skipped (fully outside) legs just produce a chord. Stops once four legs
    in a row are completely outside."""
    poly = _ccw(poly_xy)
    dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]        # N, E, S, W
    p = (float(center_xy[0]), float(center_xy[1]))
    pts, outside_run = [], 0

    def add(q):
        if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1.0:
            pts.append(q)

    for k in range(max_legs):
        n = k // 2 + 1
        dx, dy = dirs[k % 4]
        q = (p[0] + dx * n * spacing_m, p[1] + dy * n * spacing_m)
        iv = _clip_line(poly, p[0], p[1], q[0] - p[0], q[1] - p[1], clearance_m, 0.0, 1.0)
        if iv:
            add((p[0] + iv[0] * (q[0] - p[0]), p[1] + iv[0] * (q[1] - p[1])))
            add((p[0] + iv[1] * (q[0] - p[0]), p[1] + iv[1] * (q[1] - p[1])))
            outside_run = 0
        else:
            outside_run += 1
        p = q
        if outside_run >= 4:
            break
    if len(pts) < 2:
        raise ValueError("expanding square center is outside the clearance zone")
    return pts


# ------------------------------------------------ inward stage-2 patterns
def range_from_rssi(rssi_dbm, alt_m=0.0):
    """Horizontal range implied by a reading. Uses the SIMULATED path-loss model;
    replace with a model calibrated on the real RF sensor."""
    d = SIM_RSSI_REF_DIST_M * 10 ** ((SIM_RSSI_REF_DBM - rssi_dbm) / (10 * SIM_PATHLOSS_EXP))
    return math.sqrt(max(d * d - alt_m * alt_m, 0.0))


def model_rssi(d3d_m):
    """Mean RSSI at a 3D distance (the SIMULATED model; calibrate for real RF)."""
    return SIM_RSSI_REF_DBM - 10 * SIM_PATHLOSS_EXP * math.log10(max(d3d_m, 1.0) / SIM_RSSI_REF_DIST_M)


def estimate_target(readings, alt_m, init_xy, span_m=220.0):
    """Least-squares position of the transmitter from (x, y, rssi) readings:
    try every point on a grid around `init_xy`, keep the one whose predicted RSSI
    matches the readings best. A straight pass can't tell left from right of the
    track; readings taken while circling can."""
    pts = [(x, y, r) for x, y, r in readings if r >= RSSI_DETECT_DBM - 8.0]
    if len(pts) < 6:
        return init_xy

    def cost(cx, cy):
        return sum((r - model_rssi(math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + alt_m ** 2))) ** 2
                   for x, y, r in pts)

    best, step = (init_xy[0], init_xy[1], cost(*init_xy)), 10.0
    for pass_span, pass_step in ((span_m, 10.0), (12.0, 2.0)):
        cx0, cy0 = best[0], best[1]
        n = int(pass_span / pass_step)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                cx, cy = cx0 + i * pass_step, cy0 + j * pass_step
                c = cost(cx, cy)
                if c < best[2]:
                    best = (cx, cy, c)
    return (best[0], best[1])


def _clip_polyline(poly, pts, clearance_m):
    """Keep only the parts of a path that lie inside the (convex) region."""
    out = []

    def add(q):
        if not out or math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > 1.0:
            out.append(q)

    for i in range(len(pts) - 1):
        p0, p1 = pts[i], pts[i + 1]
        iv = _clip_line(poly, p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1],
                        clearance_m, 0.0, 1.0)
        if iv:
            add((p0[0] + iv[0] * (p1[0] - p0[0]), p0[1] + iv[0] * (p1[1] - p0[1])))
            add((p0[0] + iv[1] * (p1[0] - p0[0]), p0[1] + iv[1] * (p1[1] - p0[1])))
    return out


def build_inward_spiral(poly_xy, center_xy=(0.0, 0.0), outer_m=200.0,
                        spacing_m=SQUARE_SPACING_M, clearance_m=0.0, **_):
    """Round spiral from radius `outer_m` in toward the center, one ring per
    `spacing_m`. Stops at the plane's turn radius, then ends on the center.
    Clockwise, starting north of the center."""
    poly = _ccw(poly_xy)
    cx, cy = center_xy
    r_min = TURN_RADIUS_M
    if outer_m < r_min + 1.0:
        raise ValueError("outer radius is smaller than the turn radius")
    pts, theta = [], 0.0
    while True:
        r = outer_m - spacing_m * theta / (2 * math.pi)
        if r < r_min:
            break
        pts.append((cx + r * math.sin(theta), cy + r * math.cos(theta)))
        theta += math.radians(max(10.0, min(30.0, math.degrees(60.0 / r))))
    pts.append((cx, cy))
    kept = _clip_polyline(poly, pts, clearance_m)
    if len(kept) < 2:
        raise ValueError("inward spiral does not fit in the area")
    return kept


def build_inward_square(poly_xy, center_xy=(0.0, 0.0), outer_m=200.0,
                        spacing_m=SQUARE_SPACING_M, clearance_m=0.0, **_):
    """Square spiral from half-width `outer_m` in toward the center, one ring per
    `spacing_m`, ending on the center."""
    poly = _ccw(poly_xy)
    cx, cy = center_xy
    if outer_m < 0.5 * spacing_m:
        raise ValueError("outer half-width is smaller than half the spacing")
    l, r, b, tp = cx - outer_m, cx + outer_m, cy - outer_m, cy + outer_m
    s = spacing_m
    pts = [(l, b)]
    while True:
        pts.append((r, b)); b += s
        if b >= tp:
            break
        pts.append((r, tp)); r -= s
        if r <= l:
            break
        pts.append((l, tp)); tp -= s
        if tp <= b:
            break
        pts.append((l, b)); l += s
        if l >= r:
            break
    pts.append((cx, cy))
    kept = _clip_polyline(poly, pts, clearance_m)
    if len(kept) < 2:
        raise ValueError("inward square does not fit in the area")
    return kept


def build_stage2(kind, region_xy, center_xy, outer_m, spacing_m, clearance_m):
    if kind == "inward_spiral":
        return build_inward_spiral(region_xy, center_xy, outer_m, spacing_m, clearance_m)
    if kind == "inward_square":
        return build_inward_square(region_xy, center_xy, outer_m, spacing_m, clearance_m)
    return build_expanding_square(region_xy, center_xy=center_xy, spacing_m=spacing_m,
                                  clearance_m=clearance_m)


def _inset_distance(p, poly):
    """Smallest inward distance from p to the edges of a CCW convex polygon
    (negative if p is outside)."""
    n = len(poly)
    d = 1e18
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        d = min(d, (ex * (p[1] - ay) - ey * (p[0] - ax)) / math.hypot(ex, ey))
    return d


def fit_ellipse_region(poly_xy, clearance_m=EDGE_CLEARANCE_M, n=96):
    """Last year's search-area step, in metres: convex hull -> oriented ellipse
    (PCA axis = the long direction, half-extents from the hull via max()) ->
    shrink until every outline point is `clearance_m` inside the hull.
    Returns the shrunk ellipse outline as a CCW convex polygon of n points."""
    hull = _ccw(convex_hull(poly_xy))
    cx, cy = polygon_centroid(hull)
    ang = _long_axis_angle(hull)
    ux, uy = math.cos(ang), math.sin(ang)
    vx, vy = -uy, ux
    a = max(abs((p[0] - cx) * ux + (p[1] - cy) * uy) for p in hull)
    b = max(abs((p[0] - cx) * vx + (p[1] - cy) * vy) for p in hull)

    def pts(scale, count):
        out = []
        for k in range(count):
            th = 2 * math.pi * k / count
            x, y = scale * a * math.cos(th), scale * b * math.sin(th)
            out.append((cx + x * ux + y * vx, cy + x * uy + y * vy))
        return out

    lo, hi = 0.0, 1.0
    for _i in range(40):
        mid = 0.5 * (lo + hi)
        if all(_inset_distance(q, hull) >= clearance_m for q in pts(mid, 180)):
            lo = mid
        else:
            hi = mid
    if lo < 0.05:
        raise ValueError("area too small for an ellipse with this clearance")

    # Last year's uniform shrink wastes a lot of a skewed area. Grow the ellipse
    # again (move the centre, stretch each axis) while it still keeps the clearance.
    # Deterministic hill climb; the outline never leaves the clearance zone.
    def outline(c0, c1, ea, eb, count):
        out = []
        for k in range(count):
            th = 2 * math.pi * k / count
            x, y = ea * math.cos(th), eb * math.sin(th)
            out.append((c0 + x * ux + y * vx, c1 + x * uy + y * vy))
        return out

    best = (cx, cy, lo * a, lo * b)
    rng = random.Random(0)
    step = 0.1 * min(best[2], best[3])
    for it in range(600):
        cand = (best[0] + rng.gauss(0, step), best[1] + rng.gauss(0, step),
                best[2] * (1 + rng.gauss(0, 0.04)), best[3] * (1 + rng.gauss(0, 0.04)))
        if cand[2] * cand[3] <= best[2] * best[3] or min(cand[2], cand[3]) < 1.0:
            continue
        if all(_inset_distance(q, hull) >= clearance_m for q in outline(*cand, 120)):
            best = cand
        if it % 150 == 149:
            step *= 0.5
    return outline(*best, n)


def build_ellipse(poly_xy, clearance_m=EDGE_CLEARANCE_M, start_xy=None,
                  min_turn_deg=ELLIPSE_MIN_TURN_DEG, max_wps=ELLIPSE_MAX_WAYPOINTS, **_):
    """Last year's Mission 1 boundary patrol: fly around the shrunk ellipse.
    Downsampled by minimum turn angle and waypoint cap, starting nearest the plane."""
    dense = fit_ellipse_region(poly_xy, clearance_m, 360)

    # downsample: a waypoint only when the heading has changed >= min_turn_deg
    keep, last_head = [dense[0]], None
    for i in range(1, len(dense) + 1):
        p0, p1 = keep[-1], dense[i % len(dense)]
        head = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        if last_head is None:
            last_head = head
            continue
        turn = abs((math.degrees(head - last_head) + 180) % 360 - 180)
        if turn >= min_turn_deg and i < len(dense):
            keep.append(dense[i])
            last_head = None
    while len(keep) > max_wps:                       # waypoint cap
        keep = keep[::2] if len(keep) > 2 * max_wps else keep[:-1]
    if len(keep) > max_wps:
        keep = keep[:max_wps]

    if start_xy is not None:                          # begin at the waypoint nearest the plane
        k = min(range(len(keep)), key=lambda j: math.hypot(keep[j][0] - start_xy[0],
                                                           keep[j][1] - start_xy[1]))
        keep = keep[k:] + keep[:k]
    return keep + [keep[0]]                           # close the loop


def build_ellipse_lawnmower(poly_xy, spacing_m=LANE_SPACING_M, clearance_m=EDGE_CLEARANCE_M, **_):
    """Last year's approach: boundary -> ellipse along the long side -> lawnmower
    lanes inside that ellipse. The ellipse is already inset by `clearance_m`."""
    region = fit_ellipse_region(poly_xy, clearance_m)
    return build_lawnmower(region, spacing_m=spacing_m, clearance_m=0.0, side_clearance_m=0.0)


# patterns that search inside the fitted ellipse instead of the raw polygon
REGION_PATTERNS = {"ellipse_lawnmower"}

PATTERN_BUILDERS = {
    "ellipse_lawnmower": build_ellipse_lawnmower,
    "lawnmower": build_lawnmower,
    "expanding_square": build_expanding_square,
    "ellipse": build_ellipse,
}


# ------------------------------------------------------- simulated sensor
class SimRssi:
    """Hidden stationary RF source. Swap for the Gazebo sensor later."""

    def __init__(self, target_xy, seed=None):
        self.target = target_xy
        self.rng = random.Random(seed)

    def read(self, x, y, alt_m):
        d = math.sqrt((x - self.target[0]) ** 2 + (y - self.target[1]) ** 2 + alt_m ** 2)
        d = max(d, 1.0)
        mean = SIM_RSSI_REF_DBM - 10 * SIM_PATHLOSS_EXP * math.log10(d / SIM_RSSI_REF_DIST_M)
        return mean + self.rng.gauss(0.0, SIM_NOISE_SIGMA_DB)


# ------------------------------------------------- stage state machine
class SearchLogic:
    """Pure logic (no MAVSDK) so it can be tested offline.

    update() returns None, ("square", center_xy) or ("homing", target_xy)."""

    def __init__(self, strong_handoff=True):
        self.strong_handoff = strong_handoff   # False: an inward square/spiral is flown to the end
        self.stage = 1
        self.hits = 0
        self.tracking = False          # stage 1: signal seen, waiting for the peak
        self.drops = 0
        self.recent = []               # last few raw readings (for smoothing)
        self.readings = []             # every (x, y, rssi), used to refine the target position
        self.best_rssi = -999.0        # best SMOOTHED reading so far
        self.best_xy = None

    def record(self, xy, rssi):
        """Only log the reading (no stage switching): used to finish the whole
        lawnmower before deciding where the target is."""
        if rssi is None:
            return
        self.recent = (self.recent + [rssi])[-SMOOTH_N:]
        self.readings = (self.readings + [(xy[0], xy[1], rssi)])[-3000:]
        smooth = sum(self.recent) / len(self.recent)
        if smooth > self.best_rssi:
            self.best_rssi, self.best_xy = smooth, xy

    def force_handoff(self):
        """Stage 1 pattern ended while tracking: use the best reading we have."""
        if self.stage == 1 and self.tracking:
            self.stage, self.tracking = 2, False
            return ("square", self.best_xy)
        return None

    def update(self, xy, rssi):
        if rssi is None:
            return None
        self.recent = (self.recent + [rssi])[-SMOOTH_N:]
        self.readings = (self.readings + [(xy[0], xy[1], rssi)])[-3000:]
        smooth = sum(self.recent) / len(self.recent)
        if smooth > self.best_rssi:
            self.best_rssi, self.best_xy = smooth, xy

        if self.stage == 1:
            if not self.tracking:
                self.hits = self.hits + 1 if rssi >= RSSI_DETECT_DBM else 0
                if self.hits >= RSSI_DEBOUNCE_N:
                    self.tracking = True          # keep flying until we pass the peak
            else:
                # closest approach passed when the signal falls PEAK_DROP_DB below the best
                self.drops = self.drops + 1 if smooth < self.best_rssi - PEAK_DROP_DB else 0
                if self.drops >= RSSI_DEBOUNCE_N or rssi >= RSSI_STRONG_DBM:
                    if self.best_rssi < PEAK_MIN_DBM and rssi < RSSI_STRONG_DBM:
                        # weak peak = noise at the edge of range, not a real pass
                        self.tracking, self.hits, self.drops = False, 0, 0
                        self.best_rssi, self.best_xy, self.recent = -999.0, None, []
                        return None
                    self.stage, self.tracking, self.hits = 2, False, 0
                    return ("square", self.best_xy)
        elif self.stage == 2 and self.strong_handoff:
            self.hits = self.hits + 1 if rssi >= RSSI_STRONG_DBM else 0
            if self.hits >= STRONG_DEBOUNCE_N:
                self.stage, self.hits = 3, 0
                return ("homing", self.best_xy)
        return None


# ------------------------------------------------------ raw mission builders
def _item(seq, frame, cmd, current, p1, lat, lon, z, p3=0.0):
    return MissionItem(seq, frame, cmd, current, 1, p1, 0.0, p3, 0.0,
                       int(round(lat * 1e7)), int(round(lon * 1e7)), float(z), 0)


def make_mission_items(home_lat, home_lon, home_alt_amsl, wps_ll, alt_m, rtl=True,
                       takeoff_ll=None):
    """Takeoff mission. ArduPilot convention: seq 0 = home, seq 1 = takeoff."""
    items = [_item(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 0,
                   home_lat, home_lon, home_alt_amsl)]
    # lat/lon 0,0 on a takeoff item = climb out on the current heading;
    # a real lat/lon makes the plane climb out toward that point instead
    tk_lat, tk_lon = takeoff_ll if takeoff_ll else (0.0, 0.0)
    items.append(_item(1, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_TAKEOFF, 1,
                       TAKEOFF_PITCH_DEG, tk_lat, tk_lon, alt_m))
    for lat, lon in wps_ll:
        items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           MAV_CMD_NAV_WAYPOINT, 0, 0, lat, lon, alt_m))
    if rtl:
        items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0.0, 0.0, 0))
    return items


def make_followup_items(home_lat, home_lon, home_alt_amsl, wps_ll, alt_m, rtl=True,
                        nav_cmd=MAV_CMD_NAV_WAYPOINT, loiter_radius_m=0.0):
    """In-flight replacement mission (no takeoff). Item 1 is the active one."""
    items = [_item(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 0,
                   home_lat, home_lon, home_alt_amsl)]
    for i, (lat, lon) in enumerate(wps_ll):
        items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT, nav_cmd,
                           1 if i == 0 else 0, 0, lat, lon, alt_m, p3=loiter_radius_m))
    if rtl:
        items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0.0, 0.0, 0))
    return items


async def switch_mission(drone, items):
    """Replace the mission while flying: upload, jump to item 1, make sure AUTO."""
    await drone.mission_raw.upload_mission(items)
    try:
        await drone.mission_raw.set_current_mission_item(1)
    except Exception as exc:
        print(f"[MISSION] set_current_mission_item failed ({exc})")
    try:
        await drone.mission_raw.start_mission()
    except Exception as exc:
        print(f"[MISSION] start_mission failed ({exc}); if the plane is not "
              "following the new path type  mode AUTO  in MAVProxy.")


# -------------------------------------------------------------------- main
async def wait_ready(drone, timeout_s=60.0):
    """Wait until position, home and the IMU/gyro checks are all OK."""
    async def _wait():
        async for h in drone.telemetry.health():
            if (h.is_global_position_ok and h.is_home_position_ok
                    and h.is_accelerometer_calibration_ok and h.is_gyrometer_calibration_ok):
                return
    try:
        await asyncio.wait_for(_wait(), timeout_s)
    except asyncio.TimeoutError:
        print("[MAVSDK] sensors not all reported OK after "
              f"{timeout_s:.0f} s; trying to arm anyway")


async def arm_with_retry(drone):
    """The simulator can refuse to arm right after start ('Accels inconsistent'
    while the IMUs settle). Retry instead of crashing."""
    for i in range(ARM_RETRIES):
        try:
            await drone.action.arm()
            return
        except Exception as exc:
            print(f"[MAVSDK] arm refused ({exc}); attempt {i + 1}/{ARM_RETRIES}. "
                  f"Look for the reason in the QGC banner. Retrying in {ARM_RETRY_DELAY_S:.0f} s")
            await asyncio.sleep(ARM_RETRY_DELAY_S)
    raise RuntimeError("could not arm; see the pre-arm error shown in QGC / MAVProxy")


async def main(pattern, target_xy, boundary_path=None, true_coords=False,
               spacing_m=LANE_SPACING_M, clearance_m=EDGE_CLEARANCE_M, force=False,
               stage2=STAGE2_DEFAULT, takeoff_toward_first=False, finish_first=True, pause_before_start=True):
    boundary = load_boundary(boundary_path) if boundary_path else None
    if System is None:
        print("[ERROR] the mavsdk package is not installed in this Python environment.\n"
              "        Run:  pip install \"mavsdk<4\"   (or activate the venv that has it)")
        return
    drone = System()
    await drone.connect(system_address=CONNECTION_URL)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[MAVSDK] connected")
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[MAVSDK] GPS + home OK")
            break
    async for home in drone.telemetry.home():
        break
    await wait_ready(drone)
    h_lat, h_lon, h_alt = home.latitude_deg, home.longitude_deg, home.absolute_altitude_m
    print(f"[HOME] {h_lat:.6f}, {h_lon:.6f}, {h_alt:.0f} m AMSL")

    # ---- search area: from the GCS file, or the built-in test box -------------
    keep_in_xy, keep_outs_xy = None, []
    if boundary:
        c_lat, c_lon = polygon_centroid(boundary["search"])
        conv = lambda poly: [to_local(a, b, c_lat, c_lon) for a, b in poly]
        search_xy = conv(boundary["search"])
        keep_in_xy = conv(boundary["keep_in"]) if boundary["keep_in"] else None
        keep_outs_xy = [conv(k) for k in boundary["keep_out"]]
        # sim: drop the area onto the SITL home; real flight: keep true coordinates
        o_lat, o_lon = (c_lat, c_lon) if true_coords else (h_lat, h_lon)
        print(f"[AREA] {boundary_path}: {len(search_xy)} corners, "
              f"{'true coordinates' if true_coords else 'recentered on SITL home'}")
    else:
        search_xy, o_lat, o_lon = BOUNDARY_OFFSETS_M, h_lat, h_lon
        print("[AREA] built-in test box (no --boundary given)")

    area_xy, msgs, fatal = prepare_area(search_xy, keep_in_xy, keep_outs_xy, TURN_RADIUS_M)
    for m in msgs:
        print(f"[AREA] {'ERROR' if fatal else 'note'}: {m}")
    xs = [p[0] for p in area_xy]
    ys = [p[1] for p in area_xy]
    print(f"[AREA] bounding box {max(xs) - min(xs):.0f} x {max(ys) - min(ys):.0f} m, "
          f"lane spacing <= {spacing_m:.0f} m, edge clearance {clearance_m:.0f} m")

    def ll(pts):
        return [from_local(x, y, o_lat, o_lon) for x, y in pts]

    build_kw = dict(spacing_m=spacing_m, clearance_m=clearance_m,
                    start_xy=to_local(h_lat, h_lon, o_lat, o_lon))
    # if the first try passes too close to a keep-out zone, push the lanes further in
    region_xy, sq_clear, wps_xy, problems = area_xy, clearance_m, [], []
    if pattern in REGION_PATTERNS:
        # last year's method: ellipse along the long side, lanes inside the ellipse
        for inset in (clearance_m, 1.5 * clearance_m, 2.0 * clearance_m):
            try:
                region_xy = fit_ellipse_region(area_xy, inset)
            except ValueError as exc:
                print(f"[AREA] ERROR: {exc}")
                fatal = True
                break
            wps_xy = build_lawnmower(region_xy, spacing_m=spacing_m, clearance_m=0.0,
                                     side_clearance_m=0.0)
            problems = check_path(wps_xy, keep_outs_xy, TURN_RADIUS_M) if wps_xy else []
            if not problems:
                break
            print(f"[SAFETY] ellipse inset {inset:.0f} m: {problems[0]}; trying more room")
        sq_clear = 0.0                      # the ellipse is already inset
        ex = [p[0] for p in region_xy]
        ey = [p[1] for p in region_xy]
        print(f"[AREA] search ellipse {max(ex) - min(ex):.0f} x {max(ey) - min(ey):.0f} m "
              "(fitted to the long side, inset by the clearance)")
    else:
        for side in (SIDE_CLEARANCE_M, 0.75 * TURN_RADIUS_M, TURN_RADIUS_M):
            kw = dict(build_kw, side_clearance_m=side)
            wps_xy = PATTERN_BUILDERS[pattern](area_xy, **kw)
            problems = check_path(wps_xy, keep_outs_xy, TURN_RADIUS_M) if wps_xy else []
            if not problems:
                break
            print(f"[SAFETY] side clearance {side:.0f} m: {problems[0]}; trying more room")
    if len(wps_xy) < 2:
        print("[AREA] ERROR: no waypoints fit; area too small for this clearance")
        fatal = True
    for m in problems:
        print(f"[SAFETY] {m}")
    if (fatal or problems) and not force:
        print("[ABORT] fix the area / spacing / clearance, or pass --force to fly anyway")
        return
    if len(wps_xy) <= 2:
        print("[AREA] warning: only one lane fits; consider a smaller --clearance")

    # sim target is given in the same local frame as the area (metres from the
    # area centre; with no --boundary and in sim mode that is the SITL home)
    sensor = SimRssi(target_xy)
    logic = SearchLogic(strong_handoff=stage2 not in ("inward_square", "inward_spiral"))
    t_lat, t_lon = from_local(target_xy[0], target_xy[1], o_lat, o_lon)
    print(f"[SIM] hidden target at E{target_xy[0]:.0f} N{target_xy[1]:.0f} m  "
          f"(lat {t_lat:.6f}, lon {t_lon:.6f}; the plane should end up circling this spot)")
    print(f"[STAGE 1] {pattern}: {len(wps_xy)} waypoints")
    await drone.mission_raw.upload_mission(
        make_mission_items(h_lat, h_lon, h_alt, ll(wps_xy), CRUISE_ALT_M, ADD_RTL_AT_END,
                           takeoff_ll=ll(wps_xy)[0] if takeoff_toward_first else None))
    print("[MISSION] uploaded (check QGroundControl)")
    if pause_before_start:
        print("\n[QGC] The waypoints are on the plane but QGroundControl does not draw them yet.\n"
              "      In QGC:  Plan view (top-left menu) -> File (left side) -> Vehicle: Download.\n"
              "      Then go back to the Fly view: you will see all the waypoints on the map.\n")
        await asyncio.get_running_loop().run_in_executor(
            None, input, "      Press ENTER here to arm and start the flight... ")

    await arm_with_retry(drone)
    print("[MAVSDK] armed")
    try:
        await drone.mission_raw.start_mission()
        print("[MISSION] started")
    except Exception as exc:
        print(f"[MISSION] start_mission failed ({exc}).")
        print("          Fallback: type  mode AUTO  at the MAVProxy prompt "
              "(or press Start Mission in QGC).")

    progress = {"cur": 0, "total": 0}
    mission_start = time.monotonic()
    orbit = None                       # set once we are circling the target

    async def watch_progress():
        async for p in drone.mission_raw.mission_progress():
            progress["cur"], progress["total"] = p.current, p.total
            print(f"[MISSION] item {p.current}/{p.total}")

    watcher = asyncio.create_task(watch_progress())
    last_print = 0.0
    try:
        async for pos in drone.telemetry.position():
            x, y = to_local(pos.latitude_deg, pos.longitude_deg, o_lat, o_lon)
            rssi = sensor.read(x, y, pos.relative_altitude_m)
            if finish_first and logic.stage == 1:
                logic.record((x, y), rssi)       # just listen until the lawnmower is done
                event = None
                if (ADD_RTL_AT_END and progress["total"] > 0
                        and progress["cur"] >= progress["total"] - 1
                        and time.monotonic() - mission_start > 15):
                    if logic.best_xy is None or logic.best_rssi < RSSI_DETECT_DBM:
                        print("[STAGE 1] lawnmower finished and the signal was never heard "
                              f"(best {logic.best_rssi:.1f} dBm). Returning home.")
                        break
                    est = estimate_target(logic.readings, pos.relative_altitude_m, logic.best_xy)
                    print(f"[STAGE 1] lawnmower finished; strongest signal {logic.best_rssi:.1f} dBm "
                          f"-> going to the estimated target and circling it")
                    logic.stage = 3
                    event = ("homing", est)
            else:
                event = logic.update((x, y), rssi)

            # lawnmower ended while we were still tracking a signal: hand off anyway
            if (event is None and ADD_RTL_AT_END and progress["total"] > 0
                    and progress["cur"] >= progress["total"] - 1
                    and time.monotonic() - mission_start > 15):
                event = logic.force_handoff()

            # an inward square/spiral that has been flown to its end: circle the estimate
            if (event is None and stage2 in ("inward_square", "inward_spiral")
                    and logic.stage == 2 and logic.best_xy is not None
                    and progress["total"] > 0 and progress["cur"] >= progress["total"] - 1
                    and time.monotonic() - mission_start > 15):
                print(f"[STAGE 2] {stage2} finished -> circling the best estimate")
                logic.stage = 3
                event = ("homing", logic.best_xy)

            now = time.monotonic()
            if now - last_print > 2.0:
                last_print = now
                print(f"[STAGE {logic.stage}] pos E{x:6.0f} N{y:6.0f}  "
                      f"alt {pos.relative_altitude_m:4.0f}  RSSI {rssi:6.1f} dBm  "
                      f"best {logic.best_rssi:6.1f}")

            if event and event[0] == "square" and stage2 == "orbit":
                print(f"[STAGE 2] signal detected (peak {logic.best_rssi:.1f} dBm) -> "
                      "circling the best estimate")
                logic.stage = 3
                event = ("homing", event[1])

            if event and event[0] == "square":
                cx, cy = event[1]
                print(f"[STAGE 2] signal detected -> centering on "
                      f"E{cx:.0f} N{cy:.0f} (best {logic.best_rssi:.1f} dBm)")
                # how far off could the target be? the peak reading says roughly how far
                est_r = range_from_rssi(logic.best_rssi, pos.relative_altitude_m)
                outer = min(INWARD_MAX_OUTER_M, INWARD_OUTER_MARGIN * est_r)
                sp2 = SQUARE_SPACING_M
                print(f"[STAGE 2] peak {logic.best_rssi:.1f} dBm -> target roughly "
                      f"{est_r:.0f} m from the pass; {stage2} outer radius {outer:.0f} m")
                sq = None
                if stage2 != "expanding_square" and outer < INWARD_MIN_OUTER_M:
                    print(f"[STAGE 2] estimate is tighter than the plane can turn into; "
                          f"enlarging the outer radius to {INWARD_MIN_OUTER_M:.0f} m")
                    outer = INWARD_MIN_OUTER_M
                for cl in (clearance_m, 0.5 * clearance_m, 0.0):   # small areas: relax
                    try:
                        sq = build_stage2(stage2, area_xy, (cx, cy), outer, sp2, cl)
                        break
                    except ValueError:
                        continue
                if sq is None:
                    print("[STAGE 2] no inward pattern needed/possible; "
                          "treating the best reading as the target")
                    event = ("homing", event[1])
                else:
                    print(f"[STAGE 2] {len(sq)} waypoints")
                    await switch_mission(drone, make_followup_items(
                        h_lat, h_lon, h_alt, ll(sq), CRUISE_ALT_M, ADD_RTL_AT_END))
                    progress["cur"], progress["total"] = 0, 0
                    mission_start = time.monotonic()

            if event and event[0] == "homing":
                logic.stage = 3
                tx, ty = event[1]
                err = math.hypot(tx - target_xy[0], ty - target_xy[1])
                print(f"[STAGE 3] estimated target E{tx:.0f} N{ty:.0f} "
                      f"(true error {err:.0f} m); circling it, radius {LOITER_RADIUS_M:.0f} m "
                      "(gradient homing is still a TODO)")
                await switch_mission(drone, make_followup_items(
                    h_lat, h_lon, h_alt, ll([(tx, ty)]), CRUISE_ALT_M, rtl=False,
                    nav_cmd=MAV_CMD_NAV_LOITER_UNLIM, loiter_radius_m=LOITER_RADIUS_M))
                orbit = {"center": (tx, ty), "since": time.monotonic(), "refits": 0}
                event = None

            # while circling: re-fit the target position from everything heard so far
            # (a lap of readings removes the left/right ambiguity of a straight pass)
            if orbit and time.monotonic() - orbit["since"] > ORBIT_REFIT_S:
                est = estimate_target(logic.readings, pos.relative_altitude_m, orbit["center"])
                move = math.hypot(est[0] - orbit["center"][0], est[1] - orbit["center"][1])
                err = math.hypot(est[0] - target_xy[0], est[1] - target_xy[1])
                orbit["refits"] += 1
                print(f"[STAGE 3] refit {orbit['refits']}/{ORBIT_REFITS}: target now "
                      f"E{est[0]:.0f} N{est[1]:.0f} (moved {move:.0f} m, true error {err:.0f} m)")
                if move > 15.0:
                    await switch_mission(drone, make_followup_items(
                        h_lat, h_lon, h_alt, ll([est]), CRUISE_ALT_M, rtl=False,
                        nav_cmd=MAV_CMD_NAV_LOITER_UNLIM, loiter_radius_m=LOITER_RADIUS_M))
                orbit["center"], orbit["since"] = est, time.monotonic()
                if orbit["refits"] >= ORBIT_REFITS:
                    print("[STAGE 3] done: the plane keeps circling the estimated target")
                    break

            # reached the final RTL item of the current mission without success
            if (orbit is None and ADD_RTL_AT_END and progress["total"] > 0
                    and progress["cur"] >= progress["total"] - 1
                    and time.monotonic() - mission_start > 15):
                print(f"[STAGE {logic.stage}] pattern finished with no strong signal "
                      "(stage 2 -> 1 fallback is not implemented yet). Returning home.")
                break
    finally:
        watcher.cancel()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", default="lawnmower", choices=sorted(PATTERN_BUILDERS),
                    help="stage-1 pattern, chosen pre-flight (default: lawnmower over the whole area; "
                         "ellipse_lawnmower = last year's ellipse method)")
    ap.add_argument("--boundary", help="GCS area JSON (search_area / keep_in / keep_out)")
    ap.add_argument("--true-coords", action="store_true",
                    help="fly the area at its real lat/lon (default: recenter on SITL home)")
    ap.add_argument("--spacing", type=float, default=LANE_SPACING_M, help="max lane spacing, m")
    ap.add_argument("--clearance", type=float, default=EDGE_CLEARANCE_M, help="edge clearance, m")
    ap.add_argument("--stage2", default=STAGE2_DEFAULT,
                    choices=["orbit", "inward_square", "inward_spiral", "expanding_square"],
                    help="pattern flown once a signal is detected")
    ap.add_argument("--takeoff-toward-first", action="store_true",
                    help="climb out toward the first waypoint instead of straight ahead")
    ap.add_argument("--switch-early", action="store_true",
                    help="leave the lawnmower as soon as the signal is found (default: finish it first)")
    ap.add_argument("--no-pause", action="store_true",
                    help="do not wait for you to download the mission in QGC before starting")
    ap.add_argument("--force", action="store_true", help="fly even if safety checks fail")
    ap.add_argument("--target", help="hidden test target as EAST,NORTH metres from the area "
                                     "centre (default: inside the area, ~35%% toward a corner)")
    a = ap.parse_args()
    if a.target:
        tgt = tuple(float(v) for v in a.target.split(","))
    elif a.boundary:
        b = load_boundary(a.boundary)
        c = polygon_centroid(b["search"])
        far = max((to_local(la, lo, *c) for la, lo in b["search"]), key=lambda q: math.hypot(*q))
        tgt = (0.35 * far[0], 0.35 * far[1])
    else:
        tgt = SIM_TARGET_DEFAULT
    asyncio.run(main(a.pattern, tgt, a.boundary, a.true_coords, a.spacing, a.clearance, a.force,
                     a.stage2, a.takeoff_toward_first, not a.switch_early, not a.no_pause))
