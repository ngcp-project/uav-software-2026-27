"""
Mission 2: Expanding Square Flight Pattern
           Inward circular spiral
           Inward square spiral

1. Aquire search area (defined by GCS) 
NOTE: the first mission is a lawnmower pattern, and recieves the search area from GCS.

2. Generate waypoints and fly the plane autonomously in the desired pattern, once the mission 1 is completed or 
the location is partially determined. The transition must be autonomous (not controlled by GCS).

3. Report location, altitude, heading, and detections to GCS while in pattern

4. If the location of the survivor is accurate before the end of the pattern, loiter above the survivor location and 
report to GCS. If not, continue the pattern or transition to the next mission when data gradually becomes more accurate.


Requirements: 
- It must be modular and extensible to allow different defined search area
- It must handle transitions between missions autonomously
- It must not go beyond the defined search area when turning
- It must be able to recover, in case it is outside the defined search area
- It must recover back to the original pattern when the plane is off the flight path

"""

import sys
import argparse
import json
import logging
import math
import time
import uuid
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient
from state.nav_state_utils import load_nav_state
from pymavlink import mavutil


# DEFAULT CONFIGURATION (override with CLI flags)
CONNECTION          = "udpin:0.0.0.0:14605"   # Free mavlink-router endpoint (14550 is QGC's, see start_simulation.sh)
SITL_HOME           = (33.932235, -117.631915)  # Home used in start_simulation.sh

CENTER_LAT          = SITL_HOME[0]  # Search area center / simulated aircraft home
CENTER_LON          = SITL_HOME[1]
AREA_LENGTH_FT      = 1500.0       # Along-lane size of the area (ft)
AREA_WIDTH_FT       = 1050.0       # Cross-lane size of the area (ft)
HEADING_DEG         = 90.0         # Lane direction, clockwise from north (deg)
LANE_SPACING_FT     = 350.0        # Distance between adjacent lanes (ft) — set from sensor footprint
WP_SPACING_FT       = 0.0          # Extra waypoints along each lane every N ft (0 = lane ends only)
OVERSHOOT_FT        = 150.0        # Extend each lane past the area edge so the plane is aligned before entering (ft)
CRUISE_ALT_FT       = 150.0        # Mission altitude, relative to home (ft)
ACCEPT_RADIUS_FT    = 65.0         # Waypoint acceptance radius sent in param2 (ft)
TAKEOFF_PITCH_DEG   = 15.0         # NAV_TAKEOFF minimum pitch (deg)
MIN_TURN_RADIUS_FT  = 100.0        # Approx. plane turn radius, only used for a warning (ft)
PLAN_FILE           = ""

FT_TO_M        = 0.3048
EARTH_RADIUS_M = 6_371_000.0
AUTO_MODE      = 10                # ArduPlane custom_mode for AUTO

# SEARCH PATTERN DEFAULTS (override with CLI flags, see parse_args)
PATTERN               = "inward_spiral"
SPIRAL_SPACING_FT    = 150.0      # Radial distance between successive rings (ft) — set from sensor footprint
SPIRAL_MIN_RADIUS_FT  = 100.0      # Innermost ring radius; never below MIN_TURN_RADIUS_FT (ft)
SPIRAL_MAX_RADIUS_FT  = 0.0        # Cap on the outer ring radius (ft) (0 = largest circle that fits the search area)
SPIRAL_WP_SPACING_FT  = 250.0      # Arc length between waypoints along the spiral (ft)
SPIRAL_MIN_WP_PER_REV = 8          # Minimum waypoints per revolution, keeps the inner rings round
SPIRAL_START_BEARING  = 0.0        # Bearing from the center to the first waypoint, clockwise from north (deg)
SPIRAL_CLOCKWISE      = True       # Direction the plane circles
BORDER_CLEARANCE_FT   = 100.0      # Outer ring keeps at least this distance from the search area boundary (ft)


# Geodetic helpers (flat-earth, fine for areas of a few km)
def to_local(lat, lon, o_lat, o_lon):
    east = math.radians(lon - o_lon) * EARTH_RADIUS_M * math.cos(math.radians(o_lat))
    north = math.radians(lat - o_lat) * EARTH_RADIUS_M
    return east, north


def from_local(east, north, o_lat, o_lon):
    lat = o_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = o_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(o_lat))))
    return lat, lon


def point_in_poly(px, py, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if (ay > py) != (by > py) and px < ax + (py - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def dist_to_poly_edges(px, py, poly):
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


# Inward circular spiral
def calculate_inward_spiral(center_lat, center_lon, search_area, cfg):
    """
    Archimedean spiral r(theta) = r_out - spacing * theta / 2pi around the center.

    - r_out is the largest circle around the center that keeps cfg.clearance_ft
      from every search area edge, so the whole spiral (and every turn on it) stays
      inside the search area.
    - The radius shrinks by cfg.spacing_ft per revolution until it reaches r_in,
      which is never tighter than the plane can turn.
    - Waypoints are cfg.wp_spacing_ft apart along the arc, with at least
      SPIRAL_MIN_WP_PER_REV per revolution.
    """
    poly = [to_local(lat, lon, center_lat, center_lon) for lat, lon in search_area]
    if not point_in_poly(0.0, 0.0, poly):
        raise ValueError("Spiral center is outside the search area")

    spacing_m = cfg.spacing_ft * FT_TO_M
    r_in = max(cfg.min_radius_ft, MIN_TURN_RADIUS_FT) * FT_TO_M
    r_out = dist_to_poly_edges(0.0, 0.0, poly) - cfg.clearance_ft * FT_TO_M
    if cfg.max_radius_ft > 0:
        r_out = min(r_out, cfg.max_radius_ft * FT_TO_M)
    if r_out < r_in:
        raise ValueError(f"Search area too small around center: outer radius {r_out / FT_TO_M:.0f} ft "
                         f"< inner radius {r_in / FT_TO_M:.0f} ft")

    # Fly at least one full revolution, even when r_out is close to r_in
    total_angle = max(2 * math.pi * (r_out - r_in) / spacing_m, 2 * math.pi)
    shrink_per_rad = (r_out - r_in) / total_angle

    max_step = 2 * math.pi / SPIRAL_MIN_WP_PER_REV
    wp_spacing_m = cfg.wp_spacing_ft * FT_TO_M
    direction = -1.0 if cfg.ccw else 1.0
    start = math.radians(cfg.start_bearing)

    thetas = []
    theta = 0.0
    while theta < total_angle:
        thetas.append(theta)
        r = r_out - shrink_per_rad * theta
        theta += min(wp_spacing_m / r, max_step)
    thetas.append(total_angle)

    waypoints = []
    for theta in thetas:
        r = r_out - shrink_per_rad * theta
        bearing = start + direction * theta     # compass bearing, clockwise from north
        lat, lon = from_local(r * math.sin(bearing), r * math.cos(bearing), center_lat, center_lon)
        waypoints.append({"lat": lat, "lon": lon, "alt_ft": cfg.alt_ft})

    print(f"[AUTONOMY] Inward spiral: r_out={r_out / FT_TO_M:.0f} ft, r_in={r_in / FT_TO_M:.0f} ft, "
          f"{total_angle / (2 * math.pi):.2f} revolutions, {len(waypoints)} waypoints")
    return waypoints


# Inward square spiral
def calculate_inward_square_spiral(center_lat, center_lon, search_area, cfg):
    raise NotImplementedError("inward_square_spiral is not implemented yet")


# Expanding square
def calculate_expanding_square(center_lat, center_lon, search_area, cfg):
    raise NotImplementedError("expanding_square is not implemented yet")


# Every pattern takes (center_lat, center_lon, search_area, cfg) and returns
# [{"lat", "lon", "alt_ft"}, ...]. Register new patterns here to expose them in --pattern.
PATTERNS = {
    "inward_spiral":        calculate_inward_spiral,
    "inward_square_spiral": calculate_inward_square_spiral,
    "expanding_square":     calculate_expanding_square,
}


def generate_waypoints(cfg):
    """
    Runs the pattern selected by cfg.pattern over the search area from navigation_state.

    Center: cfg.center if given, else the partial survivor estimate
    (mra_refined_loiter_target) when valid, else the search area centroid. Falls back
    to the centroid if the pattern can't fit around the estimate (e.g. near the boundary).
    """
    nav_state = load_nav_state()

    raw_search = nav_state.get("search_area", [])
    if not isinstance(raw_search, list) or not 3 <= len(raw_search) <= 6:
        raise ValueError(f"Invalid search_area. Need 3–6 points, got: {raw_search}")
    search_area = [(float(p[0]), float(p[1])) for p in raw_search]

    centers = []
    if cfg.center:
        centers.append(("--center", cfg.center[0], cfg.center[1]))
    else:
        target = nav_state.get("mra_refined_loiter_target", {})
        if target.get("valid") and target.get("lat") is not None and target.get("lon") is not None:
            centers.append(("mra_refined_loiter_target", float(target["lat"]), float(target["lon"])))
        centers.append(("search_area centroid",
                        sum(p[0] for p in search_area) / len(search_area),
                        sum(p[1] for p in search_area) / len(search_area)))

    calculate_pattern = PATTERNS[cfg.pattern]
    for source, lat, lon in centers:
        try:
            waypoints = calculate_pattern(lat, lon, search_area, cfg)
        except ValueError as exc:
            print(f"[AUTONOMY] Cannot center {cfg.pattern} on {source}: {exc}")
            continue
        print(f"[AUTONOMY] {cfg.pattern} centered on {source} ({lat:.6f}, {lon:.6f})")
        return waypoints

    raise ValueError(f"No center fits {cfg.pattern} inside the search area")


def generate_plan(cfg):
    waypoints = generate_waypoints(cfg)

    return {
        "plan_id": "search_002",
        "plan_type": "waypoint_pattern",
        "waypoints": waypoints,
    }


def publish_plan(plan: dict):
    lia = LiaClient("autonomy")

    command_id = str(uuid.uuid4())

    lia.publish(
        "cmd.submit_plan",
        {
            "command_id": command_id,
            "plan": plan,
        },
    )

    print(
        f"[AUTONOMY] Submitted "
        f"{plan['plan_id']} "
        f"command_id={command_id}"
    )

    return command_id



def loiter(vehicle, lat, lon, alt_ft):
    # ==========================================================
    # Replace this function with loiter logic
    # it should have a certain radius and altitude while loitering 
    # above the survivor location
    # ==========================================================

    
    loiter_plan = {
        "plan_id": "loiter_001",
        "plan_type": "loiter",
        "waypoints": [
            {
                "lat": lat,
                "lon": lon,
                "alt_ft": alt_ft,
            }
        ],
        "loiter_radius_ft": 50.0,  # Example radius for loitering
    }
# if condition to loiter is met, execute loiter function,
# if not, transition to the mission 3


def parse_args():
    p = argparse.ArgumentParser(description="Mission 2: generate a search pattern and submit it through LIA")
    p.add_argument("--pattern", choices=PATTERNS, default=PATTERN, help="search pattern to fly")
    p.add_argument("--dry-run", action="store_true", help="print the plan as JSON instead of publishing to LIA")
    # Pattern settings come from the constants at the top of this file
    p.set_defaults(
        # Keep the generated pattern centered on the simulated aircraft unless
        # an MRA estimate is explicitly supplied or the area cannot contain it.
        center=SITL_HOME,
        alt_ft=CRUISE_ALT_FT,
        spacing_ft=SPIRAL_SPACING_FT,
        min_radius_ft=SPIRAL_MIN_RADIUS_FT,
        max_radius_ft=SPIRAL_MAX_RADIUS_FT,
        wp_spacing_ft=SPIRAL_WP_SPACING_FT,
        clearance_ft=BORDER_CLEARANCE_FT,
        start_bearing=SPIRAL_START_BEARING,
        ccw=not SPIRAL_CLOCKWISE,
    )
    return p.parse_args()


def run_mission2():
    cfg = parse_args()
    print(f"[AUTONOMY] Starting Mission 2: {cfg.pattern} flight pattern")
    print("[AUTONOMY] Generating waypoints for the search pattern...")

    plan = generate_plan(cfg)

    if cfg.dry_run:
        print(json.dumps(plan, indent=2))
        return

    publish_plan(plan)


if __name__ == "__main__":
    run_mission2()
