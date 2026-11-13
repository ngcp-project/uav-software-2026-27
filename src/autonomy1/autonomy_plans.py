#!/usr/bin/env python3
"""Search plans in the LIA MissionPlan format.

Uses the geometry in search_skeleton.py (ellipse + lawnmower, inward square,
loiter) but outputs plain Python dictionaries:

    {"plan_id": ..., "plan_type": "waypoint_pattern", "waypoints": [{"lat","lon","alt_ft"}, ...]}
    {"plan_id": ..., "plan_type": "single_loiter",    "waypoints": [{...}], "loiter_radius_ft": ...}

Nothing here talks to MAVSDK or the aircraft.  Publishing goes through LIA
(`lia.lia_client.LiaClient`), exactly as in the MissionPlan interface doc.

Command line (writes the plan as JSON, optionally publishes it):

    python3 autonomy_plans.py stage1 --home -35.363262,149.165237 --boundary uav_area_example.json
    python3 autonomy_plans.py stage2 --home LAT,LON --boundary area.json --center LAT,LON --peak-dbm -66
    python3 autonomy_plans.py loiter --center LAT,LON
    add  --publish  to send the plan to LIA
"""
import argparse
import json
import sys
import uuid
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", category=FutureWarning)
sys.path.append(str(Path(__file__).resolve().parents[1]))

import search_skeleton as sk  # noqa: E402  (geometry + constants)

M_TO_FT = 3.28084


# ------------------------------------------------------------------ plan dicts
def _wp(lat, lon, alt_m):
    return {"lat": round(lat, 7), "lon": round(lon, 7), "alt_ft": round(alt_m * M_TO_FT, 1)}


def waypoint_plan(plan_id, latlons, alt_m=sk.CRUISE_ALT_M):
    """MissionPlan 'waypoint_pattern' from a list of (lat, lon)."""
    return {"plan_id": plan_id, "plan_type": "waypoint_pattern",
            "waypoints": [_wp(la, lo, alt_m) for la, lo in latlons]}


def loiter_plan(plan_id, lat, lon, alt_m=sk.CRUISE_ALT_M, radius_m=sk.LOITER_RADIUS_M):
    """MissionPlan 'single_loiter': fly to one point and circle it."""
    return {"plan_id": plan_id, "plan_type": "single_loiter",
            "waypoints": [_wp(lat, lon, alt_m)],
            "loiter_radius_ft": round(radius_m * M_TO_FT, 1)}


# ------------------------------------------------------------------- the area
def load_area(home_ll, boundary_path=None, true_coords=False, clearance_m=sk.EDGE_CLEARANCE_M):
    """Search area as a convex polygon in local metres plus the lat/lon origin.

    No boundary file -> the built-in test box around `home_ll`.
    Boundary file    -> recentred on `home_ll` unless `true_coords` is set."""
    keep_in, keep_outs = None, []
    if boundary_path:
        b = sk.load_boundary(boundary_path)
        c_lat, c_lon = sk.polygon_centroid(b["search"])
        conv = lambda poly: [sk.to_local(a, c, c_lat, c_lon) for a, c in poly]
        search_xy = conv(b["search"])
        keep_in = conv(b["keep_in"]) if b["keep_in"] else None
        keep_outs = [conv(k) for k in b["keep_out"]]
        o_lat, o_lon = (c_lat, c_lon) if true_coords else home_ll
    else:
        search_xy, (o_lat, o_lon) = sk.BOUNDARY_OFFSETS_M, home_ll
    area_xy, msgs, fatal = sk.prepare_area(search_xy, keep_in, keep_outs, sk.TURN_RADIUS_M)
    if fatal:
        raise ValueError("; ".join(msgs))
    return area_xy, keep_outs, (o_lat, o_lon)


def _to_ll(pts_xy, origin):
    return [sk.from_local(x, y, origin[0], origin[1]) for x, y in pts_xy]


# ------------------------------------------------------------ the three stages
def generate_stage1_plan(home_ll, boundary_path=None, true_coords=False,
                         pattern="lawnmower", spacing_m=sk.LANE_SPACING_M,
                         clearance_m=sk.EDGE_CLEARANCE_M, plan_id="search_001"):
    """Stage 1: lawnmower (inside last year's ellipse by default)."""
    area_xy, keep_outs, origin = load_area(home_ll, boundary_path, true_coords, clearance_m)
    if pattern in sk.REGION_PATTERNS:
        wps = []
        for inset in (clearance_m, 1.5 * clearance_m, 2.0 * clearance_m):
            region = sk.fit_ellipse_region(area_xy, inset)
            wps = sk.build_lawnmower(region, spacing_m=spacing_m, clearance_m=0.0,
                                     side_clearance_m=0.0)
            if wps and not sk.check_path(wps, keep_outs, sk.TURN_RADIUS_M):
                break
    else:
        wps = sk.PATTERN_BUILDERS[pattern](area_xy, spacing_m=spacing_m, clearance_m=clearance_m,
                                           side_clearance_m=sk.SIDE_CLEARANCE_M,
                                           start_xy=sk.to_local(home_ll[0], home_ll[1], *origin))
    if not wps:
        raise ValueError("no waypoints fit; area too small for this clearance")
    problems = sk.check_path(wps, keep_outs, sk.TURN_RADIUS_M)
    if problems:
        raise ValueError("path too close to a keep-out zone: " + problems[0])
    return waypoint_plan(plan_id, _to_ll(wps, origin))


def generate_stage2_plan(home_ll, center_ll, peak_rssi_dbm, boundary_path=None,
                         true_coords=False, kind="inward_square", altitude_m=sk.CRUISE_ALT_M,
                         clearance_m=sk.EDGE_CLEARANCE_M, plan_id="search_002"):
    """Stage 2: inward square/spiral around where the signal was first heard.

    The outer size comes from the peak RSSI (how far the target can be);
    it is never smaller than what the plane can turn into."""
    area_xy, _keep_outs, origin = load_area(home_ll, boundary_path, true_coords, clearance_m)
    cx, cy = sk.to_local(center_ll[0], center_ll[1], *origin)
    est_r = sk.range_from_rssi(peak_rssi_dbm, altitude_m)
    outer = max(sk.INWARD_MIN_OUTER_M, min(sk.INWARD_MAX_OUTER_M, sk.INWARD_OUTER_MARGIN * est_r))
    for cl in (clearance_m, 0.5 * clearance_m, 0.0):
        try:
            pts = sk.build_stage2(kind, area_xy, (cx, cy), outer, sk.SQUARE_SPACING_M, cl)
            break
        except ValueError:
            pts = None
    if not pts:
        raise ValueError("inward pattern does not fit the area")
    return waypoint_plan(plan_id, _to_ll(pts, origin), altitude_m)


def generate_stage3_plan(center_ll, altitude_m=sk.CRUISE_ALT_M, radius_m=sk.LOITER_RADIUS_M,
                         plan_id="loiter_001"):
    """Stage 3: circle the best target estimate."""
    return loiter_plan(plan_id, center_ll[0], center_ll[1], altitude_m, radius_m)


# ----------------------------------------------------------- publishing (LIA)
def publish_plan(plan: dict):
    """Same as the MissionPlan interface doc.  Needs the `lia` package."""
    from lia.lia_client import LiaClient
    lia = LiaClient("autonomy")
    command_id = str(uuid.uuid4())
    lia.publish("cmd.submit_plan", {"command_id": command_id, "plan": plan})
    print(f"[AUTONOMY] Submitted {plan['plan_id']} command_id={command_id}")
    return command_id


# ------------------------------------------------------------------------ CLI
def _ll(text):
    a, b = text.split(",")
    return float(a), float(b)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("stage", choices=["stage1", "stage2", "loiter"])
    ap.add_argument("--home", type=_ll, default=(33.931295, -117.631212),
                    help="LAT,LON of home / area origin (default: Prado Field airfield, Prado Regional Park)")
    ap.add_argument("--boundary", help="GCS area JSON")
    ap.add_argument("--true-coords", action="store_true", help="use the area's real lat/lon")
    ap.add_argument("--pattern", default="lawnmower", choices=sorted(sk.PATTERN_BUILDERS))
    ap.add_argument("--stage2", default="inward_square", choices=["inward_square", "inward_spiral"])
    ap.add_argument("--center", type=_ll, help="stage2/loiter: LAT,LON of the detection / estimate")
    ap.add_argument("--peak-dbm", type=float, default=-70.0, help="stage2: strongest RSSI so far")
    ap.add_argument("--out", help="write the plan JSON here")
    ap.add_argument("--publish", action="store_true", help="send the plan to LIA")
    a = ap.parse_args()

    if a.stage == "stage1":
        plan = generate_stage1_plan(a.home, a.boundary, a.true_coords, a.pattern)
    elif a.stage == "stage2":
        if not a.center:
            ap.error("stage2 needs --center LAT,LON")
        plan = generate_stage2_plan(a.home, a.center, a.peak_dbm, a.boundary,
                                    a.true_coords, a.stage2)
    else:
        if not a.center:
            ap.error("loiter needs --center LAT,LON")
        plan = generate_stage3_plan(a.center)

    text = json.dumps(plan, indent=2)
    if a.out:
        Path(a.out).write_text(text)
        print(f"[AUTONOMY] wrote {a.out}: {plan['plan_type']}, {len(plan['waypoints'])} waypoint(s)")
    else:
        print(text)
    if a.publish:
        publish_plan(plan)
