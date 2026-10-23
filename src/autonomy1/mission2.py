"""
Mission 2: Expanding Square Flight Pattern

1. Aquire search area (defined by GCS) 
NOTE: the first mission is a lawnmower pattern, and recieves the search area from GCS.

2. Generate waypoints and fly the plane autonomously in expanding square pattern, once the mission 1 is completed or 
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
from pathlib import Path
from lia.lia_client import LiaClient
from pymavlink import mavutil


# DEFAULT CONFIGURATION (override with CLI flags)
CONNECTION          = "udpin:0.0.0.0:14608"   # Free mavlink-router endpoint (14550 is QGC's, see start_simulation.sh)
SITL_HOME           = (33.932235, -117.631915)  # Home used in start_simulation.sh

CENTER_LAT          = 33.934000    # Search area center
CENTER_LON          = -117.630500
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


sys.path.append(str(Path(__file__).resolve().parents[1]))


def generate_waypoints():
    # ==========================================================
    # REPLACE THIS FUNCTION WITH YOUR AUTONOMY / WAYPOINT LOGIC
    # ==========================================================

    # Your autonomy logic should calculate and return:
    #
    # [
    #     {
    #         "lat": ...,
    #         "lon": ...,
    #         "alt_ft": ...,
    #     },
    #     ...
    # ]

    return calculate_search_waypoints()


def generate_plan():
    waypoints = generate_waypoints()

    return {
        "plan_id": "search_001",
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


def run_mission2():
    print("[AUTONOMY] Starting Mission 2: Expanding Square Flight Pattern")
    
    plan = {
        "plan_id": "search_002",
        "plan_type": "waypoint_pattern",
        "label": "Expanding Square Search Pattern",
        "waypoints": [
            {
                "lat": CENTER_LAT,
                "lon": CENTER_LON,
                "alt_ft": CRUISE_ALT_FT,
            }
        ]
    }
    


if __name__ == "__main__":
    run_mission2()