#!/usr/bin/env python3

import time
import json
from pathlib import Path
from datetime import datetime, timezone
import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.nav_state_utils import update_nav_state

#This file is for bridging target and search area information from the GCS telemetry file to the navigation state json'

TELEMETRY_FILE = Path("/tmp/telemetry.json") #A file on the PI where the GCS telemetry data is written
POLL_DT_S = 0.2

SEARCH_AREA_ZONE_ID = 1  # Change this to whatever GCS defines as search area
#^ Except last year they just gave it to us before and we can just set it manually ?
# Manually means we can either give the coordinates to our scripts or we can set it in QGC to flash onto the flight controller (Both is probably good)

def find_search_area_zone(zones):
    if not isinstance(zones, list):
        return []

    for zone in zones:
        zone_id = zone.get("zone_id")
        coords = zone.get("coordinates")

        if zone_id == SEARCH_AREA_ZONE_ID:
            return coords

    return []


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()

def read_json(path: Path):
    if not path.exists():
        return None

    try:
        with open(path, "r") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f"[STATE BRIDGE] JSON decode error in {path.name}: {e.msg}")
        return None
    except Exception as e:
        print(f"[STATE BRIDGE] Failed to read {path.name}: {e}")
        return None

#Checks if the coordinates are valid (not None and not zero)
def valid_coord(lat, lon):
    if lat is None or lon is None:
        return False

    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        return False

    return lat != 0.0 and lon != 0.0

#Just chekcs to see if the search area is valid (has at least 3 points with valid coordinates)
def valid_search_area(search_area):
    if not isinstance(search_area, list):
        return False

    if len(search_area) < 3:
        return False

    for point in search_area:
        if not isinstance(point, list) and not isinstance(point, tuple):
            return False

        if len(point) != 2:
            return False

        lat, lon = point
        if not valid_coord(lat, lon):
            return False

    return True

#Sets the target location in the navigation state json (THis would be the location sent by UGV -> GCS -> UAV)
def set_target_location(lat, lon, source, timestamp):
    update_nav_state("target_location", {
        "lat": float(lat),
        "lon": float(lon),
        "source": source,
        "timestamp": timestamp,
        "valid": True
    })


def main():
    print("[STATE BRIDGE] Starting Pi5 target/search-area bridge")

    last_search_area = None
    last_refined_msg_id = None
    last_final_msg_id = None
    last_eru_msg_id = None

    while True:
        data = read_json(TELEMETRY_FILE)

        if not data:
            time.sleep(POLL_DT_S)
            continue

        timestamp = utc_now_iso()

        #Search Area
        zones = data.get("zones", [])
        search_area = find_search_area_zone(zones)
        if valid_search_area(search_area):
            current = json.dumps(search_area, sort_keys=True)

            if current != last_search_area:
                print(f"[STATE BRIDGE] Search area received with {len(search_area)} points")

                update_nav_state("search_area", search_area) #Updates the nav state JSON
                last_search_area = current

        #THis is the refined loiter target that GCS sends so that the loiter point is better
        # FIRST Kraken transmit.
        # Updates ONLY mra_refined_loiter_target.
        # Does NOT update target_location.

        #HOWEVER!!
        #This is here because triangulation was done off board so that this location is sent from GCS, this year triangulation is going to be done on board so the refined loiter target may come from a different source and this logic will probably be moved to whatever processing it is. 
        
        refined_msg_id = data.get("mra_refined_msg_id") or data.get("mra_refined_seq")
        if refined_msg_id and refined_msg_id != last_refined_msg_id:
            refined_lat = data.get("mra_refined_lat")
            refined_lon = data.get("mra_refined_lon")

            if valid_coord(refined_lat, refined_lon):
                seq = data.get("mra_refined_seq", "N/A")
                fix_id = data.get("mra_refined_fix_id") or f"mra_refined_{seq}"
                
                print(f"[STATE BRIDGE] Processed valid transmit: msg_id={refined_msg_id}, seq={seq}, source=mra_refined_loiter_target, lat={refined_lat}, lon={refined_lon}, updated=mra_refined_loiter_target, time={timestamp}")

                update_nav_state("mra_refined_loiter_target", {
                    "lat": float(refined_lat),
                    "lon": float(refined_lon),
                    "confidence": data.get("mra_refined_confidence"),
                    "timestamp": timestamp,
                    "fix_id": fix_id,
                    "valid": True
                })

                last_refined_msg_id = refined_msg_id

        # ------------------------------------------------------------
        # 3. MRA final estimated location from Kraken
        # ------------------------------------------------------------
        # SECOND Kraken transmit.
        # Updates mra_final_estimated_location and target_location.
        final_msg_id = data.get("mra_final_msg_id") or data.get("mra_final_seq")
        if final_msg_id and final_msg_id != last_final_msg_id:
            final_lat = data.get("mra_final_lat")
            final_lon = data.get("mra_final_lon")

            if valid_coord(final_lat, final_lon):
                seq = data.get("mra_final_seq", "N/A")
                fix_id = data.get("mra_final_fix_id") or f"mra_final_{seq}"
                
                print(f"[STATE BRIDGE] Processed valid transmit: msg_id={final_msg_id}, seq={seq}, source=mra_final_estimated_location, lat={final_lat}, lon={final_lon}, updated=mra_final_estimated_location, time={timestamp}")

                update_nav_state("mra_final_estimated_location", {
                    "lat": float(final_lat),
                    "lon": float(final_lon),
                    "confidence": data.get("mra_final_confidence"),
                    "timestamp": timestamp,
                    "fix_id": fix_id,
                    "valid": True
                })

                set_target_location(
                    final_lat,
                    final_lon,
                    "mra_final_estimated_location",
                    timestamp
                )

                last_final_msg_id = final_msg_id

        # ------------------------------------------------------------
        # 4. ERU reported location from GCS
        # ------------------------------------------------------------
        # Comes from GCS PatientLocation command.
        # Updates eru_reported_location and target_location.
        eru_msg_id = data.get("eru_msg_id") or data.get("eru_seq")
        if eru_msg_id and eru_msg_id != last_eru_msg_id:
            eru_lat = data.get("eru_lat")
            eru_lon = data.get("eru_lon")

            if valid_coord(eru_lat, eru_lon):
                seq = data.get("eru_seq", "N/A")
                fix_id = data.get("eru_fix_id") or f"eru_{seq}"

                print(f"[STATE BRIDGE] Processed valid transmit: msg_id={eru_msg_id}, seq={seq}, source=eru_reported_location, lat={eru_lat}, lon={eru_lon}, updated=eru_reported_location, time={timestamp}")

                update_nav_state("eru_reported_location", {
                    "lat": float(eru_lat),
                    "lon": float(eru_lon),
                    "confidence": None,
                    "timestamp": timestamp,
                    "fix_id": fix_id,
                    "valid": True
                })

                set_target_location(
                    eru_lat,
                    eru_lon,
                    "eru_reported_location",
                    timestamp
                )

                last_eru_msg_id = eru_msg_id

        time.sleep(POLL_DT_S)


if __name__ == "__main__":
    main()