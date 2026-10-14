#!/usr/bin/env python3
import asyncio
import os
from pathlib import Path
from datetime import datetime, timezone
import json
import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))


from state.mission_state_utils import load_state, update_state, STATE_FILE 
from state.nav_state_utils import update_nav_state, load_nav_state, NAV_STATE_FILE, DEFAULTS as NAV_DEFAULTS

UPDATE_FILE = Path(__file__).resolve().parents[1] / "state" / "update.json"

def load_update_json():
    if not UPDATE_FILE.exists():
        return {}

    try:
        with UPDATE_FILE.open("r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError:
        return {}
    

STATE_POLL_HZ = 2.0 #How often to check state file

async def main():

    state_period_s = 1.0 / STATE_POLL_HZ

    print(f"Mission state file: {STATE_FILE}")
    print(f"Navigation state file: {NAV_STATE_FILE}")
    print(f"Update source file: {UPDATE_FILE}")
    print()
    print("Nav Updater ready...")
    print()


    while True:
        state = load_state()
        update_json = load_update_json()

        search_area = update_json.get("search_area", [])
        mra_refined = update_json.get("mra_refined_loiter_target", {})
        target_coords = update_json.get("target_location", {})

        search_area_send = state.get("send_searcharea_to_nav", False)
        loiter_target_send = state.get("send_loiter_target_to_nav", False)
        target_location_send = state.get("send_target_location_to_nav", False)

        clear_nav_updates = state.get("clear_nav_updates", False)

        if clear_nav_updates is True:
            update_state("clear_nav_updates", False)

            update_nav_state("search_area", NAV_DEFAULTS["search_area"])
            update_nav_state("mra_refined_loiter_target", NAV_DEFAULTS["mra_refined_loiter_target"])
            update_nav_state("mra_final_estimated_location", NAV_DEFAULTS["mra_final_estimated_location"])
            update_nav_state("eru_reported_location", NAV_DEFAULTS["eru_reported_location"])
            update_nav_state("target_location", NAV_DEFAULTS["target_location"])

            print("Cleared nav update fields from navigation_state.json.")

        elif search_area_send is True:
            update_state("send_searcharea_to_nav", False)

            if search_area:
                update_nav_state("search_area", search_area)

                print("Sending search area to nav updater...")
                print(f"Search area points: {len(search_area)}")
            else:
                print("Search area send requested, but update.json has no search_area data.")
            
            
        elif loiter_target_send is True:
            update_state("send_loiter_target_to_nav", False)

            lat = mra_refined.get("lat")
            lon = mra_refined.get("lon")
            valid = mra_refined.get("valid", False)

            if lat is not None and lon is not None:
                loiter_target = {
                    "lat": lat,
                    "lon": lon,
                    "valid": valid,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }

                update_nav_state("mra_refined_loiter_target", loiter_target)

                print("Sending loiter target to nav updater...")
                print(f"Loiter target: lat={lat}, lon={lon}, valid={valid}")
            else:
                print("Loiter target send requested, but update.json has missing lat/lon.")

        elif target_location_send is True:
            update_state("send_target_location_to_nav", False)

            lat = target_coords.get("lat")
            lon = target_coords.get("lon")
            valid = target_coords.get("valid", False)
            source = target_coords.get("source")

            if lat is not None and lon is not None:
                target_location = {
                    "source": source,
                    "lat": lat,
                    "lon": lon,
                    "valid": valid,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }

                update_nav_state("target_location", target_location)

                print("Sending target location to nav updater...")
                print(f"Target location: source={source}, lat={lat}, lon={lon}, valid={valid}")
            else:
                print("Target location send requested, but update.json has missing lat/lon.")

            

        await asyncio.sleep(state_period_s)

if __name__ == "__main__":
    asyncio.run(main())