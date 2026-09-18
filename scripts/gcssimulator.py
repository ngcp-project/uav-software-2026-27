#!/usr/bin/env python3
import json
from pathlib import Path
from datetime import datetime, timezone

MISSION_STATE_PATH = Path(__file__).resolve().parents[1] / "state" / "mission_state.json"
NAV_STATE_PATH = Path(__file__).resolve().parents[1] / "state" / "navigation_state.json"

#This file simulated GCS, but not really it was used so that I can wirelessly send commands or state updates to the UAV system for testing purposes
# Works in simulation as well
SEARCH_AREA = [
    [33.93522446917662, -117.63276100158691],
    [33.93509984969237, -117.62574434280396],
    [33.93257, -117.62892],
    [33.93246, -117.63077],
    [33.932543303485915, -117.6321331383231]
]

MRA_REFINED = {
    "lat": 33.933192,
    "lon": -117.631950,
}

TARGET_COORDS = {
    "lat": 33.933736,
    "lon": -117.630259,
}


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def load_json(path):
    if not path.exists():
        return {}
    with open(path, "r") as f:
        return json.load(f)


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    tmp.replace(path)


def target(lat, lon, valid=True, confidence=None, id_value=None):
    return {
        "lat": lat,
        "lon": lon,
        "confidence": confidence,
        "timestamp": now_utc(),
        "fix_id": id_value,
        "valid": valid,
    }


def print_sent(message, value):
    print("\033c", end="")   # clears terminal
    print("Sent this =")
    print(message)
    print(json.dumps(value, indent=2))


def send_search_area():
    state = load_json(NAV_STATE_PATH)
    state["search_area"] = SEARCH_AREA
    save_json(NAV_STATE_PATH, state)
    print_sent("search_area", SEARCH_AREA)


def send_mra_refined():
    state = load_json(NAV_STATE_PATH)

    value = target(
        MRA_REFINED["lat"],
        MRA_REFINED["lon"],
        valid=True,
        confidence=None,
        id_value="mra_refined_001",
    )

    state["mra_refined_loiter_target"] = value
    save_json(NAV_STATE_PATH, state)

    print_sent("mra_refined_loiter_target", value)


def send_mra_final():
    state = load_json(NAV_STATE_PATH)

    value = target(
        TARGET_COORDS["lat"],
        TARGET_COORDS["lon"],
        valid=True,
        confidence=None,
        id_value="mra_final_001",
    )

    state["mra_final_estimated_location"] = value
    state["target_location"] = {
        "source": "mra_final_estimated_location",
        "lat": TARGET_COORDS["lat"],
        "lon": TARGET_COORDS["lon"],
        "confidence": None,
        "timestamp": now_utc(),
        "id": "mra_final_001",
        "valid": True,
    }

    save_json(NAV_STATE_PATH, state)

    print_sent("mra_final_estimated_location + target_location", state["target_location"])


def send_eru_target():
    state = load_json(NAV_STATE_PATH)

    value = {
        "lat": TARGET_COORDS["lat"],
        "lon": TARGET_COORDS["lon"],
        "confidence": None,
        "timestamp": now_utc(),
        "report_id": "eru_001",
        "valid": True,
    }

    state["eru_reported_location"] = value
    state["target_location"] = {
        "source": "eru_reported_location",
        "lat": TARGET_COORDS["lat"],
        "lon": TARGET_COORDS["lon"],
        "confidence": None,
        "timestamp": now_utc(),
        "id": "eru_001",
        "valid": True,
    }

    save_json(NAV_STATE_PATH, state)

    print_sent("eru_reported_location + target_location", state["target_location"])


def set_autonomy_command_true():
    state = load_json(MISSION_STATE_PATH)
    state["autonomy_command"] = True

    controller_status = state.get("controller_status", {})
    controller_status["autonomy_source"] = "command_listener"
    state["controller_status"] = controller_status

    save_json(MISSION_STATE_PATH, state)

    print_sent("autonomy_command = True", {"autonomy_command": True})


def set_logging_enabled_true():
    state = load_json(MISSION_STATE_PATH)
    state["logging_enabled"] = True
    save_json(MISSION_STATE_PATH, state)

    print_sent("logging_enabled = True", {"logging_enabled": True})

def empty_target():
    return {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False,
    }


def empty_eru_target():
    return {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "report_id": None,
        "valid": False,
    }


def empty_target_location():
    return {
        "source": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "id": None,
        "valid": False,
    }


def clear_search_area():
    state = load_json(NAV_STATE_PATH)
    state["search_area"] = []
    save_json(NAV_STATE_PATH, state)
    print_sent("search_area cleared", state["search_area"])


def clear_mra_refined():
    state = load_json(NAV_STATE_PATH)
    state["mra_refined_loiter_target"] = empty_target()
    save_json(NAV_STATE_PATH, state)
    print_sent("mra_refined_loiter_target cleared", state["mra_refined_loiter_target"])


def clear_mra_final():
    state = load_json(NAV_STATE_PATH)
    state["mra_final_estimated_location"] = empty_target()

    if state.get("target_location", {}).get("source") == "mra_final_estimated_location":
        state["target_location"] = empty_target_location()

    save_json(NAV_STATE_PATH, state)
    print_sent("mra_final_estimated_location cleared", state["mra_final_estimated_location"])


def clear_eru_target():
    state = load_json(NAV_STATE_PATH)
    state["eru_reported_location"] = empty_eru_target()

    if state.get("target_location", {}).get("source") == "eru_reported_location":
        state["target_location"] = empty_target_location()

    save_json(NAV_STATE_PATH, state)
    print_sent("eru_reported_location cleared", state["eru_reported_location"])


def set_autonomy_command_false():
    state = load_json(MISSION_STATE_PATH)

    state["autonomy_command"] = False

    controller_status = state.get("controller_status", {})
    controller_status["autonomy_active"] = False
    controller_status["autonomy_source"] = None
    controller_status["safety_hold"] = None

    state["controller_status"] = controller_status

    save_json(MISSION_STATE_PATH, state)

    print_sent("autonomy_command reset", {
        "autonomy_command": False,
        "controller_status": controller_status,
    })


def set_logging_enabled_false():
    state = load_json(MISSION_STATE_PATH)
    state["logging_enabled"] = False
    save_json(MISSION_STATE_PATH, state)

    print_sent("logging_enabled = False", {"logging_enabled": False})


def reset_everything():
    nav = load_json(NAV_STATE_PATH)
    mission = load_json(MISSION_STATE_PATH)

    nav["search_area"] = []
    nav["mra_refined_loiter_target"] = empty_target()
    nav["mra_final_estimated_location"] = empty_target()
    nav["eru_reported_location"] = empty_eru_target()
    nav["target_location"] = empty_target_location()

    nav["active_plan"] = {
    "plan_id": None,
    "plan_type": None,
    "label": None,
    "status": "idle",
    "waypoints": [],
    "loiter_radius_ft": None,
    "alt_ft": None,
    }

    nav["next_plan"] = None

    nav["navigation"] = {
        "mission_phase": "idle",
        "current_waypoint": None,
        "guidance_waypoint": None,
    }

    mission["autonomy_command"] = False
    mission["logging_enabled"] = False

    mission["kraken_log"] = None
    mission["telemetry_log"] = None
    mission["fusion_log"] = None

    controller_status = mission.get("controller_status", {})
    controller_status["autonomy_active"] = False
    controller_status["autonomy_source"] = None
    controller_status["safety_hold"] = None
    mission["controller_status"] = controller_status

    save_json(NAV_STATE_PATH, nav)
    save_json(MISSION_STATE_PATH, mission)

    print_sent("everything reset for new simulation", {
        "navigation_state_reset": True,
        "mission_state_reset": True,
        "kraken_log": None,
        "telemetry_log": None,
        "fusion_log": None,
        "autonomy_command": False,
        "logging_enabled": False,
        "autonomy_active": False,
    })

def show_menu():
    print()
    print("========== GCS SIM ==========")
    print("1. Send search area")
    print("2. Send MRA refined loiter target")
    print("3. Send MRA final target")
    print("4. Send ERU target")
    print("5. Set autonomy_command = true")
    print("6. Set logging_enabled = true")
    print()
    print("7. Clear search area")
    print("8. Clear MRA refined loiter target")
    print("9. Clear MRA final target")
    print("10. Clear ERU target")
    print("11. Set autonomy_command = false")
    print("12. Set logging_enabled = false")
    print("13. Reset everything for new simulation")
    print("q. Quit")
    print("=============================")


def main():
    print(f"Nav state:     {NAV_STATE_PATH}")
    print(f"Mission state: {MISSION_STATE_PATH}")

    while True:
        show_menu()
        choice = input("Command: ").strip().lower()

        if choice == "1":
            send_search_area()
        elif choice == "2":
            send_mra_refined()
        elif choice == "3":
            send_mra_final()
        elif choice == "4":
            send_eru_target()
        elif choice == "5":
            set_autonomy_command_true()
        elif choice == "6":
            set_logging_enabled_true()
        elif choice == "7":
            clear_search_area()
        elif choice == "8":
            clear_mra_refined()
        elif choice == "9":
            clear_mra_final()
        elif choice == "10":
            clear_eru_target()
        elif choice == "11":
            set_autonomy_command_false()
        elif choice == "12":
            set_logging_enabled_false()
        elif choice == "13":
            reset_everything()
        elif choice == "q":
            break
        else:
            print("Unknown command.")


if __name__ == "__main__":
    main()