import json
import os
import time
import traceback
from copy import deepcopy
from pathlib import Path

#This file sets the defaults for the navigation state JSON and provides utility functions to read and write the navigation state JSON.
NAV_STATE_FILE = Path(__file__).resolve().parent / "navigation_state.json"

DEFAULTS = {
    "search_area": [],

    "mra_refined_loiter_target": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False,
    },

    "mra_final_estimated_location": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False,
    },

    "eru_reported_location": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "report_id": None,
        "valid": False,
    },

    "target_location": {
        "source": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "id": None,
        "valid": False,
    },

    "active_plan": {
        "plan_id": None,
        "plan_type": None,
        "label": None,
        "status": None,
        "waypoints": [],
        "loiter_radius_ft": None,
        "alt_ft": None,
    },

    "next_plan": None,

    "navigation": {
        "mission_phase": None,
        "current_waypoint": None,
        "guidance_waypoint": None,
    },

    "alt_ft": 200.0,
}


def _merge_dicts(default: dict, override: dict) -> dict:
    result = deepcopy(default)

    if not isinstance(override, dict):
        return result

    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _merge_dicts(result[k], v)
        else:
            result[k] = v

    return result


def load_nav_state() -> dict:
    if not NAV_STATE_FILE.exists():
        return deepcopy(DEFAULTS)

    last_error = None

    for _ in range(5):
        try:
            loaded = json.loads(NAV_STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)

    raise RuntimeError(f"Could not read valid navigation_state.json: {last_error}")


def _atomic_write_json(path: Path, data: dict) -> None:
    tmp_path = path.with_name(f"{path.name}.{os.getpid()}.tmp")

    try:
        tmp_path.write_text(json.dumps(data, indent=2))
        os.replace(tmp_path, path)
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass


def update_nav_state(key: str, value) -> None:
    if key in ("search_area", "mra_refined_loiter_target", "target_location"):
        print(f"[NAV_STATE WRITE] {key} -> {value}")
        if os.environ.get("DEBUG_NAV_STATE") == "1":
            traceback.print_stack(limit=4)

    state = load_nav_state()
    state[key] = value
    _atomic_write_json(NAV_STATE_FILE, state)