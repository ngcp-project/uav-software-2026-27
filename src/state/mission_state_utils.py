import json
import os
import time
import traceback
from copy import deepcopy
from pathlib import Path

#This file sets the defaults for the JSON files and provides utility functions to read and write the mission state JSON.
STATE_FILE = Path(__file__).resolve().parent / "mission_state.json"

DEFAULTS = {
    "logging_enabled": False,
    "last_command": None,
    "timestamp": None,
    "last_sender_sysid": None,
    "last_sender_compid": None,
    "autonomy_command": False,

    "kraken_log": None,
    "telemetry_log": None,
    "fusion_log": None,

    "controller_status": {
        "fc_mode": None,
        "autonomy_active": False,
        "autonomy_source": None,
        "rtl_reason": None,
        "fc_mode_last_updated": None,
        "safety_hold": None,
        "last_heartbeat_utc": None,
        "last_rtl_event": {
            "reason": None,
            "source": None,
            "timestamp": None,
            "fc_mode": None,
            "previous_fc_mode": None,
        },
    },

    "pending_action": None,
    "rtl_requested": False,
    "loiter_requested": False,

    "mission_status": {
        "active_plan_id": None,
        "last_completed_plan_id": None,
        "mission_count": 0,
        "current_mode": None,
        "active_waypoint_index": None,
    },

    "send_searcharea_to_nav": False,
    "send_loiter_target_to_nav": False,
    "send_target_location_to_nav": False,
    "clear_nav_updates": False,
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


def load_state() -> dict:
    if not STATE_FILE.exists():
        return deepcopy(DEFAULTS)

    last_error = None

    for _ in range(5):
        try:
            loaded = json.loads(STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)

    raise RuntimeError(f"Could not read valid mission_state.json: {last_error}")


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


def update_state(key: str, value) -> None:
    if key in ("logging_enabled", "telemetry_log", "kraken_log", "fusion_log"):
        print(f"[MISSION_STATE WRITE] {key} -> {value}")
        traceback.print_stack(limit=4)

    state = load_state()
    state[key] = value
    _atomic_write_json(STATE_FILE, state)