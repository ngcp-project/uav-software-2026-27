import json
import os
import time
from copy import deepcopy
from pathlib import Path

#This json file is meant to hold things that the flight controller reports and that our scripts AND what GCS needs.
# So this telemetry_state.json would be parsed by some sort of GCS adapter as well as by main_controller for the flight mode

#This would be constantly being replaced and is different from telemetry.jsonl which is more historical and used by fusion_logger / localization team
STATE_FILE = Path(__file__).resolve().parent / "telemetry_state.json"

DEFAULTS = {
    "timestamp": None,

    "flight_controller": {
        "connected": False,
        "last_heartbeat": None,
        "mode": None,
        "armed": False,
    },

    "position": {
        "latitude": None,
        "longitude": None,
        "relative_altitude_m": None,
        "absolute_altitude_m": None,
    },

    "battery": {
        "voltage_v": None,
        "remaining_percent": None,
    },
}


def merge_dicts(default: dict, override: dict) -> dict:
    result = deepcopy(default)

    if not isinstance(override, dict):
        return result

    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_dicts(result[key], value)
        else:
            result[key] = value

    return result

def load_state() -> dict:
    if not STATE_FILE.exists():
        return deepcopy(DEFAULTS)

    last_error = None

    for _ in range(5):
        try:
            loaded = json.loads(STATE_FILE.read_text())
            return merge_dicts(DEFAULTS, loaded)
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)

    raise RuntimeError(
        f"Could not read valid telemetry_state.json: {last_error}"
    )


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

def update_section(section: str, updates: dict) -> None:
    state = load_state()
    current = state.get(section, {})

    if not isinstance(current, dict):
        current = {}

    state[section] = {
        **current,
        **updates
    }
    _atomic_write_json(STATE_FILE, state)