#!/usr/bin/env python3
"""
gcs_adapter.py  --  Script 2: the adapter. It TRANSLATES between the GCS and the UAV's state files.

gcs_xbee.py (Script 1) is only a relay. It owns the GCS XBee library and hands this script:

    COMMANDS   the library's decoded command as JSON, unchanged, e.g.
               {"Command ID": 3, "Packet ID": 1, "Zone": 2, "Zone ID": 0, "Coordinates": [[lat, lon], ...]}
    TELEMETRY  (this script -> relay) the library's Telemetry fields by name, in the units the GCS expects.

COMMANDS: what each one does here
    Heartbeat        records the GCS link status in mission_state.json  ("gcs_link")
    EmergencyStop    asks the flight controller loop to loiter (loiter_requested) -- see EMERGENCY_STOP_ACTION
    AddZone          SearchArea  -> navigation_state.json "search_area"
                     KeepIn/Out  -> navigation_state.json "gcs_zones"   (stored only; nothing enforces them yet)
    PatientLocation  -> navigation_state.json "eru_reported_location" and "target_location"
                       (the same thing the old gcs_bridge.py did for the ERU's reported location)

    Every command that is applied is "receipted": its Command ID and Packet ID go into the next telemetry
    packets, which is how the GCS knows the command landed. A command that is refused (bad data, unknown
    type) is NOT receipted, so the GCS will not think it worked.

TELEMETRY: where each field comes from (5 per second)
    battery, position, altitude   telemetry_state.json
    patient found / location      localization_state.json
    speed, pitch, yaw, roll       not in the state files yet -> 0   (TODO: someone has to add them)

Pairs with gcs_xbee.py: this script BINDS the two ports (commands = PULL, telemetry = PUSH).

Settings (environment variables, all optional):
    GCS_CMD_PORT     default 6767        GCS_TLM_PORT   default 3435
    UAV_SRC          the repo's src/ folder (found automatically if this script is inside the repo)
    GCS_STATE_DIR    read and write copies of the four state files in this folder INSTEAD of src/state.
                     Use it for testing so the real state files are not touched.
"""
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import zmq

# ═══════════════════════════════ SETTINGS ═══════════════════════════════════
CMD_PORT = int(os.environ.get("GCS_CMD_PORT", 6767))
TLM_PORT = int(os.environ.get("GCS_TLM_PORT", 3435))

TELEMETRY_PERIOD_S = 0.2            # 5 telemetry packets per second

# What an EmergencyStop from the GCS does:
#   "loiter"  hold position (sets loiter_requested; main_controller already handles it)   <- default
#   "rtl"     return to launch (sets rtl_requested)
#   "none"    only record it
# A hard motor stop in flight would crash the UAV, so this is NOT one of the choices.
# TEAM DECISION: confirm "loiter" is what we want.
EMERGENCY_STOP_ACTION = "loiter"

# CHECK with the GCS team: the units and meanings they expect in telemetry.
ALTITUDE_IN_FEET = True             # CHECK: telemetry_state has metres. True = send feet, False = send metres.
LAST_UPDATED_PER_SECOND = 1000      # CHECK: 1000 = LastUpdated in milliseconds, 1 = seconds.
# CHECK: position order. Commands from the GCS arrive as (lat, lon), so telemetry is sent as (lat, lon).

# Numbers the GCS library uses (Packet/Enum/*.py and Packet/Command/*.py).
class CommandID:
    HEARTBEAT = 1
    EMERGENCY_STOP = 2
    ADD_ZONE = 3
    PATIENT_LOCATION = 4

ZONE_NAMES = {0: "KeepIn", 1: "KeepOut", 2: "SearchArea"}
ZONE_SEARCH_AREA = 2
ZONE_KEEP_IN = 0
LINK_NAMES = {0: "Connected", 1: "Unstable", 2: "Disconnected"}

MESSAGE_NONE, MESSAGE_PACKAGE, MESSAGE_PATIENT = 0, 1, 2     # Telemetry.MessageFlag (from the library's comments)
FEET_PER_METRE = 3.28084


# ═════════════════════════ FIND THE STATE FILE HELPERS ══════════════════════
def find_src():
    """The repo's src/ folder: UAV_SRC if set, otherwise look upward from this file for src/state."""
    env = os.environ.get("UAV_SRC")
    if env:
        return Path(env)
    here = Path(__file__).resolve().parent
    for folder in [here, *here.parents]:
        if (folder / "src" / "state").is_dir():
            return folder / "src"
    return None


SRC = find_src()
if SRC is None or not (SRC / "state").is_dir():
    print("[Adapter] ERROR: could not find the repo's src/state folder.")
    print("          Run this script from inside the repo, or set UAV_SRC to the repo's src folder, e.g.")
    print("              UAV_SRC=~/projects/uav-software-2026-27/src python gcs_adapter.py")
    sys.exit(1)
sys.path.insert(0, str(SRC))

from state import localization_state_utils as localization_state      # noqa: E402  (after the path is set)
from state import mission_state_utils as mission_state                # noqa: E402
from state import nav_state_utils as nav_state                        # noqa: E402
from state import telemetry_state_utils as telemetry_state            # noqa: E402


def use_state_dir(folder):
    """Point the four state helpers at copies in `folder` (for testing). Each helper reads its
    file path from a module variable at the moment it is used, so changing the variable is enough."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    telemetry_state.STATE_FILE = folder / "telemetry_state.json"
    localization_state.STATE_FILE = folder / "localization_state.json"
    mission_state.STATE_FILE = folder / "mission_state.json"
    nav_state.NAV_STATE_FILE = folder / "navigation_state.json"


# ═════════════════════════════════ HELPERS ══════════════════════════════════
def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()


def to_float(value, default=0.0):
    """A finite float, or `default` for None / text / NaN / infinity."""
    try:
        x = float(value)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def valid_coord(lat, lon):
    """A real latitude/longitude. Zero is refused, like gcs_bridge.py does: it is what missing or damaged data looks like."""
    try:
        lat, lon = float(lat), float(lon)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    return -90 <= lat <= 90 and -180 <= lon <= 180 and lat != 0.0 and lon != 0.0


def valid_polygon(points):
    """3 to 6 (lat, lon) points. The library only allows 3 to 6 per zone."""
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= 6:
        return False
    return all(isinstance(p, (list, tuple)) and len(p) == 2 and valid_coord(p[0], p[1]) for p in points)


# ═══════════════════ COMMANDS: GCS -> state files ═══════════════════════════
# Each handler returns True when the command was applied, False when it was refused.
def handle_heartbeat(command):
    code = command.get("Connection Status")
    name = LINK_NAMES.get(code)
    if name is None:
        print(f"Heartbeat rejected: unknown Connection Status {code!r}")
        return False
    mission_state.update_state("gcs_link", {
        "status": name,
        "status_code": code,
        "last_heartbeat_utc": utc_now_iso(),
        "last_heartbeat_epoch": time.time(),
    })
    if code != 0:
        print(f"[Adapter] WARNING: the GCS reports its link as {name}")
    return True


def handle_emergency_stop(command):
    stop_status = command.get("Stop Status")        # CHECK: what do the values mean? Every value is treated as "stop".
    action = EMERGENCY_STOP_ACTION
    mission_state.update_state("gcs_emergency_stop", {
        "received_utc": utc_now_iso(),
        "stop_status": stop_status,
        "action": action,
        "packet_id": command.get("Packet ID"),
    })
    if action == "loiter":
        mission_state.update_state("loiter_requested", True)
    elif action == "rtl":
        mission_state.update_state("rtl_requested", True)
    print(f"[Adapter] EMERGENCY STOP from the GCS -> {action}")
    return True


def handle_add_zone(command):
    zone_type = command.get("Zone")
    zone_id = command.get("Zone ID")
    points = command.get("Coordinates")
    if zone_type not in ZONE_NAMES:
        print(f"AddZone rejected: unknown zone type {zone_type!r}")
        return False
    if not valid_polygon(points):
        print(f"AddZone rejected: the coordinates are not 3-6 valid (lat, lon) points: {points!r}")
        return False
    points = [[float(lat), float(lon)] for lat, lon in points]

    if zone_type == ZONE_SEARCH_AREA:
        nav_state.update_nav_state("search_area", points)
        print(f"[Adapter] search area set ({len(points)} points)")
    else:
        zones = nav_state.load_nav_state().get("gcs_zones") or {}
        kind = "keep_in" if zone_type == ZONE_KEEP_IN else "keep_out"
        zones.setdefault("keep_in", {})
        zones.setdefault("keep_out", {})
        zones[kind][str(zone_id)] = {"points": points, "received_utc": utc_now_iso()}
        nav_state.update_nav_state("gcs_zones", zones)
        print(f"[Adapter] {ZONE_NAMES[zone_type]} zone {zone_id} stored ({len(points)} points). "
              f"Nothing enforces keep-in / keep-out zones yet.")
    return True


def handle_patient_location(command):
    coordinate = command.get("Coordinate")
    if not (isinstance(coordinate, (list, tuple)) and len(coordinate) == 2 and valid_coord(*coordinate)):
        print(f"PatientLocation rejected: not a valid (lat, lon): {coordinate!r}")
        return False
    lat, lon = float(coordinate[0]), float(coordinate[1])
    stamp = utc_now_iso()
    report_id = command.get("Packet ID")
    # Same two writes as the old gcs_bridge.py made for the ERU's reported location.
    nav_state.update_nav_state("eru_reported_location", {
        "lat": lat, "lon": lon, "confidence": None, "timestamp": stamp, "report_id": report_id, "valid": True,
    })
    nav_state.update_nav_state("target_location", {
        "source": "eru_reported_location", "lat": lat, "lon": lon, "confidence": None,
        "timestamp": stamp, "id": report_id, "valid": True,
    })
    print(f"[Adapter] patient location set: {lat}, {lon}")
    return True


def handle_command(command):
    """Same four cases as VehicleTest.py, but each one now does its job. Returns True if applied."""
    if not isinstance(command, dict):
        print(f"Command rejected: expected a JSON object, got {type(command).__name__}")
        return False
    try:
        match command.get("Command ID"):
            case CommandID.HEARTBEAT:
                print("Executing Heartbeat code")
                return handle_heartbeat(command)
            case CommandID.EMERGENCY_STOP:
                print("Executing Emergency Stop code")
                return handle_emergency_stop(command)
            case CommandID.ADD_ZONE:
                print("Executing Add Zone code")
                return handle_add_zone(command)
            case CommandID.PATIENT_LOCATION:
                print("Executing Patient Location code")
                return handle_patient_location(command)
            case _:
                print("Unknown Command Type")
                return False
    except Exception as e:                       # a bad command or a bad state file must not stop the adapter
        print(f"[Adapter] Could not execute the command ({e!r})")
        return False


# ═════════════════ TELEMETRY: state files -> the GCS's fields ═══════════════
def build_telemetry(receipt):
    """
    The library's Telemetry fields by name (see the top of gcs_xbee.py for the contract).
    Raises if a state file cannot be read; the caller skips that packet.
    """
    tele = telemetry_state.load_state()
    loc = localization_state.load_state()
    position, battery, patient = tele["position"], tele["battery"], loc["patient"]

    altitude = to_float(position.get("relative_altitude_m"))
    if ALTITUDE_IN_FEET:
        altitude *= FEET_PER_METRE

    updated = to_float(position.get("last_updated"))                 # epoch seconds, or 0 if never
    last_updated = int(updated * LAST_UPDATED_PER_SECOND) if updated > 0 else 0

    found = bool(patient.get("patient_found")) and valid_coord(patient.get("patient_lat"), patient.get("patient_lon"))

    return {
        "CommandID": int(receipt[0]),
        "PacketID": int(receipt[1]),
        "Speed": 0.0,                         # TODO: not in telemetry_state.json
        "Pitch": 0.0,                         # TODO: attitude is not in the state files either
        "Yaw": 0.0,
        "Roll": 0.0,
        "Altitude": altitude,
        "BatteryLife": to_float(battery.get("remaining_percent")),
        "LastUpdated": last_updated,          # 0 = no position yet. Without it the GCS can't tell 0,0 from "no fix".
        "CurrentPosition": [to_float(position.get("latitude")), to_float(position.get("longitude"))],
        "VehicleStatus": 0,                   # TODO/CHECK: the GCS's meaning for 0-255 is not defined anywhere I could find
        "MessageFlag": MESSAGE_PATIENT if found else MESSAGE_NONE,
        "MessageLat": float(patient["patient_lat"]) if found else 0.0,
        "MessageLon": float(patient["patient_lon"]) if found else 0.0,
        "PatientStatus": 1 if found else 0,   # CHECK: the GCS's meaning for this value
    }


# ═══════════════════════════════════ MAIN ═══════════════════════════════════
def main():
    state_dir = os.environ.get("GCS_STATE_DIR")
    if state_dir:
        use_state_dir(state_dir)

    context = zmq.Context()

    # Receives commands
    command_socket = context.socket(zmq.PULL)
    command_socket.bind(f"tcp://127.0.0.1:{CMD_PORT}")

    # Sends telemetry
    telemetry_socket = context.socket(zmq.PUSH)
    telemetry_socket.bind(f"tcp://127.0.0.1:{TLM_PORT}")

    print(f"[Adapter] state files: {telemetry_state.STATE_FILE.parent}"
          + ("   (GCS_STATE_DIR: test copies)" if state_dir else ""), flush=True)
    print(f"[Adapter] EmergencyStop action: {EMERGENCY_STOP_ACTION}", flush=True)
    print("Adapter is waiting for commands...", flush=True)

    last_sent = 0
    receipt = (0, 0)             # (Command ID, Packet ID) of the last command that was applied
    telemetry_errors = 0

    try:
        while True:
            # Wait up to 50 ms for a command, then move on
            if command_socket.poll(50):
                try:
                    command = json.loads(command_socket.recv())
                except (ValueError, UnicodeDecodeError) as e:      # json.JSONDecodeError is a ValueError
                    print(f"JSON Error: {e}", flush=True)
                else:
                    print("Received command:", command, flush=True)
                    applied = handle_command(command)
                    if applied:                      # (applied is only True for a JSON object)
                        ids = (command.get("Command ID"), command.get("Packet ID"))
                        if all(isinstance(i, int) and not isinstance(i, bool) for i in ids):
                            receipt = ids
                    sys.stdout.flush()

            # Send telemetry 5 times a second, whether or not a command arrived
            if time.time() - last_sent >= TELEMETRY_PERIOD_S:
                try:
                    telemetry = build_telemetry(receipt)
                except Exception as e:
                    telemetry_errors += 1
                    if telemetry_errors <= 3 or telemetry_errors % 50 == 0:
                        print(f"[Adapter] could not build telemetry from the state files ({telemetry_errors}x): {e!r}",
                              flush=True)
                else:
                    try:
                        telemetry_socket.send_json(telemetry, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        pass  # nobody is listening yet, skip this one instead of freezing
                last_sent = time.time()

    except KeyboardInterrupt:
        print("\nAdapter shutting down.", flush=True)

    finally:
        command_socket.close(linger=0)
        telemetry_socket.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
