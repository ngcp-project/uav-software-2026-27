#!/usr/bin/env python3
"""
mra_info_sender.py

Sends aircraft/Pi-side info down to the GCS over the existing MAVLink/RFD link
using chunked STATUSTEXT messages.

This replaces the idea of a fusion-only sender.

Message types sent:
  - fusion_record
  - mission_status
  - active_plan_summary
  - active_plan_full
  - search_area_zones
  - eru_reported_location
  - target_ack
  - rtl_event
  - flight_mode_event
  - autonomy_event

Each STATUSTEXT text field carries:
  D{msg_seq:04d}{chunk_idx:02d}{total:02d}:{payload}

Example:
  D00030005:{...json payload...}

Header size:
  D      = 1 char
  seq    = 4 chars
  idx    = 2 chars
  total  = 2 chars
  :      = 1 char

Total header = 10 chars
STATUSTEXT max = 50 chars
Payload per chunk = 40 chars
"""

import json
import os
import sys
import time
import hashlib
from pathlib import Path
from datetime import datetime, timezone

import requests
from pymavlink import mavutil

# Allows imports from src/state if this file is inside src/<folder>/
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state


RFD_PORT = os.environ.get("RFD_PORT", "udp:127.0.0.1:14605")
RFD_BAUD = int(os.environ.get("RFD_BAUD", 57600))

POLL_INTERVAL_S = 0.1
STATUS_HEARTBEAT_S = 1.0

# KrakenSDR DOA endpoint (same one kraken_logger.py polls)
KRAKEN_DOA_URL = os.environ.get("KRAKEN_DOA_URL", "http://127.0.0.1:8081/DOA_value.html")
KRAKEN_PROBE_INTERVAL_S = 2.0  # Don't probe every loop — every 2s is sufficient

CHUNK_PAYLOAD = 40
MAX_CHUNKS = 99

SOURCE_SYSTEM = int(os.environ.get("SOURCE_SYSTEM", 1))
SOURCE_COMPONENT = int(os.environ.get("SOURCE_COMPONENT", 191))

SRC_DIR = Path(__file__).resolve().parents[1]
NAV_STATE_FILE = Path(
    os.environ.get(
        "NAV_STATE_FILE",
        str(SRC_DIR / "state" / "navigation_state.json")
    )
)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def now_ms() -> int:
    return int(time.time() * 1000)


def stable_json_hash(obj) -> str:
    raw = json.dumps(obj, sort_keys=True, separators=(",", ":"))
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def load_nav_state() -> dict:
    try:
        with open(NAV_STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[gcs_downlink_sender] navigation_state not found: {NAV_STATE_FILE}")
    except json.JSONDecodeError:
        print("[gcs_downlink_sender] navigation_state is being written or is invalid JSON")
    except OSError as e:
        print(f"[gcs_downlink_sender] could not read navigation_state: {e}")

    return {}


def send_downlink_message(mav, msg_type: str, data: dict, msg_seq: int) -> bool:
    envelope = {
        "stream": "gcs_downlink",
        "type": msg_type,
        "msg_seq": msg_seq,
        "t_aircraft_send_ms": now_ms(),
        "timestamp": utc_now_iso(),
        "data": data,
    }

    raw = json.dumps(envelope, separators=(",", ":")).encode("utf-8")
    chunks = [raw[i:i + CHUNK_PAYLOAD] for i in range(0, len(raw), CHUNK_PAYLOAD)]
    total = len(chunks)

    if total > MAX_CHUNKS:
        print(
            f"[gcs_downlink_sender] DROP type={msg_type}, too large: "
            f"{len(raw)} bytes, {total} chunks"
        )
        return False

    for idx, chunk in enumerate(chunks):
        text = f"D{msg_seq % 10000:04d}{idx:02d}{total:02d}:{chunk.decode('utf-8')}"

        mav.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text.encode("utf-8").ljust(50, b"\x00")
        )

        time.sleep(0.04)

    return True


def read_fusion_records(fusion_path: Path) -> list[dict]:
    records = []

    try:
        with open(fusion_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return records

    for line_num, line in enumerate(lines, start=1):
        line = line.strip()

        if not line:
            continue

        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            if line_num == len(lines):
                print("[gcs_downlink_sender] last fusion line still being written, skipping for now")
            else:
                print(f"[gcs_downlink_sender] bad fusion JSON on line {line_num}, skipping")
            continue

        if isinstance(obj, dict):
            records.append(obj)

    return records


# ── KrakenSDR status probe ─────────────────────────────────────────
# Returns one of: "disconnected", "calibrating", "connected"
_cached_kraken_status = "disconnected"
_last_kraken_probe = 0.0


def poll_kraken_status() -> str:
    """Probe the KrakenSDR DOA endpoint to determine hardware state.

    Returns:
        "disconnected" — endpoint unreachable (DAQ not running)
        "calibrating"  — endpoint responds but no valid DOA data yet
        "connected"    — endpoint returns valid DOA with confidence
    """
    global _cached_kraken_status, _last_kraken_probe

    now = time.time()
    if now - _last_kraken_probe < KRAKEN_PROBE_INTERVAL_S:
        return _cached_kraken_status
    _last_kraken_probe = now

    try:
        r = requests.get(KRAKEN_DOA_URL, timeout=1)
        text = r.text.strip()
        if not text:
            _cached_kraken_status = "calibrating"
            return _cached_kraken_status

        fields = text.split(",")
        # fields[2] is confidence_0_1 — None/empty during calibration
        try:
            conf = float(fields[2].strip()) if len(fields) > 2 else None
        except (ValueError, IndexError):
            conf = None

        if conf is None:
            _cached_kraken_status = "calibrating"
        else:
            _cached_kraken_status = "connected"

    except requests.exceptions.RequestException:
        _cached_kraken_status = "disconnected"

    return _cached_kraken_status


def compact_mission_status(mission_state: dict, nav_state: dict) -> dict:
    controller_status = mission_state.get("controller_status", {})
    mission_status = mission_state.get("mission_status", {})
    active_plan = nav_state.get("active_plan", {})
    navigation = nav_state.get("navigation", {})

    return {
        "fc_mode": controller_status.get("fc_mode"),
        "autonomy_command": mission_state.get("autonomy_command"),
        "autonomy_active": controller_status.get("autonomy_active"),
        "autonomy_source": controller_status.get("autonomy_source"),
        "safety_hold": controller_status.get("safety_hold"),
        "mission_mode": mission_status.get("current_mode"),
        "mission_phase": navigation.get("mission_phase"),
        "current_waypoint": navigation.get("current_waypoint"),
        "active_plan_id": active_plan.get("plan_id"),
        "active_plan_type": active_plan.get("plan_type"),
        "active_plan_label": active_plan.get("label"),
        "active_plan_status": active_plan.get("status"),
        "kraken_status": poll_kraken_status(),
    }


def active_plan_summary(nav_state: dict) -> dict | None:
    active_plan = nav_state.get("active_plan")
    navigation = nav_state.get("navigation", {})

    if not isinstance(active_plan, dict) or not active_plan:
        return None

    waypoints = active_plan.get("waypoints", [])

    return {
        "plan_id": active_plan.get("plan_id"),
        "plan_type": active_plan.get("plan_type"),
        "label": active_plan.get("label"),
        "status": active_plan.get("status"),
        "total_waypoints": len(waypoints) if isinstance(waypoints, list) else None,
        "current_waypoint": navigation.get("current_waypoint"),
        "mission_phase": navigation.get("mission_phase"),
        "alt_ft": active_plan.get("alt_ft"),
        "loiter_radius_ft": active_plan.get("loiter_radius_ft"),
    }


def active_plan_full(nav_state: dict) -> dict | None:
    active_plan = nav_state.get("active_plan")

    if not isinstance(active_plan, dict) or not active_plan:
        return None

    return {
        "plan_id": active_plan.get("plan_id"),
        "plan_type": active_plan.get("plan_type"),
        "label": active_plan.get("label"),
        "status": active_plan.get("status"),
        "alt_ft": active_plan.get("alt_ft"),
        "loiter_radius_ft": active_plan.get("loiter_radius_ft"),
        "waypoints": active_plan.get("waypoints", []),
    }


def search_area_payload(nav_state: dict) -> dict | None:
    """
    Supports both the old format:

      "search_area": [[lat, lon], [lat, lon], ...]

    and a future zone-based format:

      "zones": [
        {
          "zone_id": 1,
          "zone_type": "search_area",
          "coordinates": [...]
        }
      ]
    """

    zones = nav_state.get("zones")

    if isinstance(zones, list) and zones:
        search_zones = []

        for zone in zones:
            if not isinstance(zone, dict):
                continue

            zone_type = zone.get("zone_type") or zone.get("type")

            if zone_type == "search_area":
                search_zones.append(zone)

        if search_zones:
            return {
                "format": "zones",
                "zones": search_zones,
            }

    search_area = nav_state.get("search_area")

    if isinstance(search_area, list) and search_area:
        return {
            "format": "legacy_search_area",
            "zones": [
                {
                    "zone_id": "search_area_0",
                    "zone_type": "search_area",
                    "coordinates": search_area,
                }
            ],
        }

    return None


def eru_payload(nav_state: dict) -> dict | None:
    eru = nav_state.get("eru_reported_location")

    if not isinstance(eru, dict):
        return None

    if eru.get("valid") is not True:
        return None

    return {
        "ack": True,
        "target": "eru_reported_location",
        "id": eru.get("report_id") or eru.get("fix_id") or eru.get("timestamp"),
        "valid": True,
        "timestamp": eru.get("timestamp"),
        "lat": eru.get("lat"),
        "lon": eru.get("lon"),
        "confidence": eru.get("confidence"),
    }


def target_ack_payload(nav_state: dict, target_name: str) -> dict | None:
    target = nav_state.get(target_name)

    if not isinstance(target, dict):
        return None

    if target.get("valid") is not True:
        return None

    target_id = (
        target.get("fix_id")
        or target.get("id")
        or target.get("timestamp")
    )

    if not target_id:
        return None

    return {
        "ack": True,
        "target": target_name,
        "id": target_id,
        "valid": True,
    }


def rtl_event_payload(mission_state: dict) -> dict | None:
    controller_status = mission_state.get("controller_status", {})

    last_rtl_event = controller_status.get("last_rtl_event")
    rtl_reason = controller_status.get("rtl_reason")

    if isinstance(last_rtl_event, dict) and last_rtl_event:
        return last_rtl_event

    if rtl_reason:
        return {
            "reason": rtl_reason,
            "source": "controller_status.rtl_reason",
            "timestamp": utc_now_iso(),
        }

    return None


def main():
    print(f"[gcs_downlink_sender] Connecting -> {RFD_PORT}")
    mav = mavutil.mavlink_connection(
        RFD_PORT,
        baud=RFD_BAUD,
        source_system=SOURCE_SYSTEM,
        source_component=SOURCE_COMPONENT,
    )

    print("[gcs_downlink_sender] Waiting for MAVLink heartbeat...")
    mav.wait_heartbeat()
    print("[gcs_downlink_sender] Heartbeat received, ready to send.")

    active_fusion_path: Path | None = None
    last_fusion_seq_sent = -1

    msg_seq = 0
    last_sent_hash = {}
    last_status_heartbeat = 0.0

    while True:
        mission_state = load_state()
        nav_state = load_nav_state()

        # ------------------------------------------------------------
        # 1. Fusion records
        # ------------------------------------------------------------
        fusion_path_str = mission_state.get("fusion_log")

        if fusion_path_str:
            fusion_path = Path(fusion_path_str)

            if fusion_path != active_fusion_path:
                print(f"[gcs_downlink_sender] New fusion session: {fusion_path.name}")
                active_fusion_path = fusion_path
                last_fusion_seq_sent = -1

            if fusion_path.exists():
                records = read_fusion_records(fusion_path)

                new_records = sorted(
                    [
                        r for r in records
                        if isinstance(r.get("telemetry_seq"), int)
                        and r["telemetry_seq"] > last_fusion_seq_sent
                    ],
                    key=lambda r: r["telemetry_seq"]
                )

                # Cap fusion records per loop so mission_status/flight_mode
                # messages aren't starved during large backlogs.
                MAX_FUSION_PER_LOOP = 3
                for record in new_records[:MAX_FUSION_PER_LOOP]:
                    if send_downlink_message(mav, "fusion_record", record, msg_seq):
                        print(
                            f"[gcs_downlink_sender] fusion_record "
                            f"telem_seq={record.get('telemetry_seq')} "
                            f"usable={record.get('usable_for_triangulation')}"
                        )
                        msg_seq += 1
                        last_fusion_seq_sent = record["telemetry_seq"]

        # ------------------------------------------------------------
        # 2. Mission status summary
        # Send on change or every STATUS_HEARTBEAT_S seconds
        # ------------------------------------------------------------
        status = compact_mission_status(mission_state, nav_state)
        status_hash = stable_json_hash(status)

        now = time.time()
        should_send_status = (
            last_sent_hash.get("mission_status") != status_hash
            or now - last_status_heartbeat >= STATUS_HEARTBEAT_S
        )

        if should_send_status:
            if send_downlink_message(mav, "mission_status", status, msg_seq):
                msg_seq += 1
                last_sent_hash["mission_status"] = status_hash
                last_status_heartbeat = now

        # ------------------------------------------------------------
        # 3. Flight mode event
        # ------------------------------------------------------------
        fc_mode = status.get("fc_mode")

        if fc_mode is not None and last_sent_hash.get("fc_mode") != fc_mode:
            payload = {
                "fc_mode": fc_mode,
                "timestamp": utc_now_iso(),
            }

            if send_downlink_message(mav, "flight_mode_event", payload, msg_seq):
                print(f"[gcs_downlink_sender] flight_mode_event fc_mode={fc_mode}")
                msg_seq += 1
                last_sent_hash["fc_mode"] = fc_mode

        # ------------------------------------------------------------
        # 4. Autonomy event
        # ------------------------------------------------------------
        autonomy_payload = {
            "autonomy_command": status.get("autonomy_command"),
            "autonomy_active": status.get("autonomy_active"),
            "autonomy_source": status.get("autonomy_source"),
        }

        autonomy_hash = stable_json_hash(autonomy_payload)

        if last_sent_hash.get("autonomy_event") != autonomy_hash:
            if send_downlink_message(mav, "autonomy_event", autonomy_payload, msg_seq):
                print(
                    "[gcs_downlink_sender] autonomy_event "
                    f"command={autonomy_payload.get('autonomy_command')} "
                    f"active={autonomy_payload.get('autonomy_active')}"
                )
                msg_seq += 1
                last_sent_hash["autonomy_event"] = autonomy_hash

        # ------------------------------------------------------------
        # 5. Active plan summary
        # ------------------------------------------------------------
        plan_summary = active_plan_summary(nav_state)

        if plan_summary:
            plan_summary_hash = stable_json_hash(plan_summary)

            if last_sent_hash.get("active_plan_summary") != plan_summary_hash:
                if send_downlink_message(mav, "active_plan_summary", plan_summary, msg_seq):
                    print(
                        "[gcs_downlink_sender] active_plan_summary "
                        f"plan_id={plan_summary.get('plan_id')} "
                        f"status={plan_summary.get('status')}"
                    )
                    msg_seq += 1
                    last_sent_hash["active_plan_summary"] = plan_summary_hash

        # ------------------------------------------------------------
        # 6. Full active plan
        # Send only when plan content changes
        # ------------------------------------------------------------
        plan_full = active_plan_full(nav_state)

        if plan_full:
            plan_full_hash = stable_json_hash(plan_full)

            if last_sent_hash.get("active_plan_full") != plan_full_hash:
                if send_downlink_message(mav, "active_plan_full", plan_full, msg_seq):
                    print(
                        "[gcs_downlink_sender] active_plan_full "
                        f"plan_id={plan_full.get('plan_id')}"
                    )
                    msg_seq += 1
                    last_sent_hash["active_plan_full"] = plan_full_hash

        # ------------------------------------------------------------
        # 7. Search area zones
        # Send only when changed
        # ------------------------------------------------------------
        search_area = search_area_payload(nav_state)

        if search_area:
            search_hash = stable_json_hash(search_area)

            if last_sent_hash.get("search_area_zones") != search_hash:
                if send_downlink_message(mav, "search_area_zones", search_area, msg_seq):
                    print("[gcs_downlink_sender] search_area_zones sent")
                    msg_seq += 1
                    last_sent_hash["search_area_zones"] = search_hash

        # ------------------------------------------------------------
        # 8. ERU reported location
        # Send only when valid and changed
        # ------------------------------------------------------------
        eru = eru_payload(nav_state)

        if eru:
            eru_hash = stable_json_hash(eru)

            if last_sent_hash.get("eru_reported_location") != eru_hash:
                if send_downlink_message(mav, "eru_reported_location", eru, msg_seq):
                    print(
                        "[gcs_downlink_sender] eru_reported_location "
                        f"id={eru.get('id')}"
                    )
                    msg_seq += 1
                    last_sent_hash["eru_reported_location"] = eru_hash

        # ------------------------------------------------------------
        # 9. Target ACKs
        # These are small confirmation messages.
        # They do NOT resend the full target.
        # ------------------------------------------------------------
        for target_name in [
            "mra_refined_loiter_target",
            "mra_final_estimated_location",
        ]:
            ack = target_ack_payload(nav_state, target_name)

            if ack:
                ack_key = f"target_ack:{target_name}"
                ack_hash = stable_json_hash(ack)

                if last_sent_hash.get(ack_key) != ack_hash:
                    if send_downlink_message(mav, "target_ack", ack, msg_seq):
                        print(
                            "[gcs_downlink_sender] target_ack "
                            f"target={target_name} id={ack.get('id')}"
                        )
                        msg_seq += 1
                        last_sent_hash[ack_key] = ack_hash

        # ------------------------------------------------------------
        # 10. RTL event
        # ------------------------------------------------------------
        rtl = rtl_event_payload(mission_state)

        if rtl:
            rtl_hash = stable_json_hash(rtl)

            if last_sent_hash.get("rtl_event") != rtl_hash:
                if send_downlink_message(mav, "rtl_event", rtl, msg_seq):
                    print("[gcs_downlink_sender] rtl_event sent")
                    msg_seq += 1
                    last_sent_hash["rtl_event"] = rtl_hash

        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    main()