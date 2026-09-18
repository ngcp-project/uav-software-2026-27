#!/usr/bin/env python3
"""
mra_info_receiver.py

Receives chunked aircraft/Pi info-stream records from the RFD900x/MAVLink link
and forwards completed JSON messages over UDP to the custom Kraken/GCS app.

This replaces the old fusion-only receiver.

Expected STATUSTEXT frame format:

  D{msg_seq:04d}{chunk_idx:02d}{total:02d}:{payload}

Example:

  D00030005:{...json payload...}

The completed JSON message looks like:

{
  "stream": "gcs_downlink",
  "type": "fusion_record",
  "msg_seq": 3,
  "t_aircraft_send_ms": 123456789,
  "timestamp": "...",
  "data": {...}
}
"""

import json
import os
import socket
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from pymavlink import mavutil


RFD_PORT = os.environ.get("RFD_PORT", "udp:127.0.0.1:14551")
RFD_BAUD = int(os.environ.get("RFD_BAUD", 57600))

APP_HOST = os.environ.get("GCS_APP_HOST", "127.0.0.1")
APP_PORT = int(os.environ.get("GCS_APP_PORT", 5051))

LOG_DIR = Path(os.environ.get("GCS_DOWNLINK_LOG_DIR", "logs/gcs_downlink"))
LOG_DIR.mkdir(parents=True, exist_ok=True)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def now_ms() -> int:
    return int(time.time() * 1000)


def main():
    print(f"[gcs_downlink_receiver] Listening on {RFD_PORT}")
    mav = mavutil.mavlink_connection(
        RFD_PORT,
        baud=RFD_BAUD,
        input=True,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[gcs_downlink_receiver] Forwarding -> {APP_HOST}:{APP_PORT}")

    log_path = LOG_DIR / f"gcs_downlink_{datetime.utcnow():%Y%m%d_%H%M%S}.jsonl"
    print(f"[gcs_downlink_receiver] Log -> {log_path}")

    # { msg_seq: { chunk_idx: payload_str, "_total": int, "_first_seen_ms": int } }
    buf = defaultdict(dict)

    count = 0

    with open(log_path, "w", encoding="utf-8") as log_f:
        while True:
            msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)

            if msg is None:
                continue

            text = msg.text.rstrip("\x00").strip()

            # Only process downlink messages from this new bridge.
            if not text.startswith("D"):
                continue

            try:
                header, payload = text.split(":", 1)

                msg_seq = int(header[1:5])
                chunk_idx = int(header[5:7])
                total = int(header[7:9])

            except ValueError:
                continue

            if total <= 0 or total > 99:
                continue

            if chunk_idx < 0 or chunk_idx >= total:
                continue

            if "_first_seen_ms" not in buf[msg_seq]:
                buf[msg_seq]["_first_seen_ms"] = now_ms()

            buf[msg_seq][chunk_idx] = payload
            buf[msg_seq]["_total"] = total

            # If complete, rebuild JSON.
            if all(i in buf[msg_seq] for i in range(total)):
                raw = "".join(buf[msg_seq][i] for i in range(total))
                del buf[msg_seq]

                try:
                    envelope = json.loads(raw)
                except json.JSONDecodeError as e:
                    print(f"[gcs_downlink_receiver] Bad JSON: {e} | RAW: {raw!r}")
                    continue

                if not isinstance(envelope, dict):
                    continue

                envelope["t_gcs_rx_ms"] = now_ms()
                envelope["gcs_rx_timestamp"] = utc_now_iso()

                out = json.dumps(envelope, separators=(",", ":")).encode("utf-8")
                sock.sendto(out, (APP_HOST, APP_PORT))

                log_f.write(json.dumps(envelope, separators=(",", ":")) + "\n")
                log_f.flush()

                count += 1

                msg_type = envelope.get("type")
                data = envelope.get("data", {})

                if msg_type == "fusion_record":
                    print(
                        f"[gcs_downlink_receiver] {time.strftime('%H:%M:%S')} "
                        f"type=fusion_record "
                        f"telem_seq={data.get('telemetry_seq')} "
                        f"kraken_seq={data.get('kraken_seq')} "
                        f"usable={data.get('usable_for_triangulation')} "
                        f"total={count}"
                    )

                elif msg_type == "target_ack":
                    print(
                        f"[gcs_downlink_receiver] {time.strftime('%H:%M:%S')} "
                        f"type=target_ack "
                        f"target={data.get('target')} "
                        f"id={data.get('id')} "
                        f"ack={data.get('ack')} "
                        f"total={count}"
                    )

                elif msg_type == "active_plan_summary":
                    print(
                        f"[gcs_downlink_receiver] {time.strftime('%H:%M:%S')} "
                        f"type=active_plan_summary "
                        f"plan_id={data.get('plan_id')} "
                        f"status={data.get('status')} "
                        f"wp={data.get('current_waypoint')}/{data.get('total_waypoints')} "
                        f"total={count}"
                    )

                else:
                    print(
                        f"[gcs_downlink_receiver] {time.strftime('%H:%M:%S')} "
                        f"type={msg_type} "
                        f"msg_seq={envelope.get('msg_seq')} "
                        f"total={count}"
                    )

            # Prune stale incomplete buffers.
            # This prevents old partial chunk sets from living forever.
            now_time = now_ms()
            stale_keys = []

            for seq_id, chunks in buf.items():
                first_seen = chunks.get("_first_seen_ms", now_time)

                if now_time - first_seen > 10_000:
                    stale_keys.append(seq_id)

            for seq_id in stale_keys:
                del buf[seq_id]


if __name__ == "__main__":
    main()