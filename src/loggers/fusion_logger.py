import argparse
import bisect
import json
import time
from pathlib import Path

import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state

DEFAULT_MAX_TIME_DIFF_MS     = 250 #Difference in telemetry t_rx_ms and kraken t_rx_ms (Can't be over 250 cuz that means its too far apart)
DEFAULT_MIN_CONFIDENCE       = 0.3
DEFAULT_MAX_ROLL_DEG         = 60.0
DEFAULT_MIN_GROUND_SPEED_FT_S = 0.0

SCRIPT_NAME = "fusion_logger.py"

BASE_DIR   = Path(__file__).resolve().parents[2]


FUSION_DIR = BASE_DIR / "logs" / "fusion"
FUSION_DIR.mkdir(parents=True, exist_ok=True)

IDLE_POLL_INTERVAL_S   = 1.0 # How often the idle loop polls mission_state for a new telemetry file in seconds

ACTIVE_POLL_INTERVAL_S = 0.5 # How often an active fusion session re-reads both logs and rewrites the fusion output in seconds

def load_jsonl(path: Path) -> list[dict]:
    records = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"{path.name}:{lineno} — skipping bad JSON: {e}")
    return records

def build_timestamp_index(records: list[dict]) -> list[int]:
    return [r["t_rx_ms"] for r in records]


def find_nearest(timestamps: list[int], target: int) -> int:
    n = len(timestamps)
    if n == 0:
        raise ValueError("timestamps list is empty")

    pos = bisect.bisect_left(timestamps, target)

    if pos == 0:
        return 0
    if pos == n:
        return n - 1

    
    before = pos - 1
    after  = pos
    if abs(timestamps[before] - target) <= abs(timestamps[after] - target):
        return before
    return after

# For triangulation to see if usable in confidence, roll, and ground speed constraints
def is_usable(
    confidence_0_1: float | None,
    roll_deg: float | None,
    ground_speed_ft_s: float | None,
    dt_ms: int,
    *,
    max_time_diff_ms: int,
    min_confidence: float,
    max_roll_deg: float,
    min_ground_speed_ft_s: float,
) -> bool:
    if abs(dt_ms) > max_time_diff_ms:
        print(f"FAIL Time Difference: {dt_ms}")
        return False
    if confidence_0_1 is None or confidence_0_1 < min_confidence:
        print(f"FAIL Confidence: {confidence_0_1}")
        return False
    if roll_deg is None or abs(roll_deg) > max_roll_deg:
        print(f"FAIL Roll Deg: {roll_deg}")
        return False
    if ground_speed_ft_s is None or ground_speed_ft_s < min_ground_speed_ft_s:
        print(f"FAIL Ground Speed Ft/s: {ground_speed_ft_s}")
        return False
    return True

def fuse(
    kraken_records: list[dict],
    tel_records: list[dict],
    *,
    max_time_diff_ms: int,
    min_confidence: float,
    max_roll_deg: float,
    min_ground_speed_ft_s: float,
) -> list[dict]:
    
    # For each Telem record, find the nearest telemetry record by t_rx_ms.
    # Don't use if abs(dt_ms) > max_time_diff_ms.
    # Returns a list of fused records.
    
    # tel_timestamps = build_timestamp_index(tel_records)
    fused = []
    kraken_timestamps = build_timestamp_index(kraken_records) if kraken_records else []

    for t in tel_records:
        t_ts = t["t_rx_ms"]
        
        matched_k = None
        dt_ms = None

        if kraken_timestamps:
            idx = find_nearest(kraken_timestamps, t_ts)
            candidate = kraken_records[idx]
            candidate_dt_ms = t_ts - candidate["t_rx_ms"]

            confidence = candidate.get("confidence_0_1")

            # keep Kraken only if it is close enough and confidence is good enough
            if abs(candidate_dt_ms) <= max_time_diff_ms and confidence is not None:
                matched_k = candidate
                dt_ms = candidate_dt_ms

        roll = t.get("roll_deg")
        ground_speed = t.get("ground_speed_ft_s")

        usable = False
        if matched_k is not None and dt_ms is not None:
            usable = is_usable(
                matched_k.get("confidence_0_1"),
                roll,
                ground_speed,
                dt_ms,
                max_time_diff_ms=max_time_diff_ms,
                min_confidence=min_confidence,
                max_roll_deg=max_roll_deg,
                min_ground_speed_ft_s=min_ground_speed_ft_s,
            )

        record = {
            
            "run_id":           t.get("run_id") or (matched_k.get("run_id") if matched_k else None),

            "kraken_seq":       matched_k.get("seq") if matched_k else None,
            "telemetry_seq":    t.get("seq"),
            "t_rx_ms":          t_ts, #Telem time is the fusion anchor
            "dt_ms":            dt_ms,#Telemetry.t_rx_ms - kraken.t_rx_ms

            
            "doa_deg": matched_k.get("doa_deg") if matched_k else None,
            "confidence_0_1": matched_k.get("confidence_0_1") if matched_k else None,

            "lat_deg": t.get("lat_deg"),
            "lon_deg": t.get("lon_deg"),
            "altitude_rel_ft": t.get("altitude_rel_ft"),
            "yaw_deg": t.get("yaw_deg"),
            "pitch_deg": t.get("pitch_deg"),
            "roll_deg": roll,
            "ground_speed_ft_s": ground_speed,
            "vel_north_m_s": t.get("vel_north_m_s"),
            "vel_east_m_s": t.get("vel_east_m_s"),
            "vel_down_m_s": t.get("vel_down_m_s"),

            
            "usable_for_triangulation": usable,
        }
        fused.append(record)

    return fused


def main() -> None:
    parser = argparse.ArgumentParser(description="Fused Kraken DOA + telemetry logs.")
    parser.add_argument("--max_time_diff_ms",      type=int,   default=DEFAULT_MAX_TIME_DIFF_MS)
    parser.add_argument("--min_confidence",        type=float, default=DEFAULT_MIN_CONFIDENCE)
    parser.add_argument("--max_roll_deg",          type=float, default=DEFAULT_MAX_ROLL_DEG)
    parser.add_argument("--min_ground_speed_ft_s", type=float, default=DEFAULT_MIN_GROUND_SPEED_FT_S)
    args = parser.parse_args()

    active_session: tuple[str, str] | None = None
    out_file: Path | None = None
    run_id: str | None = None
    t_fusion_start_ms: int | None = None

    
    print("[fusion_logger] Waiting for kraken_log and telemetry_log in mission_state ...")
    


    # Main continuous loop 
    while True:
        # Re-read mission_state on every iteration so we always see the latest paths written by other loggers.
        #This is so if the telemetry/kraken log changes mid-run, we detect a new session and update the active session accordingly.
        state = load_state()

        kraken_path_str = state.get("kraken_log")
        tel_path_str    = state.get("telemetry_log")


        # Kraken log need valid path in state before fusing
        if not kraken_path_str:
            print("[fusion_logger] Waiting: kraken_logger not in state yet …")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        #Telemetry needs valid path
        if not tel_path_str:
            print("[fusion_logger] Waiting: telemetry_loggger not in state yet ...")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        session = (kraken_path_str, tel_path_str)

        kraken_file = Path(kraken_path_str)
        tel_file    = Path(tel_path_str)   # tel_path_str is not None here

        # If a new session / if the active session changes, fires once when telemetry_log first changes 
        # Active_session tracks the (kraken_log, telemetry_log) pair being fused
        if session != active_session: #So if it's different v
            # Extract run_id from the kraken filename, same as the original code.
            run_id = kraken_file.stem.replace("doa_", "")

            out_file = FUSION_DIR / f"fusion_{run_id}.jsonl" #Names it the run id 

            # Record start time of fusion session for meta file
            t_fusion_start_ms = int(time.time() * 1000)

            active_session = session

            print(f"\n[fusion_logger] New fusion session detected.")
            print(f"  run_id      : {run_id}")
            print(f"  Kraken      : {kraken_file.name}")
            print(f"  Telemetry   : {tel_file.name}")
            print(f"  Output      : {out_file.name}")

            update_state("fusion_log", str(out_file))

        #Makes sure it exists first
        if kraken_file.exists():
            kraken_records = load_jsonl(kraken_file)
        else:
            kraken_records = []

        if not tel_file.exists():
            print(f"[fusion_logger] Waiting: Telemetry file does not exist yet: {tel_file}")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue
        # Re-read both files completely on every cycle
        # JSONL files are append-only, loading them fresh each time is the simplest way to pick up newly appended records
        
        tel_records    = load_jsonl(tel_file)

        # Sort telemetry by timestamp in case records arrive out of order
        tel_records.sort(key=lambda r: r["t_rx_ms"])

        if not tel_records:
            print("[fusion_logger] Waiting: telemetry log exists but no data yet")
            time.sleep(ACTIVE_POLL_INTERVAL_S)
            continue

        # kraken_records can be empty now, because the kraken log may not have any records yet, but we still care about telemetry/location of UAV

        fused = fuse(
            kraken_records,
            tel_records,
            max_time_diff_ms=args.max_time_diff_ms,
            min_confidence=args.min_confidence,
            max_roll_deg=args.max_roll_deg,
            min_ground_speed_ft_s=args.min_ground_speed_ft_s,
        )

        n_usable    = sum(1 for r in fused if r["usable_for_triangulation"])
        n_with_kraken = sum(1 for r in fused if r["kraken_seq"] is not None)
        n_tel_only = len(fused) - n_with_kraken

        print(
            f"[fusion_logger] {time.strftime('%H:%M:%S')} — "
            f"kraken={len(kraken_records)} | tel={len(tel_records)} | "
            f"rows={len(fused)} | matched={n_with_kraken} | "
            f"usable={n_usable} | tel_only={n_tel_only}"
        )
        
        if out_file is None or run_id is None:
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        # Overwrite the fusion file completely on each cycle
        with open(out_file, "w", encoding="utf-8") as f: #Open -> overwrite the file
            for record in fused: 
                f.write(json.dumps(record) + "\n")


        #Meta to keep track of what parameters for fusion
        # updated each cycle
        meta = {
            "script":                  SCRIPT_NAME,
            "run_id":                  run_id,
            "kraken_file":             str(kraken_file),
            "telemetry_files":         [str(tel_file)],
            "fusion_file":             str(out_file),
            "fusion_time_ms":          t_fusion_start_ms,
            "kraken_records_in":       len(kraken_records),
            "telemetry_records_in":    len(tel_records),
            "fused_records_out":       len(fused),
            "matched_kraken": n_with_kraken,
            "telemetry_only": n_tel_only,
            "usable_for_triangulation": n_usable,

            "max_time_diff_ms":        args.max_time_diff_ms,
            "min_confidence":          args.min_confidence,
            "max_roll_deg":            args.max_roll_deg,
            "min_ground_speed_ft_s":   args.min_ground_speed_ft_s,
        }
        meta_file = FUSION_DIR / f"fusion_{run_id}_meta.json"
        meta_file.write_text(json.dumps(meta, indent=2))

        # Sleep before the next fusion cycle
        time.sleep(ACTIVE_POLL_INTERVAL_S)


if __name__ == "__main__":
    main()