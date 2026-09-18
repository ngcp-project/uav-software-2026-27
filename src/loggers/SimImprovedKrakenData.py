#!/usr/bin/env python3
import time
import json
import math
import random
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import update_state, load_state

#This script simulates Kraken DOA data for testing purposes, writing it to a JSON Lines file. 
# I lowk had claude write a lot of this but it's meant to simulate realistic Kraken DOA data for testing purposes
# We didn't really need it last year because simulations we just used coordinates that were "calculated" but for triangulation simulations using/updating this file may be a good starting point


BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

RUN_ID = time.strftime("%Y%m%d_%H%M%S")
OUT_FILE = LOG_DIR / f"doa_{RUN_ID}.jsonl"

OUT_FILE.touch(exist_ok=True)

update_state("kraken_log", str(OUT_FILE))

# SITL home area
HOME_LAT = -35.35251
HOME_LON = 149.15863

# Simulated RF emitter location near SITL home
# Change these if you want the target somewhere else
EMITTER_LAT = -35.35180 
EMITTER_LON = 149.16020

# Confidence / noise behavior
BASE_NOISE_STD_DEG = 4.0
LOW_CONF_NOISE_STD_DEG = 15.0
DROPOUT_PROB = 0.08          # 8% chance of a bad read
MEDIUM_CONF_PROB = 0.20      # 20% chance of medium-quality read

POLL_INTERVAL = 0.05         # how often to check telemetry file for new lines


def wrap_angle_360(angle_deg: float) -> float:
    return angle_deg % 360.0

def wrap_angle_180(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0

def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)

    x = math.sin(dlon_rad) * math.cos(lat2_rad)
    y = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    )

    brng = math.degrees(math.atan2(x, y))
    return wrap_angle_360(brng)

def distance_m_approx(lat1, lon1, lat2, lon2) -> float:
    mean_lat_rad = math.radians((lat1 + lat2) / 2.0)
    dlat_m = (lat2 - lat1) * 111320.0
    dlon_m = (lon2 - lon1) * 111320.0 * math.cos(mean_lat_rad)
    return math.sqrt(dlat_m**2 + dlon_m**2)

def compute_confidence(angle_error_deg: float, distance_m: float, moving: bool) -> float:
    """
    Build a somewhat realistic confidence model:
    - confidence drops if angular error is large
    - confidence drops a bit if aircraft is barely moving
    - confidence drops a bit with distance
    """
    # angular error contribution
    angle_term = max(0.0, 1.0 - (abs(angle_error_deg) / 45.0))

    # distance contribution
    # strong nearby, lower farther away
    distance_term = max(0.0, 1.0 - (distance_m / 2000.0))

    # motion contribution
    motion_term = 1.0 if moving else 0.55

    conf = 0.15 + 0.55 * angle_term + 0.20 * distance_term + 0.10 * motion_term
    return max(0.0, min(1.0, conf))

def get_telemetry_log_path() -> Path | None:
    state = load_state()
    path_str = state.get("telemetry_log")
    if not path_str:
        return None
    return Path(path_str)

def parse_json_line(line: str):
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None

#Main Loop
def main():
    print(f"Writing simulated Kraken log to: {OUT_FILE}")
    print("Waiting for telemetry log path in mission_state.json ...")

    telemetry_file = None
    telem_fp = None

    with open(OUT_FILE, "a") as out_f:
        seq = 0

        while True:
            # Get telemetry log path from state if we do not have one yet
            if telemetry_file is None:
                telemetry_file = get_telemetry_log_path()
                if telemetry_file is None or not telemetry_file.exists():
                    time.sleep(POLL_INTERVAL)
                    continue

                print(f"Following telemetry file: {telemetry_file}")
                telem_fp = open(telemetry_file, "r")
                telem_fp.seek(0, 2)  # jump to end so we only process new samples

            line = telem_fp.readline()

            if not line:
                time.sleep(POLL_INTERVAL)
                continue

            telem = parse_json_line(line)
            if telem is None:
                continue

            # Pull aircraft position from real telemetry
            aircraft_lat = telem.get("lat_deg", HOME_LAT)
            aircraft_lon = telem.get("lon_deg", HOME_LON)
            ground_speed_ft_s = telem.get("ground_speed_ft_s", 0.0)

            # IMPORTANT:
            # reuse the exact telemetry timestamp for fusion alignment
            t_rx_ms = telem.get("t_rx_ms", int(time.time() * 1000))

            # True bearing from aircraft to emitter
            true_doa = bearing_deg(
                aircraft_lat,
                aircraft_lon,
                EMITTER_LAT,
                EMITTER_LON
            )

            # Simulate read quality
            r = random.random()
            if r < DROPOUT_PROB:
                # bad read / fluke
                noisy_doa = wrap_angle_360(true_doa + random.gauss(0.0, 35.0))
                confidence = random.uniform(0.02, 0.20)
            elif r < DROPOUT_PROB + MEDIUM_CONF_PROB:
                # medium read
                noise = random.gauss(0.0, LOW_CONF_NOISE_STD_DEG)
                noisy_doa = wrap_angle_360(true_doa + noise)
                distance_m = distance_m_approx(
                    aircraft_lat, aircraft_lon, EMITTER_LAT, EMITTER_LON
                )
                confidence = compute_confidence(
                    angle_error_deg=wrap_angle_180(noisy_doa - true_doa),
                    distance_m=distance_m,
                    moving=ground_speed_ft_s > 3.0
                )
                confidence *= random.uniform(0.55, 0.80)
            else:
                # good read
                noise = random.gauss(0.0, BASE_NOISE_STD_DEG)
                noisy_doa = wrap_angle_360(true_doa + noise)
                distance_m = distance_m_approx(
                    aircraft_lat, aircraft_lon, EMITTER_LAT, EMITTER_LON
                )
                confidence = compute_confidence(
                    angle_error_deg=wrap_angle_180(noisy_doa - true_doa),
                    distance_m=distance_m,
                    moving=ground_speed_ft_s > 3.0
                )
                confidence *= random.uniform(0.90, 1.00)

            record = {
                "t_rx_ms": t_rx_ms,                  # EXACT match to telemetry sample
                "run_id": RUN_ID,
                "seq": seq,
                "kraken_counter": t_rx_ms,          # can be same if you want
                "doa_deg": round(noisy_doa, 4),
                "confidence_0_1": round(max(0.0, min(1.0, confidence)), 4),

                # optional metadata
                "lat_deg": round(aircraft_lat, 7),
                "lon_deg": round(aircraft_lon, 7),
                "gps_heading_deg": round(telem.get("yaw_deg", 0.0), 4),

                # debug / truth fields for validation
                "true_doa_deg": round(true_doa, 4),
                "emitter_lat_deg": EMITTER_LAT,
                "emitter_lon_deg": EMITTER_LON,
            }

            out_f.write(json.dumps(record) + "\n")
            out_f.flush()

            seq += 1

if __name__ == "__main__":
    main()