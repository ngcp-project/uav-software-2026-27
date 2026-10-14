import time
import random
import json
from pathlib import Path #Needs to import path

import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import update_state #Makes it so this output updates mission-state.json

#THis file is basically useless now, it just generates random telemetry data for testing, its basically meant for pipeline testing
#THe reason it's useless now is because we can just use the regular telemetry_logger.py and it will log the "real telemetry" for simulations too.

BASE_DIR   = Path(__file__).resolve().parents[2]
LOG_DIR    = BASE_DIR / "logs" / "telemetry"
LOG_DIR.mkdir(parents=True, exist_ok=True)

RUN_ID   = time.strftime("%Y%m%d_%H%M%S")
filepath = LOG_DIR / f"telemetry_{RUN_ID}.jsonl"

update_state("telemetry_log", str(filepath))
RATE_HZ = 10
INTERVAL = 1.0 / RATE_HZ


seq = 0

with open(filepath, "a") as f:
    while True:
        
        record = {
            "t_rx_ms":          int(time.time() * 1000),
            "run_id":           "SIM_TEST",
            "seq":              seq,
            "pitch_deg":        round(random.uniform(-10, 10), 4),
            "yaw_deg":          round(random.uniform(0, 359), 4),
            "roll_deg":         round(random.uniform(-15, 15), 4),
            "lat_deg":          34.0 + random.uniform(-0.01, 0.01),
            "lon_deg":          -118.0 + random.uniform(-0.01, 0.01),
            "altitude_rel_ft":  round(random.uniform(20, 100), 4),
            "ground_speed_ft_s": round(random.uniform(5, 60), 4),
            "vel_north_m_s":    round(random.uniform(-10, 10), 4),
            "vel_east_m_s":     round(random.uniform(-10, 10), 4),
            "vel_down_m_s":     round(random.uniform(-1, 1), 4),
            "battery_remain_pct": round(random.uniform(50, 100), 4),
        }
        seq += 1


        f.write(json.dumps(record) + "\n")
        f.flush()

        time.sleep(INTERVAL)