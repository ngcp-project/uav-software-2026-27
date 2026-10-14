import time
import random
import json
from pathlib import Path #Needs to import path

import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import update_state #Makes it so this output updates mission-state.json

#This is an older version of the Kraken data simulation script and it just gives random values

BASE_DIR   = Path(__file__).resolve().parents[2]
LOG_DIR    = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

RUN_ID   = time.strftime("%Y%m%d_%H%M%S")
filepath = LOG_DIR / f"doa_{RUN_ID}.jsonl"

update_state("kraken_log", str(filepath))
RATE_HZ = 10
INTERVAL = 1.0 / RATE_HZ


seq = 0

with open(filepath, "a") as f:
    while True:
        epoch = float(int(time.time() * 1000))
        record = {
            "t_rx_ms":        int(time.time() * 1000),
            "run_id":         "SIM_TEST",
            "seq":            seq,
            "kraken_counter": epoch,
            "doa_deg":        float(random.randint(0, 359)),  # Random DOA angle in degrees
            "confidence_0_1": random.uniform(0, 1),         # Confidence score in the 0.0–1.0 range
            "lat_deg":        34.0 + random.uniform(-0.01, 0.01),  # Small random offset around a nominal latitude
            "lon_deg":        -118.0 + random.uniform(-0.01, 0.01),  # Small random offset around a nominal longitude
            "gps_heading_deg": float(random.randint(0, 359)),  # Simulated GPS heading in degrees
        }
        seq += 1


        f.write(json.dumps(record) + "\n")
        f.flush()

        time.sleep(INTERVAL)