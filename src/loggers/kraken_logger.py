#!/usr/bin/env python3
import time
import json
import requests
from pathlib import Path

import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import update_state #For the mission_state.json

# This script logs data from the Kraken DOA endpoint to a JSON Lines file, with a unique run ID for each session.

# Kraken DOA endpoint (Hosted on Pi)
DOA_URL = "http://127.0.0.1:8081/DOA_value.html" #This is where the KrakenSDR sends data so it's where this script will take the information from

# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate

BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

START_STRUCT = time.localtime()
RUN_ID   = time.strftime("%Y%m%d_%H%M%S", START_STRUCT)

OUT_FILE = LOG_DIR / f"doa_{RUN_ID}.jsonl" # Output JSON log file (JSON Lines format)

seq: int   = 0
last_kraken_counter: float = -1 

# Logs a single data point from the Kraken DOA endpoint to the JSON Lines file
def log_once(f):
    global seq, last_kraken_counter
    try:
        t_rx_ms = int(time.time() * 1000) #Pi receipt time, needed for fusion with telemetry data

        try:
            r = requests.get(DOA_URL, timeout=1)
        except requests.exceptions.RequestException:
            return  #Endpoint not ready or no signal yet

        line = r.text.strip()

        if not line:
            return  # empty response
        fields = line.split(',')
        
        kraken_counter = float(fields[0])

        # Forces unfinished values to be null
        def safe_float(val):
            try:
                return float(val.strip())
            except:
                return None


        if kraken_counter == last_kraken_counter:
            return
        
        # Build JSON object with selected fields
        entry = {
            "t_rx_ms": t_rx_ms,
            "run_id": RUN_ID,
            "seq": seq,
            "kraken_counter": kraken_counter,

            "doa_deg": safe_float(fields[1]),               # unit-circle DoA (0°=East CCW)
            "confidence_0_1": safe_float(fields[2]),

            "lat_deg": safe_float(fields[9]),
            "lon_deg": safe_float(fields[10]),
            "gps_heading_deg": safe_float(fields[11]),
        }

        line_out = json.dumps(entry)

        f.write(line_out + "\n")
        f.flush()

        print(line_out)

        last_kraken_counter = kraken_counter
        seq += 1

    except Exception as e:
        print("Error:", e)
        print("LINE:", repr(line))
    

def main():
    print(f"Logging in {OUT_FILE}  (run_id={RUN_ID})")

    with open(OUT_FILE, "a", encoding="utf-8") as f:
        update_state("kraken_log", str(OUT_FILE))
        while True:
            loop_start = time.time()
            log_once(f)
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, UPDATE_RATE - elapsed))
    



if __name__ == "__main__":
    main()
