#!/usr/bin/env python3

#This function listents to the PIXHAWK's flight controller heartbeat messages and updates the mission state with the current flight mode
#THis makes it so that our autonomous scripts can react to changes in the flight mode

import time
import os
from pymavlink import mavutil
from pathlib import Path
import sys

# Allow import of state utils
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state

MAVLINK_PORT = os.environ.get("MAVLINK_PORT", "udp:127.0.0.1:14608")

def main():
    print(f"[FC MODE] Connecting to MAVLink on {MAVLINK_PORT}...")
    master = mavutil.mavlink_connection(MAVLINK_PORT)

    master.wait_heartbeat()
    print("[FC MODE] Heartbeat received. Connected to Pixhawk.")

    last_mode = None

    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True)
        if msg is None:
            continue

        mode = mavutil.mode_string_v10(msg)

        if mode != last_mode:
            print(f"[FC MODE] Mode changed: {last_mode} -> {mode}")
            last_mode = mode

            # Load current state
            state = load_state()
            controller_status = state.get("controller_status", {})

            if mode == "AUTO":
                update_state("autonomy_command", True)

                update_state("controller_status", {
                    **controller_status,
                    "fc_mode": mode,
                    "autonomy_source": "fc_mode",
                    "safety_hold": None,
                    "fc_mode_last_updated": time.time(),
                    "fc_mode_source": "pixhawk_heartbeat"
                })

            elif mode in ["LOITER", "RTL", "MANUAL", "FBWA", "FBWB", "CRUISE", "STABILIZE"]:
                update_state("autonomy_command", False)

                update_state("controller_status", {
                    **controller_status,
                    "fc_mode": mode,
                    "autonomy_source": None,
                    "safety_hold": mode.lower(),
                    "fc_mode_last_updated": time.time(),
                    "fc_mode_source": "pixhawk_heartbeat"
                })


        
        


if __name__ == "__main__":
    main()
