import asyncio
import time
import json
from pathlib import Path
import os

import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state, STATE_FILE #For the mission_state.json
from mavsdk import System #Needs MAVSDK to get telemetry data

#This script is for logging telemetry data from the UAV (Works in SITL as well)
SYSTEM_ADDRESS = os.getenv("MAVSDK_SYSTEM_ADDRESS", "udpin://0.0.0.0:14602")

LOG_HZ = 5.0  #logging rate (Hz)
STATE_POLL_HZ = 2.0 #How often to check state file

#Unit Conversions
M_TO_FT = 3.280839895
MS_TO_FTS = 3.280839895

START_STRUCT = time.localtime()
RUN_ID    = time.strftime("%Y%m%d_%H%M%S", START_STRUCT)

BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR  = BASE_DIR / "logs" / "telemetry"
LOG_DIR.mkdir(parents=True, exist_ok=True)

#Connection Checks
async def wait_connected(drone: System):
    print(f"Connecting via {SYSTEM_ADDRESS} ...")
    await drone.connect(system_address=SYSTEM_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to Ardupilot")
            return
        
async def wait_position_ok(drone: System):
    print("Waiting for global position + home position ...")
    print()
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK + Home OK")
            print()
            return
        
async def main():
    drone = System()
    await wait_connected(drone)

    latest = {
        "position": None,   #lat/lon/alt
        "vel_ned": None,    #north/east/down velocities
        "att_euler": None,  #roll/pitch/yaw in degrees
        "battery": None,    #remaining percent, voltage
    }

    async def pump_position():
        async for msg in drone.telemetry.position():
            latest["position"] = msg

    async def pump_velocity():
        async for msg in drone.telemetry.velocity_ned():
            latest["vel_ned"] = msg

    async def pump_attitude():
        async for msg in drone.telemetry.attitude_euler():
            latest["att_euler"] = msg   

    async def pump_battery():
        async for msg in drone.telemetry.battery():
            latest["battery"] = msg

    tasks = [
            asyncio.create_task(pump_position()),
            asyncio.create_task(pump_velocity()),
            asyncio.create_task(pump_attitude()),
            asyncio.create_task(pump_battery()),
        ]

    period_s = 1.0 / LOG_HZ
    state_period_s = 1.0 / STATE_POLL_HZ

    logging_enabled = False
    f = None
    out_file = None
    last_state_check = 0.0
    seq = 0
    

    print(f"State file: {STATE_FILE}")
    print()
    print("Telemetry logger ready (waiting for START_LOG)...")
    print()

    try:
        while True:
            loop_start = time.time()

            # Poll state file at STATE_POLL_HZ
            if loop_start - last_state_check >= state_period_s:
                last_state_check = loop_start
                state = load_state()
                should_log = state.get("logging_enabled", False) #Because we had a system where we sent a command to start logging

                if should_log and not logging_enabled: #Starts a new logging session 
                    session_ts = time.strftime("%Y%m%d_%H%M%S")
                    out_file = LOG_DIR / f"telemetry_{RUN_ID}_{session_ts}.jsonl"
                    f = open(out_file, "a", encoding="utf-8")
                    update_state("telemetry_log", str(out_file))
                    logging_enabled = True #Set to true so that logging starts
                    print(f"START_LOG: logging to {out_file}")

                elif not should_log and logging_enabled: #Stops logging
                    if f:
                        f.close()
                        f = None
                    logging_enabled = False
                    print(f"STOP_LOG: closed {out_file}")
                    out_file = None

            if logging_enabled and f is not None: #Starts logging telemetry data
                t_rx_ms = int(time.time() * 1000)

                pos = latest["position"]
                vel = latest["vel_ned"]
                att = latest["att_euler"]
                bat = latest["battery"]

                ground_speed_ft_s = None
                if vel is not None:
                    vn = vel.north_m_s
                    ve = vel.east_m_s
                    ground_speed_ft_s = round(((vn * vn + ve * ve) ** 0.5) * MS_TO_FTS, 4)

                record = {
                    "t_rx_ms": t_rx_ms,
                    "run_id": RUN_ID,
                    "seq": seq,
                    "pitch_deg": round(att.pitch_deg, 4) if att is not None else None,
                    "yaw_deg": round(att.yaw_deg, 4)   if att is not None else None,
                    "roll_deg": round(att.roll_deg, 4)  if att is not None else None,
                    
                    "lat_deg": (pos.latitude_deg if pos is not None else None),
                    "lon_deg": (pos.longitude_deg if pos is not None else None),
                    
                    "altitude_rel_ft": round(pos.relative_altitude_m * M_TO_FT, 4) if pos is not None else None,

                    "ground_speed_ft_s": ground_speed_ft_s,
                    "vel_north_m_s": round(vel.north_m_s, 4) if vel is not None else None,
                    "vel_east_m_s": round(vel.east_m_s,  4) if vel is not None else None,
                    "vel_down_m_s": round(vel.down_m_s,  4) if vel is not None else None,
 
                    "battery_remain_pct": round(bat.remaining_percent * 100.0, 4) if bat is not None and bat.remaining_percent is not None else None,
                }
                f.write(json.dumps(record) + "\n")
                f.flush()
                seq += 1

            dt = time.time() - loop_start
            await asyncio.sleep(max(0.0, period_s - dt))

    except KeyboardInterrupt:
        print("Stopping telemetry logger")
    finally:
        if f:
            f.close()
        for t in tasks:
            t.cancel()


if __name__ == "__main__":
    asyncio.run(main())