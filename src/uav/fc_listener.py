import time
import json
from pymavlink import mavutil
from pathlib import Path
import sys

# Allow import of state utils
sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.telemetry_state_utils import update_section

CONFIG_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "mavlink_address.json"
)

with open(CONFIG_FILE) as f:
    MAVLINK_ENDPOINTS = json.load(f)

MAVLINK_PORT = MAVLINK_ENDPOINTS["fc_listener"]

def main():
    print(f"[FC Listener] Connecting to MAVLink on {MAVLINK_PORT}...")
    master = mavutil.mavlink_connection(MAVLINK_PORT)

    master.wait_heartbeat()
    print("[FC Listener] Heartbeat received. Connected to FC.")

    while True:
        msg = master.recv_match(blocking=True)
        if msg is None:
            continue

        msg_type = msg.get_type()

        # HEARTBEAT
        if msg_type == "HEARTBEAT":

            mode = mavutil.mode_string_v10(msg)

            armed = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            update_section("flight_controller", {
                "connected": True,
                "mode": mode,
                "armed": armed,
                "last_heartbeat": time.time()
            })

        # POSITION / ALTITUDE
        elif msg_type == "GLOBAL_POSITION_INT":

            update_section("position", {
                "latitude": msg.lat / 1e7,
                "longitude": msg.lon / 1e7,
                "absolute_altitude_m": msg.alt / 1000.0,
                "relative_altitude_m": msg.relative_alt / 1000.0,
                "last_updated": time.time()
            })

        # BATTERY
        elif msg_type == "SYS_STATUS":

            voltage = (
                msg.voltage_battery / 1000.0
                if msg.voltage_battery != 65535
                else None
            )

            remaining = (
                msg.battery_remaining
                if msg.battery_remaining != -1
                else None
            )

            update_section("battery", {
                "voltage_v": voltage,
                "remaining_percent": remaining,
                "last_updated": time.time()
            })



if __name__ == "__main__":
    main()