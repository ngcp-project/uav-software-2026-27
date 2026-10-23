import logging
import sys
from pathlib import Path
import json

from pymavlink import mavutil

sys.path.append(str(Path(__file__).resolve().parents[1]))

from uav.flight_executor import FlightExecutor
from uav.plan_receiver import listen_for_messages

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

log = logging.getLogger("flight_main")


CONFIG_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "mavlink_address.json"
)

with open(CONFIG_FILE) as f:
    MAVLINK_ENDPOINTS = json.load(f)


def main():
    mavlink_port = MAVLINK_ENDPOINTS["flight_executor"]

    log.info(
        "Connecting FlightExecutor to MAVLink on %s",
        mavlink_port,
    )

    mav = mavutil.mavlink_connection(mavlink_port)

    log.info("Waiting for FC heartbeat...")
    mav.wait_heartbeat()

    log.info("Connected to flight controller")

    executor = FlightExecutor(mav)

    listen_for_messages(executor)


if __name__ == "__main__":
    main()