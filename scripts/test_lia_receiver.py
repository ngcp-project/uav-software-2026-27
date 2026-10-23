import sys
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src")
)

from uav.flight_executor import FlightExecutor
from uav.plan_receiver import listen_for_plans


def main():

    executor = FlightExecutor(mav=None)

    print("Waiting for MissionPlans through LIA...")

    listen_for_plans(executor)


if __name__ == "__main__":
    main()