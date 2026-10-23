
import sys
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src")
)

from uav.flight_executor import (
    FlightExecutor,
    ExecutorStatus,
    build_waypoint_items,
)


def test_executor():

    executor = FlightExecutor(mav=None)

    print("Initial state:")
    print(executor.get_status())

    executor.update_flight_mode("FBWA")
    assert executor.autonomy_enabled is False

    executor.update_flight_mode("AUTO")
    assert executor.autonomy_enabled is True

    executor.update_flight_mode("RTL")
    assert executor.autonomy_enabled is False

    print("Mode checks passed")


def test_waypoint_builder():

    plan = {
        "plan_id": "test_001",
        "plan_type": "waypoint_pattern",
        "alt_ft": 150,
        "waypoints": [
            {"lat": 34.1000, "lon": -117.7000},
            {"lat": 34.1010, "lon": -117.7000},
            {"lat": 34.1010, "lon": -117.7010}
        ]
    }

    items = build_waypoint_items(plan)

    assert len(items) == 3

    print("\nGenerated MAVLink mission items:")

    for index, item in enumerate(items):
        print(
            f"Waypoint {index}: "
            f"lat={item['x'] / 1e7}, "
            f"lon={item['y'] / 1e7}, "
            f"alt={item['z']:.2f} m"
        )

    print("Waypoint checks passed")


if __name__ == "__main__":
    test_executor()
    test_waypoint_builder()
    print("\nAll tests passed!")
