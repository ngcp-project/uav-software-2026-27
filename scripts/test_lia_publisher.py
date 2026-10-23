import sys
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src")
)

from uav.lia_client import publish_plan

plan = {
    "plan_id": "lawnmower_001",
    "plan_type": "waypoint_pattern",
    "alt_ft": 150,
    "waypoints": [
        {"lat": 34.1000, "lon": -117.7000},
        {"lat": 34.1010, "lon": -117.7000},
        {"lat": 34.1010, "lon": -117.7010},
    ],
}

publish_plan(plan)
