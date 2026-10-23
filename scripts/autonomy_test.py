import sys
import uuid
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))

from lia.lia_client import LiaClient


def main():
    lia = LiaClient("test_autonomy")

    command_id = str(uuid.uuid4())

    plan = {
        "plan_id": "test_loiter_001",
        "plan_type": "single_loiter",
        "waypoints": [
            {
                "lat": 34.0432,
                "lon": -117.8144,
                "alt_ft": 200.0,
            }
        ],
        "loiter_radius_ft": 165.0,
    }

    lia.publish(
        "cmd.submit_plan",
        {
            "command_id": command_id,
            "plan": plan,
        },
    )

    print(
        f"[TEST AUTONOMY] Submitted "
        f"{plan['plan_id']} "
        f"command_id={command_id}"
    )


if __name__ == "__main__":
    main()