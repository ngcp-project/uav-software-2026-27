# Autonomy MissionPlan Interface

## Purpose

Autonomy scripts should output a Python dictionary that follows the `MissionPlan` format below.

Plans are submitted through LIA using:

```python
from lia.lia_client import LiaClient
```

---

## MissionPlan Format

### Waypoint Pattern

Use `waypoint_pattern` for a route containing multiple waypoints.

```python
plan = {
    "plan_id": "search_001",
    "plan_type": "waypoint_pattern",
    "waypoints": [
        {
            "lat": 34.0432,
            "lon": -117.8144,
            "alt_ft": 200.0,
        },
        {
            "lat": 34.0435,
            "lon": -117.8148,
            "alt_ft": 200.0,
        },
        {
            "lat": 34.0431,
            "lon": -117.8152,
            "alt_ft": 200.0,
        },
    ],
}
```

Required fields:

```text
plan_id
plan_type
waypoints
```

Each waypoint should contain:

```text
lat
lon
alt_ft
```

---

### Single Loiter

Use `single_loiter` when the aircraft should fly to one location and loiter there.

```python
plan = {
    "plan_id": "loiter_001",
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
```

A `single_loiter` plan should contain one waypoint.

---

## Publishing a Plan Through LIA

```python
import sys
import uuid
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient


def publish_plan(plan: dict):
    lia = LiaClient("autonomy")

    command_id = str(uuid.uuid4())

    lia.publish(
        "cmd.submit_plan",
        {
            "command_id": command_id,
            "plan": plan,
        },
    )

    print(
        f"[AUTONOMY] Submitted "
        f"{plan['plan_id']} "
        f"command_id={command_id}"
    )

    return command_id
```

Example:

```python
plan = {
    "plan_id": "search_001",
    "plan_type": "waypoint_pattern",
    "waypoints": [
        {
            "lat": 34.0432,
            "lon": -117.8144,
            "alt_ft": 200.0,
        },
        {
            "lat": 34.0435,
            "lon": -117.8148,
            "alt_ft": 200.0,
        },
    ],
}

publish_plan(plan)
```

---

## Generating Plans From Autonomy Code

Autonomy functions can calculate waypoints and return a `MissionPlan` dictionary directly.

```python
def generate_search_plan(search_area):
    waypoints = calculate_search_waypoints(search_area)

    return {
        "plan_id": "search_001",
        "plan_type": "waypoint_pattern",
        "waypoints": waypoints,
    }


plan = generate_search_plan(search_area)

publish_plan(plan)
```

The output of the autonomy algorithm should be a Python dictionary matching the `MissionPlan` format.

---

## Combined Example: Generate and Publish a Plan

Use this as the main template for autonomy scripts.

The autonomy code should generate and return a waypoint list. The rest of the LIA publication code can remain unchanged.

```python
import sys
import uuid
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient


def generate_waypoints():
    # ==========================================================
    # REPLACE THIS FUNCTION WITH YOUR AUTONOMY / WAYPOINT LOGIC
    # ==========================================================

    # Your autonomy logic should calculate and return:
    #
    # [
    #     {
    #         "lat": ...,
    #         "lon": ...,
    #         "alt_ft": ...,
    #     },
    #     ...
    # ]

    return calculate_search_waypoints()


def generate_plan():
    waypoints = generate_waypoints()

    return {
        "plan_id": "search_001",
        "plan_type": "waypoint_pattern",
        "waypoints": waypoints,
    }


def publish_plan(plan: dict):
    lia = LiaClient("autonomy")

    command_id = str(uuid.uuid4())

    lia.publish(
        "cmd.submit_plan",
        {
            "command_id": command_id,
            "plan": plan,
        },
    )

    print(
        f"[AUTONOMY] Submitted "
        f"{plan['plan_id']} "
        f"command_id={command_id}"
    )

    return command_id


if __name__ == "__main__":
    plan = generate_plan()
    publish_plan(plan)
```

The autonomy portion is:

```python
def generate_waypoints():
    return calculate_search_waypoints()
```

Whatever waypoint-generation function is used must return a list in this format:

```python
[
    {
        "lat": 34.0432,
        "lon": -117.8144,
        "alt_ft": 200.0,
    },
    {
        "lat": 34.0435,
        "lon": -117.8148,
        "alt_ft": 200.0,
    },
]
```

---
Workflow:

```text
Search / Navigation Algorithm
          ↓
Generate Waypoints
          ↓
Build MissionPlan
          ↓
publish_plan(plan)
          ↓
LIA
```