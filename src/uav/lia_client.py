import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient


def publish_plan(plan: dict):
    import uuid
    command_id = str(uuid.uuid4())
    LiaClient("autonomy").publish("cmd.submit_plan", {
        "command_id": command_id,
        "plan": plan,
    })

    print(f"[AUTONOMY] Submitted plan: {plan['plan_id']}")

    return command_id


def publish_message(topic: str, value, sender: str):
    LiaClient(sender).publish(topic, value)
