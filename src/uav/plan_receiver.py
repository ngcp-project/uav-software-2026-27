import logging
from pathlib import Path
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient

log = logging.getLogger(__name__)

def listen_for_messages(executor):

    processed_commands = {}

    client = LiaClient("flight_executor")
    print("[FLIGHT EXECUTOR] Subscribed to LIA")
    for message in client.subscribe():

        if message.get("type") == "shutdown":
            print("[FLIGHT EXECUTOR] LIA shutting down")
            break

        topic = message.get("topic")
        value = message.get("value")

        if topic == "telemetry.fc_mode":
            if isinstance(value, str):
                executor.update_flight_mode(value)
            continue

        if topic != "cmd.submit_plan":
            continue

        if not isinstance(value, dict):
            log.warning("Invalid command payload")
            continue

        command_id = value.get("command_id")
        plan = value.get("plan")

        if not command_id:
            log.warning("Missing command_id")
            continue

        # Check whether command was already processed.
        if command_id in processed_commands:
            ack = processed_commands[command_id]
        else:
            try:
                plan_id = executor.submit_plan(plan)
                executor.upload_pending_plan()
                ack = {"command_id": command_id, "plan_id": plan_id, "status": "accepted"}
            except (TypeError, ValueError) as exc:
                ack = {
                    "command_id": command_id,
                    "plan_id": (
                        plan.get("plan_id")
                        if isinstance(plan, dict)
                        else None
                    ),
                    "status": "rejected",
                    "reason": str(exc),
                }

            except (RuntimeError, TimeoutError) as exc:
                ack = {
                    "command_id": command_id,
                    "plan_id": (
                        plan.get("plan_id")
                        if isinstance(plan, dict)
                        else None
                    ),
                    "status": "upload_failed",
                    "reason": str(exc),
                }
            processed_commands[command_id] = ack

        client.publish("cmd.plan_ack", ack)

        print(f"[FLIGHT EXECUTOR] ACK: {ack}")
