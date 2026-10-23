#!/usr/bin/env python3

import time
import zmq

GCS_INPUT_PORT = 6768
GCS_OUTPUT_PORT = 6769

context = zmq.Context()

# Sends commands to gateway
command_socket = context.socket(zmq.PUSH)
command_socket.connect(
    f"tcp://127.0.0.1:{GCS_INPUT_PORT}"
)

# Receives telemetry from gateway
telemetry_socket = context.socket(zmq.PULL)
telemetry_socket.connect(
    f"tcp://127.0.0.1:{GCS_OUTPUT_PORT}"
)

time.sleep(0.5)

command = {
    "type": "mavlink_command",
    "msg_type": "COMMAND_LONG",
    "command": "START_SEARCH",
    "target_system": 1,
    "target_component": 1,
    "parameters": {
        "search_area": 3
    }
}

command_socket.send_json(command)
print("[GCS] Command sent:")
print(command)

try:
    while True:
        telemetry = telemetry_socket.recv_json()

        print("[GCS] Telemetry received:")
        print(telemetry)

except KeyboardInterrupt:
    print("\n[GCS] Stopped.")

finally:
    command_socket.close()
    telemetry_socket.close()
    context.term()