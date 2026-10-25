"""
adapter.py -- the streaming adapter (stands in for the UAV side). TEST VERSION.

gcs_xbee.py is only a relay, so THIS script is where translating happens. In the finished adapter:
    * commands arrive here in the library's own format ({"Command ID": 3, "Packet ID": 1, ...}) and
      this script decides what each one means and what to do with it;
    * telemetry leaves here already in the fields and units the GCS expects, using the library's names.

This test version does the bare minimum of each:
    * prints every command it receives, and remembers its Command ID / Packet ID as the receipt;
    * sends telemetry 5 times a second with battery 87 and the last receipt (everything else 0).

Ports can be changed with the GCS_CMD_PORT / GCS_TLM_PORT environment variables.
"""
import os
import time
import zmq

CMD_PORT = int(os.environ.get("GCS_CMD_PORT", 6767))
TLM_PORT = int(os.environ.get("GCS_TLM_PORT", 3435))

context = zmq.Context()

# Receives commands
command_socket = context.socket(zmq.PULL)
command_socket.bind(f"tcp://127.0.0.1:{CMD_PORT}")

# Sends telemetry
telemetry_socket = context.socket(zmq.PUSH)
telemetry_socket.bind(f"tcp://127.0.0.1:{TLM_PORT}")

print("Adapter is waiting for commands...", flush=True)

last_sent = 0
receipt = (0, 0)        # (Command ID, Packet ID) of the last command handled

try:
    while True:
        # Wait up to 50 ms for a command, then move on
        if command_socket.poll(50):
            command = command_socket.recv_json()
            print("Received command:", command, flush=True)
            # Eventually: translate the command and act on it. For now, just note its ids.
            receipt = (command["Command ID"], command["Packet ID"])

        # Send telemetry 5 times a second, whether or not a command arrived
        if time.time() - last_sent >= 0.2:
            # Eventually, this will be built from the vehicle's state files.
            telemetry = {
                "CommandID": receipt[0],
                "PacketID": receipt[1],
                "Speed": 0.0,
                "Pitch": 0.0,
                "Yaw": 0.0,
                "Roll": 0.0,
                "Altitude": 0.0,
                "BatteryLife": 87.0,
                "LastUpdated": 0,
                "CurrentPosition": [0.0, 0.0],
                "VehicleStatus": 0,
                "MessageFlag": 0,
                "MessageLat": 0.0,
                "MessageLon": 0.0,
                "PatientStatus": 0,
            }
            try:
                telemetry_socket.send_json(telemetry, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass  # nobody is listening yet, skip this one instead of freezing
            last_sent = time.time()

except KeyboardInterrupt:
    print("\nAdapter shutting down.")

finally:
    command_socket.close(linger=0)
    telemetry_socket.close(linger=0)
    context.term()
