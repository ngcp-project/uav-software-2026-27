import time
import zmq

context = zmq.Context()

# Receives commands
command_socket = context.socket(zmq.PULL)
command_socket.bind("tcp://127.0.0.1:6767")

# Sends telemetry
telemetry_socket = context.socket(zmq.PUSH)
telemetry_socket.bind("tcp://127.0.0.1:3435")

print("Adapter is waiting for commands...")

last_sent = 0

try:
    while True:
        # Wait up to 50 ms for a command, then move on
        if command_socket.poll(50):
            command = command_socket.recv_json()
            print("Received command:", command)

        # Send telemetry 5 times a second, whether or not a command arrived
        if time.time() - last_sent >= 0.2:
            # Eventually, this will come from vehicle data
            telemetry = {
                "type": "telemetry",
                "status": "streaming",
                "battery": 87
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