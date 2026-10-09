import zmq

context = zmq.Context()

# Receives commands
command_socket = context.socket(zmq.PULL)
command_socket.bind("tcp://127.0.0.1:6767")

# Sends telemetry
telemetry_socket = context.socket(zmq.PUSH)
telemetry_socket.bind("tcp://127.0.0.1:3435")

print("Adapter is waiting for commands...")

try:
    while True:
        # Wait for a command
        command = command_socket.recv_json()

        print("Received command:", command)
        print("hello")

        # Eventually, this will come from vehicle data
        telemetry = {
            "type": "telemetry",
            "status": "command received",
            "battery": 87
        }

        telemetry_socket.send_json(telemetry)
        print("Telemetry sent.")

except KeyboardInterrupt:
    print("\nAdapter shutting down.")

finally:
    command_socket.close()
    telemetry_socket.close()
    context.term()