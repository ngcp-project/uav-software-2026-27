import zmq
import time

context = zmq.Context()

# Sends commands
command_socket = context.socket(zmq.PUSH)
command_socket.connect("tcp://127.0.0.1:6767")

# Receives telemetry
telemetry_socket = context.socket(zmq.PULL)
telemetry_socket.connect("tcp://127.0.0.1:3435")

# Give ZeroMQ time to establish both connections
time.sleep(0.5)

command = {
    "type": "move",
    "speed": 20,
    "direction": "left"
}

command_socket.send_json(command)
print("Command sent.")

telemetry = telemetry_socket.recv_json()
print("Telemetry received:", telemetry)

command_socket.close()
telemetry_socket.close()
context.term()