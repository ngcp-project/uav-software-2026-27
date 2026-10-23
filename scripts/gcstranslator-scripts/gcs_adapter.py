from Packet.Command import *
from Packet.Enum import *
from Packet.Infrastructure.InfrastructureInterface import *
from PacketLibrary.PacketLibrary import PacketLibrary
from Telemetry.Telemetry import Telemetry
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
            Command = command_socket.recv_pyobj()
            print("Received command:", Command)

            try:
                if (isinstance(Command, str)):
                    CommandData = json.loads(Command)

                match (Command):
                    case Heartbeat():
                        print("Executing Heartbeat code")
                    case EmergencyStop():
                        print("Executing Emergency Stop code")
                    case AddZone():
                        print("Executing Add Zone code")
                    case PatientLocation():
                        print("Executing Patient Location code")
                    case _:
                        print("Unknown Command Type")

            except json.JSONDecodeError as e:
                print(f"JSON Error: {e}")
                        

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