#!/usr/bin/env python3

import json
import threading
import time
import zmq

# Existing ports
ADAPTER_COMMAND_PORT = 6767
ADAPTER_TELEMETRY_PORT = 3435

# Temporary local ports for simulating the XBee/GCS connection
GCS_INPUT_PORT = 6768
GCS_OUTPUT_PORT = 6769

context = zmq.Context()
stop_event = threading.Event()


def command_thread():
    """
    Receives commands from GCS/XBee and forwards them to the adapter.
    """

    # Receives commands from simulated GCS
    gcs_socket = context.socket(zmq.PULL)
    gcs_socket.bind(f"tcp://127.0.0.1:{GCS_INPUT_PORT}")

    # Sends commands to the adapter
    adapter_socket = context.socket(zmq.PUSH)
    adapter_socket.connect(
        f"tcp://127.0.0.1:{ADAPTER_COMMAND_PORT}"
    )

    poller = zmq.Poller()
    poller.register(gcs_socket, zmq.POLLIN)

    print(f"[Gateway] Listening for GCS commands on port {GCS_INPUT_PORT}")

    try:
        while not stop_event.is_set():
            events = dict(poller.poll(500))

            if gcs_socket in events:
                command = gcs_socket.recv_json()

                print("[Gateway] Received command from GCS:")
                print(command)

                # Forward the command unchanged to the adapter
                adapter_socket.send_json(command)

                print("[Gateway] Forwarded command to adapter")

    finally:
        gcs_socket.close(linger=0)
        adapter_socket.close(linger=0)


def telemetry_thread():
    """
    Receives telemetry from the adapter and sends it to GCS/XBee.
    """

    # Receives telemetry from adapter
    adapter_socket = context.socket(zmq.PULL)
    adapter_socket.connect(
        f"tcp://127.0.0.1:{ADAPTER_TELEMETRY_PORT}"
    )

    # Sends telemetry to simulated GCS
    gcs_socket = context.socket(zmq.PUSH)
    gcs_socket.bind(f"tcp://127.0.0.1:{GCS_OUTPUT_PORT}")

    poller = zmq.Poller()
    poller.register(adapter_socket, zmq.POLLIN)

    print(
        f"[Gateway] Listening for adapter telemetry on "
        f"port {ADAPTER_TELEMETRY_PORT}"
    )

    try:
        while not stop_event.is_set():
            events = dict(poller.poll(500))

            if adapter_socket in events:
                telemetry = adapter_socket.recv_json()

                print("[Gateway] Received telemetry from adapter:")
                print(telemetry)

                # Temporary simulated XBee send
                # Later, replace this with:
                # xbee.send(telemetry)
                gcs_socket.send_json(telemetry)

                print("[Gateway] Sent telemetry to GCS/XBee")

    finally:
        adapter_socket.close(linger=0)
        gcs_socket.close(linger=0)


def main():
    command_worker = threading.Thread(
        target=command_thread,
        daemon=True
    )

    telemetry_worker = threading.Thread(
        target=telemetry_thread,
        daemon=True
    )

    command_worker.start()
    telemetry_worker.start()

    print("[Gateway] Running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[Gateway] Shutting down...")
        stop_event.set()

        command_worker.join()
        telemetry_worker.join()

        context.term()
        print("[Gateway] Stopped.")


if __name__ == "__main__":
    main()