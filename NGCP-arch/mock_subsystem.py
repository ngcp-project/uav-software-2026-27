#!/usr/bin/env python3
"""
Mock subsystem — generic subscriber.

Prints every message received. Reconnects if LIA drops.
Exits cleanly on shutdown broadcast.
"""
import json
import socket
import time

HOST, PORT = "127.0.0.1", 5588


def connect_with_retry(host, port, retry_delay=2):
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=5)
            sock.settimeout(None)
            print(f"[mock_subsystem] connected to {host}:{port}")
            return sock
        except OSError as e:
            print(f"[mock_subsystem] connection failed ({e}), retrying in {retry_delay}s...")
            time.sleep(retry_delay)


def main():
    while True:
        with connect_with_retry(HOST, PORT) as sock:
            sock.sendall(
                (json.dumps({"type": "subscribe", "sender": "mock_subsystem"}) + "\n").encode()
            )
            print("[mock_subsystem] subscribed, waiting for messages...")
            try:
                buf = b""
                for chunk in iter(lambda: sock.recv(4096), b""):
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        msg = json.loads(line.decode())
                        if msg.get("type") == "shutdown":
                            print("[mock_subsystem] shutdown received — exiting")
                            return
                        print(
                            f"[mock_subsystem] received {msg.get('topic')} = {msg.get('value')!r} "
                            f"(from {msg.get('sender')}, ts={msg.get('ts')})"
                        )
            except (OSError, ConnectionResetError, BrokenPipeError):
                print("[mock_subsystem] connection lost — reconnecting...")


if __name__ == "__main__":
    main()
