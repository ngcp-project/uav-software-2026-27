#!/usr/bin/env python3
"""
Mock main controller (stands in for ATM).

Two connections to LIA:
  - subscribe socket : receives all broadcasts; prints lcl.* messages
  - publish socket   : sends state updates and a heartbeat every 3 s when idle

Reconnects automatically if LIA drops. Exits cleanly on LIA shutdown broadcast.
"""
import json
import socket
import threading
import time

HOST, PORT    = "127.0.0.1", 5588
OWNER         = "atm"
HEARTBEAT_S   = 3.0

_shutdown = threading.Event()   # set when LIA sends {"type":"shutdown"}


def connect_with_retry(host, port, retry_delay=2):
    while not _shutdown.is_set():
        try:
            sock = socket.create_connection((host, port), timeout=5)
            sock.settimeout(None)
            print(f"[mock_main_controller] connected to {host}:{port}")
            return sock
        except OSError as e:
            print(f"[mock_main_controller] connection failed ({e}), retrying in {retry_delay}s...")
            time.sleep(retry_delay)
    return None


def publish(sock, topic, value):
    ts  = time.time()
    msg = {"type": "publish", "topic": topic, "value": value,
           "sender": "mock_main_controller", "ts": ts}
    sock.sendall((json.dumps(msg) + "\n").encode())
    print(f"[mock_main_controller] published {topic} = {value!r}")


def subscribe_loop(host, port):
    """Subscribe connection — prints lcl.* updates, handles shutdown."""
    while not _shutdown.is_set():
        sock = connect_with_retry(host, port)
        if sock is None:
            break
        try:
            sock.sendall(
                (json.dumps({"type": "subscribe", "sender": "mock_main_controller_sub"}) + "\n").encode()
            )
            print("[mock_main_controller] subscribe connection open — watching for lcl data...")
            buf = b""
            for chunk in iter(lambda: sock.recv(4096), b""):
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    msg = json.loads(line.decode())
                    if msg.get("type") == "shutdown":
                        print("[mock_main_controller] shutdown received — exiting")
                        _shutdown.set()
                        return
                    if msg.get("topic", "").startswith("lcl."):
                        print(
                            f"[mock_main_controller] lcl update: "
                            f"{msg['topic']} = {msg['value']!r}  (ts={msg.get('ts')})"
                        )
        except (OSError, ConnectionResetError, BrokenPipeError):
            if not _shutdown.is_set():
                print("[mock_main_controller] subscribe connection lost — reconnecting...")
        finally:
            try:
                sock.close()
            except OSError:
                pass


def publish_loop(host, port):
    """Publish connection — sends state, then heartbeats until shutdown."""
    # One-time state publishes
    state_publishes = [
        ("meta.mission_phase",   "searching"),
        ("atm.active_task",      "search_pattern"),
        ("atm.target_waypoint",  {"lat": 34.0432, "lon": -117.8144, "alt_ft": 200.0}),
        ("cmd.send_target_location", {"lat": 34.0432, "lon": -117.8144}),
    ]
    published = False

    while not _shutdown.is_set():
        sock = connect_with_retry(host, port)
        if sock is None:
            break
        try:
            if not published:
                for topic, value in state_publishes:
                    if _shutdown.is_set():
                        break
                    publish(sock, topic, value)
                    time.sleep(3)
                published = True
                print("[mock_main_controller] done publishing state — sending heartbeats")

            # heartbeat loop
            while not _shutdown.is_set():
                publish(sock, f"{OWNER}.heartbeat", time.time())
                time.sleep(HEARTBEAT_S)

        except (OSError, BrokenPipeError, ConnectionResetError):
            if not _shutdown.is_set():
                print("[mock_main_controller] publish connection lost — reconnecting...")
        finally:
            try:
                sock.close()
            except OSError:
                pass


def main():
    sub_thread = threading.Thread(target=subscribe_loop, args=(HOST, PORT), daemon=True)
    sub_thread.start()
    time.sleep(0.3)

    pub_thread = threading.Thread(target=publish_loop, args=(HOST, PORT), daemon=True)
    pub_thread.start()

    # Block until shutdown
    _shutdown.wait()
    print("[mock_main_controller] stopped")


if __name__ == "__main__":
    main()
