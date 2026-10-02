#!/usr/bin/env python3
"""
Mock localization (stands in for LCL).

Publishes position at 10 Hz (faster than the 0.5s persist throttle in LIA).
Sends a heartbeat every 3 s when between position bursts.
Reconnects automatically if LIA drops. Exits on shutdown broadcast.
"""
import json
import socket
import threading
import time

HOST, PORT  = "127.0.0.1", 5588
OWNER       = "lcl"
HEARTBEAT_S = 3.0

STEPS = [(34.0432 + i * 0.0001, -117.8144 - i * 0.0001) for i in range(100)]

_shutdown = threading.Event()


def connect_with_retry(host, port, retry_delay=2):
    while not _shutdown.is_set():
        try:
            sock = socket.create_connection((host, port), timeout=5)
            sock.settimeout(None)
            print(f"[mock_lcl] connected to {host}:{port}")
            return sock
        except OSError as e:
            print(f"[mock_lcl] connection failed ({e}), retrying in {retry_delay}s...")
            time.sleep(retry_delay)
    return None


def publish(sock, topic, value):
    ts  = time.time()
    msg = {"type": "publish", "topic": topic, "value": value,
           "sender": "mock_lcl", "ts": ts}
    sock.sendall((json.dumps(msg) + "\n").encode())


def subscribe_loop(host, port):
    """Watch for shutdown broadcast."""
    while not _shutdown.is_set():
        sock = connect_with_retry(host, port)
        if sock is None:
            break
        try:
            sock.sendall(
                (json.dumps({"type": "subscribe", "sender": "mock_lcl_sub"}) + "\n").encode()
            )
            buf = b""
            for chunk in iter(lambda: sock.recv(4096), b""):
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    msg = json.loads(line.decode())
                    if msg.get("type") == "shutdown":
                        print("[mock_lcl] shutdown received — exiting")
                        _shutdown.set()
                        return
        except (OSError, ConnectionResetError, BrokenPipeError):
            if not _shutdown.is_set():
                print("[mock_lcl] subscribe connection lost — reconnecting...")
        finally:
            try:
                sock.close()
            except OSError:
                pass


def publish_loop(host, port):
    step_idx      = 0
    last_heartbeat = 0.0

    while not _shutdown.is_set():
        sock = connect_with_retry(host, port)
        if sock is None:
            break
        try:
            while not _shutdown.is_set():
                now = time.time()

                if step_idx < len(STEPS):
                    lat, lon = STEPS[step_idx]
                    publish(sock, "lcl.position",
                            {"lat": round(lat, 6), "lon": round(lon, 6), "alt_ft": 200.0})
                    print(f"[mock_lcl] tick {step_idx}: {lat:.6f}, {lon:.6f}")
                    step_idx += 1
                    time.sleep(0.1)
                else:
                    # no more position data — just heartbeat
                    if now - last_heartbeat >= HEARTBEAT_S:
                        publish(sock, f"{OWNER}.heartbeat", now)
                        last_heartbeat = now
                    time.sleep(0.1)

                # heartbeat alongside position publishes (every 3 s)
                if now - last_heartbeat >= HEARTBEAT_S:
                    publish(sock, f"{OWNER}.heartbeat", now)
                    last_heartbeat = now

        except (OSError, BrokenPipeError, ConnectionResetError):
            if not _shutdown.is_set():
                print("[mock_lcl] publish connection lost — reconnecting...")
                step_idx = len(STEPS)  # don't replay position history on reconnect
        finally:
            try:
                sock.close()
            except OSError:
                pass


def main():
    sub_thread = threading.Thread(target=subscribe_loop, args=(HOST, PORT), daemon=True)
    sub_thread.start()
    time.sleep(0.3)

    publish_loop(HOST, PORT)
    _shutdown.wait()
    print("[mock_lcl] stopped")


if __name__ == "__main__":
    main()
