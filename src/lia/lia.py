#!/usr/bin/env python3
"""
LIA broker —  pub/sub message router with durable state persistence.

State files:
    telemetry_state.json    ← aircraft truth: FC mode, position, battery, etc.
    navigation_state.json   ← autonomy-facing destination and plan state
    mission_state.json      ← mission/controller state
    localization_state.json ← human-facing localization result/debug state
    heartbeats.json         ← last heartbeat timestamp per producer

Topic routing rules:
    telemetry.*     → telemetry_state.json
    nav.*           → navigation_state.json
    mission.*       → mission_state.json
    localization.* → localization_state.json
    *.heartbeat     → heartbeats.json
    cmd.*           → transient, broadcast only

Durable publications are broadcast immediately and persisted separately.
High-rate state may be throttled for disk writes, while commands remain
transient and are never written to a command JSON file.

Shutdown:
    A client sends {"type": "shutdown"} → LIA broadcasts {"type": "shutdown"}
    to all subscribers then exits. Subscribers should exit instead of retrying.
"""

from __future__ import annotations

import json
import socketserver
import threading
import time
from pathlib import Path

HOST, PORT = "127.0.0.1", 5588
ROOT_DIR = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT_DIR / "state"

QUIET_TOPICS = {
    "telemetry.position",
    "telemetry.battery",
    "telemetry.fc_mode",
    "telemetry.armed",
}
QUIET_SUFFIXES = {".heartbeat"}


STATE_FILES = {
    "telemetry": STATE_DIR / "telemetry_state.json",
    "nav": STATE_DIR / "navigation_state.json",
    "mission": STATE_DIR / "mission_state.json",
    "localization": STATE_DIR / "localization_state.json",
    "heartbeat": STATE_DIR / "heartbeats.json",
}

# High-rate state is broadcast live, but disk writes are coalesced.
PERSIST_THROTTLE_S = {"telemetry": 0.2, "heartbeat": 1.0}

_lock              = threading.Lock()
_subscribers       = []
_last_persist_time = {}
_shutdown_flag     = threading.Event()


# ── file helpers ──────────────────────────────────────────────────────────────

def load_file(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_file(data: dict, path: Path) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


# ── state updates ─────────────────────────────────────────────────────────────

def _set_nested(state: dict, key: str, value) -> None:
    """Update a topic field without flattening existing state-file sections."""
    aliases = {
        "fc_mode": ("flight_controller", "mode"),
        "armed": ("flight_controller", "armed"),
    }
    if key in aliases:
        parent, child = aliases[key]
        state.setdefault(parent, {})[child] = value
    elif isinstance(value, dict) and isinstance(state.get(key), dict):
        state[key] = {**state[key], **value}
    else:
        state[key] = value


# ── persistence ───────────────────────────────────────────────────────────────

def persist(topic: str, value) -> bool:
    namespace, _, field = topic.partition(".")
    # Heartbeats use producer names as their keys, for example
    # fc_listener.heartbeat -> heartbeats.json["fc_listener"].
    is_heartbeat = field == "heartbeat"
    if (namespace not in STATE_FILES and not is_heartbeat) or not field:
        return False

    now = time.time()
    # Throttle by semantic state category rather than by individual topic.
    persist_namespace = "heartbeat" if is_heartbeat else namespace
    limit = PERSIST_THROTTLE_S.get(persist_namespace)

    # Heartbeats can share a heartbeat rate, but semantic state should
    # throttle independently by topic.
    throttle_key = persist_namespace if is_heartbeat else topic

    with _lock:
        if limit and now - _last_persist_time.get(throttle_key, 0.0) < limit:
            return False

        if limit:
            _last_persist_time[throttle_key] = now

        path = STATE_FILES["heartbeat"] if is_heartbeat else STATE_FILES[namespace]
        state = load_file(path)

        if is_heartbeat:
            state[namespace] = value
        else:
            _set_nested(state, field, value)

        save_file(state, path)
        return True


# ── broadcast ─────────────────────────────────────────────────────────────────

def broadcast(message: dict) -> None:
    line = (json.dumps(message) + "\n").encode("utf-8")
    with _lock:
        dead = []
        for sock in _subscribers:
            try:
                sock.sendall(line)
            except OSError:
                dead.append(sock)
        for sock in dead:
            _subscribers.remove(sock)


# ── request handler ───────────────────────────────────────────────────────────

class LiaHandler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            for raw_line in self.rfile:
                msg = json.loads(raw_line.decode().strip())

                # A shutdown is a broker-wide event, so notify subscribers
                # before the server exits.
                if msg.get("type") == "shutdown":
                    broadcast({"type": "shutdown"})
                    _shutdown_flag.set()
                    return

                # A subscriber keeps this socket open for live broadcasts.
                if msg.get("type") == "subscribe":
                    with _lock:
                        _subscribers.append(self.request)
                    while not _shutdown_flag.is_set():
                        time.sleep(0.2)
                    return

                # Publications are routed live even when disk persistence is
                # throttled or skipped for transient command topics.
                if msg.get("type") != "publish":
                    continue

                topic = msg["topic"]
                namespace = topic.split(".", 1)[0]
                durable = namespace in STATE_FILES or topic.endswith(".heartbeat")
                persisted = persist(topic, msg.get("value")) if durable else False
                status = "persisted" if persisted else (
                    "throttled" if durable else "transient"
                )
                should_log = (
                    topic not in QUIET_TOPICS
                    and not any(topic.endswith(suffix) for suffix in QUIET_SUFFIXES)
                )
                if should_log:
                    print(
                        f"[LIA] {msg.get('sender', '?')} -> "
                        f"{topic} ({status})"
                    )
                broadcast({
                    "topic": topic,
                    "value": msg.get("value"),
                    "sender": msg.get("sender", "?"),
                    "ts": msg.get("ts", time.time()),
                })
        except (ConnectionResetError, BrokenPipeError, json.JSONDecodeError, KeyError):
            pass
        finally:
            with _lock:
                if self.request in _subscribers:
                    _subscribers.remove(self.request)


# ── server lifecycle ───────────────────────────────────────────────────────────

class LiaServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    # Fail early if the semantic state files have not been initialized.
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    missing = [str(path) for path in STATE_FILES.values() if not path.exists()]
    if missing:
        raise SystemExit(f"Missing state files: {', '.join(missing)}")
    print(f"[LIA] starting on {HOST}:{PORT}")
    server = LiaServer((HOST, PORT), LiaHandler)
    # Run the socket loop in a daemon thread so the main thread can wait for
    # the broker-wide shutdown event.
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    _shutdown_flag.wait()
    server.shutdown()


if __name__ == "__main__":
    main()
