#!/usr/bin/env python3
"""
LIA broker — pub/sub message router with durable state persistence.

State files:
    mission_state.json  ← meta block only
    atm.json / fcl.json / gcs.json / lcl.json  ← owner fields (flat)
    heartbeats.json     ← last heartbeat timestamp per owner

Topic routing rules:
    meta.*          → mission_state.json  (meta block)
    *.heartbeat     → heartbeats.json     (owner key)
    atm.* fcl.* gcs.* lcl.*  → <owner>.json
    cmd.*           → transient, forward only

Shutdown:
    A client sends {"type": "shutdown"} → LIA broadcasts {"type": "shutdown"}
    to all subscribers then exits. Subscribers should exit instead of retrying.
"""
import json
import socketserver
import threading
import time
from pathlib import Path

HOST, PORT = "127.0.0.1", 5588

STATE_DIR      = Path(__file__).resolve().parent
META_FILE      = STATE_DIR / "mission_state.json"
HEARTBEAT_FILE = STATE_DIR / "heartbeats.json"
DURABLE_OWNERS = ("atm", "fcl", "gcs", "lcl")

STATE_FILES = {
    "meta":      META_FILE,
    "heartbeat": HEARTBEAT_FILE,
    **{owner: STATE_DIR / f"{owner}.json" for owner in DURABLE_OWNERS},
}

PERSIST_THROTTLE_S = {
    "lcl": 0.5,   # position arrives at 10 Hz; only persist at ~2 Hz
}

_lock              = threading.Lock()
_subscribers       = []
_last_persist_time = {}
_shutdown_flag     = threading.Event()


# ── file helpers ──────────────────────────────────────────────────────────────

def load_file(path: Path) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def save_file(data: dict, path: Path) -> None:
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    tmp.replace(path)


# ── persistence ───────────────────────────────────────────────────────────────

def persist(topic: str, value) -> bool:
    owner, _, field = topic.partition(".")
    if not field:
        return False

    with _lock:
        now   = time.time()
        # throttle check uses owner, not topic
        limit = PERSIST_THROTTLE_S.get(owner)
        if limit is not None:
            last = _last_persist_time.get(owner, 0.0)
            if now - last < limit:
                return False
            _last_persist_time[owner] = now

        # ── heartbeat: any owner's .heartbeat goes to heartbeats.json ─────────
        if field == "heartbeat":
            hb = load_file(HEARTBEAT_FILE)
            hb[owner] = value   # value is the sender's ts float
            save_file(hb, HEARTBEAT_FILE)

        elif owner == "meta":
            state = load_file(META_FILE)
            state["meta"][field] = value
            save_file(state, META_FILE)

        elif owner in DURABLE_OWNERS:
            owner_data = load_file(STATE_FILES[owner])
            owner_data[field] = value
            save_file(owner_data, STATE_FILES[owner])
            # stamp last_updated in mission_state.json
            meta = load_file(META_FILE)
            meta["meta"]["last_updated"][owner] = now
            save_file(meta, META_FILE)

        else:
            return False

        return True


# ── broadcast ─────────────────────────────────────────────────────────────────

def broadcast(message: dict) -> None:
    line = (json.dumps(message) + "\n").encode()
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

                # ── shutdown ──────────────────────────────────────────────────
                if msg.get("type") == "shutdown":
                    sender = msg.get("sender", "?")
                    print(f"[LIA] shutdown requested by {sender} — broadcasting and exiting")
                    broadcast({"type": "shutdown"})
                    _shutdown_flag.set()
                    return

                # ── subscribe ─────────────────────────────────────────────────
                if msg.get("type") == "subscribe":
                    with _lock:
                        _subscribers.append(self.request)
                    print(f"[LIA] subscriber connected ({msg.get('sender', '?')})")
                    while not _shutdown_flag.is_set():
                        time.sleep(0.2)
                    return

                # ── publish ───────────────────────────────────────────────────
                if msg.get("type") == "publish":
                    topic  = msg["topic"]
                    value  = msg.get("value")
                    sender = msg.get("sender", "?")
                    ts     = msg.get("ts", time.time())   # prefer sender's timestamp
                    owner  = topic.split(".", 1)[0]
                    field  = topic.split(".", 1)[1] if "." in topic else ""
                    durable = (
                        owner in DURABLE_OWNERS or
                        topic.startswith("meta.") or
                        field == "heartbeat"
                    )

                    if durable:
                        wrote  = persist(topic, value)
                        status = "persisted" if wrote else f"throttled (<{PERSIST_THROTTLE_S.get(owner)}s)"
                    else:
                        status = "transient, routing only"

                    print(f"[LIA] {sender} -> {topic} = {value!r}  ({status})")
                    broadcast({"topic": topic, "value": value, "sender": sender, "ts": ts})

        except (ConnectionResetError, BrokenPipeError, json.JSONDecodeError):
            pass
        finally:
            with _lock:
                if self.request in _subscribers:
                    _subscribers.remove(self.request)


class LiaServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True


def main():
    required = list(STATE_FILES.values()) + [HEARTBEAT_FILE]
    missing  = [str(p) for p in required if not p.exists()]
    if missing:
        raise SystemExit(f"Missing state files: {', '.join(missing)}")

    print(f"[LIA] starting on {HOST}:{PORT}")
    for owner, path in STATE_FILES.items():
        print(f"[LIA]   {owner:9s} -> {path.name}")

    server = LiaServer((HOST, PORT), LiaHandler)

    # Run server in a daemon thread so we can watch the shutdown flag
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    _shutdown_flag.wait()   # block until shutdown is requested
    print("[LIA] shutting down")
    server.shutdown()


if __name__ == "__main__":
    main()
