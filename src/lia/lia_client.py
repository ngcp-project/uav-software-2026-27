"""
Reusable client-side interface for the local LIA broker.

The client owns the common socket and newline-delimited JSON details so
subsystems only need to provide a sender, topic, and value.

Supported operations:
    publish()   → send a durable state publication or transient command
    subscribe() → receive live broadcasts with optional reconnects
    shutdown()  → request a broker-wide shutdown

This is intentionally not an RPC or ACK layer. Commands remain transient;
explicit acknowledgement can be added later for commands that require it.
"""

from __future__ import annotations

import json
import socket
import time
from typing import Iterator

LIA_HOST = "127.0.0.1"
LIA_PORT = 5588


# ── client ────────────────────────────────────────────────────────────────────

class LiaClient:
    def __init__(self, sender: str, host: str = LIA_HOST, port: int = LIA_PORT,
                 connect_timeout: float = 5.0, retry_delay: float = 1.0):
        self.sender = sender
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.retry_delay = retry_delay

    def _send(self, message: dict) -> None:
        # Publish and shutdown connections are short-lived. This keeps one
        # failed send from poisoning the next message.
        with socket.create_connection(
            (self.host, self.port), timeout=self.connect_timeout
        ) as sock:
            sock.sendall((json.dumps(message) + "\n").encode("utf-8"))

    def publish(self, topic: str, value) -> None:
        # Topics are namespaced so LIA can distinguish state from commands.
        if not topic or "." not in topic:
            raise ValueError("LIA topics must use a namespace, for example telemetry.position")
        self._send({
            "type": "publish",
            "topic": topic,
            "value": value,
            "sender": self.sender,
            "ts": time.time(),
        })

    def subscribe(self, retry: bool = True) -> Iterator[dict]:
        """Yield broadcasts until LIA shuts down; reconnect after broker loss."""
        while True:
            try:
                with socket.create_connection(
                    (self.host, self.port), timeout=self.connect_timeout
                ) as sock:
                    sock.settimeout(None)
                    sock.sendall((json.dumps({
                        "type": "subscribe", "sender": self.sender
                    }) + "\n").encode("utf-8"))
                    with sock.makefile("r", encoding="utf-8") as stream:
                        for line in stream:
                            message = json.loads(line)
                            yield message
                            # Shutdown is terminal even when reconnects are enabled.
                            if message.get("type") == "shutdown":
                                return
                if not retry:
                    return
            except (OSError, ConnectionError, json.JSONDecodeError):
                if not retry:
                    raise
                time.sleep(self.retry_delay)

    def shutdown(self) -> None:
        # The broker broadcasts shutdown to every active subscriber.
        self._send({"type": "shutdown", "sender": self.sender})
