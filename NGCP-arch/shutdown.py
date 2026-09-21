#!/usr/bin/env python3
"""
Send a shutdown signal to LIA. Run this in any terminal to stop the whole system.
LIA will broadcast the shutdown to all subscribers, then exit.
"""
import json
import socket
import sys

HOST, PORT = "127.0.0.1", 5588

try:
    with socket.create_connection((HOST, PORT), timeout=2) as sock:
        msg = {"type": "shutdown", "sender": "shutdown.py"}
        sock.sendall((json.dumps(msg) + "\n").encode())
        print("[shutdown] signal sent — LIA and all subscribers will exit")
except OSError as e:
    print(f"[shutdown] could not connect to LIA: {e}", file=sys.stderr)
    sys.exit(1)
