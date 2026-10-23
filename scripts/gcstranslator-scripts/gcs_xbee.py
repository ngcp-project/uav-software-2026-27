#!/usr/bin/env python3
"""
gcs_xbee.py  --  Script 1: a plain RELAY between the GCS library and the adapter.   REAL XBEE VERSION.

Built from VehicleTest.py: same library calls, same one-loop structure.

This script does NO translating. The adapter does that.

    COMMANDS    GCS -> XBee -> ReceiveCommand() -> [this script] -> adapter
                  The adapter gets exactly what the library decoded, as JSON, e.g.
                  {"Command ID": 3, "Packet ID": 1, "Zone": 1, "Zone ID": 0, "Coordinates": [[x, y], ...]}
                  (the library's own field names; the adapter decides what each command means).

    TELEMETRY   adapter -> [this script] -> Telemetry(**fields) + SendTelemetry() -> XBee -> GCS
                  The adapter sends the library's own Telemetry fields, already in the units the GCS
                  expects, as JSON:
                  {"CommandID": 1, "PacketID": 5, "Speed": 0.0, "Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0,
                   "Altitude": 0.0, "BatteryLife": 87.0, "LastUpdated": 0, "CurrentPosition": [0.0, 0.0],
                   "VehicleStatus": 0, "MessageFlag": 0, "MessageLat": 0.0, "MessageLon": 0.0,
                   "PatientStatus": 0}
                  Types matter: CommandID, PacketID, LastUpdated, VehicleStatus, MessageFlag and
                  PatientStatus must be whole numbers; the rest may be floats. CommandID/PacketID are
                  the receipt: the adapter puts the last command it applied there.

What this script still does itself (it is the only thing that touches the library):
    start the XBee, pull commands, push telemetry at <= 5 per second (newest only), report problems,
    and shut down cleanly.

Setup (once, from the repo's top folder, with your venv active):
    pip install -e lib/gcs-packet -e lib/xbee-python -e lib/gcs-infrastructure pyzmq

Optional settings (environment variables):
    GCS_XBEE_PORT   serial port of the XBee         default COM3    (Linux example: /dev/ttyUSB0)
    GCS_MAC         MAC address of the GCS XBee     default 0013A2004283A0EC  (the one in VehicleTest.py)
    GCS_CMD_PORT    adapter command port            default 6767
    GCS_TLM_PORT    adapter telemetry port          default 3435

Pairs with the streaming adapter: it BINDS the two ports above (commands = PULL,
telemetry = PUSH); this script CONNECTS to both.
"""
import sys as _sys

# ── Library imports: exactly as in VehicleTest.py ────────────────────────────
try:
    from Command import *
    from Enum import *
    from Infrastructure.InfrastructureInterface import *
    from PacketLibrary.PacketLibrary import PacketLibrary
    from Telemetry.Telemetry import Telemetry
except NameError as _e:
    print(f"[XBee] ERROR: the GCS library failed to load: {_e}")
    if "Telemetry" in str(_e) and _sys.version_info < (3, 14):
        print(f"       You are running Python {_sys.version.split()[0]}. lib/gcs-packet/Packet/Telemetry/Telemetry.py")
        print("       needs Python 3.14, or this line added as the VERY FIRST line of that file:")
        print("           from __future__ import annotations")
        print("       (It is the GCS team's file, so ask them to add it.)")
    _sys.exit(1)
except ImportError as _e:
    print(f"[XBee] ERROR: could not import the GCS library: {_e}")
    print("       From the repo's top folder, with your venv active, run:")
    print("           pip install -e lib/gcs-packet -e lib/xbee-python -e lib/gcs-infrastructure")
    _sys.exit(1)

# Our own imports come AFTER the star-imports above, so the library can never shadow them.
import contextlib
import inspect
import io
import json
import os
import sys
import threading
import time

import zmq

# ═══════════════════════════════ SETTINGS ═══════════════════════════════════
XBEE_PORT = os.environ.get("GCS_XBEE_PORT", "COM3")
GCS_MAC = os.environ.get("GCS_MAC", "0013A2004283A0EC")
ADAPTER_COMMAND_PORT = int(os.environ.get("GCS_CMD_PORT", 6767))      # we PUSH commands here
ADAPTER_TELEMETRY_PORT = int(os.environ.get("GCS_TLM_PORT", 3435))    # we PULL telemetry here

TELEMETRY_SEND_HZ = 5            # most telemetry packets we hand to the radio per second
SEND_PERIOD = 1.0 / TELEMETRY_SEND_HZ
MAX_RADIO_BACKLOG = 20           # if the library's send queue holds more than this, skip sending

QUIET_LIBRARY = True             # hide the library's "Telemetry Queued" line (5 times a second)


# ═════════════════ COMMANDS: library -> adapter (no translating) ═════════════
def command_for_adapter(command):
    """
    `command` is what ReceiveCommand(..., DecodeFormat.JSON) returned: JSON text. We only check that it
    really is JSON, then hand the parsed result on unchanged. Returns None (after saying why) if not.
    JSON is used because it keeps the Packet ID the GCS really sent; DecodeFormat.Class replaces it
    with the library's own counter.
    """
    try:
        return json.loads(command)
    except (TypeError, json.JSONDecodeError) as e:
        print(f"JSON Error: {e}")
        return None


# ═════════════════ TELEMETRY: adapter -> library (no translating) ════════════
TELEMETRY_FIELDS = list(inspect.signature(Telemetry).parameters)     # the library's own argument names


def telemetry_from_adapter(fields, warned):
    """
    Telemetry(**fields), using the library's names and the adapter's values as they are.
    Fields the adapter leaves out take the library's default (0). Unknown fields are ignored.
    Each kind of problem is mentioned once (in `warned`), not 5 times a second.
    """
    unknown = sorted(set(fields) - set(TELEMETRY_FIELDS))
    missing = sorted(set(TELEMETRY_FIELDS) - set(fields))
    if unknown and ("unknown", tuple(unknown)) not in warned:
        warned.add(("unknown", tuple(unknown)))
        print(f"[XBee] The adapter's telemetry has fields the library does not have, ignoring: {', '.join(unknown)}")
    if missing and ("missing", tuple(missing)) not in warned:
        warned.add(("missing", tuple(missing)))
        print(f"[XBee] The adapter's telemetry leaves out: {', '.join(missing)}  (the library's default, 0, is sent)")
    return Telemetry(**{k: v for k, v in fields.items() if k in TELEMETRY_FIELDS})


@contextlib.contextmanager
def quiet_library():
    if not QUIET_LIBRARY:
        yield
        return
    with contextlib.redirect_stdout(io.StringIO()):
        yield


def radio_backlog():
    try:
        return TelemetryQueue.qsize()                    # the library's own send queue
    except Exception:
        return 0


# ═════════════════════════════ XBEE START-UP CHECKS ═════════════════════════
def mac_is_valid(mac):
    return len(mac) == 16 and all(c in "0123456789abcdefABCDEF" for c in mac)


def launch_xbee():
    """
    LaunchVehicleXBee() never raises and never returns anything: if the port can't be opened it
    just prints "Error: ..." and quietly starts nothing. So we check that its threads are alive.
    """
    before = set(threading.enumerate())
    LaunchVehicleXBee(XBEE_PORT)
    time.sleep(0.5)
    started = [t for t in threading.enumerate() if t not in before and t.is_alive()]
    return len(started) >= 2


# ═══════════════════════════════════ MAIN ═══════════════════════════════════
def main():
    # 1. The GCS MAC address (the library only prints a warning if it is bad).
    if not mac_is_valid(GCS_MAC):
        print(f"[XBee] ERROR: GCS_MAC {GCS_MAC!r} must be exactly 16 hex characters.")
        return 1
    PacketLibrary.SetGCSMACAddress(GCS_MAC)

    # 2. Start the XBee exactly as VehicleTest.py does. Fail fast if it did not start.
    if not launch_xbee():
        print(f"[XBee] ERROR: the XBee did not start on {XBEE_PORT} (see the library's message above).")
        print("       Check the port name, that the XBee is plugged in, and that nothing else is using it.")
        return 1
    print(f"[XBee] XBee running on {XBEE_PORT}, GCS MAC {GCS_MAC}")

    # 3. Connect to the adapter.
    ctx = zmq.Context()
    cmd_sock = ctx.socket(zmq.PUSH)                     # commands go OUT to the adapter
    cmd_sock.setsockopt(zmq.LINGER, 0)
    cmd_sock.connect(f"tcp://127.0.0.1:{ADAPTER_COMMAND_PORT}")
    tlm_sock = ctx.socket(zmq.PULL)                     # telemetry comes IN from the adapter
    tlm_sock.setsockopt(zmq.LINGER, 0)
    tlm_sock.connect(f"tcp://127.0.0.1:{ADAPTER_TELEMETRY_PORT}")
    print(f"[XBee] Talking to the adapter: commands -> {ADAPTER_COMMAND_PORT}, telemetry <- {ADAPTER_TELEMETRY_PORT}")
    print("[XBee] Running. Press Ctrl+C to stop.")

    latest = None                  # newest telemetry from the adapter not yet sent to the radio
    warned = set()                 # telemetry problems already mentioned
    last_send = 0.0
    last_tlm_at = time.time()
    warned_quiet = False
    forwarded = sent = send_errors = skipped = bad_tlm = recv_errors = 0
    sent_at_last_report, next_report = 0, time.time() + 5

    try:
        while True:
            # ── COMMANDS: radio -> adapter ──────────────────────────────────
            try:
                command = ReceiveCommand(False, DecodeFormat.JSON)
            except Exception as e:
                recv_errors += 1
                if recv_errors <= 3 or recv_errors % 50 == 0:
                    print(f"[XBee] ReceiveCommand failed ({recv_errors}x): {e!r}")
                command = None
                time.sleep(0.2)

            if command is not None:
                print(command)
                msg = command_for_adapter(command)
                if msg is not None:
                    try:
                        cmd_sock.send_json(msg, flags=zmq.NOBLOCK)
                        forwarded += 1
                        print(f"[XBee] command -> adapter: {msg}")
                    except zmq.Again:
                        print("[XBee] adapter not reachable, command dropped")

            # ── TELEMETRY: adapter -> radio ─────────────────────────────────
            while True:                                  # drain, keeping only the NEWEST
                try:
                    latest = tlm_sock.recv_json(flags=zmq.NOBLOCK)
                    last_tlm_at, warned_quiet = time.time(), False
                except zmq.Again:
                    break
                except ValueError:
                    bad_tlm += 1

            now = time.time()
            if latest is not None and now - last_send >= SEND_PERIOD:
                if radio_backlog() > MAX_RADIO_BACKLOG:
                    skipped += 1
                    if skipped <= 3 or skipped % 50 == 0:
                        print(f"[XBee] radio is not keeping up ({radio_backlog()} packets waiting), "
                              f"skipping telemetry ({skipped}x)")
                else:
                    try:
                        telemetry = telemetry_from_adapter(latest, warned)
                        telemetry.Encode()               # packing errors would otherwise be hidden in the library
                        with quiet_library():
                            SendTelemetry(telemetry)
                        sent += 1
                    except Exception as e:
                        send_errors += 1
                        if send_errors <= 3 or send_errors % 50 == 0:
                            print(f"[XBee] could not send telemetry ({send_errors}x): {e!r}")
                latest = None                            # never re-send old data as if it were new
                last_send = now

            if now - last_tlm_at > 5 and not warned_quiet:
                print("[XBee] No telemetry from the adapter for 5 s, so nothing is being sent "
                      "to the GCS. Is the adapter running?")
                warned_quiet = True

            if now >= next_report:
                rate = (sent - sent_at_last_report) / 5
                print(f"[XBee] status: telemetry sent {sent} ({rate:.1f}/s), commands forwarded {forwarded}, "
                      f"send errors {send_errors}, skipped {skipped}, bad telemetry {bad_tlm}")
                sent_at_last_report, next_report = sent, now + 5

            tlm_sock.poll(20)                            # sleep up to 20 ms, wake early on new data

    except KeyboardInterrupt:
        print("\n[XBee] Stopped by Ctrl+C.")
    finally:
        cmd_sock.close(linger=0)
        tlm_sock.close(linger=0)
        ctx.term()
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    # The library starts background threads that can never be stopped, so a normal exit would
    # hang forever. os._exit ends the process right away.
    os._exit(code)
