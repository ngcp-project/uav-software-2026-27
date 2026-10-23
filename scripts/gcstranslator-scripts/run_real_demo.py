"""
run_real_demo.py -- test gcs_xbee.py through the REAL GCS library, with a pretend radio.
No XBee needed.   Works on Linux / WSL (it uses a pretend serial port, which Windows doesn't have).

How it works
    gcs_xbee.py opens a pretend serial port as if it were the XBee. This script plays the radio:
      * it sends real command packets (built with the real gcs-packet classes) into the port, and
      * it reads the real telemetry packets coming out and decodes them with the library's own
        Telemetry.Decode, exactly as the GCS would.
    Everything between (the XBee class, the packet framing, the library's threads, ReceiveCommand,
    SendTelemetry) is the real code. Only the physical radio is missing.

Run (with your venv active, from this folder):     python run_real_demo.py
Close any other adapter / gateway terminals first (the ports would be busy).

Which adapter is used:   python run_real_demo.py                       the simple test adapter.py
                         ADAPTER_SCRIPT=gcs_adapter.py python run_real_demo.py     the real adapter, working on
                                                                                   COPIES of the state files in a
                                                                                   temporary folder (your src/state
                                                                                   files are not touched)
"""
import os
import pty
import select
import signal
import json
import subprocess
import sys
import tempfile
import threading
import time
import tty

try:
    from Command import *
    from Enum import *
    from Telemetry.Telemetry import Telemetry
except (ImportError, NameError) as e:
    print(f"Could not load the GCS library: {e}")
    if isinstance(e, NameError) and sys.version_info < (3, 14):
        print(f"You are on Python {sys.version.split()[0]}. lib/gcs-packet/Packet/Telemetry/Telemetry.py needs Python 3.14,")
        print("or this as its very first line:   from __future__ import annotations")
    else:
        print("Run from the repo's top folder, with your venv active:")
        print("    pip install -e lib/gcs-packet -e lib/xbee-python -e lib/gcs-infrastructure pyzmq")
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
GCS_MAC = "0013A2004283A0EC"                       # gcs_xbee.py's default
RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 9.0


# ───────────────────────── XBee frames, as the radio sees them ─────────────────────────
def rx_frame(payload: bytes) -> bytes:
    """An XBee 'receive packet' (0x90) frame carrying `payload`, as if the GCS radio sent it."""
    body = b"\x90" + bytes.fromhex(GCS_MAC) + b"\xff\xfe" + b"\x01" + payload
    return b"\x7e" + len(body).to_bytes(2, "big") + body + bytes([0xFF - (sum(body) & 0xFF)])


def parse_tx_frames(buf: bytearray):
    """Take complete XBee 'transmit request' (0x00) frames out of buf -> [(destination MAC, payload)]."""
    found = []
    while True:
        start = buf.find(b"\x7e")
        if start < 0:
            buf.clear()
            break
        del buf[:start]
        if len(buf) < 3:
            break
        length = int.from_bytes(buf[1:3], "big")
        if len(buf) < 4 + length:
            break
        body, checksum = bytes(buf[3:3 + length]), buf[3 + length]
        del buf[:4 + length]
        if (0xFF - (sum(body) & 0xFF)) != checksum:
            continue
        if body[0] == 0x00:                          # transmit request: type, frame id, 8-byte address, options, data
            found.append((body[2:10].hex().upper(), body[11:]))
    return found


# ───────────────────────────── pretend serial port ─────────────────────────────
master_fd, slave_fd = pty.openpty()
tty.setraw(slave_fd)                                 # raw bytes, no echo or line editing
port = os.ttyname(slave_fd)

ADAPTER_SCRIPT = os.environ.get("ADAPTER_SCRIPT", "adapter.py")
env = dict(os.environ, GCS_XBEE_PORT=port, GCS_MAC=GCS_MAC, PYTHONUNBUFFERED="1")
state_dir = None
if ADAPTER_SCRIPT != "adapter.py":                   # the real adapter: give it test copies of the state files
    state_dir = tempfile.mkdtemp(prefix="gcs_state_")
    with open(os.path.join(state_dir, "telemetry_state.json"), "w") as f:
        json.dump({"position": {"latitude": 33.9325, "longitude": -117.6308, "relative_altitude_m": 60.96,
                                "last_updated": time.time()},
                   "battery": {"remaining_percent": 87.0, "last_updated": time.time()}}, f)
    env["GCS_STATE_DIR"] = state_dir
output = {"adapter": [], "xbee": []}
ready = threading.Event()


def pump(name, proc):
    for line in proc.stdout:
        line = line.rstrip()
        output[name].append(line)
        print(f"[{name:<7}] {line}")
        if name == "xbee" and "Running. Press Ctrl+C" in line:
            ready.set()


def start(name, script):
    proc = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, script)], cwd=HERE, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    threading.Thread(target=pump, args=(name, proc), daemon=True).start()
    return proc


telemetry_seen = []                                  # (time, Telemetry object, destination MAC)
stop_reading = threading.Event()


def read_radio():
    buf = bytearray()
    while not stop_reading.is_set():
        if select.select([master_fd], [], [], 0.1)[0]:
            try:
                buf += os.read(master_fd, 4096)
            except OSError:
                return
            for dest, payload in parse_tx_frames(buf):
                telemetry_seen.append((time.time(), Telemetry.Decode(payload), dest))


threading.Thread(target=read_radio, daemon=True).start()

print(f"Pretend serial port: {port}\nStarting {ADAPTER_SCRIPT} and gcs_xbee.py...\n")
adapter = start("adapter", ADAPTER_SCRIPT)
time.sleep(0.7)
xbee = start("xbee", "gcs_xbee.py")

if not ready.wait(timeout=15):
    print("\ngcs_xbee.py never reached 'Running'. See its output above.")
    xbee.kill(); adapter.kill()
    sys.exit(1)

# ─────────────────────── the pretend GCS: send commands over the radio ───────────────────────
ZONE_POINTS = [(33.9330, -117.6300), (33.9330, -117.6290), (33.9320, -117.6295)]     # a keep-out zone near Pomona
delivered = []          # (command id, packet id, name) of commands that should reach the adapter
skipped = []            # ones made but deliberately broken, which must NOT be acknowledged


def send(command, name):
    os.write(master_fd, rx_frame(command.EncodePacket()))
    delivered.append((command.COMMAND_ID, command.PacketID, name))
    print(f"[radio  ] GCS sends {name}  (command {command.COMMAND_ID}, packet id {command.PacketID})")


t0 = time.time()
plan = [
    (0.5, lambda: send(Heartbeat(ConnectionStatus.Connected), "Heartbeat")),
    (1.0, lambda: send(AddZone(ZoneType.KeepOut, ZONE_POINTS), "AddZone")),
    (1.5, "corrupt"),
    (2.0, lambda: send(PatientLocation((33.935, -117.631)), "PatientLocation")),
    (2.5, "text"),
    (3.0, lambda: send(EmergencyStop(0), "EmergencyStop")),
    (3.5, lambda: send(Heartbeat(ConnectionStatus.Unstable), "Heartbeat")),
]
for when, action in plan:
    time.sleep(max(0, t0 + when - time.time()))
    if action == "corrupt":
        broken = Heartbeat(ConnectionStatus.Connected)       # takes a packet id, but the frame is damaged
        frame = bytearray(rx_frame(broken.EncodePacket()))
        frame[-1] ^= 0xFF                                    # wrong checksum: the library must ignore it
        os.write(master_fd, bytes(frame))
        skipped.append((broken.COMMAND_ID, broken.PacketID))
        print(f"[radio  ] GCS sends a DAMAGED Heartbeat (packet id {broken.PacketID}); it should be ignored")
    elif action == "text":
        os.write(master_fd, rx_frame(b"hello GCS"))
        print("[radio  ] GCS sends a plain text packet")
    else:
        action()

time.sleep(max(0, t0 + RUN_SECONDS - time.time()))

# ─────────────────────────────── shut everything down ───────────────────────────────
stop_started = time.time()
xbee.send_signal(signal.SIGINT)
try:
    xbee.wait(timeout=5)
    xbee_exit = xbee.returncode
except subprocess.TimeoutExpired:
    xbee.kill(); xbee_exit = None
shutdown_s = time.time() - stop_started
adapter.send_signal(signal.SIGINT)
try:
    adapter.wait(timeout=5)
except subprocess.TimeoutExpired:
    adapter.kill()
stop_reading.set()
time.sleep(0.3)

# ───────────────────────────────────── results ─────────────────────────────────────
problems = []
streaming = telemetry_seen[0][0] if telemetry_seen else None
span = (telemetry_seen[-1][0] - streaming) if len(telemetry_seen) > 1 else 0
rate = (len(telemetry_seen) - 1) / span if span > 0 else 0
if len(telemetry_seen) < 10:
    problems.append(f"only {len(telemetry_seen)} telemetry packets came out of the radio")
elif rate < 3:
    problems.append(f"telemetry rate {rate:.1f}/s is far below 5/s")
if any(dest != GCS_MAC for _, _, dest in telemetry_seen):
    problems.append("telemetry was addressed to the wrong MAC")
if any(t is None for _, t, _ in telemetry_seen):
    problems.append("a telemetry packet could not be decoded by Telemetry.Decode")
seen_pairs = {(t.CommandID, t.PacketID) for _, t, _ in telemetry_seen if t}
adapter_text = "\n".join(output["adapter"])
warnings = []
zone_refused = "AddZone rejected" in adapter_text
if zone_refused:
    line = next((l for l in output["adapter"] if "AddZone rejected" in l), "")
    sent_ok = all(f"[{lat}, {lon}]" in line for lat, lon in ZONE_POINTS)
    if sent_ok:
        warnings.append("The adapter refused the AddZone command: " + line.split("points:")[-1].strip())
    else:
        warnings.append("The adapter REFUSED the AddZone command because its coordinates arrived damaged. The GCS sent "
                        f"{ZONE_POINTS} but the adapter got {line.split('points:')[-1].strip()}. That is a bug in the GCS "
                        "library (Packet/Command/AddZone.py, DecodePacket: use (i + 4) and (i + 5), not (i + 2) and "
                        "(i + 3)). The adapter did the right thing; the GCS gets no confirmation for that command.")
for cmd_id, pkt_id, name in delivered:
    if name == "AddZone" and zone_refused:
        continue
    if (cmd_id, pkt_id) not in seen_pairs:
        problems.append(f"{name} (packet id {pkt_id}) was never acknowledged in telemetry")
for cmd_id, pkt_id in skipped:
    if (cmd_id, pkt_id) in seen_pairs:
        problems.append(f"the damaged Heartbeat (packet id {pkt_id}) was acknowledged, but it should have been ignored")
got = [l for l in output["adapter"] if l.startswith("Received command:")]
if len(got) != len(delivered):
    problems.append(f"the adapter received {len(got)} commands, expected {len(delivered)}")
if any(t and abs(t.BatteryLife - 87.0) > 0.01 for _, t, _ in telemetry_seen):
    problems.append("BatteryLife in the packets is not the adapter's 87")
text = "\n".join(output["xbee"] + output["adapter"])
if "Traceback" in text:
    problems.append("a Python error (Traceback) appeared above")
if "could not send telemetry" in text:
    problems.append("gcs_xbee.py reported it could not send telemetry")
if xbee_exit != 0 or shutdown_s > 3:
    problems.append(f"gcs_xbee.py did not stop cleanly on Ctrl+C (exit {xbee_exit}, {shutdown_s:.1f} s)")

print("\n" + "=" * 68)
print(f"Telemetry packets out of the radio : {len(telemetry_seen)}   ({rate:.1f}/s, decoded by the library's Telemetry.Decode)")
print(f"Commands delivered to the adapter  : {len(got)} of {len(delivered)} sent  (damaged one correctly ignored)")
print(f"gcs_xbee.py stopped on Ctrl+C      : exit code {xbee_exit} in {shutdown_s:.1f} s")
if telemetry_seen:
    t = telemetry_seen[-1][1]
    print("Last packet as the GCS would see it:")
    print("   " + str(t).replace(", ", "\n   "))
for w in warnings:
    print("  WARNING:", w)
for p in problems:
    print("  PROBLEM:", p)
print("RESULT: " + ("PASS" if not problems else "FAIL"))
print("=" * 68)
sys.exit(0 if not problems else 1)
