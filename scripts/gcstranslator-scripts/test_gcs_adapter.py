"""
test_gcs_adapter.py -- tests gcs_adapter.py on its own. No XBee, no pretend radio, no GCS library needed
(the library is used for one extra check if it is installed). Works on Linux, WSL and macOS.

This script plays the part of gcs_xbee.py: it sends commands to the adapter and reads the telemetry the
adapter sends back. The adapter works on COPIES of the state files in a temporary folder
(GCS_STATE_DIR), so your real src/state files are never touched.

Run (venv active, from this folder):     python test_gcs_adapter.py
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import zmq

HERE = Path(__file__).resolve().parent
CMD_PORT, TLM_PORT = 16767, 13435            # not the real ports, so a running adapter is never disturbed

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   [{detail}]" if detail and not ok else ""))


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2))


def read_json(path):
    return json.loads(path.read_text())


# ───────────────────────── a temporary set of state files ─────────────────────────
state_dir = Path(tempfile.mkdtemp(prefix="gcs_state_"))
NOW = time.time()
write_json(state_dir / "telemetry_state.json", {
    "flight_controller": {"connected": True, "last_heartbeat": NOW, "mode": "GUIDED", "armed": True},
    "position": {"latitude": 33.9325, "longitude": -117.6308, "relative_altitude_m": 60.96,
                 "absolute_altitude_m": 300.0, "last_updated": NOW},
    "battery": {"voltage_v": 15.8, "remaining_percent": 87.0, "last_updated": NOW},
})
write_json(state_dir / "localization_state.json", {
    "patient": {"patient_found": True, "patient_lat": 33.9331, "patient_lon": -117.6312, "last_updated": NOW}})

env = dict(os.environ, GCS_STATE_DIR=str(state_dir), GCS_CMD_PORT=str(CMD_PORT), GCS_TLM_PORT=str(TLM_PORT),
           PYTHONUNBUFFERED="1")
adapter = subprocess.Popen([sys.executable, "-u", str(HERE / "gcs_adapter.py")], env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
lines = []
import threading
threading.Thread(target=lambda: [lines.append(l.rstrip()) for l in adapter.stdout], daemon=True).start()

# ───────────────────────────── play the part of gcs_xbee.py ─────────────────────────────
ctx = zmq.Context()
cmd = ctx.socket(zmq.PUSH); cmd.setsockopt(zmq.LINGER, 0); cmd.connect(f"tcp://127.0.0.1:{CMD_PORT}")
tlm = ctx.socket(zmq.PULL); tlm.setsockopt(zmq.LINGER, 0); tlm.connect(f"tcp://127.0.0.1:{TLM_PORT}")


def latest(seconds=0.6):
    """The newest telemetry packet seen in the next `seconds`, and how many arrived."""
    end, last, count = time.time() + seconds, None, 0
    while time.time() < end:
        if tlm.poll(50):
            last, count = tlm.recv_json(), count + 1
    return last, count


def send(obj):
    cmd.send_json(obj)


def send_raw(raw):
    cmd.send(raw)


try:
    # wait for the adapter to be up
    first, _ = latest(2.0)
    print("\nTelemetry (state files -> GCS fields)")
    check("telemetry arrives", first is not None)
    t, n = latest(2.0)
    check("about 5 packets per second", 7 <= n <= 12, f"{n} in 2 s")
    check("battery comes from telemetry_state", t and t["BatteryLife"] == 87.0, t and t["BatteryLife"])
    check("position comes from telemetry_state (lat, lon)", t and t["CurrentPosition"] == [33.9325, -117.6308], t and t["CurrentPosition"])
    check("altitude converted metres -> feet (60.96 m = 200 ft)", t and abs(t["Altitude"] - 200.0) < 0.01, t and t["Altitude"])
    check("LastUpdated is in milliseconds", t and abs(t["LastUpdated"] - int(NOW * 1000)) < 1, t and t["LastUpdated"])
    check("patient found -> MessageFlag 2 with its location",
          t and t["MessageFlag"] == 2 and t["MessageLat"] == 33.9331 and t["MessageLon"] == -117.6312)
    check("no command yet -> receipt 0/0", t and (t["CommandID"], t["PacketID"]) == (0, 0))
    try:                                        # does the real library accept exactly these fields?
        from Telemetry.Telemetry import Telemetry
        Telemetry(**t).Encode()
        check("the library's Telemetry(**fields).Encode() accepts the packet", True)
    except ImportError:
        print("  (skipped)  library not installed here: Telemetry(**fields) check")
    except Exception as e:
        check("the library's Telemetry(**fields).Encode() accepts the packet", False, repr(e))

    print("\nCommands (what each one does to the state files)")
    send({"Command ID": 1, "Packet ID": 7, "Connection Status": 0})
    t, _ = latest()
    link = read_json(state_dir / "mission_state.json").get("gcs_link", {})
    check("Heartbeat: gcs_link recorded as Connected", link.get("status") == "Connected", link)
    check("Heartbeat: receipted in telemetry (1/7)", t and (t["CommandID"], t["PacketID"]) == (1, 7), t and (t["CommandID"], t["PacketID"]))

    area = [[33.935, -117.632], [33.935, -117.625], [33.932, -117.629], [33.9325, -117.6321]]
    send({"Command ID": 3, "Packet ID": 8, "Zone": 2, "Zone ID": 0, "Coordinates": area})
    t, _ = latest()
    check("AddZone SearchArea: navigation_state search_area set", read_json(state_dir / "navigation_state.json").get("search_area") == area)
    check("AddZone SearchArea: receipted (3/8)", t and (t["CommandID"], t["PacketID"]) == (3, 8))

    keep_out = [[33.9330, -117.6300], [33.9330, -117.6290], [33.9320, -117.6295]]
    send({"Command ID": 3, "Packet ID": 9, "Zone": 1, "Zone ID": 1, "Coordinates": keep_out})
    t, _ = latest()
    nav = read_json(state_dir / "navigation_state.json")
    check("AddZone KeepOut: stored under gcs_zones.keep_out", nav.get("gcs_zones", {}).get("keep_out", {}).get("1", {}).get("points") == keep_out)
    check("AddZone KeepOut: search_area left alone", nav.get("search_area") == area)
    check("AddZone KeepOut: receipted (3/9)", t and (t["CommandID"], t["PacketID"]) == (3, 9))

    damaged = [[1, 0], [0.0, 4.0], [4.0, 0.0], [4.0, 4.0]]       # what the GCS library's AddZone bug produces
    send({"Command ID": 3, "Packet ID": 10, "Zone": 2, "Zone ID": 2, "Coordinates": damaged})
    t, _ = latest()
    check("AddZone with damaged coordinates: refused, state unchanged", read_json(state_dir / "navigation_state.json").get("search_area") == area)
    check("AddZone with damaged coordinates: NOT receipted", t and (t["CommandID"], t["PacketID"]) == (3, 9), t and (t["CommandID"], t["PacketID"]))

    send({"Command ID": 4, "Packet ID": 11, "Coordinate": [33.9340, -117.6290]})
    t, _ = latest()
    nav = read_json(state_dir / "navigation_state.json")
    check("PatientLocation: eru_reported_location set and valid",
          nav["eru_reported_location"]["lat"] == 33.934 and nav["eru_reported_location"]["valid"] is True)
    check("PatientLocation: target_location set from it",
          nav["target_location"]["source"] == "eru_reported_location" and nav["target_location"]["lon"] == -117.629 and nav["target_location"]["valid"] is True)
    check("PatientLocation: receipted (4/11)", t and (t["CommandID"], t["PacketID"]) == (4, 11))

    mission = read_json(state_dir / "mission_state.json")
    check("before EmergencyStop: loiter_requested is False", mission.get("loiter_requested") is False)
    send({"Command ID": 2, "Packet ID": 12, "Stop Status": 0})
    t, _ = latest()
    mission = read_json(state_dir / "mission_state.json")
    check("EmergencyStop: loiter_requested set True", mission.get("loiter_requested") is True)
    check("EmergencyStop: nothing else requested (no RTL)", mission.get("rtl_requested") is False)
    check("EmergencyStop: receipted (2/12)", t and (t["CommandID"], t["PacketID"]) == (2, 12))

    print("\nBad input (the adapter must survive and not receipt it)")
    for label, payload in [("unknown command id", {"Command ID": 99, "Packet ID": 13}),
                           ("bad patient coordinate", {"Command ID": 4, "Packet ID": 14, "Coordinate": [0.0, 0.0]}),
                           ("bad heartbeat status", {"Command ID": 1, "Packet ID": 15, "Connection Status": 9}),
                           ("JSON list instead of an object", [1, 2, 3]),
                           ("missing fields", {"Command ID": 3, "Packet ID": 16})]:
        send(payload)
    send_raw(b"this is not json {{{")
    t, _ = latest(1.0)
    check("none of those were receipted (still 2/12)", t and (t["CommandID"], t["PacketID"]) == (2, 12), t and (t["CommandID"], t["PacketID"]))
    check("adapter still running", adapter.poll() is None)
    send({"Command ID": 1, "Packet ID": 17, "Connection Status": 1})
    t, _ = latest()
    link = read_json(state_dir / "mission_state.json").get("gcs_link", {})
    check("...and still works afterwards (Heartbeat Unstable, 1/17)", link.get("status") == "Unstable" and (t["CommandID"], t["PacketID"]) == (1, 17))

    print("\nA broken state file")
    good = (state_dir / "telemetry_state.json").read_text()
    (state_dir / "telemetry_state.json").write_text("{ not valid json")
    time.sleep(1.2)
    check("adapter survives a corrupt telemetry_state.json", adapter.poll() is None)
    latest(0.3)                                 # drain what was queued before the corruption
    t, n = latest(1.0)
    (state_dir / "telemetry_state.json").write_text(good)
    t2, n2 = latest(1.5)
    check("telemetry resumes once the file is fixed", n2 >= 4 and t2 and t2["BatteryLife"] == 87.0, f"{n2} packets")

finally:
    time.sleep(0.2)
    adapter.send_signal(signal.SIGINT)
    try:
        adapter.wait(timeout=5)
        code = adapter.returncode
    except subprocess.TimeoutExpired:
        adapter.kill()
        code = None
    time.sleep(0.2)
    cmd.close(linger=0); tlm.close(linger=0); ctx.term()

print("\nShutdown")
check("Ctrl+C stops the adapter cleanly (exit code 0)", code == 0, code)
text = "\n".join(lines)
check("no Python error (Traceback) in the adapter's output", "Traceback" not in text)

print("\n--- adapter output (commands only) ---")
for l in lines:
    if l.startswith(("Received", "Executing", "Unknown", "JSON Error", "[Adapter]", "AddZone", "PatientLocation", "Heartbeat", "Command rejected", "[NAV_STATE")):
        print("   ", l[:150])

print("\n" + "=" * 60)
print(f"{sum(results)} of {len(results)} checks passed")
print("RESULT: " + ("PASS" if all(results) else "FAIL"))
print("=" * 60)
sys.exit(0 if all(results) else 1)
