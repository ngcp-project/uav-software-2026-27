#!/usr/bin/env python3
"""Search autonomy over LIA:  lawnmower  ->  fly the whole thing  ->  go to the target and circle it.

What it does
  1. waits for the first telemetry.position (or uses --home),
  2. publishes the stage-1 lawnmower plan (cmd.submit_plan) and waits for the flight executor's ack,
  3. watches telemetry.position while the aircraft is in AUTO; the lawnmower counts as finished when
     the aircraft has passed every waypoint in order,
  4. fits the target position from all the signal readings and publishes a single_loiter plan there,
  5. keeps listening while circling; re-fits twice and moves the loiter if the estimate shifts > 15 m.

Takeoff and the switch to AUTO are done by the pilot / QGC (see main_controller.py); this script never
arms or changes flight mode.

Signal strength
  --rssi-source sim     hidden simulated transmitter at --target EAST,NORTH metres from home (default)
  --rssi-source topic   read it from a LIA topic (--rssi-topic). The real topic name / value format is NOT
                        confirmed yet: a float, or a dict with rssi_dbm / rssi / dbm, is accepted.

Run (from the repo root, with lia.py and fc_listener.py + uav/main.py already running):
    python3 autonomy/autonomy_runner.py --target=-150,-130
"""
import argparse
import math
import sys
import time
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", category=FutureWarning)
sys.path.append(str(Path(__file__).resolve().parents[1]))

from lia.lia_client import LiaClient  # noqa: E402

import autonomy_plans as ap  # noqa: E402
import search_skeleton as sk  # noqa: E402

WP_REACHED_M = 90.0        # a waypoint counts as passed inside this distance (ArduPlane's default WP_RADIUS)
ACK_TIMEOUT_S = 15.0
MOVE_REPUBLISH_M = 15.0    # re-publish the loiter only if the refit moved the target this much


def _rssi_value(v):
    if isinstance(v, (int, float)):
        return float(v)
    if isinstance(v, dict):
        for k in ("rssi_dbm", "rssi", "dbm", "value"):
            if isinstance(v.get(k), (int, float)):
                return float(v[k])
    return None


class Runner:
    def __init__(self, a):
        self.a = a
        self.client = LiaClient("autonomy")
        self.phase = "WAIT_POS"
        self.pos = None                # (lat, lon, rel_alt_m)
        self.mode, self.armed = None, False
        self.home_ll = tuple(a.home) if a.home else None
        self.origin = None
        self.wps_ll, self.wps_xy, self.idx = [], [], 0
        self.readings, self.best = [], (-999.0, None)
        self.recent = []
        self.rssi_topic_value = None
        self.sim = None
        self.cmd_id, self.cmd_sent = None, 0.0
        self.orbit = None
        self.ack = {}
        self.t_last_print = 0.0

    # ---- helpers
    def xy(self, lat, lon):
        return sk.to_local(lat, lon, *self.origin)

    def log(self, msg):
        print(f"[RUNNER] {msg}", flush=True)

    def send(self, plan):
        self.cmd_id = ap.publish_plan(plan)
        self.cmd_sent = time.time()

    def read_rssi(self, x, y, alt):
        if self.a.rssi_source == "sim":
            return self.sim.read(x, y, alt)
        return self.rssi_topic_value

    def record(self, x, y, rssi):
        if rssi is None:
            return
        self.readings = (self.readings + [(x, y, rssi)])[-4000:]
        self.recent = (self.recent + [rssi])[-sk.SMOOTH_N:]
        smooth = sum(self.recent) / len(self.recent)
        if smooth > self.best[0]:
            self.best = (smooth, (x, y))

    # ---- message handling
    def handle(self, m):
        topic, value = m.get("topic"), m.get("value")
        if topic == "telemetry.position" and isinstance(value, dict):
            self.pos = (value["latitude"], value["longitude"], value.get("relative_altitude_m", 0.0))
            self.on_position()
        elif topic == "telemetry.fc_mode" and isinstance(value, str):
            if value != self.mode:
                self.log(f"flight mode: {self.mode} -> {value}")
            self.mode = value
        elif topic == "telemetry.armed":
            self.armed = bool(value)
        elif topic == "cmd.plan_ack" and isinstance(value, dict):
            self.ack[value.get("command_id")] = value
        elif topic == self.a.rssi_topic and self.a.rssi_source == "topic":
            self.rssi_topic_value = _rssi_value(value)

    # ---- the state machine
    def on_position(self):
        lat, lon, alt = self.pos
        a = self.a

        if self.phase == "WAIT_POS":
            self.home_ll = self.home_ll or (lat, lon)
            _area, _ko, self.origin = ap.load_area(self.home_ll, a.boundary, a.true_coords)
            plan = ap.generate_stage1_plan(self.home_ll, a.boundary, a.true_coords, a.pattern,
                                           a.spacing, plan_id="search_001")
            self.wps_ll = [(w["lat"], w["lon"]) for w in plan["waypoints"]]
            self.wps_xy = [self.xy(*p) for p in self.wps_ll]
            if a.rssi_source == "sim":
                self.sim = sk.SimRssi(tuple(a.target))
                t_lat, t_lon = sk.from_local(a.target[0], a.target[1], *self.origin)
                self.log(f"[SIM] hidden target at E{a.target[0]:.0f} N{a.target[1]:.0f} m "
                         f"(lat {t_lat:.6f}, lon {t_lon:.6f})")
            self.log(f"stage 1: lawnmower, {len(self.wps_ll)} waypoints; publishing plan")
            self.send(plan)
            self.phase = "WAIT_ACK1"
            return

        x, y = self.xy(lat, lon)
        rssi = self.read_rssi(x, y, alt)

        if self.phase in ("WAIT_ACK1", "WAIT_ACK2"):
            ack = self.ack.get(self.cmd_id)
            if ack:
                if ack.get("status") != "accepted":
                    self.log(f"ERROR: plan {ack.get('plan_id')} {ack.get('status')}: {ack.get('reason')}")
                    self.phase = "ERROR"
                    return
                if self.phase == "WAIT_ACK1":
                    self.log("lawnmower accepted by the flight executor. "
                             "Take off and switch to AUTO (pilot / QGC); I will follow the aircraft.")
                    self.phase = "LAWNMOWER"
                else:
                    self.log("loiter plan accepted by the flight executor")
                    self.phase = "ORBIT"
                    self.orbit["since"] = time.time()
            elif time.time() - self.cmd_sent > ACK_TIMEOUT_S:
                self.log("ERROR: no ack from the flight executor (is uav/main.py running?)")
                self.phase = "ERROR"
            if self.phase != "ORBIT":
                if self.phase == "WAIT_ACK2":
                    self.record(x, y, rssi)
                return

        if self.phase == "LAWNMOWER":
            self.record(x, y, rssi)
            if self.mode == "AUTO":
                while self.idx < len(self.wps_xy) and \
                        math.hypot(x - self.wps_xy[self.idx][0], y - self.wps_xy[self.idx][1]) < WP_REACHED_M:
                    self.idx += 1
                    self.log(f"passed waypoint {self.idx}/{len(self.wps_xy)}")
            if self.idx >= len(self.wps_xy):
                self.finish_lawnmower(alt)
            elif time.time() - self.t_last_print > 5:
                self.t_last_print = time.time()
                self.log(f"lawnmower {self.idx}/{len(self.wps_xy)}  pos E{x:.0f} N{y:.0f}  "
                         f"best {self.best[0]:.1f} dBm  mode {self.mode}")
            return

        if self.phase == "WAIT_ACK2" or self.phase == "ORBIT":
            self.record(x, y, rssi)
        if self.phase == "ORBIT" and time.time() - self.orbit["since"] > a.refit_s:
            est = sk.estimate_target(self.readings, alt, self.orbit["center"])
            move = math.hypot(est[0] - self.orbit["center"][0], est[1] - self.orbit["center"][1])
            self.orbit["refits"] += 1
            err = ""
            if self.sim:
                err = f", true error {math.hypot(est[0] - a.target[0], est[1] - a.target[1]):.0f} m"
            self.log(f"refit {self.orbit['refits']}/{sk.ORBIT_REFITS}: target now "
                     f"E{est[0]:.0f} N{est[1]:.0f} (moved {move:.0f} m{err})")
            if move > MOVE_REPUBLISH_M:
                self.publish_loiter(est, alt, f"loiter_{self.orbit['refits'] + 1:03d}", wait=False)
            self.orbit["center"], self.orbit["since"] = est, time.time()
            if self.orbit["refits"] >= sk.ORBIT_REFITS:
                self.log("done: the aircraft keeps circling the estimated target")
                self.phase = "DONE"

    def finish_lawnmower(self, alt):
        if self.best[1] is None or self.best[0] < sk.RSSI_DETECT_DBM:
            self.log(f"lawnmower finished; the signal was never heard (best {self.best[0]:.1f} dBm). "
                     "Nothing to circle.")
            self.phase = "DONE"
            return
        est = sk.estimate_target(self.readings, alt, self.best[1])
        err = ""
        if self.sim:
            err = f" (true error {math.hypot(est[0] - self.a.target[0], est[1] - self.a.target[1]):.0f} m)"
        self.log(f"lawnmower finished; strongest signal {self.best[0]:.1f} dBm -> "
                 f"target estimated at E{est[0]:.0f} N{est[1]:.0f}{err}; publishing loiter plan")
        self.orbit = {"center": est, "since": time.time(), "refits": 0}
        self.publish_loiter(est, alt, "loiter_001", wait=True)

    def publish_loiter(self, est_xy, alt, plan_id, wait):
        lat, lon = sk.from_local(est_xy[0], est_xy[1], *self.origin)
        self.send(ap.generate_stage3_plan((lat, lon), plan_id=plan_id))
        if wait:
            self.phase = "WAIT_ACK2"

    def run(self):
        self.log(f"subscribed to LIA; waiting for telemetry.position (rssi source: {self.a.rssi_source})")
        for m in self.client.subscribe():
            if m.get("type") == "shutdown":
                self.log("LIA shutting down")
                break
            self.handle(m)
            if self.phase in ("DONE", "ERROR") and self.a.exit_when_done:
                break


def _ll(text):
    a, b = text.split(",")
    return float(a), float(b)


if __name__ == "__main__":
    ap_ = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap_.add_argument("--home", type=_ll, help="LAT,LON area origin (default: where the aircraft is at start)")
    ap_.add_argument("--boundary", help="GCS area JSON")
    ap_.add_argument("--true-coords", action="store_true", help="use the area's real lat/lon")
    ap_.add_argument("--pattern", default="lawnmower", choices=sorted(sk.PATTERN_BUILDERS))
    ap_.add_argument("--spacing", type=float, default=sk.LANE_SPACING_M)
    ap_.add_argument("--rssi-source", default="sim", choices=["sim", "topic"])
    ap_.add_argument("--rssi-topic", default="payload.rssi", help="LIA topic with the signal strength (UNCONFIRMED)")
    ap_.add_argument("--target", type=_ll, default=(-150.0, -130.0), help="sim: EAST,NORTH metres from home")
    ap_.add_argument("--refit-s", type=float, default=sk.ORBIT_REFIT_S, help="seconds of circling per refit")
    ap_.add_argument("--exit-when-done", action="store_true")
    Runner(ap_.parse_args()).run()
