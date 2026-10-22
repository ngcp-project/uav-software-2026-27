import json
import logging
from enum import Enum
from pymavlink import mavutil
from pathlib import Path

log = logging.getLogger(__name__)

#References
FT_TO_M = 0.3048
LOITER_ALT_FT = 200
LOITER_RADIUS_FT = 165

MISSION_DIR = Path(__file__).resolve().parents[1] / "missions"
with open(MISSION_DIR / "test_mission.json") as f:
    plan = json.load(f)

class ExecutorStatus(str, Enum):
    IDLE = "IDLE"
    UPLOADING = "UPLOADING"
    UPLOADED = "UPLOADED"
    RUNNING = "RUNNING"
    REPLACING = "REPLACING"
    ERROR = "ERROR"


class FlightExecutor:

    def __init__(self, mav):
        # Existing pymavlink connection
        self.mav = mav

        # Observed ArduPilot state
        self.flight_mode = "UNKNOWN"

        # Mission information
        self.current_plan = None
        self.pending_plan = None

        # Executor status
        self.status = ExecutorStatus.IDLE

    @property
    def autonomy_enabled(self):
        #Only permit reactive autonomy in AUTO.
        return self.flight_mode == "AUTO"

    def update_flight_mode(self, mode):
        #Observes flight mode changes from ArduPilot and updates the internal state.
        previous = self.flight_mode
        self.flight_mode = mode.upper()

        if previous != self.flight_mode:
            log.info(
                "Flight mode: %s -> %s",
                previous,
                self.flight_mode
            )

    def get_status(self):
        return {
            "flight_mode": self.flight_mode, #ArduPilot flight mode
            "autonomy_enabled": self.autonomy_enabled, #Whether autonomy is enabled (i.e. flight mode is AUTO)
            "executor_status": self.status.value, #Current status of the executor (IDLE, UPLOADING, UPLOADED, RUNNING, REPLACING, ERROR)
            "active_plan_id": ( #ID of the currently active mission plan
                self.current_plan.get("plan_id")
                if self.current_plan else None
            ),
            "pending_plan_id": (
                self.pending_plan.get("plan_id")
                if self.pending_plan else None
            ),
        }


    def submit_plan(self, plan: dict) -> str:

        if not isinstance(plan, dict):
            raise TypeError("MissionPlan must be a dictionary")

        if not plan.get("plan_id"):
            raise ValueError("MissionPlan missing plan_id")

        if plan.get("plan_type") not in (
            "waypoint_pattern",
            "single_loiter",
        ):
            raise ValueError("Unsupported plan_type")

        if not isinstance(plan.get("waypoints"), list):
            raise ValueError("MissionPlan waypoints must be a list")

        if not plan["waypoints"]:
            raise ValueError("MissionPlan has no waypoints")

        # Store proposed mission.
        self.pending_plan = plan.copy()

        log.info(
            "[FLIGHT EXECUTOR] Received MissionPlan: %s",
            plan["plan_id"],
        )

        return plan["plan_id"]


    #Uses the function build_waypoint_items to convert a plan into MAVLink mission items, then uploads them to the autopilot.
    def upload_plan(self, plan):

        if self.status == ExecutorStatus.UPLOADING:
            raise RuntimeError("Upload already in progress")

        if not plan.get("plan_id"):
            raise ValueError("Missing plan_id")

        if not plan.get("waypoints"):
            raise ValueError("Mission has no waypoints")

        # Convert based on plan type
        plan_type = plan.get("plan_type")

        if plan_type == "waypoint_pattern":
            items = build_waypoint_items(plan)

        elif plan_type == "single_loiter":
            items = build_loiter_items(plan)

        else:
            raise ValueError(f"Unsupported plan type: {plan_type}")

        self.status = ExecutorStatus.UPLOADING

        try:
            # Existing helper must complete only
            # after ArduPilot accepts the mission.
            mav_upload_mission_items(self.mav, items)

        except Exception:
            self.status = ExecutorStatus.ERROR
            log.exception("Mission upload failed")
            raise

        self.current_plan = plan
        self.status = ExecutorStatus.UPLOADED

        log.info(
            "Mission %s accepted by ArduPilot",
            plan["plan_id"]
        )


#Translates a waypoint pattern plan into a list of MAVLink mission items.
def build_waypoint_items(plan: dict) -> list:
    waypoints = plan.get("waypoints", [])
    if not waypoints:
        raise ValueError("waypoint_pattern plan has no waypoints")

    items = []

    for i, wp in enumerate(waypoints):
        lat_i = int(float(wp["lat"]) * 1e7)
        lon_i = int(float(wp["lon"]) * 1e7)
        alt_ft = float(wp.get("alt_ft", plan.get("alt_ft", LOITER_ALT_FT)))
        alt_m = alt_ft * FT_TO_M

        items.append(
            dict(
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                current=1 if i == 0 else 0,
                autocontinue=1,
                param1=0,
                param2=0,
                param3=0,
                param4=float("nan"),
                x=lat_i,
                y=lon_i,
                z=alt_m,
            )
        )
    return items


# Builds a list of mission items for a single loiter point, including a waypoint to the loiter center and an unlimited loiter command
def build_loiter_items(plan: dict) -> list:
    waypoints = plan.get("waypoints", [])
    if not waypoints:
        raise ValueError("single_loiter plan has no waypoint")

    wp = waypoints[0]
    lat = float(wp["lat"])
    lon = float(wp["lon"])
    alt_ft = float(wp.get("alt_ft", plan.get("alt_ft", LOITER_ALT_FT)))    
    loiter_radius_ft = float(plan.get("loiter_radius_ft", LOITER_RADIUS_FT))

    alt_m = alt_ft * FT_TO_M
    loiter_radius_m = loiter_radius_ft * FT_TO_M

    lat_i = int(lat * 1e7)
    lon_i = int(lon * 1e7)

    return [
        # Fly to the loiter center first
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=1,
            autocontinue=1,
            param1=0,
            param2=0,
            param3=0,
            param4=float("nan"),
            x=lat_i,
            y=lon_i,
            z=alt_m,
        ),

        # Stay in this loiter forever until a new mission is uploaded
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=0,
            autocontinue=1,
            param1=0,
            param2=0,
            param3=loiter_radius_m,
            param4=float("nan"),
            x=lat_i,
            y=lon_i,
            z=alt_m,
        ),
    ]

def make_do_jump_item(target_index: int, repeat_count: int = -1) -> dict:
    return dict(
        command=mavutil.mavlink.MAV_CMD_DO_JUMP,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current=0,
        autocontinue=1,
        param1=target_index,
        param2=repeat_count,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0,
    )

#Tells Ardupilot how many mission items exists and uploads them one by one, waiting for requests and acknowledgements
def mav_upload_mission_items(mav, items: list) -> None:
    mav.mav.mission_count_send(
        mav.target_system,
        mav.target_component,
        len(items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

    ack = None
    while ack is None:
        msg = mav.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=5,
        )
        if msg is None:
            raise TimeoutError("No MISSION_REQUEST received — vehicle not responding")

        t = msg.get_type()
        if t in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = msg.seq

            if seq < 0 or seq >= len(items):
                raise RuntimeError(
                    f"Vehicle requested invalid mission seq={seq}, but only {len(items)} items exist"
                )
            
            item = items[seq]

            mav.mav.mission_item_int_send(
            mav.target_system,
            mav.target_component,
            seq,
            item["frame"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
            item["x"],
            item["y"],
            item["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        elif t == "MISSION_ACK":
            ack = msg

    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission rejected by vehicle: MAV_MISSION result={ack.type}")

    log.info("Mission accepted by ArduPilot (%d items)", len(items))

