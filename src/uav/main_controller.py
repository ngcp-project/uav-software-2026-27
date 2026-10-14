import asyncio 
import logging
import time
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state #For the mission_state.json
from state.nav_state_utils import load_nav_state, update_nav_state #For waypoints and what not
from mavsdk import System

from pymavlink import mavutil

# Main controller for the UAV system, responsible for interacting with the flight controller via pymavlink,

#Configs and parameters 
# Enable INFO level logging by default so that INFO messages are shown
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S")

log = logging.getLogger("main_controller")

#Parameters
STATE_POLL_HZ = 2.0 #How often to check state file

TELEMETRY_TIMEOUT_S = 5 #How old telemetry can be before it is considered bad

FT_TO_M = 0.3048
MIN_ALT_FT = 50 #Min alt feet
LOITER_ALT_FT = 200 #TArget loiter altitutde 
LOITER_RADIUS_FT = 165 #Acceptance radius for the loiter waypoint (Big cuz fixed wing

RTL_REASON_GCS = "gcs_requested_rtl"
RTL_REASON_RC = "pilot_or_rc_requested_rtl"
RTL_REASON_LOW_BATTERY = "low_battery_failsafe"
RTL_REASON_AUTONOMY = "autonomy_safety_rtl"
RTL_REASON_UNKNOWN = "unknown_external_rtl"
RTL_REASON_MISSION_COMPLETE = "mission_complete_rtl"
rtl_reason = RTL_REASON_MISSION_COMPLETE

#From ARDUPILOT
ARDUPLANE_MODES = {
    "MANUAL": 0, 
    "CIRCLE": 1, 
    "STABILIZE": 2, 
    "TRAINING": 3,
    "ACRO": 4,  
    "FBWA": 5,   
    "FBWB": 6,     
    "CRUISE": 7,
    "AUTO": 10, 
    "RTL": 11,   
    "LOITER": 12,  
    "GUIDED": 15,
}


#PYMAVLINK Functions

#Heartbeat
def mav_connect(connection_string: str):
    # Opens a pymavlink connection
    mav = mavutil.mavlink_connection(connection_string)
    log.info("pymavlink: waiting for heartbeat on %s …", connection_string)
    mav.wait_heartbeat()
    log.info(
        "pymavlink: heartbeat received  system=%d  component=%d",
        mav.target_system, mav.target_component,
    )
    return mav

#Mode
def mav_set_mode(mav, mode_name: str) -> bool:
    mode_id = ARDUPLANE_MODES.get(mode_name.upper())
    if mode_id is None:
        raise ValueError(f"Unknown ArduPlane mode: {mode_name}")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    # Wait for heartbeat to echo the new mode (up to 5 tries)
    for _ in range(5):
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            log.info("pymavlink: mode set to %s (%d)", mode_name, mode_id)
            return True
    log.warning("pymavlink: mode ACK not confirmed for %s", mode_name)
    return False

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
                param2=100.0,
                param3=0,
                param4=float("nan"),
                x=lat_i,
                y=lon_i,
                z=alt_m,
            )
        )

    items.append(make_do_jump_item(target_index=1, repeat_count=-1))

    return items




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


def mav_upload_plan(mav, plan: dict) -> None:
    plan_type = plan.get("plan_type")

    if plan_type == "single_loiter":
        items = build_loiter_items(plan)
    elif plan_type == "waypoint_pattern":
        items = build_waypoint_items(plan)
    else:
        raise ValueError(f"Unknown plan_type: {plan_type}")
    
    log.info("========== GENERATED MISSION ITEMS ==========")
    log.info("[MISSION] Uploading plan_type=%s with %d items", plan_type, len(items))

    for i, item in enumerate(items):
        command = item["command"]

        if command == mavutil.mavlink.MAV_CMD_DO_JUMP:
            log.info(
                "ITEM%02d: DO_JUMP target_index=%s repeat_count=%s",
                i,
                item["param1"],
                item["param2"],
            )
            continue

        lat = item["x"] / 1e7
        lon = item["y"] / 1e7
        alt_m = item["z"]
        alt_ft = alt_m / FT_TO_M

        log.info(
            "ITEM%02d: command=%s current=%s lat=%.7f lon=%.7f "
            "alt_m=%.1f alt_ft=%.1f p1=%s p2=%s p3=%s p4=%s",
            i,
            command,
            item["current"],
            lat,
            lon,
            alt_m,
            alt_ft,
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
        )

    log.info("=============================================")


    mav_upload_mission_items(mav, items)
    # temp comment for debugging
    mav_set_current_mission_item(mav, 0)

def mav_set_current_mission_item(mav, seq: int = 0) -> None:
    mav.mav.mission_set_current_send(
        mav.target_system,
        mav.target_component,
        seq,
    )
    log.info("Set current mission item to seq=%d", seq)

    msg = mav.recv_match(type="MISSION_CURRENT", blocking=True, timeout=2)
    if msg is not None:
        log.info("[MISSION CURRENT CONFIRM] seq=%s after reset request", msg.seq)
    else:
        log.warning("[MISSION CURRENT CONFIRM] no MISSION_CURRENT received after reset request")



def mav_start_mission(mav) -> None:
    """Switch to AUTO so ArduPilot executes the uploaded mission."""
    mav_set_mode(mav, "AUTO")
    log.info("pymavlink: AUTO mode commanded — mission executing")

def mav_loiter_in_place(mav) -> None:
    mav_set_mode(mav, "LOITER")
    log.info("pymavlink: LOITER mode commanded")



def set_rtl_reason(reason: str, source: str = "main_controller", details: dict | None = None):
 
    # Stores the most likely reason RTL was requested.
  
    state = load_state()
    controller_status = state.get("controller_status", {})

    rtl_event = {
        "reason": reason,
        "source": source,
        "timestamp": time.time(),
        "details": details or {},
    }

    update_state("controller_status", {
        **controller_status,
        "rtl_reason": reason,
        "last_rtl_event": rtl_event,
    })

    log.warning(f"RTL reason set: {reason} | source={source} | details={details or {}}")

async def run():
    #Connection
    drone = System()
    
    
    #For Real
    # await drone.connect(system_address="udp://:14604")

    #For Sitl
    await drone.connect(system_address="udpin://0.0.0.0:14604")
  
    log.info("Waiting for autopilot connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            log.info("Plane connected.")
            break
    
    log.info("Waiting for plane to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log.info("Global position estimate OK")
            break
    

    #Heartbeat for pymavlink (Will need to change the UDP port when incorporated everything else)
    mav = mav_connect("udpin:0.0.0.0:14603")
    

    autonomy_active = False
    controller_status = load_state().get("controller_status", {})
    mission_status = load_state().get("mission_status", {})
    
    update_state("controller_status", {
        **controller_status,
        "autonomy_active": False,
        "safety_hold": None,
        "last_heartbeat_utc": None,
    })

    nav_state = load_nav_state()
    active_plan = nav_state.get("active_plan", {})

    if active_plan.get("plan_id") == 1 and active_plan.get("status") == "error":
        log.warning("[STARTUP] Mission 1 was in error state. Resetting to ready for retry.")
        update_nav_state("active_plan", {
            **active_plan,
            "status": "ready",
        })
    last_telemetry_time = time.time()
    state_period_s = 1.0 / STATE_POLL_HZ
    last_state_check = 0.0
    last_executed_plan_id = None
    last_status_log = 0

    controller_status = load_state().get("controller_status", {})
    autonomy_active = controller_status.get("autonomy_active", False)
    last_fc_mode = controller_status.get("fc_mode")

    #We need telemetry because minimum height is needed for autonomy and loiter height is set
    telemetry = {
        "rel_alt_m": 0.0,
        "flight_mode": None,
        "armed": False,
    }

    async def watch_telemetry():
        nonlocal last_telemetry_time
        async for pos in drone.telemetry.position():
            telemetry["rel_alt_m"] = pos.relative_altitude_m
            last_telemetry_time = time.time()

    async def watch_flight_mode():
        async for mode in drone.telemetry.flight_mode():
            telemetry["flight_mode"] = str(mode)

    async def watch_armed():
        async for is_armed in drone.telemetry.armed():
            telemetry["armed"] = is_armed

    asyncio.ensure_future(watch_telemetry())
    asyncio.ensure_future(watch_flight_mode())
    asyncio.ensure_future(watch_armed())

    log.info("Background telemetry listeners started. Entering main control loop.")


    #MAIN CONTROL LOOP

    while True:
        loop_start = time.time()

        if loop_start - last_state_check < state_period_s:
            await asyncio.sleep(0.05)
            continue
            
        last_state_check = loop_start

        state = load_state()

        controller_status = state.get("controller_status", {})
        fc_mode = controller_status.get("fc_mode")
        mode_changed = (fc_mode != last_fc_mode)

        previous_fc_mode = last_fc_mode

        if mode_changed:
            log.info("FC mode transition: %s -> %s", previous_fc_mode, fc_mode)
            last_fc_mode = fc_mode

        if mode_changed and fc_mode == "LOITER":
            log.info("LOITER transition detected. Pausing autonomy.")
            autonomy_active = False

            controller_status = load_state().get("controller_status", {})
            mission_status = load_state().get("mission_status", {})

            update_state("controller_status", {
                **controller_status,
                "autonomy_active": False,
                "safety_hold": "loiter",
            })
            controller_status = load_state().get("controller_status", {})
            mission_status = load_state().get("mission_status", {})

            update_state("mission_status", {
                **mission_status,
                "current_mode": "Loitering",
            })

            await asyncio.sleep(state_period_s)
            continue

        elif mode_changed and fc_mode == "RTL":
            log.info("RTL transition detected. Pausing autonomy.")
            state = load_state()

            controller_status = load_state().get("controller_status", {})
            mission_status = load_state().get("mission_status", {})

            existing_rtl_reason = controller_status.get("rtl_reason")

            if existing_rtl_reason:
                rtl_reason = existing_rtl_reason
            else:
                rtl_reason = RTL_REASON_MISSION_COMPLETE

            log.warning(f"RTL transition detected. Pausing autonomy. Reason: {rtl_reason}")

            autonomy_active = False

            rtl_event = {
                "reason": rtl_reason,
                "source": "flight_mode_transition",
                "timestamp": time.time(),
                "fc_mode": fc_mode,
                "previous_fc_mode": previous_fc_mode,
            }

            update_state("controller_status", {
                **controller_status,
                "safety_hold": "rtl",
                 "rtl_reason": rtl_reason,
                "last_rtl_event": rtl_event,
            })

            controller_status = load_state().get("controller_status", {})
            mission_status = load_state().get("mission_status", {})

            update_state("mission_status", {
                **mission_status,
                "current_mode": "RTL",
            })

        controller_status = load_state().get("controller_status", {})
        mission_status = load_state().get("mission_status", {})

        update_state("controller_status", {
            **controller_status,
            "autonomy_active": autonomy_active,
            "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        })

        nav_state = load_nav_state()
        active_plan = nav_state.get("active_plan", {})

        mission_status = state.get("mission_status", {})
        should_autonomy = state.get("autonomy_command", False)
       

        
        rtl_requested = state.get("rtl_requested", False)

        mode_str = telemetry.get("flight_mode") or "UNKNOWN"

        plan_id = active_plan.get("plan_id")
        plan_type = active_plan.get("plan_type")
        plan_status = str(active_plan.get("status", "")).strip()
         #Minimum height safety check

        rel_alt = telemetry.get("rel_alt_m", 0.0)
        armed   = telemetry.get("armed", False)

        reupload_requested = active_plan.get("reupload_requested", False)

        upload_condition = (
            plan_id is not None
            and (
                plan_status in ("ready", "error")
                or reupload_requested
            )
            and (
                plan_id != last_executed_plan_id
                or reupload_requested
            )
        )

        if upload_condition:
            log.info("New plan ready to upload: id=%s type=%s", plan_id, plan_type)

            log.info("========== ACTIVE PLAN DEBUG ==========")
            log.info("plan_id=%s", active_plan.get("plan_id"))
            log.info("plan_type=%s", active_plan.get("plan_type"))
            log.info("label=%s", active_plan.get("label"))
            log.info("status=%s", active_plan.get("status"))
            log.info("num_waypoints=%d", len(active_plan.get("waypoints", [])))

            for i, wp in enumerate(active_plan.get("waypoints", [])):
                log.info(
                    "PLAN_WP%02d: lat=%s lon=%s alt_ft=%s alt_m=%s",
                    i + 1,
                    wp.get("lat"),
                    wp.get("lon"),
                    wp.get("alt_ft"),
                    wp.get("alt_m"),
                )

            log.info("=======================================")

            try:
                log.info(
                    "[MISSION EXEC] Uploading plan_id=%s type=%s status=%s",
                    plan_id,
                    plan_type,
                    plan_status,
                )

                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: mav_upload_plan(mav, active_plan),
                )

                log.info("[MISSION EXEC] Upload finished for plan_id=%s", plan_id)

                last_executed_plan_id = plan_id

                nav_state = load_nav_state()

                update_nav_state("active_plan", {
                    **active_plan,
                    "status": "uploaded",
                    "reupload_requested": False,
                })

                update_state("mission_status", {
                    **mission_status,
                    "active_plan_id": plan_id,
                    "current_mode": "MissionUploaded",
                })

                log.info("[MISSION EXEC] Plan %s uploaded. Waiting for RC/QGC AUTO.", plan_id)




            except Exception as exc:
                log.error("Failed to upload plan %s: %s", plan_id, exc)

                update_nav_state("active_plan", {
                    **active_plan,
                    "status": "error",
                })

                update_state("mission_status", {
                    **mission_status,
                    "active_plan_id": plan_id,
                    "current_mode": "PlanError",
                })

        if time.time() - last_status_log > 5:
            log.info(
                "[CTRL STATUS] should_autonomy=%s autonomy_active=%s armed=%s alt=%.1f mode=%s",
                should_autonomy,
                autonomy_active,
                telemetry.get("armed"),
                telemetry.get("rel_alt_m", -1),
                mode_str,
            )

            log.info(
                "[PLAN GATE] plan_id=%s status=%s type=%s last_executed_plan_id=%s autonomy_active=%s armed=%s alt_m=%.1f",
                plan_id,
                plan_status,
                plan_type,
                last_executed_plan_id,
                autonomy_active,
                armed,
                rel_alt,
            )
            last_status_log = time.time()

        loiter_requested = state.get("loiter_requested", False)
        #Loiter from GCS 
        if loiter_requested:
            log.info("Loitering")
            autonomy_active = False
            controller_status = load_state().get("controller_status", {})
            mission_status = load_state().get("mission_status", {})
            update_state("controller_status", {
                **controller_status,
                "autonomy_active": False, 
                
            })

            

            if mission_status.get("current_mode") != "Loitering":
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: mav_loiter_in_place(mav),
                )

            

            mission_status = load_state().get("mission_status", {})

            update_state("mission_status", {
                **mission_status,
                "current_mode": "Loitering",
            })

            update_state("loiter_requested", False)
            await asyncio.sleep(state_period_s)
            continue

        #RTL !
        if rtl_requested:
            log.info("RTL requested. Commanding return to launch.")
            set_rtl_reason(
                RTL_REASON_GCS,
                source="main_controller",
                details={"trigger": "rtl_requested_flag"},
            )

            try:
                await drone.action.return_to_launch()
                update_state("rtl_requested", False)
                autonomy_active = False
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "rtl",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "RTL",
                })
                log.info("RTL command accepted. Autonomy paused.")
            except Exception as exc:
                log.error("RTL command failed: %s", exc)
            await asyncio.sleep(state_period_s)
            continue

        #Telemetry for safety 
        telemetry_age = time.time() - last_telemetry_time
        if telemetry_age > TELEMETRY_TIMEOUT_S:
            log.warning(
                "Telemetry stale (%.1f s). Pausing autonomy until link recovers.",
                telemetry_age,
            )
            if autonomy_active:
                autonomy_active = False
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "telemetry_lost",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })
            await asyncio.sleep(state_period_s)
            continue


        #Manual Override
        flight_mode = telemetry.get("flight_mode", "")
        manual_override_modes = {
            "MANUAL",
            "STABILIZE",
            "FBWA",
            "FBWB",
            "CRUISE",
            "TRAINING",
            "ACRO",
            "FlightMode.MANUAL",
            "FlightMode.STABILIZE",
            "FlightMode.FBWA",
            "FlightMode.FBWB",
            "FlightMode.CRUISE",
            "FlightMode.TRAINING",
            "FlightMode.ACRO",
        }
        pilot_in_control = flight_mode in manual_override_modes

        if pilot_in_control:
            if autonomy_active or mission_status.get("current_mode") != "Paused_PilotOverride":
                log.info("Pilot Override detected. Pausing Autonomy (mode=%s)", flight_mode)

                autonomy_active = False
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "pilot_override",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })
            await asyncio.sleep(state_period_s)
            continue

        current_mode = mission_status.get("current_mode", "")
        if (
            not pilot_in_control
            and current_mode == "Paused_PilotOverride"
            and should_autonomy
        ):
            log.info("Autopilot mode restored. Resuming autonomy.")
            update_state("mission_status", {**mission_status, "current_mode": "Idle"})

        rtl_modes = {
            "RTL",
            "RETURN_TO_LAUNCH",
            "FlightMode.RETURN_TO_LAUNCH",
        }

        if flight_mode in rtl_modes:
            if autonomy_active:
                log.info("RTL/RETURN_TO_LAUNCH detected. Pausing autonomy.")
                autonomy_active = False

                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "rtl",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })

                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "RTL",
                })

            await asyncio.sleep(state_period_s)
            continue

        auto_modes = {
            "MISSION",
            "AUTO",
            "FlightMode.MISSION",
            "FlightMode.AUTO",
        }

        if flight_mode in auto_modes:
            if plan_status not in ("uploaded", "running", "ready", "building"):
                log.warning(
                    "AUTO/MISSION detected, but active_plan status is %s. "
                    "Not marking autonomy active because no valid mission is confirmed uploaded.",
                    plan_status,
                )

                autonomy_active = False

                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "mission_not_uploaded",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })

                await asyncio.sleep(state_period_s)
                continue

            if not autonomy_active:
                log.info("AUTO/MISSION detected from RC/QGC. Autonomy is now active.")

            autonomy_active = True
            if not autonomy_active:
                log.info("AUTO/MISSION detected from RC/QGC. Autonomy is now active.")

            autonomy_active = True

            controller_status = load_state().get("controller_status", {})
            update_state("controller_status", {
                **controller_status,
                "autonomy_active": True,
                "safety_hold": None,
                "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            })

            if plan_status == "uploaded":
                update_nav_state("active_plan", {
                    **active_plan,
                    "status": "running",
                })

                update_state("mission_status", {
                    **mission_status,
                    "active_plan_id": plan_id,
                    "current_mode": "Executing",
                })

                log.info("Plan %s is now running because aircraft entered AUTO/MISSION.", plan_id)

        if not autonomy_active:
            await asyncio.sleep(state_period_s)
            continue

        msg = mav.recv_match(type="MISSION_CURRENT", blocking=False)
        if msg is not None:
            log.info("[MISSION CURRENT] seq=%s", msg.seq)

        if not armed:
            log.debug("Aircraft not armed. Waiting.")
            await asyncio.sleep(state_period_s)
            continue

        min_alt_m = MIN_ALT_FT * FT_TO_M

        if rel_alt < min_alt_m:
            log.debug(
                "Altitude %.1f m / %.1f ft is below minimum %.1f ft. Waiting for climb.",
                rel_alt,
                rel_alt / FT_TO_M,
                MIN_ALT_FT,
            )
            await asyncio.sleep(state_period_s)
            continue

if __name__ == "__main__":
  asyncio.run(run())    
