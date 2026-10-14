#!/usr/bin/env bash

VENV="$HOME/mavsdk-venv/bin/activate"
PROJECT_DIR="$HOME/Projects/ngcp-uav-software"
ARDUPILOT_DIR="$HOME/ardupilot"
MAVLINK_ROUTER_DIR="$HOME/mavlink-router"
QGC_APP="$HOME/Downloads/QGroundControl-x86_64.AppImage"

open_terminal() {
    local title="$1"
    local command="$2"

    gnome-terminal --title="$title" -- bash -lc "
        source '$VENV'
        $command
        echo
        echo '[$title exited]'
        echo 'Press Enter to close this terminal...'
        read
    "
}

open_terminal "1 - ArduPilot SITL" "
    cd '$ARDUPILOT_DIR'
./Tools/autotest/sim_vehicle.py -v ArduPlane --no-mavproxy -l 33.932235,-117.631915,100,90 -A '--serial0=udpclient:127.0.0.1:14551'"

sleep 5

open_terminal "2 - MAVLink Router" "
    cd '$MAVLINK_ROUTER_DIR'
    mavlink-routerd \
      -e 127.0.0.1:14550 \
      -e 127.0.0.1:14601 \
      -e 127.0.0.1:14602 \
      -e 127.0.0.1:14603 \
      -e 127.0.0.1:14604 \
      -e 127.0.0.1:14605 \
      -e 127.0.0.1:14606 \
      -e 127.0.0.1:14607 \
      -e 127.0.0.1:14608 \
      0.0.0.0:14551
"

sleep 3

open_terminal "3 - QGroundControl" "
    cd '$HOME/Downloads'
    '$QGC_APP'
"

sleep 2

open_terminal "4 - Telemetry Logger" "
    cd '$PROJECT_DIR'
    python3 src/loggers/telemetry_logger.py
"

sleep 1

open_terminal "5 - Sim Kraken Data" "
    cd '$PROJECT_DIR'
    python3 src/loggers/SimKrakenData.py
"

sleep 1

open_terminal "6 - Fusion Logger" "
    cd '$PROJECT_DIR'
    python3 src/loggers/fusion_logger.py
"

sleep 1

open_terminal "7 - FC Mode Listener" "
    cd '$PROJECT_DIR'
    python3 src/uav/fc_mode_listener.py
"

sleep 1

open_terminal "8 - Mission 1 Waypoint Builder" "
    cd '$PROJECT_DIR'
    python3 src/autonomy/mission1_waypoint.py
"

sleep 1

open_terminal "9 - Main Controller" "
    cd '$PROJECT_DIR'
    python3 src/uav/main_controller.py
"

sleep 1

open_terminal "10 - GCS Simulator" "
    cd '$PROJECT_DIR'
    python3 scripts/gcssimulator.py
"