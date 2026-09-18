#!/usr/bin/env bash
# =============================================================================
# ngcp-autostart.sh — Single unified autostart launcher for NGCP UAV software
# Place at: ~/ngcp-autostart.sh  (or any stable path)
# Triggered by: ~/.config/autostart/ngcp-autostart.desktop
#
# HOW TO ADD A NEW PROGRAM:
#   Copy one of the launch_* blocks below and append it in the desired position.
#   Each block follows the same pattern:
#       launch_<name> [delay_seconds]
#   Adjust DELAY inside the function if the program needs more startup time.
#
# HOW TO REORDER PROGRAMS:
#   Move the launch_* call lines in the "── Launch sequence ──" section.
#   Do NOT reorder the function definitions — only the calls at the bottom matter.
# =============================================================================

# ── Global config ─────────────────────────────────────────────────────────────

DISPLAY="${DISPLAY:-:0}"
export DISPLAY

# Working directory for UAV software Python scripts
UAV_SRC="/home/ngcp25/Projects/ngcp-uav-software/src"

# KrakenSDR install directory
KRAKEN_DIR="/home/ngcp25/krakensdr_doa"

# ngcp-mavproxy-telemetry script
MAVPROXY_TELEMETRY_SCRIPT="/home/ngcp25/work/ngcp-pixhawk-pi5-companion/scripts/ngcp-mavproxy-telemetry.sh"

# ── Helper: open a persistent gnome-terminal tab running a command ─────────────
# Usage: open_terminal <window-title> <bash-command>
open_terminal() {
    local title="$1"
    local cmd="$2"
    gnome-terminal --title="$title" -- bash -ic "$cmd; exec bash"
}

# ── Program launch functions ───────────────────────────────────────────────────
# Each function encapsulates one program.
# ADD NEW PROGRAMS by adding a new function here, then calling it in the sequence.

# 1. KrakenSDR DOA backend (requires sudo; runs as a background service)
launch_kraken_doa() {
    local delay="${1:-0}"
    sleep "$delay"
    cd "$KRAKEN_DIR" && sudo ./kraken_doa_start.sh &
}

# GCS bridge
launch_gcs_bridge() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "GCS Bridge" \
        "python3 $UAV_SRC/state/gcs_bridge.py" &
}

# Fusion logger
launch_fusion_logger() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "Fusion Logger" \
        "python3 $UAV_SRC/loggers/fusion_logger.py" &
}

# 2. Kraken data logger (Python, keeps terminal open)
launch_kraken_logger() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "Kraken Logger" \
        "python3 $UAV_SRC/loggers/kraken_logger.py" &
}

# 6. Telemetry data logger (Python, keeps terminal open)
launch_telemetry_logger() {
    local delay="${1:-10}"  # wait for telemetry pipeline to be ready
    sleep "$delay"
    open_terminal "Telemetry Logger" \
        "python3 $UAV_SRC/loggers/telemetry_logger.py" &
}

# 3. Ground-station command listener (Python, keeps terminal open)
launch_command_listener() {
    local delay="${1:-5}"   # brief pause so logger is up first
    sleep "$delay"
    open_terminal "Command Listener" \
        "python3 $UAV_SRC/comms/command_listener.py" &
}

# Fusion sender
# launch_fusion_sender() {
#     local delay="${1:-0}"
#     sleep "$delay"
#     open_terminal "Fusion Sender" \
#         "python3 $UAV_SRC/comms/fusion_sender.py" &
# }

# mra info sender
launch_info_sender() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "MRA Info Sender" \
        "python3 $UAV_SRC/comms/mra_info_sender.py" &
}

# 9. Main controller
launch_main_controller() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "Main Controller" \
        "python3 $UAV_SRC/uav/main_controller.py" &
}

# 10. System controller
launch_system_controller() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "System Controller" \
        "python3 $UAV_SRC/uav/system_controller.py" &
}


# Nav updater: copies requested update.json fields into navigation_state.json
launch_nav_updater() {
    local delay="${1:-7}"
    sleep "$delay"
    open_terminal "Nav Updater" \
        "python3 $UAV_SRC/autonomy/nav_updater.py" &
}

# 8. Flight controller mode listener
launch_fc_mode_listener() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "FC Mode Listener" \
        "python3 $UAV_SRC/uav/fc_mode_listener.py" &
}


# 4. MAVProxy autostart binary (no terminal window)
launch_mavproxy() {
    local delay="${1:-0}"
    sleep "$delay"
    /home/ngcp25/.local/bin/ngcp-mavproxy-autostart &
}

# 5. MAVProxy telemetry pipeline (keeps terminal open)
launch_mavproxy_telemetry() {
    local delay="${1:-0}"
    sleep "$delay"
    export DISPLAY=:0
    open_terminal "MAVProxy Telemetry" \
        "$MAVPROXY_TELEMETRY_SCRIPT" &
}

# 11. Mission 1 waypoint processor
launch_mission1_waypoint() {
    local delay="${1:-0}"
    sleep "$delay"
    open_terminal "Mission1 Waypoint" \
        "python3 $UAV_SRC/autonomy/mission1_waypoint.py" &
}

# 7. Firefox pointing at the KrakenSDR web UI
launch_firefox_kraken_ui() {
    local delay="${1:-20}"   # wait for the DOA backend to bind its port
    sleep "$delay"
    xdg-open "http://0.0.0.0:8080" &
}

# ── Launch sequence ────────────────────────────────────────────────────────────
# TO REORDER: move lines here. Each call is independent of function order above.
# Optional per-call delay argument (seconds) overrides the function default.

launch_kraken_doa          # 1. KrakenSDR DOA backend
launch_gcs_bridge          # 2. gcs_bridge.py
launch_fusion_logger       # 3. fusion_logger.py
launch_kraken_logger       # 4. kraken_logger.py
launch_telemetry_logger    # 5. telemetry_logger.py
launch_command_listener    # 6. command_listener.py
launch_nav_updater         # 7. nav_updater.py
#launch_fusion_sender       #  fusion_sender.py
launch_info_sender
launch_main_controller     # 8. main_controller.py
launch_system_controller   # 9. system_controller.py
launch_fc_mode_listener    # 10. fc_mode_listener.py
launch_mavproxy            # 11. ngcp-mavproxy-autostart
launch_mavproxy_telemetry  # 12. ngcp-mavproxy-telemetry.sh
launch_mission1_waypoint   # 13. mission1_waypoint.py
launch_firefox_kraken_ui   # 14. Firefox → http://0.0.0.0:8080



#launch_kraken_logger       # 2. kraken_logger.py
#launch_command_listener    # 3. command_listener.py             (default  5 s delay)
#launch_mavproxy            # 4. ngcp-mavproxy-autostart
#launch_mavproxy_telemetry  # 5. ngcp-mavproxy-telemetry.sh
#launch_telemetry_logger    # 6. telemetry_logger.py             (default 10 s delay)
#launch_firefox_kraken_ui   # 7. Firefox → http://0.0.0.0:8080  (default 10 s delay)


# Keep the launcher process alive so GNOME doesn't reap child terminals
wait
