#!/usr/bin/env bash

echo "Stopping NGCP simulation stack..."

pkill -f "sim_vehicle.py"
pkill -f "mavlink-routerd"
pkill -f "QGroundControl"
pkill -f "telemetry_logger.py"
pkill -f "SimKrakenData.py"
pkill -f "fusion_logger.py"
pkill -f "fc_mode_listener.py"
pkill -f "mission1_waypoint.py"
pkill -f "main_controller.py"
pkill -f "gcssimulator.py"

echo "Done."
