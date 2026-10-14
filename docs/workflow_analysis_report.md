# Telemetry/Autonomy Workflow Analysis (feat/len branch)

## Overview of the Workflow
The `feat/len` branch establishes a clear separation of concerns between the companion computer (Raspberry Pi 5) onboard the UAV and the Ground Station (Commander Laptop).

### 1. Onboard the UAV (Raspberry Pi 5)
The Pi5 acts as the central hub onboard the aircraft. It connects directly to the Pixhawk flight controller via a physical UART connection (`/dev/ttyAMA0:57600`).
*   **MAVLink-Router:** It uses `mavlink-routerd` to take the serial stream from the Pixhawk and split it into several local UDP endpoints:
    *   `14601`: `command_listener.py`
    *   `14602`: `telemetry_logger.py`
    *   `14603 & 14604`: Telemetry & Spare
*   **Command Listener:** The `command_listener.py` script listens on `14601` for `COMMAND_LONG` MAVLink messages (like `START_LOG`, `START_AUTONOMY`). When it receives these commands, it updates a local `state.json` file.
*   **Telemetry Logger:** The `telemetry_logger.py` script listens on `14602`. It continuously polls the `state.json` file. When `logging_enabled` is set to True by the command listener, it begins recording high-frequency telemetry data directly from the Pixhawk stream.

### 2. On the Ground Station (Commander Laptop)
The ground station operator uses `ground_sender.py` to send commands to the UAV over the RFD900 modem connection.
*   The sender script targets the onboard companion computer component (`target_component=191`).
*   It sends commands like `start_log` and `auto_start` over the radio link, which the Pi5 receives and actions.

---

## Verification of MAVProxy Multiplexing
The introduction of `start_ground_mavproxy.sh` on the Commander Laptop **does not conflict** with or hinder the existing software telemetry/autonomy workflow.

1.  **No Port Conflicts:** You might notice that both `command_listener.py` (on the Pi5) and our new `start_ground_mavproxy.sh` (on the Ground Laptop) use UDP port `14601`. **This is perfectly safe and does not conflict**. Because these scripts run on completely separate physical machines (linked only by a transparent RF serial link), their `localhost` (`127.0.0.1`) networks are completely isolated.
2.  **Uninterrupted Data Flow:** MAVProxy is designed to act as a transparent MAVLink router. When `ground_sender.py` sends a command into local UDP `14601` on the laptop, MAVProxy routes it out the physical RFD900 serial port, over the air, into the Pi5's serial port, through `mavlink-routerd`, and finally into `command_listener.py`. MAVProxy handles the routing identically to how a direct serial connection would perform.
3.  **Simultaneous QGroundControl:** QGroundControl can now be opened alongside the Python `ground_sender.py` script. The laptop's MAVProxy pipes a perfect copy of the incoming telemetry stream straight to QGC's default listen port (`14550`), allowing full situational awareness for the operator while maintaining command capability.
