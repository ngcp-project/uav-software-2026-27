
# NGCP Pixhawk ↔ Raspberry Pi 5 Communication


## On the PI5 (Companion Computer)
### Setting Up Mavlink Router
#### Downloading

Update and Install
```
sudo apt update
sudo apt install -y git meson ninja-build pkg-config libsystemd-dev
```
Cd into your home
```
cd ~
git clone https://github.com/mavlink-router/mavlink-router.git
```
You will need to pull the submodule mavlink_c_library_v2/ardupilotmega
```
cd ~/mavlink-router
git submodule update --init --recursive
```

Configure + build + install
```
meson setup build
ninja -C build
sudo ninja -C build install
```

#### Running Mavlink-Router on PI 5
```
cd ~/mavlink-router
```
```
mavlink-routerd \
  -e 127.0.0.1:14601 \
  -e 127.0.0.1:14602 \
  -e 127.0.0.1:14603 \
  -e 127.0.0.1:14604 \
  /dev/ttyAMA0:57600

```
14601 → command listener \
14602 → spare port \
14603 → spare port\
14604 → telemetry

ttyAMA0: → The UART Port
57600 → The Baud Rate

### command_listener also on PI5
#### Setting up
A virtual enviroment is needed for pymavlink
```
python3 -m venv mavsdk-venv
```

Activate it with

```
source mavsdk-venv/bin/activate
```
Download pymavlink and mavsdk with
```
pip install pymavlink mavsdk
```
cd into the proper directory (can be different depending on device and repo cloning)

```
cd /Projects/ngcp-uav-software/src/comms
```
Run
```
python3 command_listener.py
```

### Running telemetry_logger 
You also need a venv that has MAVSDK for this command as well. \
But this was downloaded earlier with pymavlink \
Activate it with

```
source mavsdk-venv/bin/activate
```
cd into the proper directory (can be different depending on device and repo cloning)

```
cd /Projects/ngcp-uav-software/src/loggers
```
Run
```
python3 telemetry_logger.py
```

## Ground Computer Side
#### Setting up again
A virtual enviroment is needed for the ground computer too
```
python3 -m venv mavsdk-venv
```
Activate it with

```
source mavsdk-venv/bin/activate
```
Download pymavlink 
```
pip install pymavlink 
```
cd into the proper directory (can be different depending on device and repo cloning)

```
cd /Projects/ngcp-uav-software/src/comms
```
Run
```
python3 ground_sender.py
```
What it should say on terminal
```
> Connecting to RFD on /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HL2Y-if00-port0 
> Waiting for heartbeat (timeout 30s)
> Heartbeat received from system=x component=x
> Ground console ready.
```
Type command 
```
> start_log
```
```
> stop_log
```
And the command should be sent to the listener