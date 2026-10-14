#!/bin/bash
# Route telemetry from RFD900 modem to ground control software (QGroundControl) and local Python scripts

RFD_PORT="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HL2Y-if00-port0"
RFD_BAUD="57600"

# QGroundControl standard port
QGC_OUT="--out udp:127.0.0.1:14550"

# Port for ground_sender.py
SCRIPT_OUT="--out udp:127.0.0.1:14601"

echo "Starting MAVProxy router..."
echo "Master: $RFD_PORT ($RFD_BAUD baud)"
echo "Out 1 (QGC): udp:127.0.0.1:14550"
echo "Out 2 (Scripts): udp:127.0.0.1:14601"

# Run MAVProxy in background or foreground depending on use case.
# We run it normally here so the terminal displays link statistics.
mavproxy.py --master="$RFD_PORT" --baudrate="$RFD_BAUD" $QGC_OUT $SCRIPT_OUT --daemon
