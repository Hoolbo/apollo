#!/bin/bash

DEVICE="/dev/ttyUSB0"
BAUD="115200"

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE not found."
    echo "Check connection or try checking dmesg | tail"
    exit 1
fi

# Configure serial port
echo "Configuring $DEVICE to $BAUD..."
stty -F $DEVICE $BAUD cs8 -cstopb -parenb

# Define commands based on the debugging document
# 1. Coordinate alignment: -y, x, z (Based on installation)
CMD_COORD='echo -e "$cmd,set,coordinate,-y,x,z*ff\r\n" > '$DEVICE
# 2. Heading offset: 0 (Based on installation)
CMD_HEAD='echo -e "$cmd,set,headoffset,0*ff\r\n" > '$DEVICE
# 3. Lever arm: gnss, -0.135, 0, 0.104 (Based on installation)
CMD_ARM='echo -e "$cmd,set,leverarm,gnss,-0.135,0,0.104*ff\r\n" > '$DEVICE
# 4. Nav mode settings
CMD_NAV1='echo -e "$cmd,set,navmode,FineAlign,off*ff\r\n" > '$DEVICE
CMD_NAV2='echo -e "$cmd,set,navmode,coarsealign,off*ff\r\n" > '$DEVICE
CMD_NAV3='echo -e "$cmd,set,navmode,dynamicalign,on*ff\r\n" > '$DEVICE
CMD_NAV4='echo -e "$cmd,set,navmode,gnss,double*ff\r\n" > '$DEVICE
CMD_NAV5='echo -e "$cmd,set,navmode,carmode,on*ff\r\n" > '$DEVICE
CMD_NAV6='echo -e "$cmd,set,navmode,dmicali,off*ff\r\n" > '$DEVICE
CMD_NAV7='echo -e "$cmd,set,navmode,zupt,on*ff\r\n" > '$DEVICE
CMD_NAV8='echo -e "$cmd,set,navmode,firmwareindex,0*ff\r\n" > '$DEVICE
CMD_OK='echo -e "$cmd,config,ok*ff\r\n" > '$DEVICE
# 6. Save config
CMD_SAVE='echo -e "$cmd,save,config*ff\r\n" > '$DEVICE

echo "Sending configuration commands..."

# Execute commands with slight delays to ensure processing
echo "Setting Coordinate..."
eval $CMD_COORD
sleep 1

echo "Setting Heading Offset..."
eval $CMD_HEAD
sleep 1

echo "Setting Lever Arm..."
eval $CMD_ARM
sleep 1

echo "Setting Nav Modes..."
eval $CMD_NAV1
sleep 0.2
eval $CMD_NAV2
sleep 0.2
eval $CMD_NAV3
sleep 0.2
eval $CMD_NAV4
sleep 0.2
eval $CMD_NAV5
sleep 0.2
eval $CMD_NAV6
sleep 0.2
eval $CMD_NAV7
sleep 0.2
eval $CMD_NAV8
sleep 0.2
eval $CMD_OK
sleep 1

echo "Saving Configuration..."
eval $CMD_SAVE
sleep 1

echo "Done. Please power cycle the GNSS device manually."
