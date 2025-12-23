#!/bin/bash

DEVICE="/dev/gps_serial"
BAUD="115200"

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE not found."
    echo "Check connection or try checking dmesg | tail"
    exit 1
fi

# Configure serial port permissions
echo "Setting permissions for $DEVICE..."
sudo chmod 666 $DEVICE

# Open file descriptor 3 for reading and writing to the device
# This prevents the port from closing and resetting between commands
exec 3<> $DEVICE

# Configure serial port using the file descriptor
# -F $DEVICE is still needed for stty to target the correct hardware port, 
# but keeping fd 3 open maintains the state.
echo "Configuring $DEVICE to $BAUD..."
stty -F $DEVICE $BAUD raw cs8 -cstopb -parenb

# Commands
# Using *ff as wildcard checksum as confirmed by user
CMD_COORD='$cmd,set,coordinate,-y,x,z*ff'
CMD_HEAD='$cmd,set,headoffset,0*ff'
CMD_ARM='$cmd,set,leverarm,gnss,-0.135,0,0.104*ff'
CMD_NAV1='$cmd,set,navmode,FineAlign,off*ff'
CMD_NAV2='$cmd,set,navmode,coarsealign,off*ff'
CMD_NAV3='$cmd,set,navmode,dynamicalign,on*ff'
CMD_NAV4='$cmd,set,navmode,gnss,double*ff'
CMD_NAV5='$cmd,set,navmode,carmode,on*ff'
CMD_NAV6='$cmd,set,navmode,dmicali,off*ff'
CMD_NAV7='$cmd,set,navmode,zupt,on*ff'
CMD_NAV8='$cmd,set,navmode,firmwareindex,0*ff'
CMD_OK='$cmd,config,ok*ff'
CMD_SAVE='$cmd,save,config*ff'

echo "Sending configuration commands..."

send_cmd() {
    echo -e "$1\r" >&3
    # Optional: Read response if needed, but for now we just wait specifically
    sleep 0.2
}

echo "Setting Coordinate..."
send_cmd "$CMD_COORD"
sleep 1

echo "Setting Heading Offset..."
send_cmd "$CMD_HEAD"
sleep 1

echo "Setting Lever Arm..."
send_cmd "$CMD_ARM"
sleep 1

echo "Setting Nav Modes..."
send_cmd "$CMD_NAV1"
send_cmd "$CMD_NAV2"
send_cmd "$CMD_NAV3"
send_cmd "$CMD_NAV4"
send_cmd "$CMD_NAV5"
send_cmd "$CMD_NAV6"
send_cmd "$CMD_NAV7"
send_cmd "$CMD_NAV8"

echo "Confirming Config..."
send_cmd "$CMD_OK"
sleep 1

echo "Saving Configuration..."
send_cmd "$CMD_SAVE"
sleep 1

# Close file descriptor
exec 3>&-

echo "Done. Please power cycle the GNSS device manually."
