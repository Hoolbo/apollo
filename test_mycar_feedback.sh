#!/bin/bash

# Mycar CAN Feedback Test Script
# This script simulates feedback messages from the vehicle to test Apollo's parsing logic.

echo "--- Mycar CAN Feedback Test Sequence ---"

# 1. Simulate Remote Control Mode (0x20C)
# DBC: ACU_Remote_control at Byte 7, bit 6; ACU_Receive_info at Byte 7, bit 3
# 0x48 = 01001000 (bits for Remote Control and Receive Info)
echo "[1/4] Sending 0x20C: Switching to AUTO mode (Remote Control ON)..."
cansend can0 20C#0000000000000048
sleep 1

# 2. Simulate Speed (0x23C)
# DBC: Factor 0.1. 100 (0x64) -> 10.0 km/h
echo "[2/4] Sending 0x23C: Simulating Speed = 10.0 km/h (~2.77 m/s)..."
cansend can0 23C#0000000000640000
sleep 1

# 3. Simulate Steering Angle (0x22C)
# DBC: Factor 1, Offset -1024. 1250 (0x04E2) -> 226 degrees
# Note: Percentage depends on max_steer_angle in vehicle_param.pb.txt (approx 470 deg)
echo "[3/4] Sending 0x22C: Simulating Angle = 226 deg..."
cansend can0 22C#0000000000E20401
sleep 2

# 4. Simulate Error State (0x20C)
# DBC: ACU_Error at Byte 0, bit 0.
echo "[4/4] Sending 0x20C: Triggering CHASSIS ERROR (ACU_Error = 1)..."
cansend can0 20C#0100000000000048

echo ""
echo "Test Sequence Finished."
echo "Please verify results in 'cyber_monitor' for Channel: /apollo/canbus/chassis"
