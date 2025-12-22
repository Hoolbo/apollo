#!/usr/bin/env python3
import os
import pty
import time
import serial
import fcntl

def calculate_checksum(sentence):
    checksum = 0
    for char in sentence[1:]:
        checksum ^= ord(char)
    return hex(checksum)[2:].zfill(2).upper()

def generate_gpfpd(lat, lon, heading):
    # $GPFPD,GPSWeek,GPSTime,Heading,Pitch,Roll,Lattitude,Longitude,Altitude,Ve,Vn,Vu,Baseline,NSV,Status*Checksum
    sentence = f"$GPFPD,2200,440000.000,{heading:.3f},0.000,0.000,{lat:.8f},{lon:.8f},50.000,0.000,0.000,0.000,0.0,12,3"
    checksum = calculate_checksum(sentence)
    return f"{sentence}*{checksum}\r\n"

def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"Mock GPS Serial Port: {slave_name}")
    print("Please update /apollo_workspace/modules/drivers/mycar_pos/conf/mycar_pos.conf with this device path.")

    # Set non-blocking on master
    fl = fcntl.fcntl(master, fcntl.F_GETFL)
    fcntl.fcntl(master, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    # Initial position (near Borregas Ave if using that map)
    # 37.415889, -122.014505
    lat = 37.415889
    lon = -122.014505
    heading = 0.0

    try:
        while True:
            # Generate and write GPS data
            gpfpd = generate_gpfpd(lat, lon, heading)
            try:
                os.write(master, gpfpd.encode())
                print(f"Sent: {gpfpd.strip()}")
            except OSError:
                pass

            # Simulate slow movement
            lat += 0.00001
            lon += 0.00001
            heading = (heading + 1.0) % 360.0

            time.sleep(0.1)  # 10Hz
    except KeyboardInterrupt:
        print("Stopping mock GPS...")

if __name__ == "__main__":
    main()
