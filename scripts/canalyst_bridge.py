#!/usr/bin/env python3
"""
CANalyst-II → SocketCAN (can0) 桥接脚本

将 CANalyst-II 的 USB HID 接口桥接到 Linux SocketCAN 接口 (can0)，
使 candump / cansend / Apollo canbus 等工具可以直接使用。

用法:
  sudo python3 canalyst_bridge.py              # 默认 channel=0, can0
  sudo python3 canalyst_bridge.py --channel 1  # 用 CH2
  sudo python3 canalyst_bridge.py --vcan        # 用 vcan0 (无需物理CAN)

桥接后可在另一个终端使用:
  candump can0
  cansend can0 233#DEADBEEF
"""

import argparse
import os
import subprocess
import sys
import threading
import time
import socket
import struct

def setup_vcan(ifname):
    """创建 vcan 接口"""
    subprocess.run(["modprobe", "vcan"], check=True)
    # 先删除可能已存在的接口
    subprocess.run(["ip", "link", "del", ifname], capture_output=True)
    subprocess.run(["ip", "link", "add", "dev", ifname, "type", "vcan"], check=True)
    subprocess.run(["ip", "link", "set", ifname, "up"], check=True)
    print(f"✅ vcan 接口 {ifname} 已创建")

def open_socketcan(ifname):
    """打开 SocketCAN raw socket"""
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((ifname,))
    return s

def usb_to_socketcan(usb_bus, sock, stats):
    """CANalyst-II → SocketCAN"""
    CAN_FRAME_FMT = "=IB3x8s"  # can_id, can_dlc, pad, data
    while True:
        try:
            msg = usb_bus.recv(timeout=0.5)
            if msg:
                frame = struct.pack(CAN_FRAME_FMT,
                                    msg.arbitration_id,
                                    msg.dlc,
                                    bytes(msg.data).ljust(8, b'\x00'))
                sock.send(frame)
                stats['rx'] += 1
        except Exception as e:
            print(f"\n⚠️ USB→CAN 错误: {e}")
            break

def socketcan_to_usb(sock, usb_bus, stats):
    """SocketCAN → CANalyst-II"""
    import can as can_mod
    CAN_FRAME_FMT = "=IB3x8s"
    while True:
        try:
            data = sock.recv(16)
            if data:
                can_id, dlc, payload = struct.unpack(CAN_FRAME_FMT, data)
                msg = can_mod.Message(
                    arbitration_id=can_id & 0x1FFFFFFF,
                    data=payload[:dlc],
                    is_extended_id=bool(can_id & 0x80000000)
                )
                usb_bus.send(msg)
                stats['tx'] += 1
        except Exception as e:
            print(f"\n⚠️ CAN→USB 错误: {e}")
            break

def main():
    parser = argparse.ArgumentParser(description="CANalyst-II ↔ SocketCAN 桥接")
    parser.add_argument("--channel", type=int, default=0, help="CANalyst-II 通道 (0=CH1, 1=CH2)")
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN 波特率 (默认 500000)")
    parser.add_argument("--ifname", type=str, default="can0", help="SocketCAN 接口名 (默认 can0)")
    parser.add_argument("--vcan", action="store_true", help="使用 vcan 类型接口")
    args = parser.parse_args()

    if os.geteuid() != 0:
        print("❌ 需要 root 权限! 请使用 sudo 运行")
        sys.exit(1)

    # 添加用户 pip 包路径
    import site
    user_site = site.getusersitepackages()
    if user_site not in sys.path:
        sys.path.insert(0, user_site)

    import can

    # 1. 创建 vcan 接口
    ifname = args.ifname
    setup_vcan(ifname)

    # 2. 连接 CANalyst-II
    print(f"正在连接 CANalyst-II (channel={args.channel}, bitrate={args.bitrate})...")
    try:
        usb_bus = can.Bus(interface='canalystii', channel=args.channel, bitrate=args.bitrate)
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        sys.exit(1)
    print(f"✅ CANalyst-II 已连接")

    # 3. 打开 SocketCAN
    sock = open_socketcan(ifname)
    print(f"✅ SocketCAN {ifname} 已打开")
    print(f"\n{'='*50}")
    print(f"  桥接运行中: CANalyst-II CH{args.channel+1} ↔ {ifname}")
    print(f"  可在另一个终端使用: candump {ifname}")
    print(f"  Ctrl+C 退出")
    print(f"{'='*50}\n")

    # 4. 启动双向桥接线程
    stats = {'rx': 0, 'tx': 0}
    t1 = threading.Thread(target=usb_to_socketcan, args=(usb_bus, sock, stats), daemon=True)
    t2 = threading.Thread(target=socketcan_to_usb, args=(sock, usb_bus, stats), daemon=True)
    t1.start()
    t2.start()

    try:
        while True:
            sys.stdout.write(f"\r  USB→CAN: {stats['rx']} 帧  |  CAN→USB: {stats['tx']} 帧  ")
            sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n\n停止桥接。总计 RX={stats['rx']}, TX={stats['tx']}")
    finally:
        sock.close()
        usb_bus.shutdown()
        subprocess.run(["ip", "link", "del", ifname], capture_output=True)
        print(f"✅ {ifname} 已删除")

if __name__ == "__main__":
    main()
