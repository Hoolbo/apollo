#!/usr/bin/env python3
"""
后车华测组合导航定位节点

通过 UDP 读取后车华测 IMU 的 $GPCHC 数据，解析后发布 LocalizationEstimate
到 /apollo/localization/pose_rear，供 MPC 控制器计算铰接角 γ。

用法:
  python3 modules/drivers/gnss/rear_localization_node.py

配置:
  修改下方 REAR_IMU_IP / REAR_IMU_PORT 为后车华测 IMU 的地址。
"""

import math
import socket
import struct
import time
import argparse

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.localization_msgs.localization_pb2 import (
    LocalizationEstimate,
)

# ─── 配置 ────────────────────────────────────────────────────────────
REAR_IMU_IP = "192.168.1.110"   # 工控机 GNSS (gnss-gongkongji/conf/gnss_conf.pb.txt)
REAR_IMU_PORT = 9900            # UDP 端口
TOPIC_OUT = "/apollo/localization/pose_rear"

# UTM zone 50 投影参数 (与前车 gnss_conf.pb.txt 中 proj4_text 对应)
# WGS84 ellipsoid
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2
UTM_K0 = 0.9996
UTM_ZONE = 50
UTM_LON0 = math.radians((UTM_ZONE - 1) * 6 - 180 + 3)  # 中央经线
UTM_FE = 500000.0  # false easting

# 地图原点 (UTM) — 来自 current_start_point.txt，不要改
UTM_BASE_X = 427961.691935
UTM_BASE_Y = 4412026.565194

# ★★★ 手动校准偏移量（米）★★★
# 如果车辆在地图上偏右了，增大 CALIB_DX；偏上了，增大 CALIB_DY
# 方法：把车开到地图上一个已知位置，对比地图坐标和 GNSS 坐标，差值填这里
CALIB_DX = 143.4
CALIB_DY = 275.9

# 最终原点 = 基础 + 校准
UTM_ORIGIN_X = UTM_BASE_X + CALIB_DX
UTM_ORIGIN_Y = UTM_BASE_Y + CALIB_DY
# ─────────────────────────────────────────────────────────────────────


def latlon_to_utm(lat_deg, lon_deg):
    """WGS84 经纬度 → UTM zone 50 (x=Easting, y=Northing)"""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    dlon = lon - UTM_LON0

    N = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = (WGS84_E2 / (1 - WGS84_E2)) * math.cos(lat) ** 2
    A_ = dlon * math.cos(lat)

    # Meridional arc
    e2 = WGS84_E2
    e4 = e2 ** 2
    e6 = e2 ** 3
    M = WGS84_A * (
        (1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat
        - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * lat)
        + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * lat)
        - (35 * e6 / 3072) * math.sin(6 * lat)
    )

    x = UTM_K0 * N * (
        A_
        + (1 - T + C) * A_ ** 3 / 6
        + (5 - 18 * T + T ** 2 + 72 * C - 58 * (e2 / (1 - e2))) * A_ ** 5 / 120
    ) + UTM_FE

    y = UTM_K0 * (
        M
        + N * math.tan(lat) * (
            A_ ** 2 / 2
            + (5 - T + 9 * C + 4 * C ** 2) * A_ ** 4 / 24
            + (61 - 58 * T + T ** 2 + 600 * C
               - 330 * (e2 / (1 - e2))) * A_ ** 6 / 720
        )
    )
    return x, y


def azimuth_to_yaw(azimuth_deg):
    """华测 Heading(方位角, 度, 北=0 顺时针) → ENU yaw (rad, 东=0 逆时针)
       与 Apollo parser_common.h 中 azimuth_deg_to_yaw_rad 一致"""
    return math.radians(90.0 - azimuth_deg)


def parse_gpchc(line):
    """解析 $GPCHC 消息, 返回 dict 或 None
    格式: $GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,
           GyroX,GyroY,GyroZ,AccX,AccY,AccZ,
           Lat,Lon,Alt,Ve,Vn,Vu,V,
           NSV1,NSV2,Status,Age,Warning*CS
    """
    # 去掉校验码
    line = line.strip()
    if '*' in line:
        line = line[:line.rfind('*')]

    fields = line.split(',')
    if len(fields) < 24:
        return None
    if fields[0] not in ('$GPCHC', '$GPCHCX'):
        return None

    try:
        d = {
            'gps_week': int(fields[1]),
            'gps_time': float(fields[2]),
            'heading': float(fields[3]),    # azimuth, deg
            'pitch': float(fields[4]),      # deg
            'roll': float(fields[5]),       # deg
            'gyro_x': float(fields[6]),     # deg/s
            'gyro_y': float(fields[7]),
            'gyro_z': float(fields[8]),
            'acc_x': float(fields[9]),      # g → m/s²
            'acc_y': float(fields[10]),
            'acc_z': float(fields[11]),
            'lat': float(fields[12]) if fields[12] else 0.0,
            'lon': float(fields[13]) if fields[13] else 0.0,
            'alt': float(fields[14]) if fields[14] else 0.0,
            've': float(fields[15]),        # m/s
            'vn': float(fields[16]),
            'vu': float(fields[17]),
            'v': float(fields[18]),
            'nsv1': int(fields[19]),
            'nsv2': int(fields[20]),
            'status': int(fields[21]),
            'age': float(fields[22]),
        }
        return d
    except (ValueError, IndexError):
        return None


def build_localization_msg(d):
    """从解析后的 GPCHC 数据构造 LocalizationEstimate"""
    msg = LocalizationEstimate()
    msg.header.timestamp_sec = time.time()
    msg.header.module_name = "rear_localization"
    msg.header.sequence_num = d['status']  # GNSS 状态码，通过 sequence_num 传给 web

    # GPS 时间戳
    gps_ts = d['gps_week'] * 604800 + d['gps_time']
    msg.measurement_time = gps_ts

    # 位置: 经纬度 → UTM → 局部坐标（减去地图原点）
    utm_x, utm_y = latlon_to_utm(d['lat'], d['lon'])
    msg.pose.position.x = utm_x - UTM_ORIGIN_X
    msg.pose.position.y = utm_y - UTM_ORIGIN_Y
    msg.pose.position.z = d['alt']

    # 航向: azimuth → ENU yaw
    yaw = azimuth_to_yaw(d['heading'])
    pitch = math.radians(-d['pitch'])  # 与 Apollo 一致: 取反
    roll = math.radians(d['roll'])
    msg.pose.heading = yaw

    # 欧拉角 → 四元数 (ZYX: yaw-pitch-roll)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    msg.pose.orientation.qw = cr * cp * cy + sr * sp * sy
    msg.pose.orientation.qx = sr * cp * cy - cr * sp * sy
    msg.pose.orientation.qy = cr * sp * cy + sr * cp * sy
    msg.pose.orientation.qz = cr * cp * sy - sr * sp * cy

    # 速度 (ENU)
    msg.pose.linear_velocity.x = d['ve']
    msg.pose.linear_velocity.y = d['vn']
    msg.pose.linear_velocity.z = d['vu']

    # 角速度 (RFU → FLU, 与 Apollo rfu_to_flu 一致)
    msg.pose.angular_velocity.x = math.radians(d['gyro_y'])   # F = Y
    msg.pose.angular_velocity.y = math.radians(-d['gyro_x'])  # L = -R
    msg.pose.angular_velocity.z = math.radians(d['gyro_z'])    # U = U

    # 加速度 (RFU → FLU)
    G = 9.806
    msg.pose.linear_acceleration.x = d['acc_y'] * G
    msg.pose.linear_acceleration.y = -d['acc_x'] * G
    msg.pose.linear_acceleration.z = d['acc_z'] * G

    return msg


def main():
    parser = argparse.ArgumentParser(description="华测 IMU 定位节点")
    parser.add_argument("--ip", default=REAR_IMU_IP,
                        help="华测 IMU 的 IP 地址")
    parser.add_argument("--port", type=int, default=REAR_IMU_PORT,
                        help="华测 IMU 的 UDP 端口")
    parser.add_argument("--topic", default=TOPIC_OUT,
                        help="发布 topic (默认 /apollo/localization/pose_rear)")
    parser.add_argument("--name", default="rear_localization_node",
                        help="Cyber 节点名 (前后车需不同)")
    args = parser.parse_args()

    if not args.ip or args.port == 0:
        print("❌ 请指定后车华测 IMU 的 IP 和端口:")
        print("   python3 rear_localization_node.py --ip 192.168.1.111 --port 9901")
        return

    # Cyber 初始化
    cyber.init()
    node = cyber.Node(args.name)
    writer = node.create_writer(args.topic, LocalizationEstimate)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(2.0)

    print(f"✅ 后车定位节点已启动")
    print(f"   监听: 0.0.0.0:{args.port} (等待 {args.ip} 发来的数据)")
    print(f"   发布: {args.topic}")
    print(f"   Ctrl+C 退出")

    msg_count = 0
    buffer = ""

    try:
        while not cyber.is_shutdown():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            # 可能一次收多行
            buffer += data.decode("ascii", errors="ignore")
            lines = buffer.split("\n")
            buffer = lines[-1]  # 可能不完整的最后一行留到下次

            for line in lines[:-1]:
                line = line.strip()
                if not line.startswith("$GPCHC"):
                    continue

                d = parse_gpchc(line)
                if d is None:
                    continue

                msg = build_localization_msg(d)
                writer.write(msg)
                msg_count += 1

                if msg_count % 100 == 1:
                    print(f"  [{msg_count}] heading={d['heading']:.1f}° "
                          f"lat={d['lat']:.7f} lon={d['lon']:.7f} "
                          f"v={d['v']:.2f}m/s status={d['status']}")

    except KeyboardInterrupt:
        print(f"\n✅ 已停止。共发送 {msg_count} 条消息。")
    finally:
        sock.close()
        cyber.shutdown()


if __name__ == "__main__":
    main()
