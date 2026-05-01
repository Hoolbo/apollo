#!/usr/bin/env python3
"""
GNSS 校准工具：监听 GNSS UDP 数据，实时显示 UTM 和局部坐标。
用于校准地图原点偏移。

用法:
  1. 把车开到地图上一个你能确认的位置（比如路口、环岛中心）
  2. 运行此脚本，记下显示的局部坐标
  3. 对比地图上该点的坐标，差值就是校正量

python3 modules/drivers/gnss/gnss_calibrate.py --ip 192.168.1.103 --port 8680
"""

import argparse
import math
import socket
import time

# --- UTM 参数 ---
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2
UTM_K0 = 0.9996
UTM_ZONE = 50
UTM_LON0 = math.radians((UTM_ZONE - 1) * 6 - 180 + 3)
UTM_FE = 500000.0

# 地图原点
UTM_ORIGIN_X = 427961.691935
UTM_ORIGIN_Y = 4412026.565194


def latlon_to_utm(lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    e2 = WGS84_E2
    a = WGS84_A
    e_prime2 = e2 / (1 - e2)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = e_prime2 * math.cos(lat) ** 2
    A_ = (lon - UTM_LON0) * math.cos(lat)
    M = a * ((1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat
              - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat)
              + (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat)
              - (35*e2**3/3072) * math.sin(6*lat))
    x = UTM_FE + UTM_K0 * N * (A_ + (1-T+C)*A_**3/6 + (5-18*T+T**2+72*C-58*e_prime2)*A_**5/120)
    y = UTM_K0 * (M + N * math.tan(lat) * (A_**2/2 + (5-T+9*C+4*C**2)*A_**4/24 + (61-58*T+T**2+600*C-330*e_prime2)*A_**6/720))
    return x, y


def parse_gpchc(line):
    parts = line.strip().split(',')
    if len(parts) < 20 or parts[0] != '$GPCHC':
        return None
    try:
        return {
            'lat': float(parts[7]),
            'lon': float(parts[8]),
            'alt': float(parts[9]),
            'heading': float(parts[4]),
            'status': int(parts[16]),
        }
    except (ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description="GNSS 校准工具")
    parser.add_argument("--ip", default="192.168.1.103", help="GNSS UDP IP")
    parser.add_argument("--port", type=int, default=8680, help="GNSS UDP 端口")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print(f"监听 {args.ip}:{args.port} ...")
    print(f"地图原点 UTM: ({UTM_ORIGIN_X:.3f}, {UTM_ORIGIN_Y:.3f})")
    print("-" * 70)
    print(f"{'#':>4} | {'status':>6} | {'lat':>12} {'lon':>13} | {'local_x':>10} {'local_y':>10} | {'utm_x':>12} {'utm_y':>14}")
    print("-" * 70)

    count = 0
    while True:
        data, addr = sock.recvfrom(4096)
        for line in data.decode('ascii', errors='ignore').split('\n'):
            d = parse_gpchc(line)
            if d is None:
                continue
            count += 1
            utm_x, utm_y = latlon_to_utm(d['lat'], d['lon'])
            local_x = utm_x - UTM_ORIGIN_X
            local_y = utm_y - UTM_ORIGIN_Y
            status_str = f"{d['status']}"
            pos_quality = d['status'] % 10
            quality_names = {0: "无解", 1: "单点", 2: "差分", 4: "RTK固定", 5: "RTK浮"}
            q_name = quality_names.get(pos_quality, f"?{pos_quality}")

            print(f"[{count:4d}] {status_str:>6}({q_name:>5}) | "
                  f"{d['lat']:12.7f} {d['lon']:13.7f} | "
                  f"{local_x:10.2f} {local_y:10.2f} | "
                  f"{utm_x:12.3f} {utm_y:14.3f}")

            if count == 1:
                print(f"\n  >> 在网页上找到你实际所在的地图坐标")
                print(f"  >> 校正量 = 实际坐标 - 上面的 local_x/y")
                print(f"  >> 然后修改 rear_localization_node.py 里的 UTM_ORIGIN_X/Y\n")


if __name__ == '__main__':
    main()
