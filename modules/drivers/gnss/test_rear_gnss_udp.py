#!/usr/bin/env python3
"""
最简 UDP 测试: 验证能否收到工控机 GNSS 组合导航数据 ($GPCHC)

从 gnss-gongkongji/conf/gnss_conf.pb.txt 读取的配置:
  address: 192.168.1.110
  port:    9900
  format:  HUACE_TEXT ($GPCHC)

用法:
  python3 modules/drivers/gnss/test_rear_gnss_udp.py

  # 或指定其他端口:
  python3 modules/drivers/gnss/test_rear_gnss_udp.py --port 9901
"""

import socket
import time
import sys
import argparse

# 从 gnss-gongkongji/conf/gnss_conf.pb.txt 拿到的配置
DEFAULT_IP = "192.168.1.110"
DEFAULT_PORT = 9900


def parse_gpchc(line):
    """解析 $GPCHC, 返回关键字段 dict 或 None"""
    line = line.strip()
    if '*' in line:
        line = line[:line.rfind('*')]

    fields = line.split(',')
    if len(fields) < 24:
        return None
    if fields[0] not in ('$GPCHC', '$GPCHCX'):
        return None

    try:
        return {
            'gps_week': int(fields[1]),
            'gps_time': float(fields[2]),
            'heading':  float(fields[3]),
            'pitch':    float(fields[4]),
            'roll':     float(fields[5]),
            'lat':      float(fields[12]) if fields[12] else 0.0,
            'lon':      float(fields[13]) if fields[13] else 0.0,
            'alt':      float(fields[14]) if fields[14] else 0.0,
            've':       float(fields[15]),
            'vn':       float(fields[16]),
            'vu':       float(fields[17]),
            'v':        float(fields[18]),
            'nsv1':     int(fields[19]),
            'nsv2':     int(fields[20]),
            'status':   int(fields[21]),
        }
    except (ValueError, IndexError) as e:
        print(f"  ⚠ 解析失败: {e}")
        return None


STATUS_MAP = {
    0: "初始化",
    1: "单点定位",
    2: "伪距差分",
    3: "无效解",
    4: "RTK 固定解 ★",
    5: "RTK 浮点解",
    6: "纯惯导",
    22: "浮点窄巷",
    50: "窄巷固定解",
    56: "组合导航+单点",
    57: "组合导航+伪距差分",
    58: "组合导航+RTK浮点",
    59: "组合导航+RTK固定 ★",
    60: "惯导推算(DR)",
    61: "组合导航+窄巷固定 ★",
}


def main():
    parser = argparse.ArgumentParser(description="测试能否收到工控机 GNSS UDP 数据")
    parser.add_argument("--ip", default=DEFAULT_IP,
                        help=f"GNSS 设备 IP (默认 {DEFAULT_IP}, 仅显示用)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"监听 UDP 端口 (默认 {DEFAULT_PORT})")
    parser.add_argument("--timeout", type=int, default=10,
                        help="无数据超时秒数 (默认 10)")
    parser.add_argument("--count", type=int, default=0,
                        help="收到 N 条后退出 (0=不限)")
    parser.add_argument("--raw", action="store_true",
                        help="同时打印原始数据")
    args = parser.parse_args()

    print("=" * 65)
    print("  GNSS UDP 数据接收测试")
    print("=" * 65)
    print(f"  配置来源: gnss-gongkongji/conf/gnss_conf.pb.txt")
    print(f"  设备 IP:  {args.ip}")
    print(f"  监听端口: 0.0.0.0:{args.port}")
    print(f"  数据格式: HUACE_TEXT ($GPCHC)")
    print(f"  超时:     {args.timeout}s")
    print("-" * 65)

    # 创建 UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", args.port))
        sock.settimeout(args.timeout)
        print(f"  ✅ Socket 绑定成功: 0.0.0.0:{args.port}")
    except OSError as e:
        print(f"  ❌ Socket 绑定失败: {e}")
        print(f"     可能端口被占用，试试: ss -ulnp | grep {args.port}")
        sys.exit(1)

    print(f"\n  等待数据 (最多 {args.timeout}s)...\n")

    msg_count = 0
    gpchc_count = 0
    other_count = 0
    first_data_time = None
    buffer = ""

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                if msg_count == 0:
                    print(f"  ❌ 超时 {args.timeout}s，未收到任何 UDP 数据！")
                    print(f"\n  排查建议:")
                    print(f"    1. 确认工控机 IP 是 {args.ip}")
                    print(f"    2. ping {args.ip} 看网络通不通")
                    print(f"    3. 确认 GNSS 设备配置输出到本机 IP 的 {args.port} 端口")
                    print(f"    4. 检查防火墙: sudo iptables -L -n")
                    print(f"    5. 抓包: sudo tcpdump -i any udp port {args.port} -c 5")
                else:
                    print(f"\n  ⚠ 数据中断 {args.timeout}s")
                break

            msg_count += 1

            if first_data_time is None:
                first_data_time = time.time()
                print(f"  ✅ 收到第一包数据! 来源: {addr[0]}:{addr[1]}")
                print(f"     数据长度: {len(data)} bytes\n")

            # 解码
            try:
                text = data.decode("ascii", errors="ignore")
            except Exception:
                print(f"  ⚠ 包 #{msg_count}: 无法解码 ({len(data)} bytes)")
                continue

            if args.raw:
                print(f"  [RAW #{msg_count}] {text.strip()}")

            # 拆行处理
            buffer += text
            lines = buffer.split("\n")
            buffer = lines[-1]

            for line in lines[:-1]:
                line = line.strip()
                if not line:
                    continue

                if line.startswith("$GPCHC"):
                    gpchc_count += 1
                    d = parse_gpchc(line)
                    if d:
                        status_str = STATUS_MAP.get(d['status'], f"未知({d['status']})")
                        print(
                            f"  📡 [{gpchc_count:4d}] "
                            f"lat={d['lat']:.7f}  lon={d['lon']:.7f}  "
                            f"alt={d['alt']:.1f}m  "
                            f"heading={d['heading']:.1f}°  "
                            f"v={d['v']:.2f}m/s  "
                            f"卫星={d['nsv1']}/{d['nsv2']}  "
                            f"状态={status_str}"
                        )
                    else:
                        print(f"  ⚠ $GPCHC 解析失败: {line[:80]}...")
                else:
                    other_count += 1
                    if other_count <= 5:
                        print(f"  [其他] {line[:100]}")
                    elif other_count == 6:
                        print(f"  [其他] ... (后续省略)")

            # 达到指定数量退出
            if args.count > 0 and gpchc_count >= args.count:
                print(f"\n  ✅ 已收到 {args.count} 条 $GPCHC，测试完成。")
                break

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    # 汇总
    elapsed = time.time() - first_data_time if first_data_time else 0
    print(f"\n{'=' * 65}")
    print(f"  测试汇总")
    print(f"{'=' * 65}")
    print(f"  耗时:         {elapsed:.1f}s")
    print(f"  UDP 包总数:   {msg_count}")
    print(f"  $GPCHC 条数:  {gpchc_count}")
    print(f"  其他消息:     {other_count}")
    if elapsed > 0 and gpchc_count > 0:
        hz = gpchc_count / elapsed
        print(f"  数据频率:     {hz:.1f} Hz")
    print(f"{'=' * 65}")

    if gpchc_count > 0:
        print(f"  ✅ 结论: 可以收到 $GPCHC 数据，rear_localization_node.py 可用")
        print(f"     启动命令:")
        print(f"     python3 modules/drivers/gnss/rear_localization_node.py --ip {args.ip} --port {args.port}")
    else:
        print(f"  ❌ 结论: 未收到 $GPCHC 数据，请检查 GNSS 设备配置")


if __name__ == "__main__":
    main()
