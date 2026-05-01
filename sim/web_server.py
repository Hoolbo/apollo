#!/usr/bin/env python3
"""
铰接车实时 Web 可视化后端 — 零依赖版 (仅 Python 标准库)

用法：python3 sim/web_server.py
浏览器：http://localhost:8888
"""

import asyncio
import base64
import datetime
import hashlib
import json
import math
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from socketserver import ThreadingMixIn

# ── Cyber RT ──
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.planning_msgs.planning_command_pb2 import PlanningCommand
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand

try:
    from modules.canbus_vehicle.articulated.proto.articulated_pb2 import Articulated
    HAS_ARTICULATED_PROTO = True
    print("  ✓ Articulated protobuf loaded")
except ImportError:
    HAS_ARTICULATED_PROTO = False
    print("  ⚠ Articulated protobuf not found, chassis_detail unavailable")

# ── 路径 ──
SCRIPT_DIR = Path(__file__).parent.resolve()
STATIC_DIR = SCRIPT_DIR / "static"
APOLLO_ROOT = SCRIPT_DIR.parent
PLANNER_CONFIG = APOLLO_ROOT / "modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt"
VEHICLE_CONFIG = APOLLO_ROOT / "modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json"

# ── 全局状态 ──
state_lock = threading.Lock()
vehicle_state = {"x": 0.0, "y": 0.0, "theta": 0.0, "gamma": 0.0, "speed": 0.0, "rear_theta": 0.0,
                 "front_gnss_status": 0, "rear_gnss_status": 0}
vehicle_trail = []
cilqr_traj = []
global_path = []
vehicle_params = None
map_cache = None

control_cmd_state = {
    "v_front": 0.0, "delta_front_deg": 0.0,
    "v_rear": 0.0, "delta_rear_deg": 0.0,
    "speed_cmd": 0.0, "steering_cmd_deg": 0.0, "timestamp": 0.0,
}
can_feedback_state = {
    "front": {"speed_kmh": 0.0, "eps_angle_deg": 0.0, "eps_enable": False,
              "motor_enable": False, "gear": "N", "driving_mode": "MANUAL",
              "vcu_error": False, "vcu_remote": False},
    "rear": {"speed_mps": 0.0, "steer_angle_rad": 0.0, "vehicle_state": "UNKNOWN",
             "control_mode": "UNKNOWN", "battery_v": 0.0, "fault_high": 0,
             "fault_low": 0, "motor1_rpm": 0, "motor1_current_a": 0.0,
             "motor2_rpm": 0, "motor2_current_a": 0.0, "motor3_rpm": 0,
             "motor3_current_a": 0.0, "driver_temp_1": 0, "motor_temp_1": 0},
}

ws_clients = []  # list of socket objects
ws_clients_lock = threading.Lock()

# Topic 最后收到时间戳（用于检测模块是否存活）
topic_last_time = {
    "localization": 0.0, # /apollo/localization/pose
    "rear_loc": 0.0,     # /apollo/localization/pose_rear
    "canbus": 0.0,       # /apollo/canbus/chassis
    "cilqr": 0.0,        # /apollo/planning
    "mpc": 0.0,          # /apollo/control
}
MODULE_ALIVE_TIMEOUT = 3.0  # 超过 3 秒没收到数据视为离线

def get_module_status():
    now = time.time()
    return {k: (now - v) < MODULE_ALIVE_TIMEOUT for k, v in topic_last_time.items()}

# ── 录包状态 ──
RECORD_DIR = SCRIPT_DIR / "recordings"
RECORD_DIR.mkdir(exist_ok=True)
record_lock = threading.Lock()
is_recording = False
record_frames = []
record_start_time = 0.0
record_filename = ""

def build_state_snapshot():
    """构建当前帧的状态快照"""
    with state_lock:
        return {
            "vehicle": dict(vehicle_state),
            "trail": vehicle_trail[-500:],
            "cilqr_traj": cilqr_traj,
            "global_path": global_path,
            "control_cmd": dict(control_cmd_state),
            "can_feedback": {
                "front": dict(can_feedback_state["front"]),
                "rear": dict(can_feedback_state["rear"]),
            },
            "module_status": get_module_status(),
        }

# ═══════════════════════════════════════════════════════════
#  WebSocket 协议 (RFC 6455)
# ═══════════════════════════════════════════════════════════
WS_MAGIC = b"258EAFA5-E914-47DA-95CA-5AB5DC47B11E"

def ws_handshake_response(key):
    accept = base64.b64encode(hashlib.sha1(key.encode() + WS_MAGIC).digest()).decode()
    return (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
    )

def ws_send_text(sock, text):
    """发送 WebSocket 文本帧"""
    data = text.encode('utf-8')
    frame = bytearray()
    frame.append(0x81)  # FIN + TEXT
    length = len(data)
    if length < 126:
        frame.append(length)
    elif length < 65536:
        frame.append(126)
        frame.extend(struct.pack('!H', length))
    else:
        frame.append(127)
        frame.extend(struct.pack('!Q', length))
    frame.extend(data)
    try:
        sock.sendall(bytes(frame))
        return True
    except Exception:
        return False

def ws_recv_frame(sock):
    """接收一帧，返回 (opcode, payload) 或 None"""
    try:
        hdr = sock.recv(2)
        if len(hdr) < 2:
            return None
        opcode = hdr[0] & 0x0F
        masked = (hdr[1] & 0x80) != 0
        length = hdr[1] & 0x7F
        if length == 126:
            length = struct.unpack('!H', sock.recv(2))[0]
        elif length == 127:
            length = struct.unpack('!Q', sock.recv(8))[0]
        mask_key = sock.recv(4) if masked else None
        payload = bytearray()
        while len(payload) < length:
            chunk = sock.recv(min(4096, length - len(payload)))
            if not chunk:
                return None
            payload.extend(chunk)
        if masked and mask_key:
            payload = bytearray(b ^ mask_key[i % 4] for i, b in enumerate(payload))
        return (opcode, bytes(payload))
    except Exception:
        return None


# ═══════════════════════════════════════════════════════════
#  Cyber RT 回调
# ═══════════════════════════════════════════════════════════

def on_localization(msg):
    with state_lock:
        vehicle_state["x"] = msg.pose.position.x
        vehicle_state["y"] = msg.pose.position.y
        vehicle_state["theta"] = msg.pose.heading
        vx = msg.pose.linear_velocity.x
        vy = msg.pose.linear_velocity.y
        vehicle_state["speed"] = math.sqrt(vx * vx + vy * vy)
        vehicle_trail.append((msg.pose.position.x, msg.pose.position.y))
        if len(vehicle_trail) > 2000:
            vehicle_trail.pop(0)
        topic_last_time["localization"] = time.time()
        vehicle_state["front_gnss_status"] = msg.header.sequence_num

def on_rear_localization(msg):
    with state_lock:
        vehicle_state["rear_theta"] = msg.pose.heading
        gamma = vehicle_state["theta"] - msg.pose.heading
        while gamma > math.pi: gamma -= 2 * math.pi
        while gamma < -math.pi: gamma += 2 * math.pi
        vehicle_state["gamma"] = gamma
        vehicle_state["rear_gnss_status"] = msg.header.sequence_num
        topic_last_time["rear_loc"] = time.time()

def on_chassis(msg):
    with state_lock:
        vehicle_state["speed"] = msg.speed_mps
        topic_last_time["canbus"] = time.time()

def on_trajectory(msg):
    global cilqr_traj
    pts = [(tp.path_point.x, tp.path_point.y) for tp in msg.trajectory_point]
    with state_lock:
        cilqr_traj = pts
        topic_last_time["cilqr"] = time.time()

def on_global_trajectory(msg):
    global global_path
    pts = [(tp.path_point.x, tp.path_point.y) for tp in msg.trajectory_point]
    if not pts: return
    with state_lock:
        if not global_path:
            global_path = pts
        else:
            old_end, new_end = global_path[-1], pts[-1]
            if math.sqrt((old_end[0]-new_end[0])**2 + (old_end[1]-new_end[1])**2) > 5.0:
                global_path = pts

def on_control_command(msg):
    with state_lock:
        control_cmd_state["speed_cmd"] = msg.speed
        control_cmd_state["steering_cmd_deg"] = msg.steering_target
        control_cmd_state["timestamp"] = msg.header.timestamp_sec
        topic_last_time["mpc"] = time.time()
        debug_str = ""
        if msg.header.HasField('status') and msg.header.status.HasField('msg'):
            debug_str = msg.header.status.msg
        if debug_str and 'v_front' in debug_str:
            for item in debug_str.split(','):
                m = re.match(r'(\w+)=([-\d.eE+]+)', item.strip())
                if m:
                    key, val = m.group(1), float(m.group(2))
                    if key == 'v_front': control_cmd_state["v_front"] = val
                    elif key == 'delta_front': control_cmd_state["delta_front_deg"] = math.degrees(val)
                    elif key == 'v_rear': control_cmd_state["v_rear"] = val
                    elif key == 'delta_rear': control_cmd_state["delta_rear_deg"] = math.degrees(val)

def on_chassis_detail(msg):
    with state_lock:
        f = can_feedback_state["front"]
        r = can_feedback_state["rear"]
        if msg.HasField('front_drivemotor_acu_572'):
            fb = msg.front_drivemotor_acu_572
            if fb.HasField('drive_motor_speed'): f["speed_kmh"] = fb.drive_motor_speed
            if fb.HasField('drive_motor_enable'): f["motor_enable"] = fb.drive_motor_enable
            if fb.HasField('drive_motor_shift'):
                f["gear"] = {0:"P",1:"D",2:"R",3:"N"}.get(fb.drive_motor_shift, "?")
        if msg.HasField('front_eps_acu_556'):
            eps = msg.front_eps_acu_556
            if eps.HasField('eps_angle'): f["eps_angle_deg"] = eps.eps_angle
            if eps.HasField('eps_enable'): f["eps_enable"] = eps.eps_enable
        if msg.HasField('front_vcu_acu_general_524'):
            vcu = msg.front_vcu_acu_general_524
            if vcu.HasField('acu_error'): f["vcu_error"] = vcu.acu_error
            if vcu.HasField('acu_remote_control'): f["vcu_remote"] = vcu.acu_remote_control
            if vcu.HasField('acu_control_mode') and vcu.acu_control_mode:
                f["driving_mode"] = "AUTO"
            elif vcu.HasField('acu_remote_control') and vcu.acu_remote_control:
                f["driving_mode"] = "REMOTE"
            else:
                f["driving_mode"] = "MANUAL"
        if msg.HasField('rear_motion_feedback_545'):
            rmf = msg.rear_motion_feedback_545
            if rmf.HasField('linear_speed'): r["speed_mps"] = rmf.linear_speed
            if rmf.HasField('steering_angle'): r["steer_angle_rad"] = rmf.steering_angle
        if msg.HasField('rear_chassis_status_529'):
            rcs = msg.rear_chassis_status_529
            if rcs.HasField('vehicle_state'):
                r["vehicle_state"] = {0:"NORMAL",1:"E-STOP",2:"FAULT"}.get(rcs.vehicle_state,"UNKNOWN")
            if rcs.HasField('control_mode'):
                r["control_mode"] = {0:"STANDBY",1:"CAN",2:"REMOTE"}.get(rcs.control_mode,"UNKNOWN")
            if rcs.HasField('battery_voltage'): r["battery_v"] = rcs.battery_voltage
            if rcs.HasField('fault_high'): r["fault_high"] = rcs.fault_high
            if rcs.HasField('fault_low'): r["fault_low"] = rcs.fault_low
        for field, prefix, keys in [
            ('rear_motor_feedback_high_1_593', 'motor1', ('motor1_speed','motor1_current')),
            ('rear_motor_feedback_high_2_594', 'motor2', ('motor2_speed','motor2_current')),
            ('rear_motor_feedback_high_3_595', 'motor3', ('motor3_speed','motor3_current')),
        ]:
            if msg.HasField(field):
                sub = getattr(msg, field)
                if sub.HasField(keys[0]): r[f"{prefix}_rpm"] = getattr(sub, keys[0])
                if sub.HasField(keys[1]): r[f"{prefix}_current_a"] = getattr(sub, keys[1])
        if msg.HasField('rear_motor_feedback_low_1_609'):
            ml = msg.rear_motor_feedback_low_1_609
            if ml.HasField('driver_temp_1'): r["driver_temp_1"] = ml.driver_temp_1
            if ml.HasField('motor_temp_1'): r["motor_temp_1"] = ml.motor_temp_1


# ═══════════════════════════════════════════════════════════
#  配置加载
# ═══════════════════════════════════════════════════════════

def load_vehicle_params():
    global vehicle_params
    for p in [str(VEHICLE_CONFIG),
              "/apollo_workspace/modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json"]:
        if os.path.exists(p):
            try:
                raw = re.sub(r'//.*', '', open(p).read())
                d = json.loads(raw)
                vehicle_params = {
                    "L_f": d["front_body"]["pivot_distance"],
                    "L_r": d["rear_body"]["pivot_distance"],
                    "L_f_body": d["front_body"]["body_length"],
                    "L_r_body": d["rear_body"]["body_length"],
                    "W_f_body": d["front_body"]["body_width"],
                    "W_r_body": d["rear_body"]["body_width"],
                }
                print(f"  ✓ Vehicle params loaded")
                return
            except Exception as e:
                print(f"  ⚠ vehicle.json error: {e}")
    print("  ⚠ vehicle.json not found")

def load_map_data():
    global map_cache
    try:
        map_path = None
        for cp in [str(PLANNER_CONFIG),
                    "/apollo_workspace/modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt"]:
            if os.path.exists(cp):
                for line in open(cp):
                    if 'map_file' in line:
                        map_path = line.split('"')[1].strip()
                        break
                if map_path: break
        if not map_path or not os.path.exists(map_path):
            print(f"  ⚠ Map not found: {map_path}")
            return
        print(f"  Loading map: {map_path}")
        raw = json.load(open(map_path))
        meta = raw.get('metadata', raw)
        dims = meta.get('dimensions', meta)
        map_cache = {
            "width": dims.get("width", meta.get("width", 0)),
            "height": dims.get("height", meta.get("height", 0)),
            "resolution": dims.get("resolution", meta.get("resolution", 1.0)),
            "origin": meta.get("origin", [0.0, 0.0]),
            "data": raw["data"],
        }
        print(f"  ✓ Map: {map_cache['width']}×{map_cache['height']}")
    except Exception as e:
        print(f"  ⚠ Map error: {e}")


# ═══════════════════════════════════════════════════════════
#  Cyber RT
# ═══════════════════════════════════════════════════════════

cyber_node = None
goal_writer = None
ctrl_writer = None

def cyber_thread_func():
    global cyber_node, goal_writer, ctrl_writer
    cyber.init()
    cyber_node = cyber.Node("web_visualizer")
    cyber_node.create_reader("/apollo/localization/pose", LocalizationEstimate, on_localization)
    cyber_node.create_reader("/apollo/localization/pose_rear", LocalizationEstimate, on_rear_localization)
    cyber_node.create_reader("/apollo/canbus/chassis", Chassis, on_chassis)
    cyber_node.create_reader("/apollo/planning", ADCTrajectory, on_trajectory)
    cyber_node.create_reader("/apollo/planning/global_path", ADCTrajectory, on_global_trajectory)
    cyber_node.create_reader("/apollo/control", ControlCommand, on_control_command)
    if HAS_ARTICULATED_PROTO:
        cyber_node.create_reader("/apollo/canbus/chassis_detail", Articulated, on_chassis_detail)
        print("  ✓ Subscribed to chassis_detail")
    goal_writer = cyber_node.create_writer("/apollo/planning/command", PlanningCommand)
    ctrl_writer = cyber_node.create_writer("/apollo/control", ControlCommand)
    print("  ✓ Cyber RT initialized")
    while True:
        time.sleep(1)


# ═══════════════════════════════════════════════════════════
#  WebSocket 广播线程
# ═══════════════════════════════════════════════════════════

def ws_broadcast_thread():
    """20Hz 向所有 WebSocket 客户端推送"""
    global is_recording, record_frames
    while True:
        with ws_clients_lock:
            if not ws_clients:
                time.sleep(0.05)
                continue
        snapshot = build_state_snapshot()
        data = json.dumps(snapshot)

        # 录包
        with record_lock:
            if is_recording:
                t = time.time() - record_start_time
                record_frames.append({"t": round(t, 3), **snapshot})

        dead = []
        with ws_clients_lock:
            for sock in ws_clients:
                if not ws_send_text(sock, data):
                    dead.append(sock)
            for s in dead:
                ws_clients.remove(s)
                try: s.close()
                except: pass
        time.sleep(0.05)


def ws_accept_thread():
    """独立 WebSocket TCP 服务器（端口 8889）"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", 8889))
    srv.listen(5)
    print("  ✓ WebSocket server on :8889")
    while True:
        client, addr = srv.accept()
        threading.Thread(target=ws_handle_new_client, args=(client,), daemon=True).start()

def ws_handle_new_client(sock):
    """处理新 WebSocket 连接：读 HTTP 升级请求 → 握手 → 进入消息循环"""
    try:
        # 读 HTTP 请求头
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = sock.recv(4096)
            if not chunk:
                sock.close()
                return
            data += chunk
        header_str = data.decode('utf-8', errors='ignore')
        # 提取 Sec-WebSocket-Key
        key = ""
        for line in header_str.split("\r\n"):
            if line.lower().startswith("sec-websocket-key:"):
                key = line.split(":", 1)[1].strip()
                break
        if not key:
            sock.close()
            return
        # 发送握手响应
        sock.sendall(ws_handshake_response(key).encode())
        # 进入 WebSocket 消息循环
        ws_client_handler(sock)
    except Exception as e:
        print(f"  WS handshake error: {e}")
        try: sock.close()
        except: pass


def ws_client_handler(sock):
    """WebSocket 消息循环"""
    init_msg = json.dumps({
        "type": "init",
        "vehicle_params": vehicle_params,
        "has_chassis_detail": HAS_ARTICULATED_PROTO,
    })
    ws_send_text(sock, init_msg)

    with ws_clients_lock:
        ws_clients.append(sock)
    print(f"  WS connected ({len(ws_clients)} clients)")

    try:
        while True:
            frame = ws_recv_frame(sock)
            if frame is None:
                break
            opcode, _ = frame
            if opcode == 0x8:  # close
                break
            if opcode == 0x9:  # ping → pong
                ws_send_text(sock, "")
    except Exception:
        pass
    finally:
        with ws_clients_lock:
            if sock in ws_clients:
                ws_clients.remove(sock)
        try: sock.close()
        except: pass
        print(f"  WS disconnected ({len(ws_clients)} clients)")


# ═══════════════════════════════════════════════════════════
#  HTTP 请求处理
# ═══════════════════════════════════════════════════════════

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self._ws_hijacked = False
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, format, *args):
        pass  # 静默日志

    def finish(self):
        if not self._ws_hijacked:
            super().finish()

    def do_GET(self):
        # WebSocket 升级
        conn_hdr = self.headers.get('Connection', '')
        if self.path == '/ws' and 'upgrade' in conn_hdr.lower():
            print(f"  [WS] Upgrade request from {self.client_address}")
            key = self.headers.get('Sec-WebSocket-Key', '')
            if not key:
                print("  [WS] ERROR: No Sec-WebSocket-Key")
                self.send_error(400, "Missing WebSocket key")
                return
            try:
                resp = ws_handshake_response(key)
                self.request.sendall(resp.encode())
                print(f"  [WS] Handshake sent, key={key[:8]}...")
                self._ws_hijacked = True
                self.close_connection = True
                ws_client_handler(self.request)
            except Exception as e:
                print(f"  [WS] ERROR: {e}")
            return

        # API
        if self.path == '/api/map':
            self._json_response(map_cache or {"error": "not loaded"})
            return

        # HTTP 轮询后备（WebSocket 不通时用）
        if self.path == '/api/state':
            self._json_response(build_state_snapshot())
            return

        # 录包文件列表
        if self.path == '/api/records':
            files = []
            for f in sorted(RECORD_DIR.glob('*.json'), reverse=True):
                try:
                    meta = json.loads(f.read_text())[:1]  # 不行就用 stat
                except:
                    pass
                size = f.stat().st_size
                files.append({"name": f.name, "size_kb": round(size / 1024, 1)})
            self._json_response(files)
            return

        # 下载录包文件
        if self.path.startswith('/api/record/'):
            fname = self.path.split('/')[-1]
            fpath = RECORD_DIR / fname
            if fpath.exists() and fpath.suffix == '.json':
                body = fpath.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(body))
                self.end_headers()
                self.wfile.write(body)
            else:
                self._json_response({"error": "not found"}, 404)
            return

        # 静态文件
        if self.path == '/':
            self.path = '/index.html'
        super().do_GET()

    def do_POST(self):
        content_len = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(content_len)) if content_len > 0 else {}

        if self.path == '/api/goal':
            x, y, theta = body.get('x', 0), body.get('y', 0), body.get('theta', 0)
            if goal_writer:
                msg = PlanningCommand()
                msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                msg.header.module_name = "web_visualizer"
                wp = msg.lane_follow_command.routing_request.waypoint.add()
                wp.pose.x, wp.pose.y, wp.heading = x, y, theta
                goal_writer.write(msg)
                print(f"  Goal: ({x:.1f}, {y:.1f}, θ={math.degrees(theta):.1f}°)")
            self._json_response({"status": "ok", "x": x, "y": y})

        elif self.path == '/api/emergency':
            if ctrl_writer:
                msg = ControlCommand()
                msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                msg.header.module_name = "web_emergency"
                msg.speed = 0.0
                msg.steering_target = 0.0
                msg.acceleration = -5.0
                msg.header.status.msg = "v_front=0.0,delta_front=0.0,v_rear=0.0,delta_rear=0.0"
                ctrl_writer.write(msg)
                print("  ⚠ EMERGENCY STOP!")
            self._json_response({"status": "ok", "action": "emergency_stop"})

        elif self.path == '/api/start':
            x, y, theta = body.get('x', -160), body.get('y', -11), body.get('theta', 0.1)
            subprocess.run(["pkill", "-f", "sim_vehicle.py"], capture_output=True)
            with state_lock:
                vehicle_trail.clear()
            subprocess.Popen(["python3", "sim/sim_vehicle.py",
                              "--x", str(x), "--y", str(y), "--theta", str(theta)],
                             cwd="/apollo_workspace")
            self._json_response({"status": "ok", "x": x, "y": y})

        elif self.path == '/api/module':
            name, action = body.get('name', ''), body.get('action', 'start')

            # Python 脚本模块（用 pkill/Popen）
            script_map = {
                "sim": ("sim/sim_vehicle.py", []),
                "rear_loc": ("modules/drivers/gnss/rear_localization_node.py", []),
                # 前车定位：复用 rear_localization_node.py，改 topic 和端口
                # ⚠ 请根据实际前车 GNSS 的 IP/端口修改下面的参数
                "localization": ("modules/drivers/gnss/rear_localization_node.py",
                                 ["--topic", "/apollo/localization/pose",
                                  "--ip", "192.168.1.103", "--port", "8680",
                                  "--name", "front_localization_node"]),
            }
            # cyber_launch 模块
            launch_map = {
                "gnss": "modules/drivers/gnss/launch/gnss.launch",
                "canbus": "modules/canbus/launch/canbus.launch",
                "cilqr": "modules/planning/cilqr_planner/launch/cilqr_planner.launch",
                "mpc": "modules/control/mpc_controller/launch/mpc_controller.launch",
            }

            if name in script_map:
                script, extra_args = script_map[name]
                # 用完整命令作为 pkill 匹配模式，避免杀错进程
                kill_pattern = " ".join([script] + extra_args) if extra_args else script
                if action == 'start':
                    subprocess.Popen(["python3", script] + extra_args, cwd="/apollo_workspace")
                    print(f"  Module {name}: started {script} {' '.join(extra_args)}")
                else:
                    subprocess.run(["pkill", "-f", kill_pattern], capture_output=True)
                    print(f"  Module {name}: stopped")
                self._json_response({"status": "ok"})
            elif name in launch_map:
                cmd = ["cyber_launch", action, launch_map[name]]
                subprocess.Popen(cmd, cwd="/apollo_workspace")
                print(f"  Module {name}: {' '.join(cmd)}")
                self._json_response({"status": "ok"})
            else:
                self._json_response({"error": f"unknown: {name}"}, 400)
        elif self.path == '/api/record/start':
            with record_lock:
                is_recording = True
                record_frames.clear()
                record_start_time = time.time()
                ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                record_filename = f"rec_{ts}.json"
            print(f"  ⏺ Recording started: {record_filename}")
            self._json_response({"status": "ok", "filename": record_filename})

        elif self.path == '/api/record/stop':
            with record_lock:
                is_recording = False
                if record_frames:
                    duration = record_frames[-1]["t"] if record_frames else 0
                    out = {
                        "filename": record_filename,
                        "duration": round(duration, 1),
                        "frame_count": len(record_frames),
                        "frames": record_frames[:],
                    }
                    fpath = RECORD_DIR / record_filename
                    fpath.write_text(json.dumps(out, separators=(',', ':')))
                    size_kb = fpath.stat().st_size / 1024
                    print(f"  ⏹ Recording saved: {record_filename} ({len(record_frames)} frames, {duration:.1f}s, {size_kb:.0f}KB)")
                    self._json_response({"status": "ok", "filename": record_filename,
                                         "frames": len(record_frames), "duration": round(duration, 1)})
                    record_frames.clear()
                else:
                    self._json_response({"status": "ok", "frames": 0})

        elif self.path == '/api/records/delete':
            fname = body.get('filename', '')
            fpath = RECORD_DIR / fname
            if fpath.exists() and fpath.suffix == '.json':
                fpath.unlink()
                self._json_response({"status": "ok"})
            else:
                self._json_response({"error": "not found"}, 404)

        else:
            self._json_response({"error": "not found"}, 404)

    def _json_response(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ═══════════════════════════════════════════════════════════
#  启动
# ═══════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  铰接车实时 Web 可视化（零依赖版）")
    print("=" * 60)

    load_vehicle_params()
    load_map_data()

    # Cyber RT 线程
    threading.Thread(target=cyber_thread_func, daemon=True).start()
    time.sleep(1)

    # Cyber RT 会覆盖 SIGINT，在它初始化后重新设置强制退出
    def _force_exit(*_):
        print("\n  已停止")
        os._exit(0)
    signal.signal(signal.SIGINT, _force_exit)
    signal.signal(signal.SIGTERM, _force_exit)

    # WebSocket 广播线程
    threading.Thread(target=ws_broadcast_thread, daemon=True).start()

    print(f"  打开浏览器: http://localhost:8888")
    print("=" * 60)

    server = ThreadedHTTPServer(("0.0.0.0", 8888), Handler)
    server.serve_forever()

if __name__ == "__main__":
    main()
