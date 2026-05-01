#!/usr/bin/env python3
"""
铰接车实时 Web 可视化后端

用法：python3 sim/web_server.py
浏览器：http://localhost:8888
"""

import asyncio
import datetime
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import aiohttp
from aiohttp import web

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

ws_connected = set()   # set of aiohttp.web.WebSocketResponse

# ── 模块进程检测（仿照 DreamView ProcessMonitor 模式）──
# 每个模块配置 command_keywords，扫描 /proc/*/cmdline 进行关键词匹配
# 所有关键词都在某进程命令行中找到 → 该模块正在运行
MODULE_PROCESS_CONFIG = {
    # Python 脚本模块
    "sim":          {"command_keywords": ["sim_vehicle.py"]},
    "localization": {"command_keywords": ["rear_localization_node.py", "front_localization_node"]},
    "rear_loc":     {"command_keywords": ["rear_localization_node.py", "--topic", "/apollo/localization/pose_rear"]},
    # cyber_launch 模块
    "gnss":         {"command_keywords": ["cyber_launch", "gnss.launch"]},
    "canbus":       {"command_keywords": ["cyber_launch", "canbus.launch"]},
    "cilqr":        {"command_keywords": ["cyber_launch", "cilqr_planner.launch"]},
    "mpc":          {"command_keywords": ["cyber_launch", "mpc_controller.launch"]},
}
# rear_loc 默认参数没有 --topic，需特殊处理：
# 检测 rear_localization_node.py 在运行 且 不含 front_localization_node
MODULE_PROCESS_CONFIG["rear_loc"] = {
    "command_keywords": ["rear_localization_node.py"],
    "exclude_keywords": ["front_localization_node"],
}

_proc_status_cache = {}
_proc_status_cache_time = 0.0
_PROC_SCAN_INTERVAL = 1.5  # 与 DreamView 的 ProcessMonitor 一致

def _scan_running_processes():
    """扫描 /proc/*/cmdline，返回所有进程命令行列表（仿照 ProcessMonitor::RunOnce）"""
    import glob
    processes = []
    for cmdline_file in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(cmdline_file, 'rb') as f:
                raw = f.read()
            if raw:
                # /proc/<PID>/cmdline 中参数以 \0 分隔，转为空格
                cmd = raw.replace(b'\0', b' ').decode('utf-8', errors='ignore').strip()
                if cmd:
                    processes.append(cmd)
        except (IOError, OSError):
            continue
    return processes

def _match_keywords(processes, command_keywords, exclude_keywords=None):
    """检查是否有进程匹配所有 command_keywords（且不含 exclude_keywords）"""
    for cmd in processes:
        if all(kw in cmd for kw in command_keywords):
            if exclude_keywords and any(ek in cmd for ek in exclude_keywords):
                continue
            return True
    return False

def get_module_status():
    """检测各模块是否在运行（结果缓存 1.5 秒，避免频繁扫描 /proc）"""
    global _proc_status_cache, _proc_status_cache_time
    now = time.time()
    if now - _proc_status_cache_time < _PROC_SCAN_INTERVAL:
        return _proc_status_cache

    processes = _scan_running_processes()
    status = {}
    for name, config in MODULE_PROCESS_CONFIG.items():
        status[name] = _match_keywords(
            processes,
            config["command_keywords"],
            config.get("exclude_keywords"),
        )
    _proc_status_cache = status
    _proc_status_cache_time = now
    return status

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
        vehicle_state["front_gnss_status"] = msg.header.sequence_num

def on_rear_localization(msg):
    with state_lock:
        vehicle_state["rear_theta"] = msg.pose.heading
        gamma = vehicle_state["theta"] - msg.pose.heading
        while gamma > math.pi: gamma -= 2 * math.pi
        while gamma < -math.pi: gamma += 2 * math.pi
        vehicle_state["gamma"] = gamma
        vehicle_state["rear_gnss_status"] = msg.header.sequence_num

def on_chassis(msg):
    with state_lock:
        vehicle_state["speed"] = msg.speed_mps

def on_trajectory(msg):
    global cilqr_traj
    pts = [(tp.path_point.x, tp.path_point.y) for tp in msg.trajectory_point]
    with state_lock:
        cilqr_traj = pts

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
#  WebSocket + 广播（基于 aiohttp，仿照 DreamView CivetServer）
# ═══════════════════════════════════════════════════════════

async def ws_handler(request):
    """处理单个 WebSocket 连接"""
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    await ws.send_json({
        "type": "init",
        "vehicle_params": vehicle_params,
        "has_chassis_detail": HAS_ARTICULATED_PROTO,
    })
    ws_connected.add(ws)
    print(f"  WS connected ({len(ws_connected)} clients)")
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                pass  # 目前不需要处理客户端消息
    finally:
        ws_connected.discard(ws)
        print(f"  WS disconnected ({len(ws_connected)} clients)")
    return ws

async def broadcast_loop():
    """20Hz 广播 + 录包采集"""
    global is_recording, record_frames
    while True:
        snapshot = build_state_snapshot()
        with record_lock:
            if is_recording:
                t = time.time() - record_start_time
                record_frames.append({"t": round(t, 3), **snapshot})
        if ws_connected:
            data = json.dumps(snapshot)
            dead = []
            for ws in list(ws_connected):
                try:
                    await ws.send_str(data)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                ws_connected.discard(ws)
        await asyncio.sleep(0.05)


# ═══════════════════════════════════════════════════════════
#  HTTP API 路由（对标 DreamView CivetHandler）
# ═══════════════════════════════════════════════════════════

async def api_map(request):
    return web.json_response(map_cache or {"error": "not loaded"})

async def api_state(request):
    return web.json_response(build_state_snapshot())

async def api_records_list(request):
    files = []
    for f in sorted(RECORD_DIR.glob('*.json'), reverse=True):
        size = f.stat().st_size
        files.append({"name": f.name, "size_kb": round(size / 1024, 1)})
    return web.json_response(files)

async def api_record_download(request):
    fname = request.match_info['fname']
    fpath = RECORD_DIR / fname
    if fpath.exists() and fpath.suffix == '.json':
        return web.FileResponse(fpath, headers={'Content-Type': 'application/json'})
    return web.json_response({"error": "not found"}, status=404)

async def api_goal(request):
    body = await request.json()
    x, y, theta = body.get('x', 0), body.get('y', 0), body.get('theta', 0)
    if goal_writer:
        msg = PlanningCommand()
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = "web_visualizer"
        wp = msg.lane_follow_command.routing_request.waypoint.add()
        wp.pose.x, wp.pose.y, wp.heading = x, y, theta
        goal_writer.write(msg)
        print(f"  Goal: ({x:.1f}, {y:.1f}, θ={math.degrees(theta):.1f}°)")
    return web.json_response({"status": "ok", "x": x, "y": y})

async def api_emergency(request):
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
    return web.json_response({"status": "ok", "action": "emergency_stop"})

async def api_keyboard_ctrl(request):
    """键盘遥控：接收 (v_cmd, ω_γ)，做 Ackermann 逆解后发布 ControlCommand
    与 MPC 的 Layer 2+3 (AckermannAllocator) 逻辑一致"""
    body = await request.json()
    v_cmd = body.get('v_cmd', 0.0)
    omega_gamma = body.get('omega_gamma', 0.0)

    if ctrl_writer:
        # 读取当前铰接角 γ
        with state_lock:
            gamma = vehicle_state.get("gamma", 0.0)

        # Ackermann 逆解（与 MPC AckermannAllocator.allocate 一致）
        Lf = 0.77   # 前车铰接距离
        Lr = 0.77   # 后车铰接距离
        L_wb_front = 0.60  # 前车轴距
        L_wb_rear = 0.55   # 后车轴距

        L_eff = Lr + Lf * math.cos(gamma)
        if abs(L_eff) < 1e-9:
            L_eff = 1e-9

        omega_front = (v_cmd * math.sin(gamma) + Lr * omega_gamma) / L_eff
        omega_rear = omega_front - omega_gamma
        v_front = v_cmd
        v_rear = v_cmd * math.cos(gamma) + Lf * omega_front * math.sin(gamma)

        delta_front = math.atan(omega_front * L_wb_front / v_front) if abs(v_front) > 0.01 else 0.0
        delta_rear = math.atan(omega_rear * L_wb_rear / v_rear) if abs(v_rear) > 0.01 else 0.0

        # 限幅
        delta_front = max(-0.5, min(0.5, delta_front))
        delta_rear = max(-0.5, min(0.5, delta_rear))

        msg = ControlCommand()
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = "web_keyboard"
        msg.speed = v_front
        msg.steering_target = math.degrees(delta_front)
        msg.header.status.msg = (f"v_front={v_front:.4f},delta_front={delta_front:.4f},"
                                 f"v_rear={v_rear:.4f},delta_rear={delta_rear:.4f}")
        ctrl_writer.write(msg)
    return web.json_response({"status": "ok"})

async def api_start(request):
    body = await request.json()
    x, y, theta = body.get('x', -160), body.get('y', -11), body.get('theta', 0.1)
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], capture_output=True)
    with state_lock:
        vehicle_trail.clear()
    subprocess.Popen(["python3", "sim/sim_vehicle.py",
                      "--x", str(x), "--y", str(y), "--theta", str(theta)],
                     cwd="/apollo_workspace")
    return web.json_response({"status": "ok", "x": x, "y": y})

async def api_module(request):
    body = await request.json()
    name, action = body.get('name', ''), body.get('action', 'start')
    script_map = {
        "sim": ("sim/sim_vehicle.py", []),
        "rear_loc": ("modules/drivers/gnss/rear_localization_node.py", []),
        "localization": ("modules/drivers/gnss/rear_localization_node.py",
                         ["--topic", "/apollo/localization/pose",
                          "--ip", "192.168.1.103", "--port", "8680",
                          "--name", "front_localization_node"]),
    }
    launch_map = {
        "gnss": "modules/drivers/gnss/launch/gnss.launch",
        "canbus": "modules/canbus/launch/canbus.launch",
        "cilqr": "modules/planning/cilqr_planner/launch/cilqr_planner.launch",
        "mpc": "modules/control/mpc_controller/launch/mpc_controller.launch",
    }
    if name in script_map:
        script, extra_args = script_map[name]
        kill_pattern = " ".join([script] + extra_args) if extra_args else script
        if action == 'start':
            subprocess.Popen(["python3", script] + extra_args, cwd="/apollo_workspace")
            print(f"  Module {name}: started {script} {' '.join(extra_args)}")
        else:
            subprocess.run(["pkill", "-f", kill_pattern], capture_output=True)
            print(f"  Module {name}: stopped")
        return web.json_response({"status": "ok"})
    elif name in launch_map:
        cmd = ["cyber_launch", action, launch_map[name]]
        subprocess.Popen(cmd, cwd="/apollo_workspace")
        print(f"  Module {name}: {' '.join(cmd)}")
        return web.json_response({"status": "ok"})
    else:
        return web.json_response({"error": f"unknown: {name}"}, status=400)

async def api_record_start(request):
    global is_recording, record_start_time, record_filename
    with record_lock:
        is_recording = True
        record_frames.clear()
        record_start_time = time.time()
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        record_filename = f"rec_{ts}.json"
    print(f"  ⏺ Recording started: {record_filename}")
    return web.json_response({"status": "ok", "filename": record_filename})

async def api_record_stop(request):
    global is_recording
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
            resp = {"status": "ok", "filename": record_filename,
                    "frames": len(record_frames), "duration": round(duration, 1)}
            record_frames.clear()
            return web.json_response(resp)
        else:
            return web.json_response({"status": "ok", "frames": 0})

async def api_records_delete(request):
    body = await request.json()
    fname = body.get('filename', '')
    fpath = RECORD_DIR / fname
    if fpath.exists() and fpath.suffix == '.json':
        fpath.unlink()
        return web.json_response({"status": "ok"})
    return web.json_response({"error": "not found"}, status=404)

async def index_handler(request):
    return web.FileResponse(STATIC_DIR / 'index.html')


# ═══════════════════════════════════════════════════════════
#  启动（单端口 HTTP+WS，仿照 DreamView CivetServer）
# ═══════════════════════════════════════════════════════════

async def main():
    print("=" * 60)
    print("  铰接车实时 Web 可视化")
    print("=" * 60)

    load_vehicle_params()
    load_map_data()

    # Cyber RT 线程
    threading.Thread(target=cyber_thread_func, daemon=True).start()
    await asyncio.sleep(1)

    # Cyber RT 会覆盖 SIGINT，在它初始化后重新设置强制退出
    def _force_exit(*_):
        print("\n  已停止")
        os._exit(0)
    signal.signal(signal.SIGINT, _force_exit)
    signal.signal(signal.SIGTERM, _force_exit)

    # 创建 aiohttp 应用（单端口 HTTP + WS + 静态文件）
    app = web.Application()
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/map', api_map)
    app.router.add_get('/api/state', api_state)
    app.router.add_get('/api/records', api_records_list)
    app.router.add_get('/api/record/{fname}', api_record_download)
    app.router.add_post('/api/goal', api_goal)
    app.router.add_post('/api/emergency', api_emergency)
    app.router.add_post('/api/keyboard_ctrl', api_keyboard_ctrl)
    app.router.add_post('/api/start', api_start)
    app.router.add_post('/api/module', api_module)
    app.router.add_post('/api/record/start', api_record_start)
    app.router.add_post('/api/record/stop', api_record_stop)
    app.router.add_post('/api/records/delete', api_records_delete)
    app.router.add_get('/', index_handler)
    app.router.add_static('/', STATIC_DIR)

    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8888)
    await site.start()

    print(f"  ✓ 单端口服务器启动: http://localhost:8888 (HTTP + WS)")
    print("=" * 60)

    # 启动 20Hz 广播循环
    asyncio.create_task(broadcast_loop())

    # 永久运行
    await asyncio.Event().wait()

if __name__ == "__main__":
    asyncio.run(main())

