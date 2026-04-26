#!/usr/bin/env python3
"""
铰接车实时 Web 可视化后端
FastAPI + Cyber RT → WebSocket → 浏览器 Canvas

用法（在容器内）：
  python3 sim/web_server.py

浏览器打开：http://localhost:8888
"""

import asyncio
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

def _force_exit(*_):
    os._exit(0)
# ── FastAPI ──
try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import FileResponse, JSONResponse
    import uvicorn
except ImportError:
    print("请先安装: pip3 install fastapi uvicorn[standard]")
    raise

# ── Cyber RT ──
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.planning_msgs.planning_command_pb2 import PlanningCommand
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis

# ── 路径 ──
SCRIPT_DIR = Path(__file__).parent.resolve()
STATIC_DIR = SCRIPT_DIR / "static"
APOLLO_ROOT = SCRIPT_DIR.parent
PLANNER_CONFIG = APOLLO_ROOT / "modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt"
VEHICLE_CONFIG = APOLLO_ROOT / "modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json"

# ── 全局状态 ──
state_lock = threading.Lock()
vehicle_state = {
    "x": 0.0, "y": 0.0, "theta": 0.0,
    "gamma": 0.0, "speed": 0.0,
    "rear_theta": 0.0,
}
vehicle_trail = []  # [(x, y), ...] 最近 2000 个点
cilqr_traj = []     # [(x, y), ...]
global_path = []    # [(x, y), ...]
vehicle_params = None
map_cache = None    # 预加载的地图数据
modules_status = {"cilqr": False, "mpc": False, "canbus": False}

# WebSocket 客户端集合
ws_clients = set()


# ═══════════════════════════════════════════════════════════════
#  Cyber RT 回调
# ═══════════════════════════════════════════════════════════════

def on_localization(msg):
    with state_lock:
        vehicle_state["x"] = msg.pose.position.x
        vehicle_state["y"] = msg.pose.position.y
        vehicle_state["theta"] = msg.pose.heading
        # 从 localization 速度向量算速度（比 chassis topic 更稳定）
        vx = msg.pose.linear_velocity.x
        vy = msg.pose.linear_velocity.y
        vehicle_state["speed"] = math.sqrt(vx * vx + vy * vy)
        vehicle_trail.append((msg.pose.position.x, msg.pose.position.y))
        if len(vehicle_trail) > 2000:
            vehicle_trail.pop(0)


def on_rear_localization(msg):
    with state_lock:
        vehicle_state["rear_theta"] = msg.pose.heading
        theta = vehicle_state["theta"]
        gamma = theta - msg.pose.heading
        while gamma > math.pi:
            gamma -= 2 * math.pi
        while gamma < -math.pi:
            gamma += 2 * math.pi
        vehicle_state["gamma"] = gamma


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
    if not pts:
        return
    with state_lock:
        if not global_path:
            # 首次接收
            global_path = pts
        else:
            # 判断是否是全新路径（终点变了 > 5m）
            old_end = global_path[-1]
            new_end = pts[-1]
            dist = math.sqrt((old_end[0] - new_end[0])**2 + (old_end[1] - new_end[1])**2)
            if dist > 5.0:
                # 新目标，替换
                global_path = pts
            # 否则保留原始完整路径（不被截断版本覆盖）


# ═══════════════════════════════════════════════════════════════
#  配置加载
# ═══════════════════════════════════════════════════════════════

def load_vehicle_params():
    global vehicle_params
    config_path = str(VEHICLE_CONFIG)
    if not os.path.exists(config_path):
        # 容器内路径
        config_path = "/apollo_workspace/modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json"
    if not os.path.exists(config_path):
        print(f"  ⚠ vehicle.json not found")
        return
    try:
        with open(config_path, 'r') as f:
            raw = f.read()
            raw = re.sub(r'//.*', '', raw)
            data = json.loads(raw)
            vehicle_params = {
                "L_f": data["front_body"]["pivot_distance"],
                "L_r": data["rear_body"]["pivot_distance"],
                "L_f_body": data["front_body"]["body_length"],
                "L_r_body": data["rear_body"]["body_length"],
                "W_f_body": data["front_body"]["body_width"],
                "W_r_body": data["rear_body"]["body_width"],
            }
        print(f"  ✓ Vehicle params: {vehicle_params}")
    except Exception as e:
        print(f"  ⚠ Failed to load vehicle.json: {e}")


def load_map_data():
    global map_cache
    try:
        map_path = None
        config_path = str(PLANNER_CONFIG)
        if not os.path.exists(config_path):
            config_path = "/apollo_workspace/modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt"
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                for line in f:
                    if 'map_file' in line:
                        map_path = line.split('"')[1].strip()
                        break
        if not map_path or not os.path.exists(map_path):
            print(f"  ⚠ Map file not found: {map_path}")
            return
        print(f"  Loading map: {map_path}")
        with open(map_path, 'r') as f:
            raw = json.load(f)

        meta = raw.get('metadata', raw)
        dims = meta.get('dimensions', meta)
        map_cache = {
            "width": dims.get("width", meta.get("width", 0)),
            "height": dims.get("height", meta.get("height", 0)),
            "resolution": dims.get("resolution", meta.get("resolution", 1.0)),
            "origin": meta.get("origin", [0.0, 0.0]),
            "data": raw["data"],  # 2D array
        }
        print(f"  ✓ Map loaded: {map_cache['width']}×{map_cache['height']}")
    except Exception as e:
        print(f"  ⚠ Map load failed: {e}")


# ═══════════════════════════════════════════════════════════════
#  Cyber RT 初始化（在后台线程运行）
# ═══════════════════════════════════════════════════════════════

cyber_node = None
goal_writer = None


def cyber_thread_func():
    global cyber_node, goal_writer
    cyber.init()
    cyber_node = cyber.Node("web_visualizer")

    cyber_node.create_reader("/apollo/localization/pose",
                             LocalizationEstimate, on_localization)
    cyber_node.create_reader("/apollo/localization/pose_rear",
                             LocalizationEstimate, on_rear_localization)
    cyber_node.create_reader("/apollo/canbus/chassis",
                             Chassis, on_chassis)
    cyber_node.create_reader("/apollo/planning",
                             ADCTrajectory, on_trajectory)
    cyber_node.create_reader("/apollo/planning/global_path",
                             ADCTrajectory, on_global_trajectory)

    goal_writer = cyber_node.create_writer(
        "/apollo/planning/command", PlanningCommand)

    print("  ✓ Cyber RT initialized")
    # 不用 spin()，readers 是回调驱动的，只需保持线程存活
    while True:
        time.sleep(1)


# ═══════════════════════════════════════════════════════════════
#  FastAPI 应用
# ═══════════════════════════════════════════════════════════════

app = FastAPI(title="Articulated Vehicle Visualizer")


@app.on_event("startup")
async def startup():
    load_vehicle_params()
    load_map_data()

    # 启动 Cyber RT 线程
    t = threading.Thread(target=cyber_thread_func, daemon=True)
    t.start()
    await asyncio.sleep(2)  # 等待 Cyber 初始化

    # Cyber RT 的 C++ 层会覆盖 SIGINT，我们在主线程重新设置
    signal.signal(signal.SIGINT, _force_exit)
    signal.signal(signal.SIGTERM, _force_exit)

    # 启动广播任务
    asyncio.create_task(broadcast_state())


async def broadcast_state():
    """20Hz 向所有 WebSocket 客户端推送状态"""
    global ws_clients
    while True:
        if ws_clients:
            with state_lock:
                data = {
                    "vehicle": dict(vehicle_state),
                    "trail": vehicle_trail[-500:],  # 只发最近 500 个点
                    "cilqr_traj": cilqr_traj,
                    "global_path": global_path,
                }
            msg = json.dumps(data)
            dead = set()
            for ws in ws_clients:
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.add(ws)
            ws_clients -= dead
        await asyncio.sleep(0.05)  # 20Hz


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global ws_clients
    await ws.accept()
    ws_clients.add(ws)
    print(f"  WebSocket connected ({len(ws_clients)} clients)")
    try:
        # 首次连接发送配置
        init_data = {
            "type": "init",
            "vehicle_params": vehicle_params,
        }
        await ws.send_text(json.dumps(init_data))
        # 保持连接
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        ws_clients.discard(ws)
        print(f"  WebSocket disconnected ({len(ws_clients)} clients)")


@app.get("/api/map")
async def get_map():
    if map_cache is None:
        return JSONResponse({"error": "map not loaded"}, status_code=404)
    return JSONResponse(map_cache)


@app.post("/api/goal")
async def set_goal(data: dict):
    x = data.get("x", 0.0)
    y = data.get("y", 0.0)
    theta = data.get("theta", 0.0)

    if goal_writer:
        msg = PlanningCommand()
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = "web_visualizer"
        wp = msg.lane_follow_command.routing_request.waypoint.add()
        wp.pose.x = x
        wp.pose.y = y
        wp.heading = theta
        goal_writer.write(msg)
        print(f"  Goal set: ({x:.1f}, {y:.1f}, θ={math.degrees(theta):.1f}°)")
        return {"status": "ok", "x": x, "y": y}
    return JSONResponse({"error": "cyber not ready"}, status_code=503)


@app.post("/api/start")
async def set_start(data: dict):
    """设置起点：停旧 sim，以新坐标启动新 sim"""
    x = data.get("x", -160.0)
    y = data.get("y", -11.0)
    theta = data.get("theta", 0.1)

    # 停掉旧 sim
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], capture_output=True)
    await asyncio.sleep(0.5)

    # 清空轨迹
    global vehicle_trail
    with state_lock:
        vehicle_trail = []

    # 以新坐标启动
    subprocess.Popen(
        ["python3", "sim/sim_vehicle.py",
         "--x", str(x), "--y", str(y), "--theta", str(theta)],
        cwd="/apollo_workspace"
    )
    print(f"  Start set: ({x:.1f}, {y:.1f}, θ={math.degrees(theta):.1f}°)")
    return {"status": "ok", "x": x, "y": y}


@app.post("/api/module")
async def toggle_module(data: dict):
    name = data.get("name", "")
    action = data.get("action", "start")

    # sim_vehicle 特殊处理（Python 脚本，不是 cyber_launch）
    if name == "sim":
        if action == "start":
            subprocess.Popen(
                ["python3", "sim/sim_vehicle.py"],
                cwd="/apollo_workspace"
            )
            print("  Module sim: started sim_vehicle.py")
        else:
            subprocess.run(["pkill", "-f", "sim_vehicle.py"], capture_output=True)
            print("  Module sim: stopped sim_vehicle.py")
        modules_status["sim"] = (action == "start")
        return {"status": "ok"}

    launch_map = {
        "cilqr": "modules/planning/cilqr_planner/launch/cilqr_planner.launch",
        "mpc": "modules/control/mpc_controller/launch/mpc_controller.launch",
        "canbus": "modules/canbus/launch/canbus.launch",
    }

    if name in launch_map:
        launch = launch_map[name]
        cmd = ["cyber_launch", action, launch]
        print(f"  Module {name}: {' '.join(cmd)}")
        if action == "start":
            subprocess.Popen(cmd, cwd="/apollo_workspace")
        else:
            subprocess.Popen(cmd, cwd="/apollo_workspace")
        modules_status[name] = (action == "start")
        return {"status": "ok"}

    return JSONResponse({"error": f"unknown module: {name}"}, status_code=400)


# 静态文件
app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")


# ═══════════════════════════════════════════════════════════════
#  启动
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  铰接车实时 Web 可视化")
    print("=" * 60)
    print(f"  打开浏览器: http://localhost:8888")
    print("=" * 60)

    uvicorn.run(app, host="0.0.0.0", port=8888, log_level="warning")
