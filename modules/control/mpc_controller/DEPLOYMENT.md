# 铰接车 CILQR + MPC 实车部署指南

## 一、文件清单

### CILQR Planner（C++ 组件，需编译）

```
modules/planning/cilqr_planner/
├── cilqr_planner_component.h          # 组件头文件
├── cilqr_planner_component.cc         # 组件实现
├── BUILD                              # 编译规则
├── cyberfile.xml                      # 包描述
├── proto/cilqr_planner_conf.proto     # 配置 proto
├── cilqr_lib/                         # 算法核心库（整个目录）
│   ├── include/*.h
│   └── src/*.cpp
├── conf/
│   ├── cilqr_planner_config.pb.txt    # 组件配置
│   └── cilqr_json/                    # 算法参数
│       ├── main.json
│       ├── ilqr.json
│       ├── hybrid_astar.json
│       └── vehicle.json               # ★ 需要根据实车修改
├── dag/cilqr_planner.dag
├── launch/cilqr_planner.launch
└── data/maps/                         # 地图文件
    └── *.png
```

### MPC Controller（Python 脚本，无需编译）

```
modules/control/mpc_controller/
├── mpc_controller_node.py             # 控制器主程序
├── conf/
│   ├── vehicle.json                   # ★ 实车参数（必改）
│   └── mpc_params.json                # ★ MPC 权重（需调参）
├── launch/mpc_controller.launch
└── dag/mpc_controller.dag
```

---

## 二、实车部署步骤

### Step 1: 确认工控机 Apollo 环境

```bash
# 工控机上需要已安装 Apollo 9.0+ Docker 环境
docker ps | grep apollo
# 进入容器
docker exec -it <container_name> bash
```

### Step 2: 复制文件到工控机

```bash
# 在开发机上打包
cd ~/code/apollo
tar czf cilqr_mpc.tar.gz \
  modules/planning/cilqr_planner/ \
  modules/control/mpc_controller/

# 传到工控机
scp cilqr_mpc.tar.gz user@IPC_IP:/path/to/apollo_workspace/

# 工控机上解压
cd /path/to/apollo_workspace
tar xzf cilqr_mpc.tar.gz
```

### Step 3: 编译 Planner

```bash
# 容器内
buildtool build -p modules/planning/cilqr_planner -j$(nproc)
buildtool install -p modules/planning/cilqr_planner
```

### Step 4: 安装 Python 依赖

```bash
# 容器内
pip3 install osqp scipy numpy
```

### Step 5: 运行

```bash
# 终端 1: Planner
cyber_launch start modules/planning/cilqr_planner/launch/cilqr_planner.launch

# 终端 2: Controller
python3 modules/control/mpc_controller/mpc_controller_node.py
```

---

## 三、必须修改的参数

### ★ `modules/control/mpc_controller/conf/vehicle.json`

```jsonc
{
  "front_body": {
    "wheelbase": 0.90,      // ← 改成前车实际轴距 (m)
    "delta_max": 0.5236,    // ← 前车最大转角 (rad ≈ 30°)
    "delta_min": -0.5236,
    "v_max": 3.0,           // ← 前车最大速度 (m/s)
    "v_min": -0.5
  },
  "rear_body": {
    "wheelbase": 0.90,      // ← 改成后车实际轴距 (m)
    "delta_max": 0.5236,    // ← 后车最大转角 (rad)
    "delta_min": -0.5236,
    "v_max": 3.0,
    "v_min": -0.5
  },
  "articulation": {
    "Lf": 0.45,             // ← 前车中心到铰接点距离 (m)
    "Lr": 0.45,             // ← 后车中心到铰接点距离 (m)
    "gamma_max": 1.0472,    // ← 最大铰接角 (rad ≈ 60°)
    "gamma_min": -1.0472,
    "omega_gamma_max": 0.5  // ← 最大铰接角速度 (rad/s)
  }
}
```

### ★ `modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json`

```jsonc
{
  "lf": 0.77,        // ← 改成实车前体长度的一半
  "lr": 0.77,        // ← 改成实车后体长度的一半
  "len": 0.90,       // ← 车体长度
  "width": 0.40,     // ← 车体宽度
  "ego_rad": 0.30    // ← 碰撞检测半径
}
```

> **重要**：两个 `vehicle.json` 的 Lf/Lr 参数必须一致！

### `modules/control/mpc_controller/conf/mpc_params.json`

```jsonc
{
  "horizons": {
    "Np": 20,         // 预测步数（增大→更平滑但更慢）
    "Nc": 10,         // 控制步数
    "dt": 0.05        // 控制周期 (s)
  },
  "weights": {
    "Q": [10, 10, 5, 2],   // [x, y, θ, γ] 跟踪权重
    "R": [1, 1],           // [v, ω_γ] 控制代价
    "S": [0.1, 0.1]        // 控制增量平滑
  }
}
```

---

## 四、接口适配（实车必须做）

### 4.1 CAN 总线适配器

Controller 输出在 `/apollo/control` 的 `ControlCommand` 里：

```
speed            → v_front (m/s)
steering_target  → δ_front (deg)
header.status.msg → "v_front=X,delta_front=X,v_rear=X,delta_rear=X"
```

你需要写一个 **CAN bus adapter** 从 debug string 解析 4 路指令并发 CAN 帧：

```python
# 伪代码
def on_control(msg):
    ctrl = parse_debug_string(msg.header.status.msg)
    can_bus.send(FRONT_SPEED_ID, encode_float(ctrl['v_front']))
    can_bus.send(FRONT_STEER_ID, encode_float(ctrl['delta_front']))
    can_bus.send(REAR_SPEED_ID,  encode_float(ctrl['v_rear']))
    can_bus.send(REAR_STEER_ID,  encode_float(ctrl['delta_rear']))
```

### 4.2 铰接角 γ 来源

Controller 从 `/apollo/canbus/chassis` 的 `Chassis.steering_percentage` 读取 γ (deg)。

实车需要一个节点计算并发布 γ：
```
γ = θ_front_INS - θ_rear_INS    (双 INS 航向差)
或
γ = 铰接角传感器直接读数
```

### 4.3 定位来源

Planner 和 Controller 都从 `/apollo/localization/pose` 读位置。
确保实车的 INS/RTK 驱动发布到这个 channel。

### 4.4 目标点来源

Planner 从 `/apollo/planning/command` 接收目标点。
可以通过 Dreamview+ 点击发送，或写脚本发送。

---

## 五、通信拓扑

```
INS/RTK → /apollo/localization/pose → Planner + Controller
                                         ↓
铰接传感器 → /apollo/canbus/chassis → Controller
                                         ↓
目标点 → /apollo/planning/command → Planner
                                      ↓
                               /apollo/planning → Controller
                                                     ↓
                                              /apollo/control → CAN Adapter → ECU
```

---

## 六、注意事项

1. **先低速测试**：初始 `v_max` 设为 0.5 m/s，确认方向正确后再提速
2. **急停开关**：必须有物理急停按钮，不依赖软件
3. **坐标系**：确认 INS 输出的坐标系与地图坐标系一致（通常是 UTM 或 ENU）
4. **转角方向**：确认 δ 正方向与实车一致（左转为正 or 右转为正）
5. **日志**：Controller 自动写 CSV 到 `/tmp/controller/`，可用于事后分析
