# CILQR Planner 部署指南

> 将 CILQR + Hybrid A* 铰接车规划器部署到实车 Apollo 系统

---

## 1. 需要复制的文件

将以下目录 **整体复制** 到目标工控机的 Apollo 工程中：

```
<apollo_workspace>/modules/planning/cilqr_planner/
```

完整文件清单：

```
modules/planning/cilqr_planner/
├── BUILD                              # Bazel 构建文件
├── cyberfile.xml                      # Apollo 包描述 & 依赖声明
├── cilqr_planner_component.h          # Cyber RT Component 头文件
├── cilqr_planner_component.cc         # Cyber RT Component 实现
├── cilqr_lib/                         # 核心算法库（纯 C++，无 Cyber 依赖）
│   ├── include/
│   │   ├── common_types.h             # State, Point, MapData 等基础类型
│   │   ├── ilqr.h                     # CILQRSolver, Vehicle, GlobalPlan
│   │   ├── articulated_hybrid_astar.h # Hybrid A* 全局规划器
│   │   ├── config_loader.h            # JSON 配置加载
│   │   └── utils.h                    # 工具函数
│   └── src/
│       ├── ilqr.cpp
│       ├── articulated_hybrid_astar.cpp
│       ├── config_loader.cpp
│       ├── utils.cpp
│       └── vehicle.cpp
├── proto/
│   ├── BUILD
│   └── cilqr_planner_conf.proto       # Protobuf 配置定义
├── conf/
│   ├── cilqr_planner.conf             # gflag 配置
│   ├── cilqr_planner_config.pb.txt    # ★ Protobuf 配置（需修改）
│   └── cilqr_json/                    # 算法参数 JSON
│       ├── main.json
│       ├── ilqr.json
│       ├── hybrid_astar.json
│       └── vehicle.json
├── dag/
│   └── cilqr_planner.dag              # Component DAG 描述
├── launch/
│   └── cilqr_planner.launch           # cyber_launch 启动文件
├── data/
│   └── atv_terrain_global_map.json    # ★ 栅格地图（需替换为实际地图）
└── test/
    └── test_cilqr_publisher.py        # 测试脚本（可选复制）
```

---

## 2. 编译 & 安装

### 2.1 前提条件

目标工控机上必须已有：
- Apollo Docker 环境（`aem` 已安装）
- 能正常执行 `buildtool build`

### 2.2 编译步骤

```bash
# 进入容器
aem enter

# 编译 (根据工控机 CPU 核数调整 -j)
buildtool build -p modules/planning/cilqr_planner -j $(nproc) -m 16384
```

编译成功后自动安装到：
```
/opt/apollo/neo/lib/modules/planning/cilqr_planner/libcilqr_planner_component.so
/opt/apollo/neo/share/modules/planning/cilqr_planner/  (conf, dag, launch, data)
```

### 2.3 依赖说明

| 依赖 | 说明 | 安装方式 |
|---|---|---|
| `cyber` | Apollo Cyber RT 框架 | Apollo 自带 |
| `eigen` | 线性代数库 | Apollo 自带 (`@eigen`) |
| `opencv` | 图像处理（地图加载用） | `cyberfile.xml` 中已声明，自动安装 |
| `nlohmann_json` | JSON 解析（通过 `@com_github_nlohmann_json`） | Apollo 自带 |

> **不需要额外安装任何依赖**，所有依赖都已在 `BUILD` 和 `cyberfile.xml` 中声明。

---

## 3. 输入/输出接口

### 3.1 输入 Channel

| Channel | 消息类型 | 来源模块 | 说明 |
|---|---|---|---|
| `/apollo/localization/pose` | `LocalizationEstimate` | localization | 车辆位置、航向、速度 |
| `/apollo/planning/command` | `PlanningCommand` | Dreamview+ / external_command | 目标点（包含 waypoint） |
| `/apollo/prediction` | `PredictionObstacles` | prediction | 障碍物预测轨迹（可选） |
| `/apollo/canbus/chassis` | `Chassis` | canbus | 预留，当前未使用 |

### 3.2 输出 Channel

| Channel | 消息类型 | 说明 |
|---|---|---|
| `/apollo/planning` | `ADCTrajectory` | 规划轨迹 |

### 3.3 ADCTrajectory 输出格式

每个 `trajectory_point` 包含：

| 字段 | 含义 | 单位 |
|---|---|---|
| `path_point.x` | 世界坐标 X | m |
| `path_point.y` | 世界坐标 Y | m |
| `path_point.theta` | 前车航向角 | rad |
| `path_point.kappa` | **铰接角 γ**（非曲率） | rad |
| `v` | 期望线速度 | m/s |
| `relative_time` | 相对时间 | s |

> ⚠️ **注意**：`kappa` 字段存储的是铰接角 γ，不是标准曲率。后续控制器需要按此约定读取。

---

## 4. 需要修改的配置

### 4.1 ★ 地图文件（必须修改）

替换 `data/atv_terrain_global_map.json` 为实际场地的栅格地图。

地图 JSON 格式：
```json
{
  "metadata": {
    "width": 525,
    "height": 251,
    "resolution": 0.2,
    "origin": [0.0, 0.0],
    "max_elevation": 10.0
  },
  "data": [[...], ...]
}
```
- `data[row][col]`：`-1` = 障碍物，`≥0` = 可通行（值为高程）
- `resolution`：每像素的实际米数
- `origin`：地图左下角的世界坐标 `[x, y]`

修改 `conf/cilqr_planner_config.pb.txt`：
```protobuf
map_file: "modules/planning/cilqr_planner/data/你的地图文件.json"
```

### 4.2 ★ 车辆参数（必须修改）

修改 `conf/cilqr_json/vehicle.json` 中的车辆物理参数，确保与实车匹配：
- `lf` / `lr`：前后车体半长
- `width`：车体宽度
- `gamma_max`：最大铰接角

同时检查 `cilqr_planner_component.cc` 中硬编码的参数（第 103-106 行）：
```cpp
system_model_.lf = 0.77;    // ← 改为实车参数
system_model_.lr = 0.77;
system_model_.len = 0.90;
system_model_.width = 0.40;
```

### 4.3 Channel 适配（按需修改）

如果实车的 channel 名称不同，修改 `conf/cilqr_planner_config.pb.txt`：

```protobuf
localization_topic: "/apollo/localization/pose"        # 定位 channel
planning_command_topic: "/apollo/planning/command"      # 目标点 channel
prediction_topic: "/apollo/prediction"                  # 预测障碍物
chassis_topic: "/apollo/canbus/chassis"                 # 底盘状态
planning_trajectory_topic: "/apollo/planning"           # 输出轨迹
articulation_topic: "/apollo/vehicle/articulation"      # 铰接角输入（未实现）
```

### 4.4 算法参数调整（可选）

| 文件 | 内容 |
|---|---|
| `conf/cilqr_json/main.json` | 期望速度、时间步长、优化代价权重 |
| `conf/cilqr_json/ilqr.json` | iLQR 迭代次数、收敛阈值 |
| `conf/cilqr_json/hybrid_astar.json` | A* 搜索分辨率、启发式权重 |

---

## 5. 启动 & 停止

```bash
# 启动
cyber_launch start modules/planning/cilqr_planner/launch/cilqr_planner.launch

# 停止
cyber_launch stop modules/planning/cilqr_planner/launch/cilqr_planner.launch

# 检查输出
cyber_monitor   # 查看 /apollo/planning channel
```

---

## 6. 与实车系统集成检查清单

- [ ] **地图**：已替换为实际场地的栅格地图
- [ ] **车辆参数**：`vehicle.json` 和 `cilqr_planner_component.cc` 中参数与实车一致
- [ ] **定位模块**：localization 正常运行，`/apollo/localization/pose` 有数据
- [ ] **铰接角**：
  - 当前铰接角 γ 默认为 0（`cur_state_[3]`）
  - 需要一个独立 component 计算 γ = θ_front - θ_rear 并发布到 `/apollo/vehicle/articulation`
  - 或在 `cilqr_planner_component.cc` 的 chassis_reader_ 回调中直接计算
- [ ] **目标点发送**：通过 Dreamview+ 或自定义节点发送 `PlanningCommand`
- [ ] **控制器**：下游控制器能正确读取 `ADCTrajectory`，注意 `kappa` 字段是铰接角

---

## 7. 常见问题

### Q: 编译报 `-Werror` 警告错误
已在 BUILD 中通过 copts 抑制，如果出现新的警告，添加对应的 `-Wno-error=xxx`。

### Q: 启动时 "Permission denied [log/planner]"
日志已改写到 `/tmp/planner/`，如果仍有问题检查容器内 `/tmp` 权限。

### Q: planner 启动但不输出轨迹
检查是否同时满足：(1) 收到定位消息 (2) 收到目标点。可用测试脚本验证：
```bash
python3 modules/planning/cilqr_planner/test/test_cilqr_publisher.py
```

### Q: 如何切换回 Apollo 原版 planning？
停止 CILQR planner，启动原版：
```bash
cyber_launch stop modules/planning/cilqr_planner/launch/cilqr_planner.launch
cyber_launch start modules/planning/planning_component/launch/planning.launch
```
