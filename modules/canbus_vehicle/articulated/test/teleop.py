#!/usr/bin/env python3
"""
铰接车键盘遥控脚本 (支持前车/后车/整车拖挂/双独立四种控制模式)

启动后选择控制模式:
  1 - 前车控制 (驱动电机 + EPS)
  2 - 后车控制 (Hunter SE)
  3 - 整车拖挂 (前车转向 + 后车直行跟随)
  4 - 双独立控制 (WSAD控前车, IKJL控后车)

键盘控制 (模式 1/2/3):
  M / m  ->  切换模式 (Standby / CAN控制)
  W / S  ->  加速/减速 (±0.05 m/s)
  A / D  ->  左转/右转 (±0.05 rad)
  空格   ->  急停 (速度/转角归零)
  Q / q  ->  退出

键盘控制 (模式 4 — 双独立):
  W / S  ->  前车 加速/减速
  A / D  ->  前车 左转/右转
  I / K  ->  后车 加速/减速
  J / L  ->  后车 左转/右转
  空格   ->  全部急停
  Q / q  ->  退出

使用方法:
  cyber_launch start modules/canbus/launch/canbus.launch
  python3 modules/canbus_vehicle/articulated/test/teleop.py
"""

import sys
import time
import select
import termios
import tty
import threading

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.articulated.proto.articulated_pb2 import Articulated
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.control_msgs.pad_msg_pb2 import PadMessage, START, RESET

# ---------- 参数 ----------
SPEED_INC = 0.05       # m/s 每次增量
STEER_INC = 0.05      # rad 每次增量 (≈2.9°)
MAX_SPEED = 0.5       # m/s 最大限速
MAX_STEER_FRONT = 0.47  # rad 前车最大转角 (≈27°, EPS 137° / 转向比 5.074)
MAX_STEER_REAR = 0.40   # rad 后车最大转角 (≈23°, Hunter SE DBC: [-0.4, 0.4])
SEND_HZ = 50          # 发送频率
RAD2DEG = 180.0 / 3.14159265

# 前车转向比 (与 articulated_controller.cc 保持一致)
FRONT_STEER_RATIO = 5.074
FRONT_MAX_EPS_DEG = 137.0  # EPS 电机最大角度 (标定值)
REAR_SPEED_RATIO = 1.0     # 后车速度 = 前车速度 × 此比值 (拖挂模式)
# ---------------------------

# 控制模式常量
MODE_FRONT = 1
MODE_REAR = 2
MODE_TOW = 3       # 整车拖挂 (前车转向, 后车直行)
MODE_DUAL = 4      # 双独立控制 (WSAD前车, IKJL后车)


class ArticulatedTeleop:
    def __init__(self, control_mode):
        self.control_mode = control_mode

        # 前车控制量
        self.v_front = 0.0        # m/s
        self.delta_front = 0.0    # rad (前轮转角)

        # 后车控制量
        self.v_rear = 0.0         # m/s
        self.delta_rear = 0.0     # rad

        self.is_enabled = False
        self.running = True

        # 非阻塞 pad 动作队列 (由 _send_loop 消费)
        self._pending_pad_action = None
        self._pending_pad_count = 0
        self._pad_lock = threading.Lock()

        # 状态显示节流
        self._last_print_time = 0.0

        # 前车反馈数据
        self.fb_front_speed = 0.0     # km/h -> m/s
        self.fb_front_steer = 0.0     # EPS 角度 (度)
        self.fb_front_mode = "?"
        self.fb_front_motor_en = False
        self.fb_front_eps_en = False

        # 后车反馈数据
        self.fb_rear_speed = 0.0
        self.fb_rear_steer = 0.0
        self.fb_rear_mode = "?"
        self.fb_rear_battery = 0.0
        self.fb_rear_state = "?"

        cyber.init()
        self.node = cyber.Node("articulated_teleop")
        self.cmd_writer = self.node.create_writer(
            "/apollo/control", ControlCommand)
        self.node.create_reader(
            "/apollo/canbus/chassis_detail", Articulated, self._chassis_cb)
        time.sleep(0.5)

    def _chassis_cb(self, msg):
        """解析前后车反馈"""
        # ---- 后车反馈 ----
        if msg.HasField("rear_motion_feedback_545"):
            fb = msg.rear_motion_feedback_545
            self.fb_rear_speed = fb.linear_speed
            self.fb_rear_steer = fb.steering_angle
        if msg.HasField("rear_chassis_status_529"):
            cs = msg.rear_chassis_status_529
            mode_map = {0: "待机", 1: "CAN", 2: "遥控"}
            state_map = {0: "正常", 1: "急停", 2: "故障"}
            self.fb_rear_mode = mode_map.get(cs.control_mode, "?")
            self.fb_rear_state = state_map.get(cs.vehicle_state, "?")
            self.fb_rear_battery = cs.battery_voltage

        # ---- 前车反馈 ----
        if msg.HasField("front_drivemotor_acu_572"):
            dm = msg.front_drivemotor_acu_572
            self.fb_front_speed = dm.drive_motor_speed / 3.6  # km/h -> m/s
            self.fb_front_motor_en = dm.drive_motor_enable
        if msg.HasField("front_eps_acu_556"):
            eps = msg.front_eps_acu_556
            self.fb_front_steer = eps.eps_angle  # EPS 电机角度 (度)
            self.fb_front_eps_en = eps.eps_enable
        if msg.HasField("front_vcu_acu_general_524"):
            vcu = msg.front_vcu_acu_general_524
            if hasattr(vcu, 'acu_remote_control') and vcu.acu_remote_control:
                self.fb_front_mode = "遥控"
            elif hasattr(vcu, 'acu_control_mode') and vcu.acu_control_mode:
                self.fb_front_mode = "自动"
            else:
                self.fb_front_mode = "手动"

    def _make_cmd(self, pad_action=None):
        """构造 ControlCommand, 前车用标准字段, 后车用 header.status.msg"""
        cmd = ControlCommand()
        cmd.header.timestamp_sec = time.time()
        cmd.header.module_name = "articulated_teleop"

        # 后车命令编码到 header.status.msg (始终填写，controller 会解析)
        # 整车拖挂模式: 后车速度 = 前车速度 × REAR_SPEED_RATIO
        if self.control_mode == MODE_TOW:
            v_rear_actual = self.v_rear * REAR_SPEED_RATIO
        else:
            v_rear_actual = self.v_rear
        cmd.header.status.msg = (
            f"v_front={self.v_front},delta_front={self.delta_front},"
            f"v_rear={v_rear_actual},delta_rear={self.delta_rear}"
        )

        # 前车命令: speed (m/s), steering_target (前轮转角, 度)
        cmd.speed = self.v_front
        # 前轮转角: rad -> deg
        cmd.steering_target = self.delta_front * 180.0 / 3.14159265
        cmd.gear_location = Chassis.GEAR_DRIVE if self.v_front >= 0 else Chassis.GEAR_REVERSE

        if pad_action is not None:
            cmd.pad_msg.header.timestamp_sec = time.time()
            cmd.pad_msg.header.module_name = "articulated_teleop"
            cmd.pad_msg.action = pad_action
        return cmd

    def _send_loop(self):
        """后台线程: 持续以 SEND_HZ 频率发送 ControlCommand"""
        period = 1.0 / SEND_HZ
        while self.running:
            # 检查是否有待发送的 pad 动作 (M 键触发)
            pad_action = None
            with self._pad_lock:
                if self._pending_pad_count > 0:
                    pad_action = self._pending_pad_action
                    self._pending_pad_count -= 1
                    if self._pending_pad_count <= 0:
                        self._pending_pad_action = None
            cmd = self._make_cmd(pad_action=pad_action)
            self.cmd_writer.write(cmd)
            time.sleep(period)

    def _print_status(self):
        # 节流: 最多 5Hz 刷新状态显示, 避免 stdout 阻塞主循环
        now = time.time()
        if now - self._last_print_time < 0.2:
            return
        self._last_print_time = now

        ctrl_name = {MODE_FRONT: "前车", MODE_REAR: "后车", MODE_TOW: "拖挂", MODE_DUAL: "双独立"}[self.control_mode]

        # 用底盘反馈的真实模式，而不是内部 is_enabled 变量
        if self.control_mode == MODE_FRONT:
            actual_mode = self.fb_front_mode
            is_can = (self.fb_front_mode == "自动")
        elif self.control_mode == MODE_REAR:
            actual_mode = self.fb_rear_mode
            is_can = (self.fb_rear_mode == "CAN")
        else:  # MODE_TOW or MODE_DUAL
            actual_mode = f"前:{self.fb_front_mode}/后:{self.fb_rear_mode}"
            is_can = (self.fb_rear_mode == "CAN" or self.fb_front_mode == "自动")

        if is_can:
            mode_str = f"\033[92m{actual_mode}\033[0m"
        else:
            mode_str = f"\033[93m{actual_mode}\033[0m"

        parts = [f"\r  [{ctrl_name}] 状态={mode_str}  "]

        if self.control_mode in (MODE_FRONT, MODE_TOW, MODE_DUAL):
            parts.append(
                f"| 前车: cmd v={self.v_front:+.2f} δ={self.delta_front * RAD2DEG:+.1f}°  "
                f"fb v={self.fb_front_speed:+.2f} eps={self.fb_front_steer:+.1f}°  "
            )

        if self.control_mode in (MODE_REAR, MODE_TOW, MODE_DUAL):
            v_rear_display = self.v_rear * REAR_SPEED_RATIO if self.control_mode == MODE_TOW else self.v_rear
            parts.append(
                f"| 后车[{self.fb_rear_mode}/{self.fb_rear_state}]: "
                f"cmd v={v_rear_display:+.2f} δ={self.delta_rear * RAD2DEG:+.1f}°  "
                f"fb v={self.fb_rear_speed:+.3f} δ={self.fb_rear_steer * RAD2DEG:+.1f}°  "
                f"电池={self.fb_rear_battery:.1f}V  "
            )

        sys.stdout.write("".join(parts) + "   ")
        sys.stdout.flush()

    def run(self):
        import signal

        old_settings = termios.tcgetattr(sys.stdin)

        # 信号处理: Ctrl+C 直接退出
        def signal_handler(sig, frame):
            self.running = False
        signal.signal(signal.SIGINT, signal_handler)

        send_thread = threading.Thread(target=self._send_loop, daemon=True)
        send_thread.start()

        try:
            tty.setcbreak(sys.stdin.fileno())  # cbreak 比 raw 更兼容
            self._print_help()

            while self.running:
                try:
                    # 快速轮询 (20ms) + 一次性排空所有待处理按键
                    if select.select([sys.stdin], [], [], 0.02)[0]:
                        # 排空 stdin 缓冲区, 确保不丢按键
                        while select.select([sys.stdin], [], [], 0)[0]:
                            c = sys.stdin.read(1)
                            self._handle_key(c)
                            if not self.running:
                                break
                except (IOError, OSError):
                    pass
                self._print_status()

        except (Exception, KeyboardInterrupt) as e:
            if not isinstance(e, KeyboardInterrupt):
                print(f"\n错误: {e}")
        finally:
            # 恢复终端
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

            # 停车
            self.v_front = 0.0
            self.delta_front = 0.0
            self.v_rear = 0.0
            self.delta_rear = 0.0
            time.sleep(0.1)

            # 发送 RESET
            for _ in range(50):
                cmd = self._make_cmd(pad_action=RESET)
                self.cmd_writer.write(cmd)
                time.sleep(0.02)

            self.running = False
            cyber.shutdown()
            ctrl_name = {MODE_FRONT: "前车", MODE_REAR: "后车", MODE_TOW: "拖挂", MODE_DUAL: "双独立"}[self.control_mode]
            print(f"\n\n✅ {ctrl_name}已停止，Standby 模式已请求。")

    def _emergency_stop(self):
        """最高优先级急停: 归零 + 取消待发动作 + 立即发送"""
        # 1. 立即归零所有控制量
        self.v_front = 0.0
        self.delta_front = 0.0
        self.v_rear = 0.0
        self.delta_rear = 0.0

        # 2. 取消任何正在进行的 M 键 pad 动作 (如正在发 START)
        with self._pad_lock:
            self._pending_pad_action = None
            self._pending_pad_count = 0

        # 3. 主线程立即发送零命令, 不等后台线程的下一个周期
        for _ in range(3):
            cmd = self._make_cmd()
            self.cmd_writer.write(cmd)

    def _handle_key(self, c):
        # ★ 最高优先级: 急停 (空格) 和退出 (Q)
        if c == ' ':
            self._emergency_stop()
            return

        if c in ('q', 'Q', '\x03'):  # q or Ctrl+C
            self._emergency_stop()
            self.running = False
            return

        if c in ('m', 'M'):
            if not self.is_enabled:
                self.is_enabled = True
                sys.stdout.write("\r\n🟢 [M] 使能中...\r\n")
                sys.stdout.flush()
                with self._pad_lock:
                    self._pending_pad_action = START
                    self._pending_pad_count = 100
            else:
                self._emergency_stop()
                self.is_enabled = False
                sys.stdout.write("\r\n🔴 [M] 禁用中...\r\n")
                sys.stdout.flush()
                with self._pad_lock:
                    self._pending_pad_action = RESET
                    self._pending_pad_count = 50

        # ---- 前车控制 (W/S/A/D) ----
        elif c in ('w', 'W'):
            if self.control_mode in (MODE_FRONT, MODE_TOW, MODE_DUAL):
                self.v_front = min(self.v_front + SPEED_INC, MAX_SPEED)
            if self.control_mode == MODE_REAR:
                self.v_rear = min(self.v_rear + SPEED_INC, MAX_SPEED)
            if self.control_mode == MODE_TOW:  # 拖挂模式前后同步
                self.v_rear = min(self.v_rear + SPEED_INC, MAX_SPEED)

        elif c in ('s', 'S'):
            if self.control_mode in (MODE_FRONT, MODE_TOW, MODE_DUAL):
                self.v_front = max(self.v_front - SPEED_INC, -MAX_SPEED)
            if self.control_mode == MODE_REAR:
                self.v_rear = max(self.v_rear - SPEED_INC, -MAX_SPEED)
            if self.control_mode == MODE_TOW:  # 拖挂模式前后同步
                self.v_rear = max(self.v_rear - SPEED_INC, -MAX_SPEED)

        elif c in ('a', 'A'):
            if self.control_mode in (MODE_FRONT, MODE_TOW, MODE_DUAL):
                self.delta_front = min(self.delta_front + STEER_INC, MAX_STEER_FRONT)
            if self.control_mode == MODE_REAR:
                self.delta_rear = min(self.delta_rear + STEER_INC, MAX_STEER_REAR)

        elif c in ('d', 'D'):
            if self.control_mode in (MODE_FRONT, MODE_TOW, MODE_DUAL):
                self.delta_front = max(self.delta_front - STEER_INC, -MAX_STEER_FRONT)
            if self.control_mode == MODE_REAR:
                self.delta_rear = max(self.delta_rear - STEER_INC, -MAX_STEER_REAR)

        # ---- 后车独立控制 (I/K/J/L) — 仅 MODE_DUAL 生效 ----
        elif c in ('i', 'I'):
            if self.control_mode == MODE_DUAL:
                self.v_rear = min(self.v_rear + SPEED_INC, MAX_SPEED)

        elif c in ('k', 'K'):
            if self.control_mode == MODE_DUAL:
                self.v_rear = max(self.v_rear - SPEED_INC, -MAX_SPEED)

        elif c in ('j', 'J'):
            if self.control_mode == MODE_DUAL:
                self.delta_rear = min(self.delta_rear + STEER_INC, MAX_STEER_REAR)

        elif c in ('l', 'L'):
            if self.control_mode == MODE_DUAL:
                self.delta_rear = max(self.delta_rear - STEER_INC, -MAX_STEER_REAR)

    def _print_help(self):
        ctrl_name = {
            MODE_FRONT: "前车",
            MODE_REAR: "后车 (Hunter SE)",
            MODE_TOW: "整车拖挂 (前车转向+后车直行)",
            MODE_DUAL: "双独立控制",
        }[self.control_mode]
        lines = [
            "\033[2J\033[H",  # 清屏
            "=" * 60 + "\r\n",
            f"  铰接车键盘遥控 — {ctrl_name}\r\n",
            "=" * 60 + "\r\n",
            f"  M    切换模式 (Standby ↔ CAN控制)\r\n",
        ]
        if self.control_mode == MODE_DUAL:
            steer_inc_deg = STEER_INC * RAD2DEG
            lines += [
                f"  ── 前车 ──\r\n",
                f"  W/S  前车 加速/减速 (±{SPEED_INC} m/s)\r\n",
                f"  A/D  前车 左转/右转 (±{steer_inc_deg:.1f}°)\r\n",
                f"  ── 后车 ──\r\n",
                f"  I/K  后车 加速/减速 (±{SPEED_INC} m/s)\r\n",
                f"  J/L  后车 左转/右转 (±{steer_inc_deg:.1f}°)\r\n",
            ]
        else:
            steer_inc_deg = STEER_INC * RAD2DEG
            if self.control_mode == MODE_FRONT:
                max_steer_deg = MAX_STEER_FRONT * RAD2DEG
            elif self.control_mode == MODE_REAR:
                max_steer_deg = MAX_STEER_REAR * RAD2DEG
            else:  # MODE_TOW
                max_steer_deg = MAX_STEER_FRONT * RAD2DEG
            lines += [
                f"  W/S  加速/减速 (±{SPEED_INC} m/s, 限 ±{MAX_SPEED})\r\n",
                f"  A/D  左转/右转 (±{steer_inc_deg:.1f}°, 限 ±{max_steer_deg:.0f}°)\r\n",
            ]
            if self.control_mode == MODE_TOW:
                lines.append(f"       拖挂模式: 仅前车转向, 后车直行\r\n")
        lines += [
            f"  空格  急停 (速度/转角归零)\r\n",
            f"  Q    退出 (自动停车)\r\n",
            "-" * 60 + "\r\n\r\n",
        ]
        sys.stdout.write("".join(lines))
        sys.stdout.flush()


def select_mode():
    """启动时选择控制模式"""
    print("=" * 50)
    print("  铰接车键盘遥控 — 选择控制模式")
    print("=" * 50)
    print("  1 - 前车控制 (驱动电机 + EPS 转向)")
    print("  2 - 后车控制 (Hunter SE)")
    print("  3 - 整车拖挂 (前车转向 + 后车直行)")
    print("  4 - 双独立控制 (WSAD前车 + IKJL后车)")
    print("=" * 50)

    while True:
        try:
            choice = input("  请输入选项 (1/2/3/4): ").strip()
            if choice in ('1', '2', '3', '4'):
                return int(choice)
            print("  ⚠ 无效输入，请输入 1~4")
        except (EOFError, KeyboardInterrupt):
            print("\n  已取消。")
            sys.exit(0)


if __name__ == "__main__":
    mode = select_mode()
    teleop = ArticulatedTeleop(mode)
    teleop.run()
