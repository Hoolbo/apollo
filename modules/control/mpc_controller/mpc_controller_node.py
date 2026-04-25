"""
LPV-MPC Controller Component for Apollo Cyber RT.

Migrated from ROS2 mpc_node.py. Implements the 3-layer control architecture:
  Layer 1: LPV-MPC trajectory tracking  (v_cmd, omega_gamma_cmd)
  Layer 2: Kinematic allocation          (front/rear body yaw rates)
  Layer 3: Ackermann inverse mapping     (v_front, delta_front, v_rear, delta_rear)

Reads ADCTrajectory from CILQR planner, outputs CAN-ready (v, delta) per body.
"""

import json
import math
import os
import time
import csv

import numpy as np
from scipy import linalg as la

try:
    import osqp
    from scipy import sparse
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis


# =========================================================================== #
#                          Helper Math Functions                               #
# =========================================================================== #

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


# =========================================================================== #
#                         Core MPC Solver                                      #
# =========================================================================== #

class ArticulatedVehicleMPC:
    """
    LPV-MPC for articulated vehicle.
    State  x = [x, y, theta, gamma]^T
    Control u = [v, omega_gamma]^T
    """

    def __init__(self, params):
        self.Np = params['Np']
        self.Nc = params['Nc']
        self.dt = params['dt']
        self.Lf = params['Lf']
        self.Lr = params['Lr']

        self.Q = np.diag(params['Q'])
        self.R = np.diag(params['R'])
        self.S = np.diag(params.get('S', [0.0, 0.0]))

        self.v_max = params['v_max']
        self.v_min = params['v_min']
        self.omega_max = params['omega_gamma_max']
        self.gamma_max = params['gamma_max']
        self.gamma_min = params['gamma_min']
        self.dv_max = params['dv_max']
        self.domega_max = params['domega_gamma_max']
        self.v_threshold = params['v_threshold']

        self.nx = 4
        self.nu = 2
        self.nxi = self.nx + self.nu
        self.u_prev = np.zeros(self.nu)

    def _continuous_jacobians(self, x_ref, u_ref):
        theta_r = x_ref[2]
        gamma_r = x_ref[3]
        v_r = u_ref[0]
        Lf, Lr = self.Lf, self.Lr
        L = Lr + Lf * np.cos(gamma_r)

        num = (v_r * np.cos(gamma_r)) * L \
              + (v_r * np.sin(gamma_r) + Lr * u_ref[1]) * (Lf * np.sin(gamma_r))
        den = L ** 2
        a34 = num / den if abs(den) > 1e-12 else 0.0

        Ac = np.zeros((self.nx, self.nx))
        Ac[0, 2] = -v_r * np.sin(theta_r)
        Ac[1, 2] = v_r * np.cos(theta_r)
        Ac[2, 3] = a34

        Bc = np.zeros((self.nx, self.nu))
        Bc[0, 0] = np.cos(theta_r)
        Bc[1, 0] = np.sin(theta_r)
        Bc[2, 0] = np.sin(gamma_r) / L if abs(L) > 1e-12 else 0.0
        Bc[2, 1] = Lr / L if abs(L) > 1e-12 else 0.0
        Bc[3, 1] = 1.0

        return Ac, Bc

    def _augmented_model(self, Ac, Bc):
        Ad = np.eye(self.nx) + self.dt * Ac
        Bd = self.dt * Bc

        A_aug = np.zeros((self.nxi, self.nxi))
        A_aug[:self.nx, :self.nx] = Ad
        A_aug[:self.nx, self.nx:] = Bd
        A_aug[self.nx:, self.nx:] = np.eye(self.nu)

        B_aug = np.zeros((self.nxi, self.nu))
        B_aug[:self.nx, :] = Bd
        B_aug[self.nx:, :] = np.eye(self.nu)

        C_aug = np.zeros((self.nx, self.nxi))
        C_aug[:self.nx, :self.nx] = np.eye(self.nx)

        return A_aug, B_aug, C_aug

    def _prediction_matrices_lpv(self, A_aug_list, B_aug_list, C_aug):
        Np, Nc = self.Np, self.Nc
        ny, nxi, nu = self.nx, self.nxi, self.nu

        Psi = np.zeros((Np * ny, nxi))
        Theta = np.zeros((Np * ny, Nc * nu))

        A_cum = []
        A_prod = np.eye(nxi)
        for i in range(Np):
            A_prod = A_aug_list[i] @ A_prod
            A_cum.append(A_prod.copy())

        for i in range(Np):
            Psi[i*ny:(i+1)*ny, :] = C_aug @ A_cum[i]
            for j in range(min(i + 1, Nc)):
                if j == i:
                    CB = C_aug @ B_aug_list[j]
                else:
                    A_chain = np.eye(nxi)
                    for m in range(j + 1, i + 1):
                        A_chain = A_aug_list[m] @ A_chain
                    CB = C_aug @ A_chain @ B_aug_list[j]
                Theta[i*ny:(i+1)*ny, j*nu:(j+1)*nu] = CB

        return Psi, Theta

    def solve(self, x_current, ref_traj, ref_ctrl):
        Np, Nc = self.Np, self.Nc
        ny, nu = self.nx, self.nu

        A_aug_list, B_aug_list = [], []
        C_aug = None
        for i in range(Np):
            Ac_i, Bc_i = self._continuous_jacobians(ref_traj[i], ref_ctrl[i])
            A_aug_i, B_aug_i, C_aug = self._augmented_model(Ac_i, Bc_i)
            A_aug_list.append(A_aug_i)
            B_aug_list.append(B_aug_i)

        Psi, Theta = self._prediction_matrices_lpv(A_aug_list, B_aug_list, C_aug)

        x_ref0 = ref_traj[0]
        x_err = x_current - x_ref0
        x_err[2] = normalize_angle(x_err[2])
        x_err[3] = normalize_angle(x_err[3])
        xi = np.concatenate([x_err, self.u_prev])

        Y_ref = np.zeros(Np * ny)
        for i in range(Np):
            ref_err = ref_traj[i] - x_ref0
            ref_err[2] = normalize_angle(ref_err[2])
            ref_err[3] = normalize_angle(ref_err[3])
            Y_ref[i*ny:(i+1)*ny] = ref_err

        Q_bar = la.block_diag(*[self.Q] * Np)
        R_bar = la.block_diag(*[self.R] * Nc)

        H = Theta.T @ Q_bar @ Theta + R_bar

        M_tri = np.kron(np.tril(np.ones((Nc, Nc))), np.eye(nu))
        u_prev_vec = np.tile(self.u_prev, Nc)
        u_ref_vec = np.zeros(Nc * nu)
        for j in range(Nc):
            ref_idx = min(j, Np - 1)
            u_ref_vec[j*nu:(j+1)*nu] = ref_ctrl[ref_idx]

        f_ctrl = np.zeros(Nc * nu)
        if np.any(np.diag(self.S) > 0):
            S_bar = la.block_diag(*[self.S] * Nc)
            H += M_tri.T @ S_bar @ M_tri
            f_ctrl = M_tri.T @ S_bar @ (u_prev_vec - u_ref_vec)

        H = (H + H.T) / 2.0
        f = (Theta.T @ Q_bar @ (Psi @ xi - Y_ref)).flatten() + f_ctrl

        du_min = np.tile([-self.dv_max, -self.domega_max], Nc)
        du_max = np.tile([self.dv_max,  self.domega_max], Nc)

        u_abs_min = np.tile([self.v_min, -self.omega_max], Nc) - u_prev_vec
        u_abs_max = np.tile([self.v_max,  self.omega_max], Nc) - u_prev_vec

        gamma_idx = 3
        gamma_rows = list(range(gamma_idx, Np * ny, ny))
        Theta_gamma = Theta[gamma_rows, :]
        Psi_gamma = Psi[gamma_rows, :]
        psi_xi_gamma = Psi_gamma @ xi
        gamma_lb = np.full(Np, self.gamma_min) - psi_xi_gamma - x_ref0[3]
        gamma_ub = np.full(Np, self.gamma_max) - psi_xi_gamma - x_ref0[3]

        A_cons = np.vstack([M_tri, -M_tri, Theta_gamma, -Theta_gamma])
        b_cons = np.concatenate([u_abs_max, -u_abs_min, gamma_ub, -gamma_lb])

        dU_opt = self._solve_qp(H, f, A_cons, b_cons, du_min, du_max)

        du_0 = dU_opt[:nu]
        u_new = self.u_prev + du_0
        u_new[0] = np.clip(u_new[0], self.v_min, self.v_max)
        u_new[1] = np.clip(u_new[1], -self.omega_max, self.omega_max)
        self.u_prev = u_new.copy()

        return float(u_new[0]), float(u_new[1])

    def _solve_qp(self, H, f, A_ineq, b_ineq, lb, ub):
        n = H.shape[0]
        if HAS_OSQP:
            P = sparse.csc_matrix(H)
            A_box = sparse.eye(n, format='csc')
            A_all = sparse.vstack([sparse.csc_matrix(A_ineq), A_box], format='csc')
            l_all = np.concatenate([-np.inf * np.ones(A_ineq.shape[0]), lb])
            u_all = np.concatenate([b_ineq, ub])

            solver = osqp.OSQP()
            solver.setup(P, f, A_all, l_all, u_all,
                         verbose=False, warm_start=True,
                         max_iter=4000, eps_abs=1e-3, eps_rel=1e-3,
                         adaptive_rho=True, polish=False)
            result = solver.solve()
            if result.info.status == 'solved' or 'inaccurate' in result.info.status:
                return result.x
        # Fallback
        try:
            dU = -np.linalg.solve(H, f)
        except np.linalg.LinAlgError:
            dU = np.zeros(n)
        return np.clip(dU, lb, ub)


# =========================================================================== #
#          Ackermann Kinematic Allocator                                       #
# =========================================================================== #

class AckermannAllocator:
    """
    Converts MPC outputs (v_cmd, omega_gamma_cmd) into per-body
    Ackermann commands (v, delta) for CAN bus.

    Pipeline:
      (v_cmd, ω_γ) → front/rear yaw rates → Ackermann inverse → (v, δ) × 2
    """

    def __init__(self, vehicle_cfg):
        art = vehicle_cfg['articulation']
        self.Lf = art['Lf']
        self.Lr = art['Lr']

        front = vehicle_cfg['front_body']
        self.L_wb_front = front['wheelbase']
        self.delta_max_front = front['delta_max']
        self.delta_min_front = front['delta_min']

        rear = vehicle_cfg['rear_body']
        self.L_wb_rear = rear['wheelbase']
        self.delta_max_rear = rear['delta_max']
        self.delta_min_rear = rear['delta_min']

    def allocate(self, v_cmd, omega_gamma_cmd, gamma):
        """
        Convert (v_cmd, omega_gamma_cmd) to (v_front, delta_front, v_rear, delta_rear).

        Returns
        -------
        v_front, delta_front, v_rear, delta_rear : float
            CAN-ready commands for each body.
        """
        Lf, Lr = self.Lf, self.Lr

        # Step 1: Front body yaw rate (from articulated kinematic model)
        L_eff = Lr + Lf * np.cos(gamma)
        if abs(L_eff) < 1e-9:
            L_eff = 1e-9
        omega_front = (v_cmd * np.sin(gamma) + Lr * omega_gamma_cmd) / L_eff

        # Step 2: Rear body states
        omega_rear = omega_front - omega_gamma_cmd
        v_rear = v_cmd * np.cos(gamma) + Lf * omega_front * np.sin(gamma)

        # Step 3: Ackermann inverse mapping
        v_front = v_cmd

        # delta = atan(omega * L_wheelbase / v)
        if abs(v_front) > 0.01:
            delta_front = np.arctan(omega_front * self.L_wb_front / v_front)
        else:
            delta_front = 0.0

        if abs(v_rear) > 0.01:
            delta_rear = np.arctan(omega_rear * self.L_wb_rear / v_rear)
        else:
            delta_rear = 0.0

        # Clamp steering angles
        delta_front = np.clip(delta_front, self.delta_min_front, self.delta_max_front)
        delta_rear = np.clip(delta_rear, self.delta_min_rear, self.delta_max_rear)

        return v_front, delta_front, v_rear, delta_rear


# =========================================================================== #
#                    Cyber RT Controller Node                                  #
# =========================================================================== #

class MPCControllerNode:
    """
    Apollo Cyber RT controller node.
    Subscribes to localization + ADCTrajectory, runs LPV-MPC,
    outputs Ackermann commands via ControlCommand.
    """

    def __init__(self, node):
        self.node = node
        self.current_state = None   # [x, y, theta, gamma]
        self.gamma = 0.0
        self.latest_trajectory = None
        self.closest_idx = 0

        # ── Load configs ──
        conf_dir = os.path.join(os.path.dirname(__file__), 'conf')
        if not os.path.isdir(conf_dir):
            conf_dir = '/opt/apollo/neo/share/modules/control/mpc_controller/conf'

        with open(os.path.join(conf_dir, 'vehicle.json')) as f:
            self.vehicle_cfg = json.load(f)
        with open(os.path.join(conf_dir, 'mpc_params.json')) as f:
            mpc_cfg = json.load(f)

        # ── Build MPC params dict ──
        art = self.vehicle_cfg['articulation']
        front = self.vehicle_cfg['front_body']
        h = mpc_cfg['horizons']
        w = mpc_cfg['weights']
        c = mpc_cfg['constraints']

        mpc_params = {
            'Np': h['Np'], 'Nc': h['Nc'], 'dt': h['dt'],
            'Lf': art['Lf'], 'Lr': art['Lr'],
            'Q': w['Q'], 'R': w['R'], 'S': w.get('S', [0, 0]),
            'v_max': front['v_max'], 'v_min': front['v_min'],
            'omega_gamma_max': art['omega_gamma_max'],
            'gamma_max': art['gamma_max'], 'gamma_min': art['gamma_min'],
            'dv_max': c['dv_max'], 'domega_gamma_max': c['domega_gamma_max'],
            'v_threshold': c['v_threshold'],
        }

        self.mpc = ArticulatedVehicleMPC(mpc_params)
        self.allocator = AckermannAllocator(self.vehicle_cfg)
        self.dt = h['dt']

        # ── Subscribers ──
        self.node.create_reader(
            '/apollo/localization/pose',
            LocalizationEstimate,
            self._on_localization)

        self.node.create_reader(
            '/apollo/planning',
            ADCTrajectory,
            self._on_trajectory)

        # ── Publisher ──
        self.ctrl_writer = self.node.create_writer(
            '/apollo/control', ControlCommand)

        # Subscribe to chassis for articulation angle γ
        self.node.create_reader(
            '/apollo/canbus/chassis',
            Chassis,
            self._on_chassis)

        # ── CSV log ──
        os.makedirs('/tmp/controller', exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._log_path = f'/tmp/controller/mpc_ctrl_{ts}.csv'
        self._log_file = open(self._log_path, 'w', newline='')
        self._csv = csv.writer(self._log_file)
        self._csv.writerow([
            'time_s', 'x', 'y', 'theta_deg', 'gamma_deg',
            'v_cmd', 'omega_gamma_cmd',
            'v_front', 'delta_front_deg', 'v_rear', 'delta_rear_deg',
            'closest_idx',
        ])

        self._tick = 0
        print(f'[MPC Controller] Started | Np={mpc_params["Np"]} Nc={mpc_params["Nc"]} '
              f'dt={self.dt}s OSQP={"Yes" if HAS_OSQP else "No"}')
        print(f'[MPC Controller] CSV log: {self._log_path}')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _on_localization(self, msg):
        """Extract [x, y, theta] from LocalizationEstimate."""
        pos = msg.pose.position
        heading = msg.pose.heading
        self.current_state = np.array([pos.x, pos.y, heading, self.gamma])

    def _on_trajectory(self, msg):
        """Store latest ADCTrajectory from CILQR planner."""
        if len(msg.trajectory_point) < 2:
            return
        self.latest_trajectory = msg
        self.closest_idx = 0  # reset on new trajectory

    def _on_chassis(self, msg):
        """Read articulation angle γ from Chassis.steering_percentage (deg → rad)."""
        self.gamma = np.radians(msg.steering_percentage)
        if self.current_state is not None:
            self.current_state[3] = self.gamma

    # ------------------------------------------------------------------ #
    # Reference trajectory extraction from ADCTrajectory
    # ------------------------------------------------------------------ #
    def _build_ref_arrays(self):
        """
        Build Np-length reference state and control arrays from ADCTrajectory.

        ADCTrajectory fields:
          trajectory_point[i].path_point.x/y/theta → ref x, y, θ
          trajectory_point[i].path_point.kappa      → ref γ (articulation angle)
          trajectory_point[i].v                     → ref v
        """
        if self.latest_trajectory is None or self.current_state is None:
            return None, None

        traj = self.latest_trajectory
        pts = traj.trajectory_point
        n_pts = len(pts)
        if n_pts < 2:
            return None, None

        # Find closest point (forward search with cap)
        cx, cy = self.current_state[0], self.current_state[1]
        min_dist = float('inf')
        closest = self.closest_idx

        dx0 = pts[1].path_point.x - pts[0].path_point.x
        dy0 = pts[1].path_point.y - pts[0].path_point.y
        spacing = max(math.sqrt(dx0*dx0 + dy0*dy0), 1e-6)
        v_now = max(abs(self.mpc.u_prev[0]), 1.0)
        max_adv = max(int(v_now * self.dt / spacing * 20), 50)
        search_end = min(self.closest_idx + max_adv, n_pts)

        for i in range(self.closest_idx, search_end):
            dx = pts[i].path_point.x - cx
            dy = pts[i].path_point.y - cy
            d = dx*dx + dy*dy
            if d < min_dist:
                min_dist = d
                closest = i

        if min_dist > 9.0:
            for i in range(n_pts):
                dx = pts[i].path_point.x - cx
                dy = pts[i].path_point.y - cy
                d = dx*dx + dy*dy
                if d < min_dist:
                    min_dist = d
                    closest = i

        self.closest_idx = closest

        Np = self.mpc.Np
        ref_states = np.zeros((Np, 4))
        ref_ctrls = np.zeros((Np, 2))

        if closest < n_pts:
            v_ref_sample = max(abs(pts[closest].v), 0.3)
        else:
            v_ref_sample = 1.0

        arc = [0.0]
        for k in range(closest, min(closest + n_pts, n_pts - 1)):
            dx = pts[k+1].path_point.x - pts[k].path_point.x
            dy = pts[k+1].path_point.y - pts[k].path_point.y
            arc.append(arc[-1] + math.sqrt(dx*dx + dy*dy))

        for i in range(Np):
            target_s = v_ref_sample * (i + 1) * self.dt
            idx = closest
            for k, s in enumerate(arc):
                if s >= target_s:
                    idx = min(closest + k, n_pts - 1)
                    break
            else:
                idx = min(closest + len(arc) - 1, n_pts - 1)

            pt = pts[idx]
            ref_states[i] = [
                pt.path_point.x,
                pt.path_point.y,
                pt.path_point.theta,
                pt.path_point.kappa,  # γ stored in kappa
            ]
            ref_ctrls[i, 0] = pt.v

        for i in range(Np - 1):
            ref_ctrls[i, 1] = (ref_states[i+1, 3] - ref_states[i, 3]) / self.dt
        if Np >= 2:
            ref_ctrls[Np-1, 1] = ref_ctrls[Np-2, 1]
        ref_ctrls[:, 1] = np.clip(ref_ctrls[:, 1],
                                   -self.mpc.omega_max, self.mpc.omega_max)

        return ref_states, ref_ctrls

    # ------------------------------------------------------------------ #
    # Control loop (called by main loop)
    # ------------------------------------------------------------------ #
    def step(self):
        """Single control step — called by the main loop."""
        self._tick += 1

        if self._tick % 100 == 1:
            has_loc = self.current_state is not None
            has_traj = self.latest_trajectory is not None
            n_pts = len(self.latest_trajectory.trajectory_point) if has_traj else 0
            state_str = (f'[{self.current_state[0]:.2f}, {self.current_state[1]:.2f}, '
                         f'{np.degrees(self.current_state[2]):.1f}°, '
                         f'{np.degrees(self.current_state[3]):.1f}°]'
                         if has_loc else 'None')
            print(f'[MPC] tick={self._tick} | loc={has_loc} state={state_str} | '
                  f'traj={has_traj} pts={n_pts}')

        if self.current_state is None:
            return

        ref_states, ref_ctrls = self._build_ref_arrays()
        if ref_states is None:
            self._publish_stop()
            return

        # Layer 1: MPC solve
        v_cmd, omega_gamma_cmd = self.mpc.solve(
            self.current_state, ref_states, ref_ctrls)

        # Layer 2+3: Ackermann allocation
        v_front, delta_front, v_rear, delta_rear = self.allocator.allocate(
            v_cmd, omega_gamma_cmd, self.gamma)

        # CSV log
        now = time.time()
        st = self.current_state
        self._csv.writerow([
            f'{now:.4f}',
            f'{st[0]:.4f}', f'{st[1]:.4f}',
            f'{np.degrees(st[2]):.2f}', f'{np.degrees(st[3]):.2f}',
            f'{v_cmd:.4f}', f'{omega_gamma_cmd:.4f}',
            f'{v_front:.4f}', f'{np.degrees(delta_front):.2f}',
            f'{v_rear:.4f}', f'{np.degrees(delta_rear):.2f}',
            self.closest_idx,
        ])
        self._log_file.flush()

        # Publish
        self._publish_cmd(v_front, delta_front, v_rear, delta_rear)

    def _publish_cmd(self, v_front, delta_front, v_rear, delta_rear):
        """Publish ControlCommand with Ackermann outputs."""
        msg = ControlCommand()
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = 'mpc_controller'

        msg.speed = v_front
        msg.steering_target = np.degrees(delta_front)
        msg.acceleration = 0.0

        # Store all 4 commands in debug string for canbus adapter
        debug_str = (f'v_front={v_front:.4f},'
                     f'delta_front={delta_front:.4f},'
                     f'v_rear={v_rear:.4f},'
                     f'delta_rear={delta_rear:.4f}')
        msg.header.status.msg = debug_str

        self.ctrl_writer.write(msg)

    def _publish_stop(self):
        msg = ControlCommand()
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = 'mpc_controller'
        msg.speed = 0.0
        msg.steering_target = 0.0
        msg.acceleration = 0.0
        self.ctrl_writer.write(msg)


# =========================================================================== #
#                              Entry Point                                    #
# =========================================================================== #

def main():
    cyber.init()
    node = cyber.Node('mpc_controller')
    controller = MPCControllerNode(node)

    print('[MPC Controller] Running... Press Ctrl+C to stop')
    dt = controller.dt
    try:
        while not cyber.is_shutdown():
            controller.step()
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\n[MPC Controller] Stopped.')
    finally:
        cyber.shutdown()


if __name__ == '__main__':
    main()
