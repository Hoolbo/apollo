"""
LPV-MPC Trajectory Tracking Controller for Articulated Tracked Vehicle.

Implements the complete 3-layer control architecture from Chapter 4:
  Layer 1: LPV-MPC trajectory tracking  (Section 4.2)
  Layer 2: Kinematic allocation          (Section 4.3)
  Layer 3: Multi-mode track mapping      (Section 4.4)

LPV-MPC formulation: at each prediction step i the vehicle model is
linearised at the corresponding reference point (x_r(k+i), u_r(k+i))
along the planned trajectory.  The reference velocity / curvature acts
as the *scheduling variable* that drives parameter variation — hence
"Linear Parameter-Varying" rather than LTI (single fixed model) or
LTV (state-trajectory-dependent).  The QP structure is identical to
the LTV case; only the conceptual framing changes.

Author : Hoolbo (auto-generated from thesis)
"""

import numpy as np
import os
import time
import csv
import io
import contextlib
from scipy import linalg as la
from scipy.optimize import linprog  # fallback if osqp not available

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

try:
    import osqp
    from scipy import sparse
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False


# =========================================================================== #
#                          Helper Math Functions                               #
# =========================================================================== #

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


# =========================================================================== #
#                         Core MPC Solver (Chapter 4.2)                       #
# =========================================================================== #

class ArticulatedVehicleMPC:
    """
    Linear Parameter-Varying Model Predictive Controller for an articulated
    tracked vehicle, implementing the algorithm described in Section 4.2.

    At each prediction step i the continuous-time Jacobians are evaluated
    at the corresponding reference point (x_r(k+i), u_r(k+i)).  The
    reference velocity and curvature serve as the *scheduling variables*
    that drive parameter variation across the prediction horizon, making
    this an LPV (Linear Parameter-Varying) controller.

    State  x = [x, y, theta, gamma]^T          (4 states)
    Control u = [v, omega_gamma]^T              (2 inputs)
    Control increment  du = u(k) - u(k-1)      (optimisation variable)
    """

    def __init__(self, params: dict):
        # Horizons
        self.Np = params['Np']      # prediction horizon
        self.Nc = params['Nc']      # control horizon
        self.dt = params['dt']      # sampling period

        # Vehicle geometry
        self.Lf = params['Lf']
        self.Lr = params['Lr']
        self.B  = params['B']
        self.R_wheel  = params['wheel_radius']

        # Weight matrices
        self.Q = np.diag(params['Q'])       # 4x4 state weight
        self.R = np.diag(params['R'])       # 2x2 control-increment weight
        self.S = np.diag(params.get('S', [0.0, 0.0]))  # 2x2 control-tracking weight

        # Constraints
        self.v_max   = params['v_max']
        self.v_min   = params['v_min']
        self.omega_max = params['omega_gamma_max']
        self.gamma_max = params['gamma_max']
        self.gamma_min = params['gamma_min']
        self.dv_max  = params['dv_max']
        self.domega_max = params['domega_gamma_max']

        # Turning-mode threshold
        self.v_threshold = params['v_threshold']

        # Dimensions
        self.nx = 4   # state dim
        self.nu = 2   # control dim
        self.nxi = self.nx + self.nu   # augmented state dim

        # Previous control (for increment computation)
        self.u_prev = np.zeros(self.nu)

        # Diagnostics
        self._diag_counter = 0   # throttle log to every N calls
        self._logger = None      # set by ROS node after construction

    # ------------------------------------------------------------------ #
    # 4.2.1.2  Jacobian linearisation  (Eq. 4-1 ~ 4-5)
    # ------------------------------------------------------------------ #
    def _continuous_jacobians(self, x_ref, u_ref):
        """
        Compute Ac(t) and Bc(t) at a reference operating point.
        See Eq. (4-3), (4-4), (4-5).
        """
        theta_r = x_ref[2]
        gamma_r = x_ref[3]
        v_r     = u_ref[0]
        Lf, Lr  = self.Lf, self.Lr
        # Correct denominator: L = Lr + Lf*cos(gamma)  (thesis Eq. 2-10)
        L = Lr + Lf * np.cos(gamma_r)

        # a34: partial(f3)/partial(gamma)  —— Eq. (4-4)
        # d/dgamma [(v*sin(gamma) + Lr*omega) / L]
        # = [v*cos(gamma)*L + (v*sin(gamma) + Lr*omega)*Lf*sin(gamma)] / L^2
        num = (v_r * np.cos(gamma_r)) * L \
               + (v_r * np.sin(gamma_r) + Lr * u_ref[1]) * (Lf * np.sin(gamma_r))
        den = L ** 2
        a34 = num / den if abs(den) > 1e-12 else 0.0

        Ac = np.zeros((self.nx, self.nx))
        Ac[0, 2] = -v_r * np.sin(theta_r)
        Ac[1, 2] =  v_r * np.cos(theta_r)
        Ac[2, 3] = a34
        # Ac[3,:] = 0 (gamma_dot = omega_gamma, linear in u)

        Bc = np.zeros((self.nx, self.nu))
        Bc[0, 0] =  np.cos(theta_r)
        Bc[1, 0] =  np.sin(theta_r)
        Bc[2, 0] =  np.sin(gamma_r) / L if abs(L) > 1e-12 else 0.0
        Bc[2, 1] =  Lr / L if abs(L) > 1e-12 else 0.0
        Bc[3, 1] =  1.0

        return Ac, Bc

    # ------------------------------------------------------------------ #
    # 4.2.1.3  Discretisation & augmented state  (Eq. 4-6)
    # ------------------------------------------------------------------ #
    def _augmented_model(self, Ac, Bc):
        """
        Forward-Euler discretisation + state augmentation.
        Returns A_aug (nxi x nxi) and B_aug (nxi x nu).
        """
        Ad = np.eye(self.nx) + self.dt * Ac      # I + Ts * Ac
        Bd = self.dt * Bc                          # Ts * Bc

        # Augmented matrices (Eq. 4-6)
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

    # ------------------------------------------------------------------ #
    # 4.2.2  LPV prediction matrices Ψ & Θ  (Eq. 4-9, 4-10)
    # ------------------------------------------------------------------ #
    def _prediction_matrices_lpv(self, A_aug_list, B_aug_list, C_aug):
        """
        Construct **parameter-varying** stacked prediction matrices.

        Each prediction step uses a different (Ã(k+i), B̃(k+i)) linearised
        at the corresponding reference point (x_r(k+i), u_r(k+i)).
        The scheduling variables are the reference velocity and curvature
        embedded in those reference points.

        Ψ row i:   C̃ × Ã(i) × Ã(i-1) × ... × Ã(0)
        Θ[i, j]:   C̃ × Ã(i) × ... × Ã(j+1) × B̃(j)   for j ≤ i
        """
        Np, Nc = self.Np, self.Nc
        ny, nxi, nu = self.nx, self.nxi, self.nu

        Psi   = np.zeros((Np * ny, nxi))
        Theta = np.zeros((Np * ny, Nc * nu))

        # Pre-compute cumulative Ã products from left:
        # A_cum[i] = Ã(i) × Ã(i-1) × ... × Ã(0)
        A_cum = []
        A_prod = np.eye(nxi)
        for i in range(Np):
            A_prod = A_aug_list[i] @ A_prod
            A_cum.append(A_prod.copy())

        for i in range(Np):
            # Ψ row i
            Psi[i*ny:(i+1)*ny, :] = C_aug @ A_cum[i]

            # Θ columns
            for j in range(min(i + 1, Nc)):
                if j == i:
                    # C̃ × B̃(j)
                    CB = C_aug @ B_aug_list[j]
                else:
                    # C̃ × Ã(i) × Ã(i-1) × ... × Ã(j+1) × B̃(j)
                    # Use direct chain product (avoids matrix inverse)
                    A_chain = np.eye(nxi)
                    for m in range(j + 1, i + 1):
                        A_chain = A_aug_list[m] @ A_chain
                    CB = C_aug @ A_chain @ B_aug_list[j]
                Theta[i*ny:(i+1)*ny, j*nu:(j+1)*nu] = CB

        return Psi, Theta

    # ------------------------------------------------------------------ #
    # 4.2.3  Formulate & solve QP  (Eq. 4-11 ~ 4-16)
    # ------------------------------------------------------------------ #
    def solve(self, x_current, ref_traj, ref_ctrl):
        """
        LPV-MPC solve (thesis §4.2).

        At each prediction step i the system is linearised at the
        corresponding reference point (x_r(k+i), u_r(k+i)).  The
        reference velocity / curvature is the *scheduling variable*;
        as it varies along the horizon the model matrices change —
        this is the LPV (Linear Parameter-Varying) property.

        Parameters
        ----------
        x_current : np.array (4,)
            Current vehicle state [x, y, theta, gamma].
        ref_traj : np.array (Np, 4)
            Reference states for the prediction horizon.
        ref_ctrl : np.array (Np, 2)
            Reference controls for the prediction horizon.

        Returns
        -------
        v_cmd, omega_gamma_cmd : float
            Optimal front-body velocity and articulation angular rate.
        """
        Np, Nc = self.Np, self.Nc
        ny, nu = self.nx, self.nu

        # --- (1) Per-step linearisation at each reference point ---
        # Scheduling variable: reference velocity v_r(k+i) and curvature
        # encoded in (x_r(k+i), u_r(k+i)) — this is the LPV core.
        A_aug_list = []
        B_aug_list = []
        C_aug = None
        for i in range(Np):
            Ac_i, Bc_i = self._continuous_jacobians(ref_traj[i], ref_ctrl[i])
            A_aug_i, B_aug_i, C_aug = self._augmented_model(Ac_i, Bc_i)
            A_aug_list.append(A_aug_i)
            B_aug_list.append(B_aug_i)

        # --- (2) Build LPV prediction matrices ---
        Psi, Theta = self._prediction_matrices_lpv(A_aug_list, B_aug_list, C_aug)

        # --- (3) Augmented error state  ξ(k) = [x̃(k), ũ(k-1)]  ---
        # x̃(k) = x_current - x_ref(k)   (Eq. 4-2)
        x_ref0 = ref_traj[0]
        x_err = x_current - x_ref0
        x_err[2] = normalize_angle(x_err[2])  # theta wrap
        x_err[3] = normalize_angle(x_err[3])  # gamma wrap
        xi = np.concatenate([x_err, self.u_prev])

        # --- (4) Reference output sequence  Y_ref ---
        # Y_ref[i] = ref_traj[i] - x_ref(0)  (deviation from linearisation base)
        # Angle channels normalised to prevent ±π jumps.
        Y_ref = np.zeros(Np * ny)
        for i in range(Np):
            ref_err = ref_traj[i] - x_ref0
            ref_err[2] = normalize_angle(ref_err[2])
            ref_err[3] = normalize_angle(ref_err[3])
            Y_ref[i*ny:(i+1)*ny] = ref_err

        # --- (5) Block-diagonal weight matrices ---
        Q_bar = la.block_diag(*[self.Q] * Np)
        R_bar = la.block_diag(*[self.R] * Nc)

        # --- (6) QP matrices  (Eq. 4-12, extended with control tracking) ---
        H = Theta.T @ Q_bar @ Theta + R_bar

        # Control tracking cost:  ||u - u_ref||²_S
        #   u(k+j) = M_tri @ ΔU + u_prev_vec
        #   Cost = ||M_tri @ ΔU + u_prev_vec - u_ref_vec||²_S_bar
        #   H += M_tri^T @ S_bar @ M_tri
        #   f += M_tri^T @ S_bar @ (u_prev_vec - u_ref_vec)
        M_tri = np.kron(np.tril(np.ones((Nc, Nc))), np.eye(nu))
        u_prev_vec = np.tile(self.u_prev, Nc)
        u_ref_vec = np.zeros(Nc * nu)
        for j in range(Nc):
            ref_idx = min(j, Np - 1)
            u_ref_vec[j*nu:(j+1)*nu] = ref_ctrl[ref_idx]

        if np.any(np.diag(self.S) > 0):
            S_bar = la.block_diag(*[self.S] * Nc)
            H += M_tri.T @ S_bar @ M_tri
            f_ctrl = M_tri.T @ S_bar @ (u_prev_vec - u_ref_vec)
        else:
            f_ctrl = np.zeros(Nc * nu)

        H = (H + H.T) / 2.0  # enforce symmetry
        f = (Theta.T @ Q_bar @ (Psi @ xi - Y_ref)).flatten() + f_ctrl

        # ---- Constraints ----
        # 1) Control increment bounds  (Eq. 4-13)
        du_min = np.tile([-self.dv_max, -self.domega_max], Nc)
        du_max = np.tile([ self.dv_max,  self.domega_max], Nc)

        # 2) Absolute control bounds via lower-tri cumsum  (Eq. 4-14)
        #    M_tri and u_prev_vec already computed above for the tracking cost.
        u_abs_min = np.tile([self.v_min, -self.omega_max], Nc) - u_prev_vec
        u_abs_max = np.tile([self.v_max,  self.omega_max], Nc) - u_prev_vec

        # 3) State (gamma) constraint  (Eq. 4-15)
        gamma_idx = 3
        gamma_rows = list(range(gamma_idx, Np * ny, ny))
        Theta_gamma = Theta[gamma_rows, :]
        Psi_gamma   = Psi[gamma_rows, :]
        psi_xi_gamma = Psi_gamma @ xi
        gamma_lb = np.full(Np, self.gamma_min) - psi_xi_gamma - x_ref0[3]
        gamma_ub = np.full(Np, self.gamma_max) - psi_xi_gamma - x_ref0[3]

        # Stack inequality constraints:  A_cons @ dU <= b_cons
        A_cons = np.vstack([
            M_tri,           # u_abs upper
            -M_tri,          # u_abs lower
            Theta_gamma,     # gamma upper
            -Theta_gamma,    # gamma lower
        ])
        b_cons = np.concatenate([
            u_abs_max,
            -u_abs_min,
            gamma_ub,
            -gamma_lb,
        ])

        # --- (7) Solve QP ---
        dU_opt, qp_status = self._solve_qp(H, f, A_cons, b_cons, du_min, du_max)

        # Extract first control increment  (Eq. 4-16)
        du_0 = dU_opt[:nu]
        u_new = self.u_prev + du_0
        u_new[0] = np.clip(u_new[0], self.v_min, self.v_max)
        u_new[1] = np.clip(u_new[1], -self.omega_max, self.omega_max)

        self.u_prev = u_new.copy()

        # --- (8) Cost breakdown for diagnostics ---
        Y_pred = Theta @ dU_opt + Psi @ xi          # (Np*ny,) predicted state error
        e_y_mat = (Y_pred - Y_ref).reshape(Np, ny)  # (Np, 4): [ex, ey, etheta, egamma]

        # Q cost per state variable (summed over horizon)
        self._cq_x     = float(np.sum(self.Q[0, 0] * e_y_mat[:, 0] ** 2))
        self._cq_y     = float(np.sum(self.Q[1, 1] * e_y_mat[:, 1] ** 2))
        self._cq_th    = float(np.sum(self.Q[2, 2] * e_y_mat[:, 2] ** 2))
        self._cq_gm    = float(np.sum(self.Q[3, 3] * e_y_mat[:, 3] ** 2))
        self._cq_total = self._cq_x + self._cq_y + self._cq_th + self._cq_gm

        # R cost per control increment (summed over Nc)
        dU_mat = dU_opt.reshape(Nc, nu)
        self._cr_v     = float(np.sum(self.R[0, 0] * dU_mat[:, 0] ** 2))
        self._cr_omg   = float(np.sum(self.R[1, 1] * dU_mat[:, 1] ** 2))
        self._cr_total = self._cr_v + self._cr_omg

        # S control tracking cost
        if np.any(np.diag(self.S) > 0):
            u_pred_vec = M_tri @ dU_opt + u_prev_vec  # predicted absolute u over Nc
            u_err_mat  = (u_pred_vec - u_ref_vec).reshape(Nc, nu)
            self._cs_v    = float(np.sum(self.S[0, 0] * u_err_mat[:, 0] ** 2))
            self._cs_omg  = float(np.sum(self.S[1, 1] * u_err_mat[:, 1] ** 2))
            self._cs_total = self._cs_v + self._cs_omg
        else:
            self._cs_v = self._cs_omg = self._cs_total = 0.0

        self._c_total = self._cq_total + self._cr_total + self._cs_total

        # Gradient of objective w.r.t. omega_gamma increment at step 0.
        # At unconstrained optimum this ≈ 0; if nonzero, a constraint is active.
        # Positive  →  optimizer wants LESS omega_gamma (upper bound active)
        # Negative  →  optimizer wants MORE omega_gamma (lower bound active)
        grad = H @ dU_opt + f
        self._grad_omg = float(grad[1])  # gradient for the 1st step omega_gamma

        # --- Diagnostics (throttled to 1 in 20 calls) ---
        self._diag_counter += 1
        if self._logger is not None and self._diag_counter % 20 == 0:
            hdg_err = np.degrees(x_err[2])
            self._logger.info(
                f'[MPC] QP={qp_status:20s} | '
                f'err x={x_err[0]:+.3f} y={x_err[1]:+.3f} '
                f'theta={hdg_err:+.1f}deg gamma={np.degrees(x_err[3]):+.1f}deg | '
                f'cmd v={u_new[0]:+.3f}m/s omega_g={u_new[1]:+.3f}rad/s | '
                f'J={self._c_total:.2f} [Qx={self._cq_x:.2f} Qy={self._cq_y:.2f} '
                f'Qth={self._cq_th:.2f} Qgm={self._cq_gm:.2f} | '
                f'Rv={self._cr_v:.2f} Rog={self._cr_omg:.2f} | S={self._cs_total:.2f}] '
                f'grad_og={self._grad_omg:+.4f}'
            )

        return float(u_new[0]), float(u_new[1])


    def _solve_qp(self, H, f, A_ineq, b_ineq, lb, ub):
        """Solve the QP. Returns (solution_vector, status_string)."""
        n = H.shape[0]

        if HAS_OSQP:
            P = sparse.csc_matrix(H)
            q = f

            # Combine inequality + box constraints
            A_box = sparse.eye(n, format='csc')
            A_all = sparse.vstack([
                sparse.csc_matrix(A_ineq),
                A_box
            ], format='csc')
            l_all = np.concatenate([
                -np.inf * np.ones(A_ineq.shape[0]),
                lb
            ])
            u_all = np.concatenate([
                b_ineq,
                ub
            ])

            solver = osqp.OSQP()
            solver.setup(P, q, A_all, l_all, u_all,
                         verbose=False, warm_start=True,
                         max_iter=4000,    # was 200 — too few for Np=20,Nc=10
                         eps_abs=1e-3, eps_rel=1e-3,
                         eps_prim_inf=1e-4, eps_dual_inf=1e-4,
                         adaptive_rho=True, polish=False)
            result = solver.solve()
            status = result.info.status
            if status == 'solved' or 'inaccurate' in status:
                return result.x, status
            else:
                return np.zeros(n), status   # fallback to zero increment
        else:
            # Fallback: unconstrained solution (gradient descent step)
            try:
                dU = -np.linalg.solve(H, f)
                status = 'fallback-unconstrained'
            except np.linalg.LinAlgError:
                dU = np.zeros(n)
                status = 'fallback-singular'
            dU = np.clip(dU, lb, ub)
            return dU, status


# =========================================================================== #
#          Kinematic Allocation  (Chapter 4.3)                                #
# =========================================================================== #

class KinematicAllocator:
    """
    Converts MPC outputs (v_cmd, omega_gamma_cmd) into per-body states
    and per-track velocities according to Sections 4.3 and 4.4.
    """

    def __init__(self, Lf: float, Lr: float, B: float,
                 v_threshold: float, wheel_radius: float):
        self.Lf = Lf
        self.Lr = Lr
        self.B  = B
        self.v_threshold = v_threshold
        self.R  = wheel_radius

    def allocate(self, v_cmd: float, omega_gamma_cmd: float,
                 gamma: float):
        """
        Full allocation pipeline.

        Returns
        -------
        v_fl, v_fr, v_rl, v_rr : float
            Track linear velocities (m/s).
        omega_f_cmd : float
            Front body yaw rate for reference.
        """
        Lf, Lr = self.Lf, self.Lr

        # ------ 4.3.2  Front body yaw rate  (Eq. 4-31) ------
        # Correct denominator: L_eff = Lr + Lf*cos(gamma)  (thesis Eq. 2-10)
        L_eff = Lr + Lf * np.cos(gamma)
        if abs(L_eff) < 1e-9:
            L_eff = 1e-9
        omega_f_cmd = (v_cmd * np.sin(gamma)
                       + Lr * omega_gamma_cmd) / L_eff

        # ------ 4.3.3  Rear body states  (Eq. 4-35, 4-36) ------
        v_r_cmd = v_cmd * np.cos(gamma) + Lf * omega_f_cmd * np.sin(gamma)
        omega_r_cmd = omega_f_cmd - omega_gamma_cmd

        # ------ 4.4  Track velocity mapping ------
        if abs(v_cmd) <= self.v_threshold:
            # 4.4.2  Low-speed pure articulation mode  (Eq. 4-37, 4-38)
            v_fl = v_cmd
            v_fr = v_cmd
            v_rl = v_r_cmd
            v_rr = v_r_cmd
        else:
            # 4.4.3  High-speed compound turning mode  (Eq. 4-39, 4-40)
            v_fl = v_cmd   + omega_f_cmd * self.B / 2.0
            v_fr = v_cmd   - omega_f_cmd * self.B / 2.0
            v_rl = v_r_cmd + omega_r_cmd * self.B / 2.0
            v_rr = v_r_cmd - omega_r_cmd * self.B / 2.0

        return v_fl, v_fr, v_rl, v_rr, omega_f_cmd


# =========================================================================== #
#                         ROS 2 Node                                          #
# =========================================================================== #

class MPCControllerNode(Node):
    """
    ROS 2 node that:
      - Subscribes to odometry and reference trajectory.
      - Runs the LPV-MPC solver at every control tick.
      - Publishes individual track velocity commands (Float64) and
        steering joint position commands.
    """

    def __init__(self):
        super().__init__('mpc_controller')

        # ---- Declare and load parameters ----
        self._declare_params()
        params = self._load_params()

        # ---- Core algorithm objects ----
        self.mpc = ArticulatedVehicleMPC(params)
        self.mpc._logger = self.get_logger()   # wire ROS logger for diagnostics

        self.allocator = KinematicAllocator(
            Lf=params['Lf'], Lr=params['Lr'], B=params['B'],
            v_threshold=params['v_threshold'],
            wheel_radius=params['wheel_radius'],
        )

        # ---- State variables ----
        self.current_state = None       # [x, y, theta, gamma]
        self.ref_trajectory = None      # Path message
        self.ref_velocities = None      # Per-waypoint v_ref from planner
        self.gamma = 0.0                # Current articulation angle
        self.closest_idx = 0            # Persistent: only advances forward along path
        self.steering_joint_pos = 0.0   # Internal integrator for steering command

        # ---- Publishers ----
        # Independent track velocity commands (Float64, rad/s angular velocity)
        self.pub_fl = self.create_publisher(Float64, '/cmd_omega_fl', 10)
        self.pub_fr = self.create_publisher(Float64, '/cmd_omega_fr', 10)
        self.pub_rl = self.create_publisher(Float64, '/cmd_omega_rl', 10)
        self.pub_rr = self.create_publisher(Float64, '/cmd_omega_rr', 10)
        # Steering joint position command
        # NOTE: ROS2 topic segments cannot start with a digit, so we can't use
        # the Ignition topic '/model/.../0/cmd_pos' directly. We use a clean
        # ROS2 alias and bridge it to Ignition in sim.launch.py.
        self.pub_steering = self.create_publisher(
            Float64, '/atv/steering_cmd_pos', 10)

        # ---- Subscribers ----
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            Path, '/atv/reference_trajectory', self._ref_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/atv/reference_velocities', self._vel_cb, 10)

        # ---- CSV log ----
        os.makedirs('log/controller', exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._log_path = f'log/controller/lpv_mpc_controller_{ts}.csv'
        self._log_file = open(self._log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        self._csv_writer.writerow([
            'timestamp_s', 'x', 'y', 'theta_deg', 'gamma_deg',
            'ref_x', 'ref_y', 'ref_theta_deg', 'ref_gamma_deg',
            'v_cmd', 'omega_gamma_cmd',
            'err_x', 'err_y', 'err_theta_deg', 'err_gamma_deg',
            'v_fl', 'v_fr', 'v_rl', 'v_rr',
            'closest_idx', 'qp_status',
            'J_total', 'Q_x', 'Q_y', 'Q_theta', 'Q_gamma',
            'R_v', 'R_omg', 'S_v', 'S_omg', 'grad_omg'
        ])
        self.get_logger().info(f'CSV log: {self._log_path}')

        # ---- Control loop timer ----
        self.dt = params['dt']
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'MPC Controller started  |  Np={params["Np"]}  Nc={params["Nc"]}'
            f'  dt={self.dt}s  OSQP={"Yes" if HAS_OSQP else "No (fallback)"}')

    # ------------------------------------------------------------------ #
    # Parameter helpers
    # ------------------------------------------------------------------ #
    def _declare_params(self):
        """Declare all ROS parameters with defaults from mpc_params.yaml."""
        defaults = {
            'Np': 20, 'Nc': 10, 'dt': 0.05,
            'Lf': 0.45, 'Lr': 0.45, 'B': 0.493,
            'wheel_radius': 0.09047,
            'Q': [5.0, 5.0, 15.0, 2.0],
            'R': [1.0, 1.0],
            'v_max': 2.0, 'v_min': -0.5,
            'omega_gamma_max': 0.5,
            'gamma_max': 0.6, 'gamma_min': -0.6,
            'dv_max': 0.5, 'domega_gamma_max': 0.3,
            'v_threshold': 0.3,
            'S': [5.0, 0.0],
        }
        for name, val in defaults.items():
            if isinstance(val, list):
                self.declare_parameter(name, val)
            else:
                self.declare_parameter(name, val)

    def _load_params(self) -> dict:
        p = {}
        for name in ['Np', 'Nc']:
            p[name] = self.get_parameter(name).get_parameter_value().integer_value
        for name in ['dt', 'Lf', 'Lr', 'B', 'wheel_radius',
                      'v_max', 'v_min', 'omega_gamma_max',
                      'gamma_max', 'gamma_min',
                      'dv_max', 'domega_gamma_max', 'v_threshold']:
            p[name] = self.get_parameter(name).get_parameter_value().double_value
        for name in ['Q', 'R', 'S']:
            p[name] = list(
                self.get_parameter(name).get_parameter_value().double_array_value)
        return p

    # ------------------------------------------------------------------ #
    # Subscriber callbacks
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry):
        """Extract [x, y, theta] from odometry."""
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.current_state = np.array([pos.x, pos.y, yaw, self.gamma])

    def _joint_state_cb(self, msg: JointState):
        """Extract current articulation angle from joint states."""
        if 'steering_joint' in msg.name:
            idx = msg.name.index('steering_joint')
            # SDF steering_joint: parent=front_base_link, child=rear_base_link, axis=(0,0,-1)
            # Joint position = theta_front - theta_rear = gamma  (matches thesis definition)
            self.gamma = msg.position[idx]
            if self.current_state is not None:
                self.current_state[3] = self.gamma

    def _ref_cb(self, msg: Path):
        """Store the latest reference trajectory.

        Always reset closest_idx when the planner sends a new trajectory.
        The planner replans the local trajectory from the current state each
        time, so the old progress index is generally invalid for the new plan.
        """
        old_len = len(self.ref_trajectory.poses) if self.ref_trajectory else 0
        new_len = len(msg.poses)
        self.ref_trajectory = msg
        self.closest_idx = 0  # always restart — planner replans from current state
        if new_len != old_len:
            self.get_logger().info(
                f'New reference trajectory received: {new_len} waypoints, resetting path progress.')

    def _vel_cb(self, msg: Float64MultiArray):
        """Store per-waypoint reference velocities from planner."""
        self.ref_velocities = list(msg.data)

    # ------------------------------------------------------------------ #
    # Reference trajectory processing
    # ------------------------------------------------------------------ #
    def _build_ref_arrays(self):
        """
        Build Np-length reference state and control arrays from the
        stored Path message.

        Uses distance-based lookahead: each prediction step i corresponds
        to a look-ahead distance  s_i = v_current * (i+1) * dt  along
        the path arc-length, so MPC vision scales with vehicle speed.
        """
        if self.ref_trajectory is None or self.current_state is None:
            return None, None

        poses = self.ref_trajectory.poses
        if len(poses) < 2:
            return None, None

        # Find closest waypoint — search FORWARD only, with a CAPPED window.
        #
        # Why cap is essential on curved paths (e.g. a circle):
        #   If the vehicle drifts slightly inside the circle, a point on the
        #   FAR side of the circle may be geometrically closer than the correct
        #   next point. Without a cap, closest_idx jumps 1200+ points and the
        #   reference heading flips ~180°, causing instant divergence.
        #
        # Window sizing: at v_max the vehicle covers v_max*dt metres per step.
        #   Estimate point spacing from first two poses, then allow a generous
        #   20× margin.  Minimum window = 50 points.
        dx0 = poses[1].pose.position.x - poses[0].pose.position.x
        dy0 = poses[1].pose.position.y - poses[0].pose.position.y
        pt_spacing = max(np.sqrt(dx0*dx0 + dy0*dy0), 1e-6)
        v_now = max(abs(self.mpc.u_prev[0]), 1.0)
        max_advance = max(int(v_now * self.dt / pt_spacing * 20), 50)
        search_end = min(self.closest_idx + max_advance, len(poses))

        cx, cy = self.current_state[0], self.current_state[1]
        min_dist = float('inf')
        closest_idx = self.closest_idx
        for i in range(self.closest_idx, search_end):
            ps = poses[i]
            dx = ps.pose.position.x - cx
            dy = ps.pose.position.y - cy
            d = dx * dx + dy * dy
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Safety: if closest point is too far (>3m), the forward-only search
        # may have missed; do a FULL path search as fallback.
        if min_dist > 9.0:  # 3m squared
            for i in range(len(poses)):
                ps = poses[i]
                dx = ps.pose.position.x - cx
                dy = ps.pose.position.y - cy
                d = dx * dx + dy * dy
                if d < min_dist:
                    min_dist = d
                    closest_idx = i
            self.get_logger().warn(
                f'Forward search too far ({np.sqrt(min_dist):.2f}m), '
                f'full reset closest_idx: {self.closest_idx} -> {closest_idx}')

        self.closest_idx = closest_idx  # persist for next cycle


        # Use REFERENCE speed (from iLQR plan) for arc-length sampling.
        # 使用规划速度而非实际车速采样，确保 MPC 视野与 iLQR 计划匹配。
        # 问题根因：如果 v_current=5m/s 但 iLQR 以 1m/s 规划，MPC 采样距离=5倍，
        # 该范围超出计划视野，导致参考速度塑造成 v_cmd=v_max。
        if self.ref_velocities is not None and closest_idx < len(self.ref_velocities):
            v_ref_sample = max(float(self.ref_velocities[closest_idx]), 0.3)
        else:
            v_ref_sample = 1.0  # safe default
        v_current = v_ref_sample  # ← 关键修改：用规划速度采样

        # Pre-compute cumulative arc-length from closest_idx
        max_look = min(closest_idx + len(poses), len(poses) - 1)
        arc_lengths = [0.0]
        for k in range(closest_idx, max_look):
            dx = poses[k + 1].pose.position.x - poses[k].pose.position.x
            dy = poses[k + 1].pose.position.y - poses[k].pose.position.y
            arc_lengths.append(arc_lengths[-1] + np.sqrt(dx * dx + dy * dy))

        Np = self.mpc.Np
        ref_states = np.zeros((Np, 4))
        ref_ctrls  = np.zeros((Np, 2))

        for i in range(Np):
            # Target arc-length for prediction step i
            target_s = v_current * (i + 1) * self.dt

            # Find the path index whose arc-length is >= target_s
            idx = closest_idx
            for k, s in enumerate(arc_lengths):
                if s >= target_s:
                    idx = min(closest_idx + k, len(poses) - 1)
                    break
            else:
                idx = min(closest_idx + len(arc_lengths) - 1, len(poses) - 1)

            ps = poses[idx].pose
            q = ps.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny, cosy)

            ref_states[i] = [ps.position.x, ps.position.y, yaw, ps.position.z]  # z carries gamma_ref

            # Use planner-provided reference velocity for linearisation.
            # Falls back to a safe low speed if planner hasn't published yet.
            if self.ref_velocities is not None and idx < len(self.ref_velocities):
                ref_ctrls[i, 0] = self.ref_velocities[idx]
            else:
                ref_ctrls[i, 0] = 0.3  # safe fallback, not v_max!
            # Bug fix: provide omega_gamma feedforward from the gamma_ref trajectory.
            # Previously hardcoded to 0, which left the MPC with no feedforward
            # on curves, causing systematic heading lag at higher speeds.
            # Now estimated via finite difference of the reference gamma.
            ref_ctrls[i, 1] = 0.0  # will be filled in after all ref_states are known

        # Fill in omega_gamma_ref via finite difference of reference gamma.
        # ref_ctrls[i, 1] = (gamma_ref[i+1] - gamma_ref[i]) / dt
        # This gives the MPC a feedforward term for steering on curved paths.
        for i in range(Np - 1):
            ref_ctrls[i, 1] = (
                ref_states[i + 1, 3] - ref_states[i, 3]
            ) / self.dt
        ref_ctrls[Np - 1, 1] = ref_ctrls[Np - 2, 1]  # repeat last

        # Clamp feedforward to physical limits
        ref_ctrls[:, 1] = np.clip(ref_ctrls[:, 1],
                                   -self.mpc.omega_max, self.mpc.omega_max)

        # --- Diagnostics for reference trajectory ---
        self._ref_diag_counter = getattr(self, '_ref_diag_counter', 0) + 1
        if self._logger is not None and self._ref_diag_counter % 20 == 0:
            self._logger.info(
                f'[REF] i=0 | x={ref_states[0,0]:.3f} y={ref_states[0,1]:.3f} '
                f'theta={np.degrees(ref_states[0,2]):.1f}deg gamma={np.degrees(ref_states[0,3]):.1f}deg'
            )

        return ref_states, ref_ctrls

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        """Main timer callback — runs at 1/dt Hz."""
        # ---- Diagnostic heartbeat (every 50 ticks ≈ 5s @10Hz) ----
        self._loop_counter = getattr(self, '_loop_counter', 0) + 1
        if self._loop_counter % 50 == 1:
            has_odom = self.current_state is not None
            has_traj = self.ref_trajectory is not None
            traj_len = len(self.ref_trajectory.poses) if has_traj else 0
            state_str = (f'[{self.current_state[0]:.2f}, {self.current_state[1]:.2f}, '
                         f'{np.degrees(self.current_state[2]):.1f}°, '
                         f'{np.degrees(self.current_state[3]):.1f}°]'
                         if has_odom else 'None')
            self.get_logger().info(
                f'[DIAG] tick={self._loop_counter} | '
                f'odom={has_odom} state={state_str} | '
                f'traj={has_traj} len={traj_len} | '
                f'u_prev=[{self.mpc.u_prev[0]:.3f}, {self.mpc.u_prev[1]:.3f}]')

        if self.current_state is None:
            return

        ref_states, ref_ctrls = self._build_ref_arrays()
        if ref_states is None:
            # No trajectory yet — stop the vehicle
            self._publish_stop()
            return

        # ---- Layer 1: MPC solve ----
        # Suppress OSQP stdout spam during solve
        with contextlib.redirect_stdout(io.StringIO()):
            v_cmd, omega_gamma_cmd = self.mpc.solve(
                self.current_state, ref_states, ref_ctrls)

        # ---- Layer 2 + 3: Kinematic allocation + track mapping ----
        v_fl, v_fr, v_rl, v_rr, omega_f = self.allocator.allocate(
            v_cmd, omega_gamma_cmd, self.gamma)

        # ---- CSV log (every tick) ----
        if hasattr(self, '_csv_writer'):
            now_s = self.get_clock().now().nanoseconds * 1e-9
            st = self.current_state
            ref0 = ref_states[0]
            err = st - ref0
            # Cost breakdown from last solve (default 0 before first solve)
            m = self.mpc
            cq_x  = getattr(m, '_cq_x',   0.0)
            cq_y  = getattr(m, '_cq_y',   0.0)
            cq_th = getattr(m, '_cq_th',  0.0)
            cq_gm = getattr(m, '_cq_gm',  0.0)
            cr_v  = getattr(m, '_cr_v',   0.0)
            cr_og = getattr(m, '_cr_omg', 0.0)
            cs_v  = getattr(m, '_cs_v',   0.0)
            cs_og = getattr(m, '_cs_omg', 0.0)
            c_tot = getattr(m, '_c_total',0.0)
            g_og  = getattr(m, '_grad_omg',0.0)
            self._csv_writer.writerow([
                f'{now_s:.4f}',
                f'{st[0]:.4f}', f'{st[1]:.4f}',
                f'{np.degrees(st[2]):.2f}', f'{np.degrees(st[3]):.2f}',
                f'{ref0[0]:.4f}', f'{ref0[1]:.4f}',
                f'{np.degrees(ref0[2]):.2f}', f'{np.degrees(ref0[3]):.2f}',
                f'{v_cmd:.4f}', f'{omega_gamma_cmd:.4f}',
                f'{err[0]:.4f}', f'{err[1]:.4f}',
                f'{np.degrees(err[2]):.2f}', f'{np.degrees(err[3]):.2f}',
                f'{v_fl:.4f}', f'{v_fr:.4f}', f'{v_rl:.4f}', f'{v_rr:.4f}',
                self.closest_idx, '',
                f'{c_tot:.4f}',
                f'{cq_x:.4f}', f'{cq_y:.4f}', f'{cq_th:.4f}', f'{cq_gm:.4f}',
                f'{cr_v:.4f}', f'{cr_og:.4f}', f'{cs_v:.4f}', f'{cs_og:.4f}',
                f'{g_og:.6f}'
            ])
            self._log_file.flush()

        # ---- Publish commands ----
        self._publish_commands(v_fl, v_fr, v_rl, v_rr,
                               omega_gamma_cmd, omega_f)

    # ------------------------------------------------------------------ #
    # Publishing helpers
    # ------------------------------------------------------------------ #
    def _publish_commands(self, v_fl, v_fr, v_rl, v_rr,
                          omega_gamma_cmd, omega_f):
        """
        Publish individual track velocity commands to JointController plugins.

        Each track's JointController receives a Float64 angular velocity (rad/s).
        Convert from track linear velocity (m/s) to wheel angular velocity.
        """
        R = self.allocator.R  # wheel radius (m)
        for pub, v in [(self.pub_fl, v_fl), (self.pub_fr, v_fr),
                       (self.pub_rl, v_rl), (self.pub_rr, v_rr)]:
            msg = Float64()
            msg.data = v / R   # linear velocity → wheel angular velocity (rad/s)
            pub.publish(msg)

        # Steering joint position command.
        # Anchor to actual gamma feedback to prevent open-loop integrator wind-up.
        # If joint is slow to respond, the internal integrator would otherwise
        # diverge from reality; using self.gamma as the base keeps them aligned.
        self.steering_joint_pos = np.clip(
            self.gamma + omega_gamma_cmd * self.dt,
            self.mpc.gamma_min, self.mpc.gamma_max
        )

        steer_msg = Float64()
        steer_msg.data = self.steering_joint_pos
        self.pub_steering.publish(steer_msg)

    def _publish_stop(self):
        """Publish zero-velocity commands."""
        zero = Float64()
        zero.data = 0.0
        self.pub_fl.publish(zero)
        self.pub_fr.publish(zero)
        self.pub_rl.publish(zero)
        self.pub_rr.publish(zero)


# =========================================================================== #
#                              Entry Point                                    #
# =========================================================================== #

def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except RuntimeError:
                # ros_gz_bridge JointState deserialization can fail
                # with "Unable to convert call argument to Python object".
                # Ignore and continue — the MPC does not depend on JointState.
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
