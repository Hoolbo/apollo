#include "modules/control/mpc_controller/mpc_controller_component.h"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "cyber/common/log.h"
#include "osqp/osqp.h"

namespace apollo {
namespace control {

using nlohmann::json;

// ===== Helper ==============================================================

static double NormalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ===== ArticulatedVehicleMPC ===============================================

ArticulatedVehicleMPC::ArticulatedVehicleMPC(const MpcParams& p) : p_(p) {}

void ArticulatedVehicleMPC::ContinuousJacobians(
    const Eigen::Vector4d& x_ref, const Eigen::Vector2d& u_ref,
    Eigen::Matrix4d& Ac, Eigen::Matrix<double, 4, 2>& Bc) const {
  double theta_r = x_ref[2];
  double gamma_r = x_ref[3];
  double v_r = u_ref[0];
  double Lf = p_.Lf, Lr = p_.Lr;
  double L = Lr + Lf * std::cos(gamma_r);

  double num = (v_r * std::cos(gamma_r)) * L +
               (v_r * std::sin(gamma_r) + Lr * u_ref[1]) *
                   (Lf * std::sin(gamma_r));
  double den = L * L;
  double a34 = (std::abs(den) > 1e-12) ? num / den : 0.0;

  Ac.setZero();
  Ac(0, 2) = -v_r * std::sin(theta_r);
  Ac(1, 2) = v_r * std::cos(theta_r);
  Ac(2, 3) = a34;

  Bc.setZero();
  Bc(0, 0) = std::cos(theta_r);
  Bc(1, 0) = std::sin(theta_r);
  Bc(2, 0) = (std::abs(L) > 1e-12) ? std::sin(gamma_r) / L : 0.0;
  Bc(2, 1) = (std::abs(L) > 1e-12) ? Lr / L : 0.0;
  Bc(3, 1) = 1.0;
}

void ArticulatedVehicleMPC::AugmentedModel(
    const Eigen::Matrix4d& Ac, const Eigen::Matrix<double, 4, 2>& Bc,
    Eigen::MatrixXd& A_aug, Eigen::MatrixXd& B_aug,
    Eigen::MatrixXd& C_aug) const {
  Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity() + p_.dt * Ac;
  Eigen::Matrix<double, 4, 2> Bd = p_.dt * Bc;

  A_aug.setZero(kNxi, kNxi);
  A_aug.topLeftCorner(kNx, kNx) = Ad;
  A_aug.topRightCorner(kNx, kNu) = Bd;
  A_aug.bottomRightCorner(kNu, kNu) = Eigen::Matrix2d::Identity();

  B_aug.setZero(kNxi, kNu);
  B_aug.topRows(kNx) = Bd;
  B_aug.bottomRows(kNu) = Eigen::Matrix2d::Identity();

  C_aug.setZero(kNx, kNxi);
  C_aug.topLeftCorner(kNx, kNx) = Eigen::Matrix4d::Identity();
}

std::pair<double, double> ArticulatedVehicleMPC::Solve(
    const Eigen::Vector4d& x_current,
    const Eigen::MatrixXd& ref_states,
    const Eigen::MatrixXd& ref_ctrls) {
  int Np = p_.Np, Nc = p_.Nc;

  // Build LPV prediction matrices
  std::vector<Eigen::MatrixXd> A_aug_list(Np), B_aug_list(Np);
  Eigen::MatrixXd C_aug;

  for (int i = 0; i < Np; ++i) {
    Eigen::Matrix4d Ac_i;
    Eigen::Matrix<double, 4, 2> Bc_i;
    ContinuousJacobians(ref_states.row(i), ref_ctrls.row(i), Ac_i, Bc_i);
    AugmentedModel(Ac_i, Bc_i, A_aug_list[i], B_aug_list[i], C_aug);
  }

  // Prediction matrices Psi, Theta
  int ny = kNx;
  Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(Np * ny, kNxi);
  Eigen::MatrixXd Theta = Eigen::MatrixXd::Zero(Np * ny, Nc * kNu);

  // Cumulative A products
  std::vector<Eigen::MatrixXd> A_cum(Np);
  Eigen::MatrixXd A_prod = Eigen::MatrixXd::Identity(kNxi, kNxi);
  for (int i = 0; i < Np; ++i) {
    A_prod = A_aug_list[i] * A_prod;
    A_cum[i] = A_prod;
  }

  for (int i = 0; i < Np; ++i) {
    Psi.block(i * ny, 0, ny, kNxi) = C_aug * A_cum[i];
    for (int j = 0; j < std::min(i + 1, Nc); ++j) {
      Eigen::MatrixXd CB;
      if (j == i) {
        CB = C_aug * B_aug_list[j];
      } else {
        Eigen::MatrixXd A_chain =
            Eigen::MatrixXd::Identity(kNxi, kNxi);
        for (int m = j + 1; m <= i; ++m) {
          A_chain = A_aug_list[m] * A_chain;
        }
        CB = C_aug * A_chain * B_aug_list[j];
      }
      Theta.block(i * ny, j * kNu, ny, kNu) = CB;
    }
  }

  // Error state
  Eigen::Vector4d x_ref0 = ref_states.row(0);
  Eigen::Vector4d x_err = x_current - x_ref0;
  x_err[2] = NormalizeAngle(x_err[2]);
  x_err[3] = NormalizeAngle(x_err[3]);

  Eigen::VectorXd xi(kNxi);
  xi.head(kNx) = x_err;
  xi.tail(kNu) = u_prev_;

  // Reference output
  Eigen::VectorXd Y_ref = Eigen::VectorXd::Zero(Np * ny);
  for (int i = 0; i < Np; ++i) {
    Eigen::Vector4d ref_err = ref_states.row(i).transpose() - x_ref0;
    ref_err[2] = NormalizeAngle(ref_err[2]);
    ref_err[3] = NormalizeAngle(ref_err[3]);
    Y_ref.segment(i * ny, ny) = ref_err;
  }

  // Q_bar, R_bar
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(Np * ny, Np * ny);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(Nc * kNu, Nc * kNu);
  for (int i = 0; i < Np; ++i) {
    Q_bar.block(i * ny, i * ny, ny, ny) = p_.Q_diag.asDiagonal();
  }
  for (int i = 0; i < Nc; ++i) {
    R_bar.block(i * kNu, i * kNu, kNu, kNu) = p_.R_diag.asDiagonal();
  }

  // H and f
  Eigen::MatrixXd H = Theta.transpose() * Q_bar * Theta + R_bar;

  // Triangular matrix for absolute control recovery
  Eigen::MatrixXd M_tri =
      Eigen::MatrixXd::Zero(Nc * kNu, Nc * kNu);
  for (int i = 0; i < Nc; ++i) {
    for (int j = 0; j <= i; ++j) {
      M_tri.block(i * kNu, j * kNu, kNu, kNu) =
          Eigen::Matrix2d::Identity();
    }
  }

  // u_prev repeated
  Eigen::VectorXd u_prev_vec(Nc * kNu);
  Eigen::VectorXd u_ref_vec(Nc * kNu);
  for (int j = 0; j < Nc; ++j) {
    u_prev_vec.segment(j * kNu, kNu) = u_prev_;
    int ref_idx = std::min(j, Np - 1);
    u_ref_vec.segment(j * kNu, kNu) = ref_ctrls.row(ref_idx).transpose();
  }

  // Smoothing term
  if (p_.S_diag.squaredNorm() > 0) {
    Eigen::MatrixXd S_bar =
        Eigen::MatrixXd::Zero(Nc * kNu, Nc * kNu);
    for (int i = 0; i < Nc; ++i) {
      S_bar.block(i * kNu, i * kNu, kNu, kNu) = p_.S_diag.asDiagonal();
    }
    H += M_tri.transpose() * S_bar * M_tri;
  }

  H = (H + H.transpose()) / 2.0;
  Eigen::VectorXd f =
      Theta.transpose() * Q_bar * (Psi * xi - Y_ref);

  // Box constraints on dU
  Eigen::VectorXd lb(Nc * kNu), ub(Nc * kNu);
  for (int j = 0; j < Nc; ++j) {
    lb[j * kNu + 0] = -p_.dv_max;
    lb[j * kNu + 1] = -p_.domega_max;
    ub[j * kNu + 0] = p_.dv_max;
    ub[j * kNu + 1] = p_.domega_max;
  }

  // Solve QP
  Eigen::VectorXd dU = SolveQP(H, f, lb, ub);

  // Extract first control increment
  Eigen::Vector2d du_0 = dU.head(kNu);
  Eigen::Vector2d u_new = u_prev_ + du_0;
  u_new[0] = std::clamp(u_new[0], p_.v_min, p_.v_max);
  u_new[1] = std::clamp(u_new[1], -p_.omega_max, p_.omega_max);
  u_prev_ = u_new;

  return {u_new[0], u_new[1]};
}

Eigen::VectorXd ArticulatedVehicleMPC::SolveQP(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) const {
  int n = H.rows();

  // ── Convert dense H to CSC (upper-triangular) ──
  std::vector<c_float> P_data;
  std::vector<c_int> P_row, P_col_ptr;
  P_col_ptr.push_back(0);
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i <= j; ++i) {
      if (std::abs(H(i, j)) > 1e-14) {
        P_data.push_back(static_cast<c_float>(H(i, j)));
        P_row.push_back(static_cast<c_int>(i));
      }
    }
    P_col_ptr.push_back(static_cast<c_int>(P_data.size()));
  }
  c_int P_nnz = static_cast<c_int>(P_data.size());

  // ── Identity constraint matrix A = I (for box constraints) ──
  std::vector<c_float> A_data(n, 1.0);
  std::vector<c_int> A_row(n);
  std::vector<c_int> A_col_ptr(n + 1);
  for (int i = 0; i < n; ++i) {
    A_row[i] = i;
    A_col_ptr[i] = i;
  }
  A_col_ptr[n] = n;

  // ── OSQP data ──
  std::vector<c_float> q_vec(n);
  std::vector<c_float> l_vec(n), u_vec(n);
  for (int i = 0; i < n; ++i) {
    q_vec[i] = static_cast<c_float>(f[i]);
    l_vec[i] = static_cast<c_float>(lb[i]);
    u_vec[i] = static_cast<c_float>(ub[i]);
  }

  OSQPData* data = static_cast<OSQPData*>(c_calloc(1, sizeof(OSQPData)));
  data->n = n;
  data->m = n;
  data->P = csc_matrix(n, n, P_nnz, P_data.data(), P_row.data(),
                       P_col_ptr.data());
  data->A = csc_matrix(n, n, n, A_data.data(), A_row.data(),
                       A_col_ptr.data());
  data->q = q_vec.data();
  data->l = l_vec.data();
  data->u = u_vec.data();

  OSQPSettings* settings =
      static_cast<OSQPSettings*>(c_calloc(1, sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->verbose = 0;
  settings->warm_start = 1;
  settings->max_iter = 200;
  settings->eps_abs = 1e-4;
  settings->eps_rel = 1e-4;
  settings->polish = 1;

  OSQPWorkspace* work = osqp_setup(data, settings);

  Eigen::VectorXd result = Eigen::VectorXd::Zero(n);
  if (work) {
    c_int status = osqp_solve(work);
    if (status == 0 && work->info->status_val == OSQP_SOLVED) {
      for (int i = 0; i < n; ++i) {
        result[i] = work->solution->x[i];
      }
    } else {
      // Fallback: unconstrained + clamp
      AWARN << "OSQP failed (status=" << work->info->status_val
            << "), using LDLT fallback";
      result = -H.ldlt().solve(f);
      for (int i = 0; i < n; ++i) {
        result[i] = std::clamp(result[i], lb[i], ub[i]);
      }
    }
    osqp_cleanup(work);
  } else {
    AWARN << "OSQP setup failed, using LDLT fallback";
    result = -H.ldlt().solve(f);
    for (int i = 0; i < n; ++i) {
      result[i] = std::clamp(result[i], lb[i], ub[i]);
    }
  }

  // Free CSC matrices (osqp_cleanup doesn't free data->P/A)
  if (data->P) c_free(data->P);
  if (data->A) c_free(data->A);
  c_free(data);
  c_free(settings);

  return result;
}

// ===== AckermannAllocator ==================================================

AckermannOutput AckermannAllocator::Allocate(
    double v_cmd, double omega_gamma, double gamma) const {
  double Lf = p_.Lf, Lr = p_.Lr;

  // Front body yaw rate
  double L_eff = Lr + Lf * std::cos(gamma);
  if (std::abs(L_eff) < 1e-9) L_eff = 1e-9;
  double omega_front =
      (v_cmd * std::sin(gamma) + Lr * omega_gamma) / L_eff;

  // Rear body
  double omega_rear = omega_front - omega_gamma;
  double v_rear = v_cmd * std::cos(gamma) + Lf * omega_front * std::sin(gamma);

  double v_front = v_cmd;

  // Ackermann inverse: delta = atan(omega * L_wb / v)
  double delta_front = 0.0;
  if (std::abs(v_front) > 0.01) {
    delta_front = std::atan(omega_front * p_.L_wb_front / v_front);
  }
  double delta_rear = 0.0;
  if (std::abs(v_rear) > 0.01) {
    delta_rear = std::atan(omega_rear * p_.L_wb_rear / v_rear);
  }

  delta_front = std::clamp(delta_front, p_.delta_min_front, p_.delta_max_front);
  delta_rear = std::clamp(delta_rear, p_.delta_min_rear, p_.delta_max_rear);

  return {v_front, delta_front, v_rear, delta_rear};
}

// ===== MpcControllerComponent ==============================================

bool MpcControllerComponent::Init() {
  // ── Load JSON configs ──
  std::string conf_dir =
      "/apollo/modules/control/mpc_controller/conf";
  std::string ws_dir =
      "/apollo_workspace/modules/control/mpc_controller/conf";
  if (std::filesystem::exists(ws_dir)) {
    conf_dir = ws_dir;
  }

  // vehicle.json
  {
    std::ifstream f(conf_dir + "/vehicle.json");
    if (!f.is_open()) {
      AERROR << "Cannot open vehicle.json from " << conf_dir;
      return false;
    }
    json j = json::parse(f);
    auto art = j["articulation"];
    mpc_params_.Lf = art["Lf"].get<double>();
    mpc_params_.Lr = art["Lr"].get<double>();
    mpc_params_.gamma_max = art["gamma_max"].get<double>();
    mpc_params_.gamma_min = art["gamma_min"].get<double>();
    mpc_params_.omega_max = art["omega_gamma_max"].get<double>();

    auto front = j["front_body"];
    mpc_params_.v_max = front["v_max"].get<double>();
    mpc_params_.v_min = front["v_min"].get<double>();

    ack_params_.Lf = mpc_params_.Lf;
    ack_params_.Lr = mpc_params_.Lr;
    ack_params_.L_wb_front = front["wheelbase"].get<double>();
    ack_params_.delta_max_front = front["delta_max"].get<double>();
    ack_params_.delta_min_front = front["delta_min"].get<double>();

    auto rear = j["rear_body"];
    ack_params_.L_wb_rear = rear["wheelbase"].get<double>();
    ack_params_.delta_max_rear = rear["delta_max"].get<double>();
    ack_params_.delta_min_rear = rear["delta_min"].get<double>();

    // Gamma feedback compensation gain
    if (j.contains("gamma_feedback")) {
      Kp_gamma_ = j["gamma_feedback"].value("Kp", 0.5);
      AINFO << "Gamma feedback Kp=" << Kp_gamma_;
    }
  }

  // mpc_params.json
  {
    std::ifstream f(conf_dir + "/mpc_params.json");
    if (!f.is_open()) {
      AERROR << "Cannot open mpc_params.json from " << conf_dir;
      return false;
    }
    json j = json::parse(f);
    auto h = j["horizons"];
    mpc_params_.Np = h["Np"].get<int>();
    mpc_params_.Nc = h["Nc"].get<int>();
    mpc_params_.dt = h["dt"].get<double>();
    dt_ = mpc_params_.dt;

    auto w = j["weights"];
    auto Q = w["Q"].get<std::vector<double>>();
    auto R = w["R"].get<std::vector<double>>();
    auto S = w.value("S", std::vector<double>{0.0, 0.0});
    mpc_params_.Q_diag = Eigen::Vector4d(Q[0], Q[1], Q[2], Q[3]);
    mpc_params_.R_diag = Eigen::Vector2d(R[0], R[1]);
    mpc_params_.S_diag = Eigen::Vector2d(S[0], S[1]);

    auto c = j["constraints"];
    mpc_params_.dv_max = c["dv_max"].get<double>();
    mpc_params_.domega_max = c["domega_gamma_max"].get<double>();
    mpc_params_.v_threshold = c["v_threshold"].get<double>();
  }

  // ── Create solver & allocator ──
  mpc_ = std::make_unique<ArticulatedVehicleMPC>(mpc_params_);
  allocator_ = std::make_unique<AckermannAllocator>(ack_params_);

  // ── Readers ──
  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          "/apollo/localization/pose",
          [this](const auto& msg) { OnLocalization(msg); });

  trajectory_reader_ = node_->CreateReader<planning::ADCTrajectory>(
      "/apollo/planning",
      [this](const auto& msg) { OnTrajectory(msg); });

  chassis_reader_ = node_->CreateReader<canbus::Chassis>(
      "/apollo/canbus/chassis",
      [this](const auto& msg) { OnChassis(msg); });

  // 后车定位：读取后车 IMU 航向，计算铰接角 gamma
  rear_localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          "/apollo/localization/pose_rear",
          [this](const auto& msg) { OnRearLocalization(msg); });

  // ── Writer ──
  ctrl_writer_ = node_->CreateWriter<control::ControlCommand>(
      "/apollo/control");

  // ── CSV log ──
  std::filesystem::create_directories("/tmp/controller");
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << "/tmp/controller/mpc_ctrl_"
      << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
  log_file_.open(oss.str());
  log_file_ << "time_s,x,y,theta_deg,gamma_deg,"
            << "v_cmd,omega_gamma_cmd,"
            << "v_front,delta_front_deg,v_rear,delta_rear_deg,"
            << "closest_idx,gamma_ref,gamma_err,v_rear_comp\n";

  // ── Timer ──
  uint32_t period_ms = static_cast<uint32_t>(dt_ * 1000.0);
  timer_ = std::make_unique<cyber::Timer>(
      period_ms, [this]() { ControlLoop(); }, false);
  timer_->Start();

  AINFO << "MPC Controller started | Np=" << mpc_params_.Np
        << " Nc=" << mpc_params_.Nc << " dt=" << dt_ << "s";
  AINFO << "CSV log: " << oss.str();
  return true;
}

MpcControllerComponent::~MpcControllerComponent() {
  if (timer_) timer_->Stop();
  if (log_file_.is_open()) log_file_.close();
}

// ─── Callbacks ──────────────────────────────────────────────────────────────

void MpcControllerComponent::OnLocalization(
    const std::shared_ptr<localization::LocalizationEstimate>& msg) {
  std::lock_guard<std::mutex> lk(state_mutex_);
  current_state_[0] = msg->pose().position().x();
  current_state_[1] = msg->pose().position().y();
  current_state_[2] = msg->pose().heading();
  current_state_[3] = gamma_;
  odom_received_ = true;
}

void MpcControllerComponent::OnTrajectory(
    const std::shared_ptr<planning::ADCTrajectory>& msg) {
  if (msg->trajectory_point_size() < 2) return;
  std::lock_guard<std::mutex> lk(traj_mutex_);
  latest_trajectory_ = msg;
  closest_idx_ = 0;
}

void MpcControllerComponent::OnChassis(
    const std::shared_ptr<canbus::Chassis>& msg) {
  // chassis 不再用于读取 gamma，gamma 由双 IMU 航向差计算
  // 可用于读取车速等其他信息（当前未使用）
  (void)msg;
}

void MpcControllerComponent::OnRearLocalization(
    const std::shared_ptr<localization::LocalizationEstimate>& msg) {
  std::lock_guard<std::mutex> lk(state_mutex_);
  rear_theta_ = msg->pose().heading();
  rear_odom_received_ = true;
  // gamma = 前车航向 - 后车航向，归一化到 [-π, π]
  double gamma = current_state_[2] - rear_theta_;
  while (gamma > M_PI) gamma -= 2 * M_PI;
  while (gamma < -M_PI) gamma += 2 * M_PI;
  gamma_ = gamma;
  current_state_[3] = gamma_;
}

// ─── Reference trajectory ─────────────────────────────────────────────────

bool MpcControllerComponent::BuildRefArrays(
    Eigen::MatrixXd& ref_states, Eigen::MatrixXd& ref_ctrls) {
  std::shared_ptr<planning::ADCTrajectory> traj;
  Eigen::Vector4d cur;
  int closest;
  {
    std::lock_guard<std::mutex> lk1(traj_mutex_);
    std::lock_guard<std::mutex> lk2(state_mutex_);
    if (!latest_trajectory_ || !odom_received_) return false;
    traj = latest_trajectory_;
    cur = current_state_;
    closest = closest_idx_;
  }

  int n_pts = traj->trajectory_point_size();
  if (n_pts < 2) return false;

  // Find closest point
  double cx = cur[0], cy = cur[1];
  double min_dist = 1e18;

  auto& pts = traj->trajectory_point();
  double dx0 = pts[1].path_point().x() - pts[0].path_point().x();
  double dy0 = pts[1].path_point().y() - pts[0].path_point().y();
  double spacing = std::max(std::hypot(dx0, dy0), 1e-6);
  double v_now = std::max(std::abs(mpc_->u_prev()[0]), 1.0);
  int max_adv = std::max(static_cast<int>(v_now * dt_ / spacing * 20), 50);
  int search_end = std::min(closest + max_adv, n_pts);

  for (int i = closest; i < search_end; ++i) {
    double dx = pts[i].path_point().x() - cx;
    double dy = pts[i].path_point().y() - cy;
    double d = dx * dx + dy * dy;
    if (d < min_dist) { min_dist = d; closest = i; }
  }

  if (min_dist > 9.0) {
    for (int i = 0; i < n_pts; ++i) {
      double dx = pts[i].path_point().x() - cx;
      double dy = pts[i].path_point().y() - cy;
      double d = dx * dx + dy * dy;
      if (d < min_dist) { min_dist = d; closest = i; }
    }
  }

  {
    std::lock_guard<std::mutex> lk(traj_mutex_);
    closest_idx_ = closest;
  }

  int Np = mpc_params_.Np;
  ref_states.setZero(Np, 4);
  ref_ctrls.setZero(Np, 2);

  double v_ref = (closest < n_pts)
      ? std::max(std::abs(pts[closest].v()), 0.3) : 1.0;

  // Arc length
  std::vector<double> arc = {0.0};
  for (int k = closest; k < std::min(closest + n_pts, n_pts - 1); ++k) {
    double dx = pts[k+1].path_point().x() - pts[k].path_point().x();
    double dy = pts[k+1].path_point().y() - pts[k].path_point().y();
    arc.push_back(arc.back() + std::hypot(dx, dy));
  }

  for (int i = 0; i < Np; ++i) {
    double target_s = v_ref * (i + 1) * dt_;
    int idx = closest;
    for (size_t k = 0; k < arc.size(); ++k) {
      if (arc[k] >= target_s) {
        idx = std::min(closest + static_cast<int>(k), n_pts - 1);
        break;
      }
      idx = std::min(closest + static_cast<int>(arc.size()) - 1, n_pts - 1);
    }

    auto& pt = pts[idx];
    ref_states(i, 0) = pt.path_point().x();
    ref_states(i, 1) = pt.path_point().y();
    ref_states(i, 2) = pt.path_point().theta();
    ref_states(i, 3) = pt.path_point().kappa();  // γ in kappa
    ref_ctrls(i, 0) = pt.v();
  }

  // omega_gamma_ref via finite difference
  for (int i = 0; i < Np - 1; ++i) {
    ref_ctrls(i, 1) = (ref_states(i+1, 3) - ref_states(i, 3)) / dt_;
  }
  if (Np >= 2) ref_ctrls(Np-1, 1) = ref_ctrls(Np-2, 1);

  for (int i = 0; i < Np; ++i) {
    ref_ctrls(i, 1) = std::clamp(ref_ctrls(i, 1),
                                  -mpc_params_.omega_max,
                                  mpc_params_.omega_max);
  }
  return true;
}

// ─── Control loop ──────────────────────────────────────────────────────────

void MpcControllerComponent::ControlLoop() {
  ++tick_;
  if (tick_ % 100 == 1) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    AINFO << "[MPC] tick=" << tick_
          << " pos=(" << current_state_[0] << ", " << current_state_[1]
          << ") theta=" << current_state_[2] * 180.0 / M_PI << "°"
          << " gamma=" << current_state_[3] * 180.0 / M_PI << "°";
  }

  Eigen::Vector4d cur;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (!odom_received_) return;
    cur = current_state_;
  }

  Eigen::MatrixXd ref_states, ref_ctrls;
  if (!BuildRefArrays(ref_states, ref_ctrls)) {
    PublishStop();
    return;
  }

  // Layer 1: MPC solve
  auto [v_cmd, omega_gamma_cmd] = mpc_->Solve(cur, ref_states, ref_ctrls);

  // Layer 2+3: Ackermann allocation
  double gamma;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    gamma = gamma_;
  }
  auto cmd = allocator_->Allocate(v_cmd, omega_gamma_cmd, gamma);

  // Layer 4: Gamma feedback compensation
  // 用 γ 跟踪误差微调后车速度，补偿开环映射的不精确
  double gamma_ref = ref_states(0, 3);  // 期望铰接角
  double gamma_err = gamma_ref - gamma; // 正值 = γ不够大
  double v_rear_comp = -Kp_gamma_ * gamma_err;
  cmd.v_rear += v_rear_comp;

  // CSV log
  if (log_file_.is_open()) {
    log_file_ << std::fixed << std::setprecision(4)
              << cyber::Time::Now().ToSecond() << ","
              << cur[0] << "," << cur[1] << ","
              << cur[2] * 180.0 / M_PI << ","
              << cur[3] * 180.0 / M_PI << ","
              << v_cmd << "," << omega_gamma_cmd << ","
              << cmd.v_front << ","
              << cmd.delta_front * 180.0 / M_PI << ","
              << cmd.v_rear << ","
              << cmd.delta_rear * 180.0 / M_PI << ","
              << closest_idx_ << ","
              << gamma_ref << "," << gamma_err << ","
              << v_rear_comp << "\n";
    log_file_.flush();
  }

  PublishCmd(cmd);
}

void MpcControllerComponent::PublishCmd(const AckermannOutput& cmd) {
  auto msg = std::make_shared<control::ControlCommand>();
  msg->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  msg->mutable_header()->set_module_name("mpc_controller");

  msg->set_speed(cmd.v_front);
  msg->set_steering_target(cmd.delta_front * 180.0 / M_PI);
  msg->set_acceleration(0.0);

  // Debug string for CAN adapter
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4)
      << "v_front=" << cmd.v_front
      << ",delta_front=" << cmd.delta_front
      << ",v_rear=" << cmd.v_rear
      << ",delta_rear=" << cmd.delta_rear;
  msg->mutable_header()->mutable_status()->set_msg(oss.str());

  ctrl_writer_->Write(msg);
}

void MpcControllerComponent::PublishStop() {
  auto msg = std::make_shared<control::ControlCommand>();
  msg->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  msg->mutable_header()->set_module_name("mpc_controller");
  msg->set_speed(0.0);
  msg->set_steering_target(0.0);
  msg->set_acceleration(0.0);
  ctrl_writer_->Write(msg);
}

}  // namespace control
}  // namespace apollo
