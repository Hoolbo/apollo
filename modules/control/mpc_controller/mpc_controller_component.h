#pragma once

#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"

#include "cyber/cyber.h"
#include "cyber/time/time.h"

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace control {

// ─── Articulated Vehicle LPV-MPC Solver ─────────────────────────────────────

struct MpcParams {
  int Np = 20;       // prediction horizon
  int Nc = 10;       // control horizon
  double dt = 0.05;  // time step

  double Lf = 0.45;
  double Lr = 0.45;

  Eigen::Vector4d Q_diag;  // state tracking weight
  Eigen::Vector2d R_diag;  // control effort weight
  Eigen::Vector2d S_diag;  // control smoothing weight

  double v_max = 3.0;
  double v_min = -0.5;
  double omega_max = 0.5;
  double gamma_max = 1.0472;
  double gamma_min = -1.0472;
  double dv_max = 1.0;
  double domega_max = 0.5;
  double v_threshold = 0.01;
};

class ArticulatedVehicleMPC {
 public:
  explicit ArticulatedVehicleMPC(const MpcParams& p);

  // Solve one MPC step. Returns (v_cmd, omega_gamma_cmd).
  std::pair<double, double> Solve(
      const Eigen::Vector4d& x_current,
      const Eigen::MatrixXd& ref_states,   // Np x 4
      const Eigen::MatrixXd& ref_ctrls);   // Np x 2

  Eigen::Vector2d u_prev() const { return u_prev_; }

 private:
  void ContinuousJacobians(const Eigen::Vector4d& x_ref,
                           const Eigen::Vector2d& u_ref,
                           Eigen::Matrix4d& Ac,
                           Eigen::Matrix<double, 4, 2>& Bc) const;

  void AugmentedModel(const Eigen::Matrix4d& Ac,
                      const Eigen::Matrix<double, 4, 2>& Bc,
                      Eigen::MatrixXd& A_aug,
                      Eigen::MatrixXd& B_aug,
                      Eigen::MatrixXd& C_aug) const;

  Eigen::VectorXd SolveQP(const Eigen::MatrixXd& H,
                           const Eigen::VectorXd& f,
                           const Eigen::VectorXd& lb,
                           const Eigen::VectorXd& ub) const;

  MpcParams p_;
  static constexpr int kNx = 4;
  static constexpr int kNu = 2;
  static constexpr int kNxi = kNx + kNu;  // augmented state

  Eigen::Vector2d u_prev_ = Eigen::Vector2d::Zero();
};

// ─── Ackermann Kinematic Allocator ──────────────────────────────────────────

struct AckermannParams {
  double Lf, Lr;
  double L_wb_front, delta_max_front, delta_min_front;
  double L_wb_rear, delta_max_rear, delta_min_rear;
};

struct AckermannOutput {
  double v_front, delta_front, v_rear, delta_rear;
};

class AckermannAllocator {
 public:
  explicit AckermannAllocator(const AckermannParams& p) : p_(p) {}
  AckermannOutput Allocate(double v_cmd, double omega_gamma, double gamma) const;
 private:
  AckermannParams p_;
};

// ─── Cyber RT Component ─────────────────────────────────────────────────────

class MpcControllerComponent : public cyber::Component<> {
 public:
  bool Init() override;
  ~MpcControllerComponent() override;

 private:
  void ControlLoop();

  void OnLocalization(
      const std::shared_ptr<localization::LocalizationEstimate>& msg);
  void OnTrajectory(
      const std::shared_ptr<planning::ADCTrajectory>& msg);
  void OnChassis(
      const std::shared_ptr<canbus::Chassis>& msg);
  void OnRearLocalization(
      const std::shared_ptr<localization::LocalizationEstimate>& msg);

  // Build reference arrays from latest trajectory
  bool BuildRefArrays(Eigen::MatrixXd& ref_states,
                      Eigen::MatrixXd& ref_ctrls);

  void PublishCmd(const AckermannOutput& cmd);
  void PublishStop();

  // Config
  MpcParams mpc_params_;
  AckermannParams ack_params_;
  double dt_ = 0.05;

  // Solver + allocator
  std::unique_ptr<ArticulatedVehicleMPC> mpc_;
  std::unique_ptr<AckermannAllocator> allocator_;

  // State
  std::mutex state_mutex_;
  Eigen::Vector4d current_state_ = Eigen::Vector4d::Zero();
  double gamma_ = 0.0;
  double rear_theta_ = 0.0;
  bool odom_received_ = false;
  bool rear_odom_received_ = false;

  // Trajectory
  std::mutex traj_mutex_;
  std::shared_ptr<planning::ADCTrajectory> latest_trajectory_;
  int closest_idx_ = 0;

  // Readers / Writers
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> trajectory_reader_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      rear_localization_reader_;
  std::shared_ptr<cyber::Writer<control::ControlCommand>> ctrl_writer_;

  // Timer
  std::unique_ptr<cyber::Timer> timer_;
  int tick_ = 0;

  // CSV log
  std::ofstream log_file_;
};

CYBER_REGISTER_COMPONENT(MpcControllerComponent)

}  // namespace control
}  // namespace apollo
