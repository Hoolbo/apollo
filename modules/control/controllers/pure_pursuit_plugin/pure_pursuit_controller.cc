#include "modules/control/controllers/pure_pursuit_plugin/pure_pursuit_controller.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include "cyber/common/log.h"

namespace apollo {
namespace control {

using apollo::common::Status;
using apollo::common::ErrorCode;

// 1. 【适配】新的 Init 签名
Status PurePursuitController::Init(std::shared_ptr<DependencyInjector> injector) {
  AINFO << "Pure Pursuit Controller Init!";
  return Status::OK();
}

std::string PurePursuitController::Name() const { return "PURE_PURSUIT_CONTROLLER"; }

Status PurePursuitController::Reset() { return Status::OK(); }

void PurePursuitController::Stop() { AINFO << "Pure Pursuit Controller Stop"; }

// 2. 【适配】新的 ComputeControlCommand 签名 (去掉了 last_command)
Status PurePursuitController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    control::ControlCommand *cmd) {

  if (localization == nullptr || planning_published_trajectory == nullptr || chassis == nullptr) {
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Input is null");
  }

  auto pose = localization->pose();
  double current_x = pose.position().x();
  double current_y = pose.position().y();
  double current_yaw = pose.heading();
  double current_v = chassis->speed_mps(); 

  int target_ind = CalcTargetIndex(current_x, current_y, current_v, planning_published_trajectory);

  auto trajectory_points = planning_published_trajectory->trajectory_point();
  if (trajectory_points.empty()) {
      return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory is empty");
  }
  
  if (target_ind >= static_cast<int>(trajectory_points.size())) {
    target_ind = trajectory_points.size() - 1;
  }
  
  double tx = trajectory_points[target_ind].path_point().x();
  double ty = trajectory_points[target_ind].path_point().y();

  double alpha = std::atan2(ty - current_y, tx - current_x) - current_yaw;

  if (current_v < 0) {
    alpha = M_PI - alpha;
  }

  double Lf = K_ * std::abs(current_v) + Lfc_;

  // 计算 Delta. 如果实车方向反了，这里加负号
  double delta = std::atan2(2.0 * L_ * std::sin(alpha), Lf);

  double steer_percent = (delta / max_steer_angle_) * 100.0;
  
  if (steer_percent > 100.0) steer_percent = 100.0;
  if (steer_percent < -100.0) steer_percent = -100.0;

  cmd->set_steering_target(steer_percent);
  cmd->set_steering_rate(0.0); 

  // 简单纵向逻辑
  double dist_to_end = std::hypot(trajectory_points.back().path_point().x() - current_x,
                                  trajectory_points.back().path_point().y() - current_y);
  if (dist_to_end < 0.5) {
      cmd->set_speed(0.0);
      cmd->set_throttle(0.0);
      cmd->set_brake(50.0);
  } else {
      cmd->set_speed(2.0);
      cmd->set_throttle(20.0);
      cmd->set_brake(0.0);
  }

  return Status::OK();
}

int PurePursuitController::CalcTargetIndex(const double x, const double y, const double v,
                                           const planning::ADCTrajectory *trajectory) {
  int best_ind = 0;
  double min_dist = std::numeric_limits<double>::max();
  auto points = trajectory->trajectory_point();

  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    double dx = x - points[i].path_point().x();
    double dy = y - points[i].path_point().y();
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_ind = i;
    }
  }

  double Lf = K_ * std::abs(v) + Lfc_;
  double current_dist = 0.0;
  int target_ind = best_ind;

  while (current_dist < Lf && target_ind < static_cast<int>(points.size()) - 1) {
    double dx = points[target_ind + 1].path_point().x() - points[target_ind].path_point().x();
    double dy = points[target_ind + 1].path_point().y() - points[target_ind].path_point().y();
    current_dist += std::sqrt(dx * dx + dy * dy);
    target_ind++;
  }

  return target_ind;
}

}  // namespace control
}  // namespace apollo