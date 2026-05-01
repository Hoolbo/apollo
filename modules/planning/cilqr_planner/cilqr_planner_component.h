#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/cilqr_planner/proto/cilqr_planner_conf.pb.h"

#include "cyber/cyber.h"

// CILQR library headers
#include "modules/planning/cilqr_planner/cilqr_lib/include/common_types.h"
#include "modules/planning/cilqr_planner/cilqr_lib/include/config_loader.h"
#include "modules/planning/cilqr_planner/cilqr_lib/include/articulated_hybrid_astar.h"
#include "modules/planning/cilqr_planner/cilqr_lib/include/ilqr.h"
#include "modules/planning/cilqr_planner/cilqr_lib/include/utils.h"

namespace apollo {
namespace planning {
namespace cilqr {

class CilqrPlannerComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~CilqrPlannerComponent();

 private:
  // Core planning loop (called by timer)
  void PlanAndPublish();

  // Global planning (Hybrid A*)
  bool ReplanGlobal(double sx, double sy, double stheta, double sgamma);

  // Publish idle trajectory (goal reached)
  void PublishIdle(const State& s);

  // Fill ADCTrajectory from planned states and velocities
  void FillTrajectory(
      apollo::planning::ADCTrajectory* trajectory,
      const std::vector<State>& states,
      const std::vector<double>& velocities);

  // Compute reference gamma from path curvature
  static std::vector<double> ComputeGammaRef(
      const std::vector<State>& states,
      double L, double gamma_max, double gamma_min);

  // ── Config ──
  CilqrPlannerConf conf_;
  Arg arg_;
  SystemModel system_model_;
  ArticulatedHybridAStarParams ha_params_;
  RunConfig run_config_;

  // ── Map & Plan ──
  MapData bitmap_map_;
  GlobalPlan global_plan_;
  std::unique_ptr<CILQRSolver> cilqr_solver_;

  // ── Trajectory stitching cache ──
  std::vector<State> prev_trj_states_;
  std::vector<double> prev_trj_vels_;

  // ── State ──
  std::mutex state_mutex_;
  State cur_state_{0.0, 0.0, 0.0, 0.0};
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_theta_ = 0.0;
  std::vector<ObstacleData> obs_trajectories_;

  bool odom_received_ = false;
  bool goal_received_ = false;
  bool plan_valid_ = false;
  bool static_obs_injected_ = false;

  double publish_rate_ = 10.0;
  double cur_velocity_ = 0.0;
  double rear_theta_ = 0.0;
  bool rear_odom_received_ = false;

  // ── Logging ──
  std::string log_path_;
  std::ofstream log_file_;
  std::string cilqr_log_path_;
  double last_ha_solve_ms_ = 0.0;  // Hybrid A* 最近一次求解时间

  // ── Cyber readers/writers ──
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<prediction::PredictionObstacles>>
      prediction_reader_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      rear_localization_reader_;
  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingResponse>> routing_writer_;
  std::shared_ptr<cyber::Writer<ADCTrajectory>> global_path_writer_;

  // Timer for periodic planning
  std::unique_ptr<cyber::Timer> timer_;
};

CYBER_REGISTER_COMPONENT(CilqrPlannerComponent)

}  // namespace cilqr
}  // namespace planning
}  // namespace apollo
