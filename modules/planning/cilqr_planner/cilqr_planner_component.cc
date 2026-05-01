#include "modules/planning/cilqr_planner/cilqr_planner_component.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "cyber/common/file.h"
#include "cyber/time/time.h"

namespace apollo {
namespace planning {
namespace cilqr {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::prediction::PredictionObstacles;

// ── helpers ─────────────────────────────────────────────────────────────────

std::vector<double> CilqrPlannerComponent::ComputeGammaRef(
    const std::vector<State>& states,
    double L, double gamma_max, double gamma_min) {
  size_t n = states.size();
  std::vector<double> gammas(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    double ax = states[i - 1][0], ay = states[i - 1][1];
    double bx = states[i][0], by = states[i][1];
    double cx = states[i + 1][0], cy = states[i + 1][1];
    double area = 0.5 * ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax));
    double ab = std::hypot(bx - ax, by - ay);
    double bc = std::hypot(cx - bx, cy - by);
    double ca = std::hypot(ax - cx, ay - cy);
    double denom = ab * bc * ca;
    if (denom < 1e-12) {
      gammas[i] = 0.0;
      continue;
    }
    double kappa = 4.0 * area / denom;
    double arg = std::clamp(L * kappa, -1.0, 1.0);
    gammas[i] = std::asin(arg);
  }
  if (n > 1) {
    gammas[0] = gammas[1];
    gammas[n - 1] = gammas[n - 2];
  }
  for (auto& g : gammas)
    g = std::clamp(g, gamma_min, gamma_max);
  return gammas;
}

// ── Init ─────────────────────────────────────────────────────────────────────

bool CilqrPlannerComponent::Init() {
  // Load protobuf config
  if (!GetProtoConfig(&conf_)) {
    AERROR << "Unable to load cilqr planner config: " << ConfigFilePath();
    return false;
  }

  AINFO << "CILQR Planner config loaded.";

  if (conf_.config_dir().empty() || conf_.map_file().empty()) {
    AERROR << "config_dir and map_file must be set in cilqr_planner_config.";
    return false;
  }

  publish_rate_ = conf_.publish_rate();

  // ── Load CILQR JSON config ──
  std::string cfg_dir = conf_.config_dir();
  load_config(cfg_dir + "/main.json", cfg_dir + "/ilqr.json",
              cfg_dir + "/hybrid_astar.json", cfg_dir + "/vehicle.json",
              arg_, system_model_, ha_params_, run_config_);

  // Override from protobuf config if explicitly set
  if (conf_.desire_speed() > 0.0) {
    arg_.desire_speed = conf_.desire_speed();
  }
  if (conf_.horizon() > 0) {
    arg_.N = conf_.horizon();
    system_model_.N = static_cast<size_t>(conf_.horizon());
  }

  AINFO << "CILQR config: N=" << arg_.N
        << ", desire_speed=" << arg_.desire_speed
        << ", dt=" << arg_.dt;
  AINFO << "Obstacle params: ego_rad_f=" << system_model_.ego_rad_f
        << ", ego_rad_r=" << system_model_.ego_rad_r
        << ", obs_q1=" << arg_.obs_q1
        << ", obs_q2=" << arg_.obs_q2
        << ", safe_a=" << arg_.safe_a_buffer
        << ", safe_b=" << arg_.safe_b_buffer
        << ", if_cal_obs=" << arg_.if_cal_obs_cost;

  // 车辆参数全部由 vehicle.json 驱动，不再硬编码
  system_model_.dt = arg_.dt;

  // Map physical and dynamic limits to articulated A* planner params
  ha_params_.L_f = system_model_.lf;
  ha_params_.L_r = system_model_.lr;
  ha_params_.W_f_body = system_model_.body_width_f;
  ha_params_.W_r_body = system_model_.body_width_r;
  ha_params_.L_f_body = system_model_.body_length_f;
  ha_params_.L_r_body = system_model_.body_length_r;
  ha_params_.gamma_max = arg_.gamma_max;
  ha_params_.gamma_dot_max = arg_.gamma_dot_max;
  ha_params_.v_desire = arg_.desire_speed;

  ArticulatedLimits lims =
      compute_articulated_limits(system_model_, arg_.gamma_max);
  AINFO << "Vehicle limits: kappa_max=" << lims.kappa_max
        << ", gamma_dot from JSON: [" << arg_.gamma_dot_min
        << ", " << arg_.gamma_dot_max << "]";

  // ── Load map ──
  AINFO << "Loading map: " << conf_.map_file();
  bitmap_map_ = load_bitmap_map(conf_.map_file());
  if (bitmap_map_.data.empty()) {
    AERROR << "Failed to load map file: " << conf_.map_file();
    return false;
  }
  AINFO << "Map loaded: " << bitmap_map_.width << "x" << bitmap_map_.height
        << ", res=" << bitmap_map_.resolution << " m/px";

  // ── Create readers ──
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      conf_.localization_topic(),
      [this](const std::shared_ptr<LocalizationEstimate>& msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        odom_received_ = true;
        cur_state_[0] = msg->pose().position().x();
        cur_state_[1] = msg->pose().position().y();
        cur_state_[2] = msg->pose().heading();
        // gamma (State[3]) updated by rear localization reader
      });

  planning_command_reader_ = node_->CreateReader<planning::PlanningCommand>(
      conf_.planning_command_topic(),
      [this](const std::shared_ptr<planning::PlanningCommand>& msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        // Extract goal from planning command
        // For now, use waypoint[0] from lane_follow_command if available
        if (msg->has_lane_follow_command() &&
            msg->lane_follow_command().routing_request().waypoint_size() > 0) {
          const auto& wp = msg->lane_follow_command()
                               .routing_request()
                               .waypoint(
                                   msg->lane_follow_command()
                                       .routing_request()
                                       .waypoint_size() - 1);
          // Clamp goal to map bounds
          double half_x = bitmap_map_.width * bitmap_map_.resolution / 2.0;
          double half_y = bitmap_map_.height * bitmap_map_.resolution / 2.0;
          double ox = bitmap_map_.origin.size() > 0
                          ? bitmap_map_.origin[0] : -half_x;
          double oy = bitmap_map_.origin.size() > 1
                          ? bitmap_map_.origin[1] : -half_y;
          double margin = 2.0 * bitmap_map_.resolution;
          goal_x_ = std::clamp(
              wp.pose().x(), ox + margin,
              ox + bitmap_map_.width * bitmap_map_.resolution - margin);
          goal_y_ = std::clamp(
              wp.pose().y(), oy + margin,
              oy + bitmap_map_.height * bitmap_map_.resolution - margin);
          if (wp.has_heading()) {
            goal_theta_ = wp.heading();
          }
          goal_received_ = true;
          plan_valid_ = false;  // force replanning
          AINFO << "New goal: (" << goal_x_ << ", " << goal_y_
                << ", " << goal_theta_ * 180.0 / M_PI << "°)";
        }
      });

  prediction_reader_ = node_->CreateReader<PredictionObstacles>(
      conf_.prediction_topic(),
      [this](const std::shared_ptr<PredictionObstacles>& msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        obs_trajectories_.clear();
        for (const auto& obstacle : msg->prediction_obstacle()) {
          if (obstacle.trajectory_size() == 0) continue;
          State obs_state;
          const auto& point =
              obstacle.trajectory(0).trajectory_point(0);
          obs_state[0] = point.path_point().x();
          obs_state[1] = point.path_point().y();
          obs_state[2] = point.path_point().theta();
          obs_state[3] = 0.0;  // no gamma for regular obstacles

          ObstacleData obs_data;
          obs_data.trj =
              predict_obstacle_trajectory(obs_state, arg_.dt, arg_.N);
          obs_data.length = obstacle.perception_obstacle().length();
          obs_data.width = obstacle.perception_obstacle().width();
          if (obs_data.length < 0.1) obs_data.length = 1.4;
          if (obs_data.width < 0.1) obs_data.width = 1.4;
          obs_trajectories_.push_back(obs_data);
        }
      });

  chassis_reader_ = node_->CreateReader<Chassis>(
      conf_.chassis_topic(),
      [this](const std::shared_ptr<Chassis>& msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        // 从电机转速读取车速，比 GNSS 速度更准确（尤其低速）
        cur_velocity_ = msg->speed_mps();
      });

  // 后车定位：读取后车 IMU 航向，计算铰接角 gamma
  rear_localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      "/apollo/localization/pose_rear",
      [this](const std::shared_ptr<LocalizationEstimate>& msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        rear_theta_ = msg->pose().heading();
        rear_odom_received_ = true;
        // gamma = 前车航向 - 后车航向，归一化到 [-π, π]
        double gamma = cur_state_[2] - rear_theta_;
        // 归一化
        while (gamma > M_PI) gamma -= 2 * M_PI;
        while (gamma < -M_PI) gamma += 2 * M_PI;
        cur_state_[3] = gamma;
      });

  // ── Create writers ──
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      conf_.planning_trajectory_topic());
  routing_writer_ = node_->CreateWriter<routing::RoutingResponse>(
      "/apollo/routing");
  global_path_writer_ = node_->CreateWriter<ADCTrajectory>(
      "/apollo/planning/global_path");

  // ── Create planning timer ──
  uint32_t period_ms = static_cast<uint32_t>(1000.0 / publish_rate_);
  timer_ = std::make_unique<cyber::Timer>(
      period_ms,
      [this]() { PlanAndPublish(); },
      false);  // not oneshot
  timer_->Start();

  // ── CSV log ──
  {
    std::filesystem::create_directories("/tmp/planner");
    auto t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    log_path_ = std::string("/tmp/planner/cilqr_planner_") + buf + ".csv";
    log_file_.open(log_path_);
    if (log_file_.is_open()) {
      log_file_
          << "timestamp_s,x,y,theta_deg,gamma_deg,goal_x,goal_y,dist_to_goal,"
             "J_total,converged,solve_ms,trj_len,v0_cmd,global_pts,"
             "ha_solve_ms\n";
      AINFO << "CSV log: " << log_path_;
    }
    cilqr_log_path_ =
        std::string("log/planner/cilqr_cost_analysis_") + buf + ".csv";
    AINFO << "CILQR cost log: " << cilqr_log_path_;
  }

  AINFO << "CilqrPlannerComponent ready. Waiting for planning command ...";
  return true;
}

CilqrPlannerComponent::~CilqrPlannerComponent() {
  if (timer_) {
    timer_->Stop();
  }
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

// ── Global planning (Hybrid A*) ─────────────────────────────────────────────

bool CilqrPlannerComponent::ReplanGlobal(
    double sx, double sy, double stheta, double sgamma) {
  std::array<double, 4> start = {sx, sy, stheta, sgamma};
  std::array<double, 4> goal = {goal_x_, goal_y_, goal_theta_, 0.0};
  std::vector<::Point> pts;

  auto t0 = std::chrono::steady_clock::now();
  bool ok = articulated_hybrid_astar_plan(
      bitmap_map_, start, goal, ha_params_, pts);
  auto t1 = std::chrono::steady_clock::now();
  double ha_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  last_ha_solve_ms_ = ha_ms;

  if (!ok || pts.empty()) {
    AWARN << "Hybrid A* failed (" << ha_ms << " ms), fallback to straight line.";
    pts.clear();
    double dx = goal_x_ - sx, dy = goal_y_ - sy;
    double dist = std::hypot(dx, dy);
    double hdg = std::atan2(dy, dx);
    int n = std::max(10, static_cast<int>(dist / 0.5));
    for (int i = 0; i <= n; ++i) {
      double t = static_cast<double>(i) / n;
      pts.emplace_back(::Point(sx + t * dx, sy + t * dy, hdg));
    }
  }
  global_plan_.set_plan(pts);
  AINFO << "Global plan: " << pts.size() << " waypoints, Hybrid A* solve: "
        << ha_ms << " ms";

  // Publish global path as RoutingResponse for Dreamview+ visualization
  if (routing_writer_ != nullptr) {
    auto routing_msg = std::make_shared<routing::RoutingResponse>();
    routing_msg->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());
    routing_msg->mutable_header()->set_module_name("cilqr_planner");
    // Add all waypoints as a single road segment
    auto* road = routing_msg->add_road();
    auto* passage = road->add_passage();
    for (size_t i = 0; i < pts.size(); ++i) {
      auto* segment = passage->add_segment();
      segment->set_id(std::to_string(i));
      segment->set_start_s(0.0);
      segment->set_end_s(1.0);
    }
    // Store actual path geometry in measurement for visualization
    auto* measurement = routing_msg->mutable_measurement();
    measurement->set_distance(0.0);
    double total_dist = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      double dx = pts[i].x - pts[i-1].x;
      double dy = pts[i].y - pts[i-1].y;
      total_dist += std::sqrt(dx*dx + dy*dy);
    }
    measurement->set_distance(total_dist);
    routing_writer_->Write(routing_msg);
    AINFO << "Published global path to /apollo/routing: "
          << pts.size() << " pts, " << total_dist << "m";
  }

  if (global_path_writer_ != nullptr) {
    ADCTrajectory global_traj;
    global_traj.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
    global_traj.mutable_header()->set_module_name("cilqr_global_planner");
    for (size_t i = 0; i < pts.size(); ++i) {
      auto* tp = global_traj.add_trajectory_point();
      auto* pp = tp->mutable_path_point();
      pp->set_x(pts[i].x);
      pp->set_y(pts[i].y);
      pp->set_theta(pts[i].heading);
      tp->set_v(0.0);
    }
    global_path_writer_->Write(global_traj);
    AINFO << "Published global path to /apollo/planning/global_path: " << pts.size() << " pts";
  }

  return true;
}

// ── CILQR local planning + publish ──────────────────────────────────────────

void CilqrPlannerComponent::PlanAndPublish() {
  State cur_state;
  double gx, gy, gtheta;
  (void)gtheta;  // used only in logging path
  std::vector<ObstacleData> obs_copy;
  bool have_odom, have_goal, plan_ok;

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    cur_state = cur_state_;
    gx = goal_x_;
    gy = goal_y_;
    gtheta = goal_theta_;
    obs_copy = obs_trajectories_;
    have_odom = odom_received_;
    have_goal = goal_received_;
    plan_ok = plan_valid_;
  }

  if (!have_odom) {
    ADEBUG << "No localization yet.";
    return;
  }
  if (!have_goal) {
    ADEBUG << "No goal yet.";
    return;
  }

  // ── Global replan if needed ──
  if (!plan_ok) {
    if (!ReplanGlobal(cur_state[0], cur_state[1],
                      cur_state[2], cur_state[3]))
      return;
    Vehicle ego;
    ego.set_state(cur_state);
    ego.set_global_plan(global_plan_);
    ego.set_model(system_model_);
    cilqr_solver_ = std::make_unique<CILQRSolver>(
        ego, obs_copy, arg_, "atv_terrain", "cilqr", cilqr_log_path_);
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      plan_valid_ = true;
    }
    prev_trj_states_.clear();
    prev_trj_vels_.clear();
  }

  // ── Check goal reached ──
  double dist_to_goal = std::hypot(gx - cur_state[0], gy - cur_state[1]);
  if (dist_to_goal < arg_.goal_reached_dist) {
    AINFO << "Goal reached (dist=" << dist_to_goal << " m). Holding position.";
    PublishIdle(cur_state);
    prev_trj_states_.clear();
    prev_trj_vels_.clear();
    return;
  }

  // ── Trajectory stitching ──
  constexpr int stitch_ahead_min = 1;
  int stitch_ahead = std::max(
      stitch_ahead_min,
      static_cast<int>(std::round(1.0 / (publish_rate_ * arg_.dt))));
  State plan_init_state = cur_state;
  std::vector<State> stitch_prefix;
  std::vector<double> stitch_prefix_vels;
  int match_idx = -1;

  if (!prev_trj_states_.empty()) {
    double min_dist_sq = 1e18;
    for (size_t i = 0; i < prev_trj_states_.size(); ++i) {
      double dx = prev_trj_states_[i][0] - cur_state[0];
      double dy = prev_trj_states_[i][1] - cur_state[1];
      double d2 = dx * dx + dy * dy;
      if (d2 < min_dist_sq) {
        min_dist_sq = d2;
        match_idx = static_cast<int>(i);
      }
    }

    if (std::sqrt(min_dist_sq) < arg_.stitch_dist_threshold &&
        match_idx >= 0) {
      int stitch_idx = std::min(
          match_idx + stitch_ahead,
          static_cast<int>(prev_trj_states_.size()) - 1);
      plan_init_state = prev_trj_states_[stitch_idx];

      for (int k = match_idx; k <= stitch_idx; ++k) {
        stitch_prefix.push_back(prev_trj_states_[k]);
        if (k < static_cast<int>(prev_trj_vels_.size()))
          stitch_prefix_vels.push_back(prev_trj_vels_[k]);
        else
          stitch_prefix_vels.push_back(0.3);
      }

      ADEBUG << "Trajectory stitching: match=" << match_idx
             << " stitch=" << stitch_idx
             << " dist=" << std::sqrt(min_dist_sq) << "m";
    }
  }

  // ── CILQR solve ──
  if (!cilqr_solver_) return;

  Solution solution;
  try {
    cilqr_solver_->set_ros_time(cyber::Time::Now().ToSecond());
    cilqr_solver_->set_initial_velocity(cur_velocity_);
    solution = cilqr_solver_->solve(plan_init_state, obs_copy);
  } catch (const std::exception& e) {
    AWARN << "CILQR exception: " << e.what();
    return;
  }

  if (solution.ego_trj.states.empty()) {
    AWARN << "CILQR returned empty trajectory.";
    return;
  }

  // ── Merge stitched trajectory ──
  auto& trj_states = solution.ego_trj.states;
  std::vector<State> merged_states;
  std::vector<double> merged_vels;

  if (!stitch_prefix.empty()) {
    for (size_t k = 0; k + 1 < stitch_prefix.size(); ++k) {
      merged_states.push_back(stitch_prefix[k]);
      merged_vels.push_back(stitch_prefix_vels[k]);
    }
  }
  for (size_t k = 0; k < trj_states.size(); ++k) {
    merged_states.push_back(trj_states[k]);
  }

  prev_trj_states_ = merged_states;

  // Build velocity array from control sequence
  const auto& ctrl_seq = solution.control_sequence.get_control_sequence();
  std::vector<double> vel_data;
  if (!stitch_prefix.empty()) {
    for (size_t k = 0; k + 1 < stitch_prefix_vels.size(); ++k) {
      vel_data.push_back(stitch_prefix_vels[k]);
    }
  }
  for (const auto& u : ctrl_seq) {
    vel_data.push_back(u[0]);  // U[0] = linear speed
  }
  while (vel_data.size() < merged_states.size()) {
    vel_data.push_back(vel_data.empty() ? 0.0 : vel_data.back());
  }

  prev_trj_vels_.clear();
  for (auto v : vel_data) prev_trj_vels_.push_back(v);

  // ── Publish ADCTrajectory ──
  ADCTrajectory adc_trajectory;
  auto* header = adc_trajectory.mutable_header();
  header->set_timestamp_sec(cyber::Time::Now().ToSecond());
  header->set_module_name("cilqr_planner");

  FillTrajectory(&adc_trajectory, merged_states, vel_data);
  planning_writer_->Write(adc_trajectory);

  ADEBUG << "Published trajectory: " << trj_states.size()
         << " waypoints, v0=" << (ctrl_seq.empty() ? 0.0 : ctrl_seq[0][0])
         << " m/s, solve=" << solution.solve_time_ms << " ms";

  // ── CSV log entry ──
  if (log_file_.is_open()) {
    double now_s = cyber::Time::Now().ToSecond();
    double v0 = ctrl_seq.empty() ? 0.0 : ctrl_seq[0][0];
    size_t gpts = global_plan_.get_points().size();
    log_file_ << std::fixed << std::setprecision(4) << now_s << ","
              << cur_state[0] << "," << cur_state[1] << ","
              << (cur_state[2] * 180.0 / M_PI) << ","
              << (cur_state[3] * 180.0 / M_PI) << "," << gx << "," << gy
              << "," << dist_to_goal << "," << solution.final_cost << ","
              << (solution.converged ? 1 : 0) << ","
              << solution.solve_time_ms << "," << trj_states.size()
              << "," << v0 << "," << gpts << "," << last_ha_solve_ms_
              << "\n";
    log_file_.flush();
  }
}

void CilqrPlannerComponent::PublishIdle(const State& s) {
  ADCTrajectory adc_trajectory;
  auto* header = adc_trajectory.mutable_header();
  header->set_timestamp_sec(cyber::Time::Now().ToSecond());
  header->set_module_name("cilqr_planner");

  auto* tp = adc_trajectory.add_trajectory_point();
  auto* pp = tp->mutable_path_point();
  pp->set_x(s[0]);
  pp->set_y(s[1]);
  pp->set_theta(s[2]);
  pp->set_kappa(s[3]);  // gamma stored in kappa field
  tp->set_v(0.0);
  tp->set_relative_time(0.0);

  planning_writer_->Write(adc_trajectory);
}

void CilqrPlannerComponent::FillTrajectory(
    ADCTrajectory* trajectory,
    const std::vector<State>& states,
    const std::vector<double>& velocities) {
  for (size_t i = 0; i < states.size(); ++i) {
    auto* tp = trajectory->add_trajectory_point();
    auto* pp = tp->mutable_path_point();
    pp->set_x(states[i][0]);
    pp->set_y(states[i][1]);
    pp->set_theta(states[i][2]);
    // Store gamma in kappa field (articulated vehicle specific)
    pp->set_kappa(std::clamp(states[i][3], arg_.gamma_min, arg_.gamma_max));
    tp->set_v(i < velocities.size() ? velocities[i] : 0.0);
    tp->set_relative_time(i * arg_.dt);
  }
}

}  // namespace cilqr
}  // namespace planning
}  // namespace apollo
