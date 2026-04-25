/**
 * planner_node.cpp
 *
 * ROS2 node that wraps the Cilqr_cpp-CAV (Hybrid A* + CILQR) planner.
 *
 * Subscriptions:
 *   /odom                  (nav_msgs/Odometry)   — current vehicle state
 *   /goal_pose             (geometry_msgs/PoseStamped) — planning goal
 * (triggers replanning) /obstacles             (visualization_msgs/MarkerArray)
 * — dynamic obstacles
 *
 * Publications:
 *   /atv/reference_trajectory   (nav_msgs/Path)
 *   /atv/reference_velocities   (std_msgs/Float64MultiArray)
 *
 * The node maintains a global plan (Hybrid A*) and re-runs CILQR each timer
 * tick to generate a locally-optimised trajectory centred on the current state.
 */

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Cilqr library headers
#include "common_types.h"
#include "config_loader.h"
#include "articulated_hybrid_astar.h"
#include "ilqr.h"
#include "utils.h"

using namespace std::chrono_literals;

// ── helpers ──────────────────────────────────────────────────────────────────

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  return q;
}

static double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// gamma clamped to config limits (applied after compute_gamma_ref)
static std::vector<double> compute_gamma_ref(const std::vector<State> &states,
                                             double L, double gamma_max,
                                             double gamma_min) {
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
  for (auto &g : gammas)
    g = std::clamp(g, gamma_min, gamma_max);
  return gammas;
}

// ── Node ─────────────────────────────────────────────────────────────────────

class CilqrPlannerNode : public rclcpp::Node {
public:
  CilqrPlannerNode() : Node("cilqr_planner") {
    // ── parameters ──
    declare_parameter("config_dir", "");
    declare_parameter("map_file", "");
    declare_parameter("publish_rate", 10.0); // Hz
    declare_parameter("desire_speed", -1.0); // -1 = use JSON value
    declare_parameter("horizon", -1);        // -1 = use JSON value

    auto cfg_dir = get_parameter("config_dir").as_string();
    auto map_file = get_parameter("map_file").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    double speed = get_parameter("desire_speed").as_double();
    int horiz = get_parameter("horizon").as_int();

    if (cfg_dir.empty() || map_file.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "Parameters 'config_dir' and 'map_file' must be set.");
      throw std::runtime_error("missing parameters");
    }

    // ── load Cilqr config ──
    load_config(cfg_dir + "/main.json", cfg_dir + "/ilqr.json",
                cfg_dir + "/hybrid_astar.json", cfg_dir + "/vehicle.json",
                arg_, system_model_, ha_params_, run_config_);

    // JSON config is the primary source for desire_speed and N.
    // ROS parameters only override if explicitly set via launch file.
    // (The defaults declared above match the JSON-typical values.)
    if (speed > 0.0) {
      // Launch param was explicitly set — use it
      arg_.desire_speed = speed;
    }
    // else: keep the value from ilqr.json

    if (horiz > 0) {
      arg_.N = horiz;
      system_model_.N = static_cast<size_t>(horiz);
    }

    RCLCPP_INFO(get_logger(), "CILQR config: N=%d, desire_speed=%.2f, dt=%.3f",
                arg_.N, arg_.desire_speed, arg_.dt);
    RCLCPP_INFO(
        get_logger(),
        "Obstacle params: ego_rad=%.3f, "
        "obs_q1=%.3f, obs_q2=%.3f, safe_a=%.3f, safe_b=%.3f, if_cal_obs=%d",
        system_model_.ego_rad, arg_.obs_q1,
        arg_.obs_q2, arg_.safe_a_buffer, arg_.safe_b_buffer,
        arg_.if_cal_obs_cost);

    // Rescale ATV physical parameters
    system_model_.lf = 0.77;
    system_model_.lr = 0.77;
    system_model_.len = 0.90;
    system_model_.width = 0.40;
    system_model_.dt = arg_.dt;

    // Map physical and dynamic limits to articulated A* planner params
    ha_params_.L_f = system_model_.lf;
    ha_params_.L_r = system_model_.lr;
    ha_params_.W_body = system_model_.width;
    ha_params_.L_f_body = 0.77;
    ha_params_.L_r_body = 0.77;
    ha_params_.gamma_max = arg_.gamma_max;
    ha_params_.gamma_dot_max = arg_.gamma_dot_max;
    ha_params_.v_desire = arg_.desire_speed;

    // Derived limits (informational only — gamma_dot_max/min come from JSON)
    ArticulatedLimits lims =
        compute_articulated_limits(system_model_, arg_.gamma_max);
    RCLCPP_INFO(
        get_logger(),
        "Vehicle limits: kappa_max=%.3f, gamma_dot from JSON: [%.2f, %.2f]",
        lims.kappa_max, arg_.gamma_dot_min, arg_.gamma_dot_max);

    // ── load map ──
    RCLCPP_INFO(get_logger(), "Loading map: %s", map_file.c_str());
    bitmap_map_ = load_bitmap_map(map_file);
    if (bitmap_map_.data.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load map file: %s",
                   map_file.c_str());
      throw std::runtime_error("map load failed");
    }
    RCLCPP_INFO(get_logger(), "Map loaded: %dx%d, res=%.3f m/px",
                bitmap_map_.width, bitmap_map_.height, bitmap_map_.resolution);

    // ── publishers ──
    pub_path_ =
        create_publisher<nav_msgs::msg::Path>("/atv/reference_trajectory", 10);
    pub_vel_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/atv/reference_velocities", 10);
    pub_global_path_ = create_publisher<nav_msgs::msg::Path>(
        "/planner/global_path", 10); // Hybrid A* global path for visualization
    pub_local_plan_ = create_publisher<nav_msgs::msg::Path>(
        "/planner/local_plan",
        10); // CILQR local reference path for visualization

    // ── subscribers ──
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          odom_received_ = true;
          cur_state_[0] = msg->pose.pose.position.x;
          cur_state_[1] = msg->pose.pose.position.y;
          cur_state_[2] = quat_to_yaw(msg->pose.pose.orientation);
          // State[3] = gamma (articulation angle).
          // Updated separately by sub_joint_state_ from /joint_states.
          cur_velocity_ = msg->twist.twist.linear.x;
        });

    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          // Clamp goal to map bounds to avoid Hybrid A* out-of-bounds
          double half_x = bitmap_map_.width * bitmap_map_.resolution / 2.0;
          double half_y = bitmap_map_.height * bitmap_map_.resolution / 2.0;
          double ox =
              bitmap_map_.origin.size() > 0 ? bitmap_map_.origin[0] : -half_x;
          double oy =
              bitmap_map_.origin.size() > 1 ? bitmap_map_.origin[1] : -half_y;
          double margin = 2.0 * bitmap_map_.resolution; // 2-pixel margin
          goal_x_ = std::clamp(msg->pose.position.x, ox + margin,
                               ox + bitmap_map_.width * bitmap_map_.resolution -
                                   margin);
          goal_y_ = std::clamp(
              msg->pose.position.y, oy + margin,
              oy + bitmap_map_.height * bitmap_map_.resolution - margin);
          goal_theta_ = quat_to_yaw(msg->pose.orientation);
          goal_received_ = true;
          plan_valid_ = false; // force replanning
          RCLCPP_INFO(get_logger(),
                      "New goal: (%.2f, %.2f, %.1f°) [clamped from (%.2f, "
                      "%.2f)]  frame=%s  quat=[w=%.4f x=%.4f y=%.4f z=%.4f]",
                      goal_x_, goal_y_, goal_theta_ * 180.0 / M_PI,
                      msg->pose.position.x, msg->pose.position.y,
                      msg->header.frame_id.c_str(), msg->pose.orientation.w,
                      msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z);
        });

    sub_obs_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/obstacles", 10,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          obs_trajectories_.clear();
          for (const auto &marker : msg->markers) {
            if (marker.action != visualization_msgs::msg::Marker::ADD)
              continue;
            // Skip corridor wall markers (visualization only)
            if (marker.ns == "corridor_walls")
              continue;
            State obs_state;
            obs_state[0] = marker.pose.position.x;
            obs_state[1] = marker.pose.position.y;
            obs_state[2] = quat_to_yaw(marker.pose.orientation);
            try {
              obs_state[3] = marker.text.empty() ? 0.0 : std::stod(marker.text);
            } catch (...) {
              obs_state[3] = 0.0;
            }
            ObstacleData obs_data;
            obs_data.trj =
                predict_obstacle_trajectory(obs_state, arg_.dt, arg_.N);
            obs_data.length = marker.scale.x > 0.1 ? marker.scale.x : 1.4;
            obs_data.width = marker.scale.y > 0.1 ? marker.scale.y : 1.4;
            obs_trajectories_.push_back(obs_data);
          }

          // ── Inject static obstacles into bitmap_map_ (once) ──
          if (!static_obs_injected_ && !bitmap_map_.data.empty()) {
            const double buffer = 0.5; // safety margin [m]
            int count = 0;
            for (const auto &marker : msg->markers) {
              if (marker.action != visualization_msgs::msg::Marker::ADD)
                continue;
              // Skip corridor wall markers (they are TRIANGLE_LIST, not obstacles)
              if (marker.ns == "corridor_walls")
                continue;
              // Static obstacle: speed ≈ 0
              double speed = 0.0;
              try {
                speed = marker.text.empty() ? 0.0 : std::stod(marker.text);
              } catch (...) { speed = 0.0; }
              if (std::abs(speed) > 0.01) continue; // skip dynamic

              double cx = marker.pose.position.x;
              double cy = marker.pose.position.y;
              double yaw = quat_to_yaw(marker.pose.orientation);
              double half_l = (marker.scale.x > 0.1 ? marker.scale.x : 1.4) / 2.0 + buffer;
              double half_w = (marker.scale.y > 0.1 ? marker.scale.y : 1.4) / 2.0 + buffer;

              // Stamp rotated box into grid
              double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
              double ox = bitmap_map_.origin[0];
              double oy = bitmap_map_.origin[1];
              double res = bitmap_map_.resolution;
              // Scan bounding circle
              double scan_r = std::hypot(half_l, half_w);
              int r_pix = static_cast<int>(std::ceil(scan_r / res)) + 1;
              int cx_pix = static_cast<int>(std::floor((cx - ox) / res));
              int cy_pix = static_cast<int>(std::floor((cy - oy) / res));
              for (int dy = -r_pix; dy <= r_pix; ++dy) {
                for (int dx = -r_pix; dx <= r_pix; ++dx) {
                  int col = cx_pix + dx;
                  int row = cy_pix + dy;
                  if (row < 0 || row >= bitmap_map_.height ||
                      col < 0 || col >= bitmap_map_.width)
                    continue;
                  // World coords of cell center
                  double wx = ox + (col + 0.5) * res;
                  double wy = oy + (row + 0.5) * res;
                  // Transform to obstacle-local frame
                  double lx =  cos_y * (wx - cx) + sin_y * (wy - cy);
                  double ly = -sin_y * (wx - cx) + cos_y * (wy - cy);
                  if (std::abs(lx) <= half_l && std::abs(ly) <= half_w) {
                    bitmap_map_.data[row][col] = -1.0; // occupied
                  }
                }
              }
              ++count;
            }
            static_obs_injected_ = true;
            if (count > 0) {
              RCLCPP_INFO(get_logger(),
                  "Injected %d static obstacles into bitmap (buf=%.1fm)", count, buffer);
            }
          }
        });

    // ── joint state subscriber: update gamma from steering_joint ──
    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "steering_joint") {
              std::lock_guard<std::mutex> lk(state_mutex_);
              // SDF: parent=front_base_link, child=rear_base_link,
              // axis=(0,0,-1) joint_position = theta_front - theta_rear = gamma
              cur_state_[3] = msg->position[i];
              break;
            }
          }
        });

    // ── planning timer ──
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CilqrPlannerNode::plan_and_publish, this));

    // ── CSV log ──
    {
      std::filesystem::create_directories("log/planner");
      auto t = std::time(nullptr);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
      log_path_ = std::string("log/planner/cilqr_planner_") + buf + ".csv";
      log_file_.open(log_path_);
      if (log_file_.is_open()) {
        log_file_
            << "timestamp_s,x,y,theta_deg,gamma_deg,goal_x,goal_y,dist_to_goal,"
               "J_total,converged,solve_ms,trj_len,v0_cmd,global_pts\n";
        RCLCPP_INFO(get_logger(), "CSV log: %s", log_path_.c_str());
      }
      // 一次性生成 CILQR 代价日志路径（整个仿真共用一个文件，追加写入）
      cilqr_log_path_ =
          std::string("log/planner/cilqr_cost_analysis_") + buf + ".csv";
      RCLCPP_INFO(get_logger(), "CILQR cost log: %s", cilqr_log_path_.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "CilqrPlannerNode ready. Waiting for /goal_pose ...");
  }

private:
  // ── Global planning (Hybrid A*) ──
  bool replan_global(double sx, double sy, double stheta, double sgamma) {
    std::array<double,4> start = {sx, sy, stheta, sgamma};
    std::array<double,4> goal = {goal_x_, goal_y_, goal_theta_, 0.0};
    std::vector<Point> pts;
    bool ok = articulated_hybrid_astar_plan(bitmap_map_, start, goal, ha_params_, pts);
    if (!ok || pts.empty()) {
      RCLCPP_WARN(get_logger(), "Hybrid A* failed, fallback to straight line.");
      // Fallback: insert straight line waypoints
      pts.clear();
      double dx = goal_x_ - sx, dy = goal_y_ - sy;
      double dist = std::hypot(dx, dy);
      double hdg = std::atan2(dy, dx);
      int n = std::max(10, static_cast<int>(dist / 0.5));
      for (int i = 0; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        pts.emplace_back(sx + t * dx, sy + t * dy, hdg);
      }
    }
    global_plan_.set_plan(pts);
    RCLCPP_INFO(get_logger(), "Global plan: %zu waypoints", pts.size());

    // Publish global path for visualization
    nav_msgs::msg::Path global_msg;
    global_msg.header.stamp = get_clock()->now();
    global_msg.header.frame_id = "world";
    for (const auto &p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = global_msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = yaw_to_quat(p.heading);
      global_msg.poses.push_back(ps);
    }
    pub_global_path_->publish(global_msg);

    return true;
  }

  // ── CILQR local planning + publish (with trajectory stitching) ──
  void plan_and_publish() {
    State cur_state;
    double gx, gy, gtheta;
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
      RCLCPP_DEBUG(get_logger(), "No odometry yet.");
      return;
    }
    if (!have_goal) {
      RCLCPP_DEBUG(get_logger(), "No goal yet.");
      return;
    }

    // ── Global replan if needed ──
    if (!plan_ok) {
      if (!replan_global(cur_state[0], cur_state[1], cur_state[2], cur_state[3]))
        return;
      // Reset CILQR solver with new global plan
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
      // 新的全局路径 → 清空上一次轨迹缓存
      prev_trj_states_.clear();
      prev_trj_vels_.clear();
    }

    // ── Check goal reached ──
    double dist_to_goal = std::hypot(gx - cur_state[0], gy - cur_state[1]);
    if (dist_to_goal < arg_.goal_reached_dist) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Goal reached (dist=%.2f m). Holding position.",
                           dist_to_goal);
      publish_idle(cur_state);
      prev_trj_states_.clear();
      prev_trj_vels_.clear();
      return;
    }

    // ── 轨迹拼接: 确定 CILQR 初始状态 ──
    // 如果有上一次轨迹，找到车辆在上一次轨迹上的匹配点,
    // 取匹配点前方 stitch_ahead 步的状态作为 CILQR 的初始状态,
    // 并保留匹配点到拼接点之间的前缀段。
    constexpr int stitch_ahead_min = 1;
    int stitch_ahead = std::max(
        stitch_ahead_min,
        (int)std::round(1.0 / (publish_rate_ * arg_.dt))); // 一个规划周期的步数
    State plan_init_state = cur_state;      // 默认: 从车辆当前状态规划
    std::vector<State> stitch_prefix;       // 拼接前缀
    std::vector<double> stitch_prefix_vels; // 前缀速度
    int match_idx = -1;

    if (!prev_trj_states_.empty()) {
      // 在上一次轨迹上找到距车辆最近的点 (match point)
      double min_dist_sq = 1e18;
      for (size_t i = 0; i < prev_trj_states_.size(); ++i) {
        double dx = prev_trj_states_[i][0] - cur_state[0];
        double dy = prev_trj_states_[i][1] - cur_state[1];
        double d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq) {
          min_dist_sq = d2;
          match_idx = (int)i;
        }
      }

      // 如果匹配距离 < stitch_dist_threshold (m, ilqr.json 可配)，使用轨迹拼接
      if (std::sqrt(min_dist_sq) < arg_.stitch_dist_threshold &&
          match_idx >= 0) {
        int stitch_idx = std::min(match_idx + stitch_ahead,
                                  (int)prev_trj_states_.size() - 1);
        plan_init_state = prev_trj_states_[stitch_idx];

        // 收集前缀段: match_idx .. stitch_idx
        for (int k = match_idx; k <= stitch_idx; ++k) {
          stitch_prefix.push_back(prev_trj_states_[k]);
          if (k < (int)prev_trj_vels_.size())
            stitch_prefix_vels.push_back(prev_trj_vels_[k]);
          else
            stitch_prefix_vels.push_back(0.3);
        }

        RCLCPP_DEBUG(get_logger(),
                     "Trajectory stitching: match=%d stitch=%d dist=%.2fm",
                     match_idx, stitch_idx, std::sqrt(min_dist_sq));
      }
    }

    // ── CILQR solve ──
    if (!cilqr_solver_)
      return;

    Solution solution;
    try {
      // 传入 ROS 仿真时间，使 cost_analysis 日志时间戳与 cilqr_planner / mpc
      // 对齐
      cilqr_solver_->set_ros_time(get_clock()->now().seconds());
      solution = cilqr_solver_->solve(plan_init_state, obs_copy);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "CILQR exception: %s", e.what());
      return;
    }

    if (solution.ego_trj.states.empty()) {
      RCLCPP_WARN(get_logger(), "CILQR returned empty trajectory.");
      return;
    }

    // ── Build and publish Path ──
    auto &trj_states = solution.ego_trj.states;

    // ── 拼接: 将前缀段 + CILQR新轨迹合并 ──
    std::vector<State> merged_states;
    std::vector<double> merged_vels;

    // 添加前缀段 (跳过最后一个点, 避免与 CILQR 的第一个点重复)
    if (!stitch_prefix.empty()) {
      for (size_t k = 0; k + 1 < stitch_prefix.size(); ++k) {
        merged_states.push_back(stitch_prefix[k]);
        merged_vels.push_back(stitch_prefix_vels[k]);
      }
    }
    // 添加 CILQR 新轨迹
    for (size_t k = 0; k < trj_states.size(); ++k) {
      merged_states.push_back(trj_states[k]);
    }

    // 缓存本次轨迹供下一次拼接使用
    prev_trj_states_ = merged_states;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = "world";

    for (size_t i = 0; i < merged_states.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = merged_states[i][0];
      ps.pose.position.y = merged_states[i][1];
      // Use CILQR-optimised gamma directly (State[3]), not curvature
      // approximation
      ps.pose.position.z =
          std::clamp(merged_states[i][3], arg_.gamma_min, arg_.gamma_max);
      ps.pose.orientation = yaw_to_quat(merged_states[i][2]);
      path_msg.poses.push_back(ps);
    }

    pub_path_->publish(path_msg);

    // ── Publish local plan for visualization ──
    {
      auto local_pts = cilqr_solver_->get_local_plan_points();
      nav_msgs::msg::Path local_msg;
      local_msg.header.stamp = get_clock()->now();
      local_msg.header.frame_id = "world";
      for (const auto &p : local_pts) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = local_msg.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = yaw_to_quat(p.heading);
        local_msg.poses.push_back(ps);
      }
      pub_local_plan_->publish(local_msg);
    }

    // Publish reference velocities (prefix vels + CILQR control sequence)
    std_msgs::msg::Float64MultiArray vel_msg;
    // 前缀段速度
    if (!stitch_prefix.empty()) {
      for (size_t k = 0; k + 1 < stitch_prefix_vels.size(); ++k) {
        vel_msg.data.push_back(stitch_prefix_vels[k]);
      }
    }
    // CILQR 控制序列速度
    const auto &ctrl_seq = solution.control_sequence.get_control_sequence();
    for (const auto &u : ctrl_seq) {
      vel_msg.data.push_back(u[0]); // U[0] = linear speed
    }
    // Pad to match path length if shorter
    while (vel_msg.data.size() < merged_states.size())
      vel_msg.data.push_back(vel_msg.data.empty() ? 0.0 : vel_msg.data.back());

    // 缓存速度供下一次拼接使用
    prev_trj_vels_.clear();
    for (auto v : vel_msg.data)
      prev_trj_vels_.push_back(v);

    pub_vel_->publish(vel_msg);

    RCLCPP_DEBUG(
        get_logger(),
        "Published trajectory: %zu waypoints, v0=%.3f m/s, solve=%.1f ms",
        trj_states.size(), ctrl_seq.empty() ? 0.0 : ctrl_seq[0][0],
        solution.solve_time_ms);

    // ── CSV log entry ──
    if (log_file_.is_open()) {
      double now_s = get_clock()->now().seconds();
      double v0 = ctrl_seq.empty() ? 0.0 : ctrl_seq[0][0];
      size_t gpts = global_plan_.get_points().size();
      log_file_ << std::fixed << std::setprecision(4) << now_s << ","
                << cur_state[0] << "," << cur_state[1] << ","
                << (cur_state[2] * 180.0 / M_PI) << ","
                << (cur_state[3] * 180.0 / M_PI) << "," << gx << "," << gy
                << "," << dist_to_goal << "," << solution.final_cost << ","
                << (solution.converged ? 1 : 0) << "," << solution.solve_time_ms
                << "," << trj_states.size() << "," << v0 << "," << gpts << "\n";
      log_file_.flush();
    }
  }

  void publish_idle(const State &s) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = "world";

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x = s[0];
    ps.pose.position.y = s[1];
    ps.pose.position.z = 0.0;
    ps.pose.orientation = yaw_to_quat(s[2]);
    path_msg.poses.push_back(ps);

    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data.push_back(0.0);

    pub_path_->publish(path_msg);
    pub_vel_->publish(vel_msg);
  }

  // ── members ──
  Arg arg_;
  SystemModel system_model_;
  ArticulatedHybridAStarParams ha_params_;
  RunConfig run_config_;
  MapData bitmap_map_;
  GlobalPlan global_plan_;

  std::unique_ptr<CILQRSolver> cilqr_solver_;

  // 轨迹拼接: 缓存上一次的轨迹状态和速度
  std::vector<State> prev_trj_states_;
  std::vector<double> prev_trj_vels_;

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

  // ── logging ──
  std::string log_path_;
  std::ofstream log_file_;
  std::string cilqr_log_path_; // CILQR 代价日志路径（整个仿真共用）

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_plan_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      sub_obs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CilqrPlannerNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("planner"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
