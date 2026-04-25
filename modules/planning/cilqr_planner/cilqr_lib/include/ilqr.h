#ifndef ILQR_H
#define ILQR_H

#include "common_types.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace Eigen;
// #define M_PI 3.1415

struct Arg {
  // 仿真参数
  double tf = 1000;
  double dt = 0.1;
  // CILQR参数
  int N = 50; // Horizen
  double tol = 1;
  double rel_tol = 1e-5;
  int max_iter = 50;
  double lamb_init = 7.0; // 优化：从10降到7，减少lambda调节次数
  double lamb_factor = 2;
  double lamb_max = 6000;
  // 纯跟踪参数
  double kv = 0.3; // 前视距离系数
  double kp = 0.8; // 速度P控制器系数
  double ld0 = 3;  // 基础前瞻距离
  double ld_min = 3;
  double ld_max = 20;
  // 代价参数
  double desire_speed = 15;
  double v_max = 5.0;  // 速度上限 (m/s)，用于所有 clamp
  double v_min = -1.0; // 速度下限 (m/s)
  double desire_heading = 0;
  bool if_cal_obs_cost = true;
  bool if_cal_lane_cost = true;
  bool if_cal_speed_rate_cost = true;
  double v_rate_weight = 5.0; // 优化：从1.0增加到5.0，大幅增加速度平滑性

  // 铰接角gamma约束参数
  bool if_cal_gamma_barrier = true; // 对状态gamma施加上下限barrier
  double gamma_max = 1.0;           // 铰接角上限
  double gamma_max_q1 = 1.0;        // 铰接角上限barrier权重
  double gamma_max_q2 = 3.0;        // 铰接角上限barrier曲率
  double gamma_min = -1.0;          // 铰接角下限
  double gamma_min_q1 = 1.0;        // 铰接角下限barrier权重
  double gamma_min_q2 = 3.0;        // 铰接角下限barrier曲率

  // 铰接角速度gamma_dot约束参数
  bool if_cal_gamma_dot_barrier = true; // 对控制gamma_dot施加上下限barrier
  double gamma_dot_max = 0.3;     // 优化：从0.6降到0.3，大幅收紧约束以消除震荡
  double gamma_dot_max_q1 = 10.0; // 优化：从1.0增加到10.0，大幅增加barrier权重
  double gamma_dot_max_q2 = 2.0;  // 优化：保持2.0
  double gamma_dot_min = -0.3;    // 优化：从-0.6调整到-0.3
  double gamma_dot_min_q1 = 10.0; // 优化：从1.0增加到10.0
  double gamma_dot_min_q2 = 2.0;  // 优化：保持2.0

  // 加速度约束 (v_k - v_{k-1})/dt
  double acc_max = 3.0;  // 最大加速度 m/s^2 (relaxed)
  double acc_min = -4.0; // 最小加速度 m/s^2 (relaxed)
  double acc_q1 = 1.0;   // 加速度barrier权重
  double acc_q2 = 2.0;   // 加速度barrier曲率

  // 道路约束
  double trace_safe_width_left = 5;
  double trace_safe_width_right = 5;
  double lane_q1 = 5;
  double lane_q2 = 3;
  // 障碍约束
  double obs_q1 = 4.25; // 优化：从5降到4.25，降低15%
  double obs_q2 = 3.4;  // 优化：从4降到3.4，降低15%
  // double obs_length = 2.7;
  // double obs_width = 2;
  double safe_a_buffer = 0.5;
  double safe_b_buffer = 0.5;
  double stitch_dist_threshold =
      0.5; // 轨迹拼接距离阈值 (m)，小于此距离时使用上一帧轨迹起点
  // 终端代价与减速参数
  double terminal_cost_scale = 5.0;   // 终端状态代价倍率（相对 Q）
  double decel_factor = 0.5;          // 刹车距离缩放因子
  double goal_reached_dist = 1.0;     // 停车判定距离 (m)
  // double buff = 0;
  // double obs_rad = 1 + buff;
  // QR矩阵
  Matrix4d Q;
  Matrix2d R;
  // 横向偏移代价
  double ref_weight = 8.0; // 优化：从10降到8，降低20%
  Arg() {                  // 在构造函数中初始化矩阵
    Q << 0.2, 0, 0, 0,     // 优化：位置权重从1增加到1.2，增加20%
        0, 0.2, 0, 0,      // 优化：位置权重从1增加到1.2，增加20%
        0, 0, 0.1, 0,      // 航向权重保持不变
        0, 0, 0, 1.5;      // 优化：铰接角权重从1增加到1.5，增加50%

    R << 1.0, 0, // 速度控制权重从0.1增加到1.0
        0, 50;   // 优化：铰接角速率权重从8大幅增加到50
  }
};

struct BarrieInfo {
  double b;
  VectorXd d_b;
  MatrixXd dd_b;
};

// 计算两点之间距离
inline double distance(const Point &p1, const Point &p2);
// 找到路径中离自车最短距离的点
size_t find_closest_point(const std::vector<Point> &path, const State &state);
// 角度归一化到[-π, π]
inline double angle_wrap(double theta) {
  theta = fmod(theta + M_PI, 2.0 * M_PI);
  if (theta < 0.0) {
    theta += 2.0 * M_PI;
  }
  return theta - M_PI;
}

BarrieInfo barrierFunction(double q1, double q2, double c, VectorXd dc);
// 全局路径
class GlobalPlan {
private:
  std::vector<Point> points;

public:
  std::vector<Point> get_points() const { return this->points; };
  void set_plan(const std::vector<Point> &points) { this->points = points; };
};
// 局部路径
class LocalPlan {
private:
  std::vector<Point> points;

public:
  std::vector<Point> get_points() const { return this->points; };
  void set_plan(const GlobalPlan &global_plan, const State &vehicle_state,
                size_t num_points_to_extract);
};

// 系统模型
class SystemModel {
public:
  double ego_rad = 7;
  // double lf      = 1.6;
  // double lr      =  1.13;
  // double len       =  2.73;
  double lf = 3;
  double lr = 3;
  double len = 3;
  double width = 3;
  double box_length = 5;
  double dt = 0.1;
  size_t N = 50;
  SystemModel() = default;
  SystemModel(double dt, size_t N) : dt(dt), N(N) {};
  State dynamics(const State &X, const Control &U);
  Matrix4d get_jacobian_state(const Vector4d &X, const Vector2d &U);
  Matrix<double, 4, 2> get_jacobian_control(const Vector4d &X,
                                            const Vector2d &U);
};
// 车辆类
class Vehicle {
private:
  // 车辆状态
  State state;
  GlobalPlan global_plan;
  LocalPlan local_plan;
  SystemModel model;

public:
  Vehicle();

  // 设置or获取车辆状态
  void set_state(double x, double y, double heading, double v) {
    this->state << x, y, heading, v;
  };
  void set_state(const State &X) { this->state << X; };
  Vector4d get_state() const { return this->state; };

  // 设置全局路径
  void set_global_plan(const GlobalPlan &global_plan) {
    this->global_plan = global_plan;
  };
  // 获取全局路径
  GlobalPlan get_global_plan() { return this->global_plan; };
  // 设置or获取局部路径
  void set_local_plan(double desire_speed = 2.0) {
    const size_t global_size = this->global_plan.get_points().size();
    // 动态计算：N 步时域内实际行驶距离 / 全局路径点间距（0.5m）× 4 倍前瞻缓冲
    // 高速时提取更多点，避免预测状态超出参考路径末端
    const double move_step_estimate =
        0.5; // 与 hybrid_astar.json move_step 一致
    double horizon_dist = desire_speed * this->model.N * this->model.dt;
    size_t speed_based =
        static_cast<size_t>(horizon_dist / move_step_estimate) * 4;
    size_t num_points_to_extract =
        std::max(speed_based, static_cast<size_t>(this->model.N) * 3);
    num_points_to_extract = std::max(num_points_to_extract,
                                     static_cast<size_t>(this->model.N) + 10);
    num_points_to_extract = std::min(num_points_to_extract, global_size);
    this->local_plan.set_plan(this->global_plan, this->state,
                              num_points_to_extract);
  };
  LocalPlan get_local_plan() { return this->local_plan; };
  // 设置车辆模型
  void set_model(const SystemModel &model) { this->model = model; };
  SystemModel get_model() { return this->model; };
};

class Trajectory {
public:
  // 状态点集合
  std::vector<State> states;

  Trajectory() = default;
  Trajectory(const std::vector<State> &states) : states(states) {} //
  // 获取轨迹状态集合
  std::vector<State> get_states() const { return this->states; };
  void set_states(const std::vector<State> &states) { this->states = states; };
  // 添加轨迹尾的状态
  void push_back(const State &state) { this->states.push_back(state); };
  // 获取轨迹末端状态
  State back() { return this->states.back(); };
};

struct ObstacleData {
  Trajectory trj;
  double length;
  double width;
};

class ControlSequence {

public:
  std::vector<Control> controls;
  ControlSequence() = default;
  explicit ControlSequence(const std::vector<Control> &ctrls)
      : controls(ctrls) {}

  std::vector<Control> get_control_sequence() const { return controls; }
  void push_back(const Control &control) { controls.push_back(control); }
  // 添加边界检查
  Control &operator[](size_t i) {
    if (i >= controls.size())
      throw std::out_of_range("ControlSquence index out of range");
    return controls[i];
  }
  size_t size() const { return controls.size(); }
};
struct Solution {
  Solution() {}
  Solution(Trajectory ego_trj, ControlSequence control_sequence) {
    this->ego_trj = ego_trj;
    this->control_sequence = control_sequence;
  }
  Solution(const Solution &solution) {
    this->ego_trj = solution.ego_trj;
    this->control_sequence = solution.control_sequence;
    this->converged = solution.converged;
    this->iterations = solution.iterations;
    this->final_cost = solution.final_cost;
    this->solve_time_ms = solution.solve_time_ms;
  }
  Trajectory ego_trj;
  ControlSequence control_sequence;

  // 收敛信息
  bool converged = false;
  int iterations = 0;
  double final_cost = 0.0;
  double solve_time_ms = 0.0;
};

class CILQRSolver {
private:
  // double J_total = 0;
  double lamb;
  double average_gradient = 0.0;
  bool converged = false;
  Solution pre_solution;
  Vehicle ego;
  std::vector<ObstacleData> obs_list;
  Arg arg;
  std::string solver_name;
  std::string map_name;

  // 日志记录相关
  std::string log_filename;
  std::ofstream cost_log_file;
  bool enable_logging = true;
  int plan_cycle_counter = 0; // 规划周期计数，每次 solve() 递增
  double ros_time_s_ = 0.0;   // 外部传入的 ROS 时间戳 (仿真时间)

  std::vector<MatrixXd> k;
  std::vector<MatrixXd> K;
  std::vector<MatrixXd> df_dx;
  std::vector<MatrixXd> df_du;
  std::vector<MatrixXd> lx;
  std::vector<MatrixXd> lu;
  std::vector<MatrixXd> lxx;
  std::vector<MatrixXd> luu;
  std::vector<MatrixXd> lux;
  std::vector<MatrixXd> Qu;
  std::vector<MatrixXd> Quu;
  Control pure_pursuit(const State &X_cur);
  Solution get_nominal_solution(const State &init_state);
  double cal_cost(const Solution &solution);
  double cal_cost_with_logging(const Solution &solution, int iteration);
  void compute_df(const Solution &solution);
  void compute_cost_derivatives(const Solution &solution);
  void backward();
  Solution forward(const Solution &solution);

  // 日志记录方法
  void init_cost_logging();
  void log_cost_breakdown(int iteration, double total_cost, double J_position,
                          double J_heading, double J_gamma_state,
                          double J_lateral_ref, double J_velocity,
                          double J_gamma_ctrl, double J_obs, double J_lane,
                          double J_gamma_barrier, double J_gamma_dot_barrier,
                          double J_speed_rate, double J_terminal,
                          double lambda_value);
  void close_cost_logging();

public:
  // 构造函数
  // log_path:
  // 可选，指定日志文件路径（追加模式）。留空则自动创建带时间戳的新文件。
  CILQRSolver(const Vehicle &ego, const std::vector<ObstacleData> &obs_list,
              const Arg &arg, const std::string &map_name,
              const std::string &solver_name = "cilqr",
              const std::string &log_path = "")
      : ego(ego), obs_list(obs_list), arg(arg), map_name(map_name),
        solver_name(solver_name), log_filename(log_path), lamb(arg.lamb_init),
        k(arg.N), K(arg.N), df_dx(arg.N), df_du(arg.N), lx(arg.N + 1),
        lu(arg.N), lxx(arg.N + 1), luu(arg.N), lux(arg.N), Qu(arg.N),
        Quu(arg.N) {
    // 初始化矩阵向量
    for (int i = 0; i < arg.N; ++i) {
      k[i] = MatrixXd::Zero(2, 1);
      K[i] = MatrixXd::Zero(2, 4);
      df_dx[i] = MatrixXd::Zero(4, 4);
      df_du[i] = MatrixXd::Zero(4, 2);
      lu[i] = MatrixXd::Zero(2, 1);
      luu[i] = MatrixXd::Zero(2, 2);
      lux[i] = MatrixXd::Zero(2, 4);
      Qu[i] = MatrixXd::Zero(2, 1);
      Quu[i] = MatrixXd::Zero(2, 2);
    }
    for (int i = 0; i < arg.N + 1; ++i) {
      lx[i] = MatrixXd::Zero(4, 1);
      lxx[i] = MatrixXd::Zero(4, 4);
    }

    // 初始化日志记录
    init_cost_logging();
  }

  // 析构函数
  ~CILQRSolver() { close_cost_logging(); }

  // 接口
  Solution solve(const State &init_state,
                 const std::vector<ObstacleData> &obs_list);
  // 设置 ROS 仿真时间戳（planner_node 每次 solve 前调用）
  void set_ros_time(double t) { ros_time_s_ = t; }
  // 返回当前帧 ego 使用的局部参考路径点（供外部发布可视化）
  std::vector<Point> get_local_plan_points() {
    return ego.get_local_plan().get_points();
  }
};

class ALILQRSolver {
private:
  double lamb;
  double average_gradient = 0.0;
  bool converged = false;
  Solution pre_solution;
  Vehicle ego;
  std::vector<ObstacleData> obs_list;
  Arg arg;

  // iLQR matrices
  std::vector<MatrixXd> k;
  std::vector<MatrixXd> K;
  std::vector<MatrixXd> df_dx;
  std::vector<MatrixXd> df_du;
  std::vector<MatrixXd> lx;
  std::vector<MatrixXd> lu;
  std::vector<MatrixXd> lxx;
  std::vector<MatrixXd> luu;
  std::vector<MatrixXd> lux;
  std::vector<MatrixXd> Qu;
  std::vector<MatrixXd> Quu;

  // ALTRO Parameters
  int max_outer_iters = 20;
  int max_inner_iters = 50;
  double constraint_tol = 1e-3;
  double penalty_scaling = 10.0;
  double penalty_max = 1e8;

  // Constraint Data (Multipliers and Penalties)
  // State constraints (N+1 steps)
  std::vector<double> lambda_gamma_max;
  std::vector<double> mu_gamma_max;
  std::vector<double> lambda_gamma_min;
  std::vector<double> mu_gamma_min;

  std::vector<double> lambda_obs;
  std::vector<double> mu_obs;

  std::vector<double> lambda_lane;
  std::vector<double> mu_lane;

  // Control constraints (N steps)
  std::vector<double> lambda_gdot_max;
  std::vector<double> mu_gdot_max;
  std::vector<double> lambda_gdot_min;
  std::vector<double> mu_gdot_min;

  Control pure_pursuit(const State &X_cur);
  Solution get_nominal_solution(const State &init_state);
  double cal_cost(const Solution &solution);
  double cal_cost_with_logging(const Solution &solution, int iteration);
  void compute_df(const Solution &solution);
  void compute_al_derivatives(const Solution &solution);
  void backward();
  Solution forward(const Solution &solution);

  // ALTRO Helpers
  void update_constraints(const Solution &solution);
  double max_constraint_violation(const Solution &solution);
  void initialize_penalties();
  void shift_penalties();

public:
  ALILQRSolver(const Vehicle &ego, const std::vector<ObstacleData> &obs_list,
               const Arg &arg)
      : ego(ego), obs_list(obs_list), arg(arg), lamb(arg.lamb_init), k(arg.N),
        K(arg.N), df_dx(arg.N), df_du(arg.N), lx(arg.N + 1), lu(arg.N),
        lxx(arg.N + 1), luu(arg.N), lux(arg.N), Qu(arg.N), Quu(arg.N),
        lambda_gamma_max(arg.N + 1, 0.0), mu_gamma_max(arg.N + 1, 1.0),
        lambda_gamma_min(arg.N + 1, 0.0), mu_gamma_min(arg.N + 1, 1.0),
        lambda_obs(arg.N + 1, 0.0), mu_obs(arg.N + 1, 1.0),
        lambda_lane(arg.N + 1, 0.0), mu_lane(arg.N + 1, 1.0),
        lambda_gdot_max(arg.N, 0.0), mu_gdot_max(arg.N, 1.0),
        lambda_gdot_min(arg.N, 0.0), mu_gdot_min(arg.N, 1.0) {
    for (int i = 0; i < arg.N; ++i) {
      k[i] = MatrixXd::Zero(2, 1);
      K[i] = MatrixXd::Zero(2, 4);
      df_dx[i] = MatrixXd::Zero(4, 4);
      df_du[i] = MatrixXd::Zero(4, 2);
      lu[i] = MatrixXd::Zero(2, 1);
      luu[i] = MatrixXd::Zero(2, 2);
      lux[i] = MatrixXd::Zero(2, 4);
      Qu[i] = MatrixXd::Zero(2, 1);
      Quu[i] = MatrixXd::Zero(2, 2);
    }
    for (int i = 0; i < arg.N + 1; ++i) {
      lx[i] = MatrixXd::Zero(4, 1);
      lxx[i] = MatrixXd::Zero(4, 4);
    }
  }
  Solution solve(const State &init_state,
                 const std::vector<ObstacleData> &obs_list);
};
#endif
