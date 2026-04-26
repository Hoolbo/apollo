#pragma once

#include "common_types.h"
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// ============================================================
// 1. 参数结构体（对应 Python HybridAStarParams）
// ============================================================
struct ArticulatedHybridAStarParams {
    // 栅格离散化分辨率
    double grid_resolution   = 0.5;
    double heading_resolution = 5.0 * M_PI / 180.0;   // rad
    double gamma_resolution  = 5.0 * M_PI / 180.0;    // rad

    // 铰接车辆几何尺寸
    double L_f       = 3.0;   // 前车质心到铰接点距离 (m)
    double L_r       = 3.0;   // 后车质心到铰接点距离 (m)
    double W_f_body  = 3.0;   // 前车体宽度 (m)
    double W_r_body  = 3.0;   // 后车体宽度 (m)
    double L_f_body  = 5.0;   // 前车体总长 (m)
    double L_r_body  = 5.0;   // 后车体总长 (m)

    // 运动学约束
    double move_step           = 2.0;   // 前进步长 (m)
    double move_step_backwards = 2.0;   // 倒退步长 (m)
    bool   allow_reverse       = true;
    double gamma_max           = 1.0;   // 最大铰接角 (rad)
    double gamma_dot_max       = 0.3;   // 最大铰接角变化率 (rad/s)
    int    num_gamma_angles    = 3;     // 铰接角采样数
    double v_desire            = 2.0;   // 规划参考速度 (m/s)

    // 代价权重
    double steering_penalty        = 0.2;
    double steering_change_penalty = 0.5;
    double direction_change_penalty = 1.5;
    double backwards_penalty       = 1.2;
    double heuristic_weight        = 1.5;
    
    // 势场/避障参数
    double obstacle_cost_weight    = 50.0;
    double obstacle_cost_decay     = 8.0;     // m
    double max_obstacle_cost       = 1000.0;

    // 搜索控制
    int    max_iterations        = 60000;
    double goal_tolerance_xy     = 2.5;                  // m
    double goal_tolerance_heading = 15.0 * M_PI / 180.0; // rad
};

// ============================================================
// 2. 4D 节点结构体
// ============================================================
struct ArticulatedNode {
    double x, y;       // 前车体中心坐标 (m)
    double theta;      // 前车体航向角 (rad)
    double gamma;      // 铰接角 (rad)，γ = θ_f - θ_r
    double g, f;       // 代价
    bool   is_forward;
    double dgamma;     // 本步铰接角增量
    std::shared_ptr<ArticulatedNode> parent;

    bool operator<(const ArticulatedNode& o) const { return f < o.f; }
};

// 节点离散索引 (ix, iy, ith, igamma)
using NodeIndex4D = std::tuple<int,int,int,int>;

struct NodeIndex4DHash {
    size_t operator()(const NodeIndex4D& k) const {
        size_t h = 0;
        h ^= std::hash<int>()(std::get<0>(k)) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= std::hash<int>()(std::get<1>(k)) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= std::hash<int>()(std::get<2>(k)) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= std::hash<int>()(std::get<3>(k)) + 0x9e3779b9 + (h<<6) + (h>>2);
        return h;
    }
};

// ============================================================
// 3. 车体碰撞检测器声明（AABB + SAT 双层）
// ============================================================
struct Segment2D {
    Eigen::Vector2f p1, p2;
    // AABB
    float xmin, xmax, ymin, ymax;
};

class ArticulatedVehicleChecker {
public:
    ArticulatedVehicleChecker(const ArticulatedHybridAStarParams& params,
                               const std::vector<Segment2D>& segments);

    // 计算前后车体 4 角点（世界坐标）
    void getVehicleCorners(double x_f, double y_f, double theta_f, double gamma,
                           Eigen::Matrix<float,4,2>& front_corners,
                           Eigen::Matrix<float,4,2>& rear_corners) const;

    // 整体碰撞检测（返回 true 表示无碰撞）
    bool isCollisionFree(double x_f, double y_f, double theta_f, double gamma) const;

private:
    bool satPolySegment(const Eigen::Matrix<float,4,2>& poly,
                        const Segment2D& seg) const;

    const ArticulatedHybridAStarParams& params_;
    std::vector<Segment2D> segments_;
    // 预计算 AABB 数组
    std::vector<float> seg_xmin_, seg_xmax_, seg_ymin_, seg_ymax_;
};

// ============================================================
// 4. 辅助函数声明
// ============================================================
// 从占用栅格提取障碍物线段（RDP 降采样，对应 Python vectorize_map_rdp）
std::vector<Segment2D> vectorizeMapRDP(const MapData& map_data, double epsilon_px = 3.0);

// Dijkstra 预计算启发式地图（对应 Python compute_heuristic_map，正确绕障版）
std::vector<std::vector<double>> computeArticulatedDijkstraMap(
    const MapData& map_data, double goal_x, double goal_y);

// 角度归一化到 [-π, π]
inline double normalizeAngleA(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

// ============================================================
// 5. 主规划接口
//    输入 4D 起止点 (x, y, theta, gamma)
//    out_points: 输出 Point 路径 (x, y, heading)
//    out_states: 输出完整 4D 路径 {x, y, theta, gamma}
// ============================================================
bool articulated_hybrid_astar_plan(
    const MapData& map_data,
    std::array<double,4> start_state,
    std::array<double,4> goal_state,
    const ArticulatedHybridAStarParams& params,
    std::vector<Point>& out_points,
    std::vector<std::array<double,4>>* out_states = nullptr);
