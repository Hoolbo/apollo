/**
 * articulated_hybrid_astar.cpp
 *
 * 4D Hybrid A* for articulated tracked vehicles.
 * Translated from new_hybrid_a_star.py with correctness fixes:
 *   - Kinematic formula includes L_r·dγ term (paper eq. 2-10)
 *   - True Dijkstra heuristic (not Euclidean)
 *   - SAT with 3 axes
 *   - floor-based grid indexing (handles negative coordinates correctly)
 *   - Multi-point interpolation collision check (anti-tunneling)
 */

#include "articulated_hybrid_astar.h"
#include "ompl_rs.h"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <cassert>
#include <array>
#include <numeric>

// ============================================================
// Section 1: vectorizeMapRDP — obstacle contour extraction
//            Corresponds to Python vectorize_map_rdp()
// ============================================================
std::vector<Segment2D> vectorizeMapRDP(const MapData& map_data, double epsilon_px)
{
    // Build uint8 occupancy image: 1=obstacle, 0=free
    // map_data.data is [row][col], value < 0 means obstacle
    int h = map_data.height;
    int w = map_data.width;
    cv::Mat grid(h, w, CV_8UC1, cv::Scalar(0));
    for (int row = 0; row < h; ++row) {
        for (int col = 0; col < w; ++col) {
            if (map_data.data[row][col] < 0.0)
                grid.at<uchar>(row, col) = 1;
        }
    }

    // Extract contours — RETR_LIST to get all levels including interior obstacles
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(grid, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Segment2D> segments;
    double ox = map_data.origin[0];
    double oy = map_data.origin[1];
    double res = map_data.resolution;

    for (const auto& contour : contours) {
        if ((int)contour.size() < 3) continue;

        // RDP simplification
        std::vector<cv::Point2f> contour_f;
        for (const auto& p : contour)
            contour_f.emplace_back((float)p.x, (float)p.y);

        std::vector<cv::Point2f> approx;
        cv::approxPolyDP(contour_f, approx, (float)epsilon_px, true);

        if (approx.size() < 2) continue;

        // Convert to world coordinates and build segments
        int n = (int)approx.size();
        std::vector<Eigen::Vector2f> world_pts(n);
        for (int i = 0; i < n; ++i) {
            world_pts[i].x() = (float)(ox + (approx[i].x + 0.5) * res);
            world_pts[i].y() = (float)(oy + (approx[i].y + 0.5) * res);
        }
        for (int i = 0; i < n; ++i) {
            Segment2D seg;
            seg.p1 = world_pts[i];
            seg.p2 = world_pts[(i + 1) % n];
            seg.xmin = std::min(seg.p1.x(), seg.p2.x());
            seg.xmax = std::max(seg.p1.x(), seg.p2.x());
            seg.ymin = std::min(seg.p1.y(), seg.p2.y());
            seg.ymax = std::max(seg.p1.y(), seg.p2.y());
            segments.push_back(seg);
        }
    }
    return segments;
}

// ============================================================
// Section 2: ArticulatedVehicleChecker — AABB + SAT collision
//            Corresponds to Python VehicleCollisionChecker
// ============================================================
ArticulatedVehicleChecker::ArticulatedVehicleChecker(
    const ArticulatedHybridAStarParams& params,
    const std::vector<Segment2D>& segments)
    : params_(params), segments_(segments)
{
    seg_xmin_.reserve(segments.size());
    seg_xmax_.reserve(segments.size());
    seg_ymin_.reserve(segments.size());
    seg_ymax_.reserve(segments.size());
    for (const auto& s : segments) {
        seg_xmin_.push_back(s.xmin);
        seg_xmax_.push_back(s.xmax);
        seg_ymin_.push_back(s.ymin);
        seg_ymax_.push_back(s.ymax);
    }
}

void ArticulatedVehicleChecker::getVehicleCorners(
    double x_f, double y_f, double theta_f, double gamma,
    Eigen::Matrix<float,4,2>& fc,
    Eigen::Matrix<float,4,2>& rc) const
{
    float cos_f = (float)std::cos(theta_f);
    float sin_f = (float)std::sin(theta_f);
    double theta_r = theta_f - gamma;
    float cos_r = (float)std::cos(theta_r);
    float sin_r = (float)std::sin(theta_r);

    float xf = (float)x_f, yf = (float)y_f;
    float hl_f = (float)(params_.L_f_body / 2.0);
    float hw   = (float)(params_.W_body   / 2.0);

    // Front body corners
    fc << xf + hl_f*cos_f - hw*sin_f,  yf + hl_f*sin_f + hw*cos_f,
          xf + hl_f*cos_f + hw*sin_f,  yf + hl_f*sin_f - hw*cos_f,
          xf - hl_f*cos_f + hw*sin_f,  yf - hl_f*sin_f - hw*cos_f,
          xf - hl_f*cos_f - hw*sin_f,  yf - hl_f*sin_f + hw*cos_f;

    // Articulation point → rear body center
    float x_c = xf - (float)params_.L_f * cos_f;
    float y_c = yf - (float)params_.L_f * sin_f;
    float x_r = x_c - (float)params_.L_r * cos_r;
    float y_r = y_c - (float)params_.L_r * sin_r;
    float hl_r = (float)(params_.L_r_body / 2.0);

    rc << x_r + hl_r*cos_r - hw*sin_r,  y_r + hl_r*sin_r + hw*cos_r,
          x_r + hl_r*cos_r + hw*sin_r,  y_r + hl_r*sin_r - hw*cos_r,
          x_r - hl_r*cos_r + hw*sin_r,  y_r - hl_r*sin_r - hw*cos_r,
          x_r - hl_r*cos_r - hw*sin_r,  y_r - hl_r*sin_r + hw*cos_r;
}

// SAT: 4-axis test between rectangle poly (4×2) and line segment seg
// SAT: 3-axis test between rectangle poly (4×2) and line segment seg.
// Axes: 2 rectangle edge normals + 1 segment perpendicular normal.
// This is complete and correct — the segment parallel direction is NOT
// a valid SAT axis for a finite segment (a segment has only one edge → one normal).
// Returns true if collision detected (no separating axis found).
bool ArticulatedVehicleChecker::satPolySegment(
    const Eigen::Matrix<float,4,2>& poly,
    const Segment2D& seg) const
{
    Eigen::Vector2f p1 = seg.p1;
    Eigen::Vector2f p2 = seg.p2;
    Eigen::Vector2f seg_edge = p2 - p1;

    // 3 separating axes:
    //   0,1 : rectangle edge normals (2 unique axes for a rectangle)
    //   2   : segment perpendicular normal
    std::array<Eigen::Vector2f, 3> axes;
    int n_axes = 0;

    for (int i = 0; i < 2; ++i) {
        Eigen::Vector2f edge = poly.row((i+1)%4) - poly.row(i);
        Eigen::Vector2f normal(-edge.y(), edge.x());
        float len = normal.norm();
        if (len > 1e-6f) axes[n_axes++] = normal / len;
    }
    {
        float len = seg_edge.norm();
        if (len > 1e-6f) {
            Eigen::Vector2f normal(-seg_edge.y(), seg_edge.x());
            axes[n_axes++] = normal / len;
        }
    }

    // Project both shapes onto each axis; if any axis has no overlap → no collision
    Eigen::Matrix<float,2,2> seg_pts;
    seg_pts << p1.x(), p1.y(),
               p2.x(), p2.y();

    for (int a = 0; a < n_axes; ++a) {
        Eigen::Vector2f ax = axes[a];
        float poly_min =  std::numeric_limits<float>::infinity();
        float poly_max = -std::numeric_limits<float>::infinity();
        for (int r = 0; r < 4; ++r) {
            float proj = poly.row(r).dot(ax.transpose());
            poly_min = std::min(poly_min, proj);
            poly_max = std::max(poly_max, proj);
        }
        float s1 = seg_pts.row(0).dot(ax.transpose());
        float s2 = seg_pts.row(1).dot(ax.transpose());
        float seg_min = std::min(s1, s2);
        float seg_max = std::max(s1, s2);

        if (poly_max < seg_min || seg_max < poly_min)
            return false;  // separating axis found → no collision
    }
    return true;  // all axes overlap → collision
}


bool ArticulatedVehicleChecker::isCollisionFree(
    double x_f, double y_f, double theta_f, double gamma) const
{
    if (segments_.empty()) return true;

    Eigen::Matrix<float,4,2> front_poly, rear_poly;
    getVehicleCorners(x_f, y_f, theta_f, gamma, front_poly, rear_poly);

    // AABB of combined vehicle footprint
    float vx_min = front_poly.col(0).minCoeff();
    float vx_max = front_poly.col(0).maxCoeff();
    float vy_min = front_poly.col(1).minCoeff();
    float vy_max = front_poly.col(1).maxCoeff();
    vx_min = std::min(vx_min, rear_poly.col(0).minCoeff());
    vx_max = std::max(vx_max, rear_poly.col(0).maxCoeff());
    vy_min = std::min(vy_min, rear_poly.col(1).minCoeff());
    vy_max = std::max(vy_max, rear_poly.col(1).maxCoeff());

    // Coarse AABB filter — vectorized loop
    int N = (int)segments_.size();
    for (int i = 0; i < N; ++i) {
        if (vx_max < seg_xmin_[i] || vx_min > seg_xmax_[i] ||
            vy_max < seg_ymin_[i] || vy_min > seg_ymax_[i])
            continue;

        // Fine SAT check (forward only front body first for early exit)
        if (satPolySegment(front_poly, segments_[i]) ||
            satPolySegment(rear_poly,  segments_[i]))
            return false;
    }
    return true;
}

// ============================================================
// Section 3: Dijkstra heuristic map
//            Corrected version: real obstacle-avoiding distances
//            Corresponds to fixed Python compute_heuristic_map()
// ============================================================
std::vector<std::vector<double>> computeArticulatedDijkstraMap(
    const MapData& map_data, double goal_x, double goal_y)
{
    int h = map_data.height;
    int w = map_data.width;
    double res = map_data.resolution;
    double ox  = map_data.origin[0];
    double oy  = map_data.origin[1];

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> dist(h, std::vector<double>(w, INF));

    int gx = (int)std::floor((goal_x - ox) / res);
    int gy = (int)std::floor((goal_y - oy) / res);

    if (gx < 0 || gx >= w || gy < 0 || gy >= h) {
        std::cerr << "[Dijkstra] goal out of map\n";
        return dist;
    }
    if (map_data.data[gy][gx] < 0.0) {
        std::cerr << "[Dijkstra] goal in obstacle\n";
        return dist;
    }

    // Min-heap: (dist, x, y)
    using T3 = std::tuple<double,int,int>;
    std::priority_queue<T3, std::vector<T3>, std::greater<T3>> pq;
    dist[gy][gx] = 0.0;
    pq.push({0.0, gx, gy});

    // 8-connected motions
    const int dx[]   = {1,-1,0, 0, 1,-1, 1,-1};
    const int dy[]   = {0, 0,1,-1, 1,-1,-1, 1};
    const double dc[]= {1, 1, 1, 1, 1.414214, 1.414214, 1.414214, 1.414214};

    while (!pq.empty()) {
        auto [d, cx, cy] = pq.top(); pq.pop();
        if (d > dist[cy][cx]) continue;
        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (map_data.data[ny][nx] < 0.0) continue;  // obstacle
            double nd = d + dc[i] * res;
            if (nd < dist[ny][nx]) {
                dist[ny][nx] = nd;
                pq.push({nd, nx, ny});
            }
        }
    }
    return dist;
}

// ============================================================
// Section 4: Main 4D Hybrid A* planner
//            Corresponds to Python hybrid_astar_plan()
// ============================================================

// Grid index helper (floor-based, handles negative coordinates)
static NodeIndex4D makeIndex(double x, double y, double theta, double gamma,
                              const MapData& map, const ArticulatedHybridAStarParams& p)
{
    int ix  = (int)std::floor((x - map.origin[0]) / p.grid_resolution);
    int iy  = (int)std::floor((y - map.origin[1]) / p.grid_resolution);
    int ith = (int)std::floor(normalizeAngleA(theta) / p.heading_resolution);
    int ig  = (int)std::floor(gamma / p.gamma_resolution);
    return {ix, iy, ith, ig};
}

// RS Analytic Expansion (Paper 3.2.4):
// Maps RS curve to control sequence, forward simulates exact kinematics 
// with gamma_dot_max limits, and checks if it reaches goal without collision.
static std::vector<std::array<double,4>> tryRSConnect(
    const ArticulatedNode& n,
    const std::array<double,4>& goal_state,
    const ArticulatedHybridAStarParams& p,
    const ArticulatedVehicleChecker& checker)
{
    // Minimal turning radius
    double R_min = p.L_f / std::tan(p.gamma_max / 2.0);
    
    // Generate shortest RS path ignoring obstacles
    auto rs_path = reeds_shepp_ompl::getPath(
        n.x, n.y, n.theta,
        goal_state[0], goal_state[1], goal_state[2], R_min);
        
    if (rs_path.totalLength_ > 9e8) return {}; // No solution
    
    // Check if path contains reverse motions and reject if so
    for (int i = 0; i < 5; ++i) {
        if (rs_path.type_[i] != reeds_shepp_ompl::RS_NOP && rs_path.length_[i] < -1e-6) {
            return {}; // Reject reversing RS paths
        }
    }

    std::vector<std::array<double,4>> simulated_path;
    
    // Initial state for forward simulation
    double cx = n.x, cy = n.y, cth = n.theta, cgamma = n.gamma;
    double sim_step = p.grid_resolution * 0.5; // Fine integration step
    
    for (int idx = 0; idx < 5; ++idx) {
        if (rs_path.type_[idx] == reeds_shepp_ompl::RS_NOP) continue;
        double seg_len_norm = rs_path.length_[idx];
        if (std::abs(seg_len_norm) < 1e-6) continue;

        // Un-normalize length to meters
        double seg_len_m = std::abs(seg_len_norm) * R_min;
        bool is_forward  = (seg_len_norm > 0);
        
        int steps = std::max(1, (int)std::ceil(seg_len_m / sim_step));
        double actual_step = seg_len_m / steps;
        
        // Target articulation angle based on RS steering commands
        double target_gamma = 0.0;
        if (rs_path.type_[idx] == reeds_shepp_ompl::RS_LEFT) target_gamma = p.gamma_max;
        else if (rs_path.type_[idx] == reeds_shepp_ompl::RS_RIGHT) target_gamma = -p.gamma_max;

        double dir_sign = is_forward ? 1.0 : -1.0;
        double ds = dir_sign * actual_step;
        
        for (int i = 0; i < steps; ++i) {
            // Apply gamma_dot limit
            double max_dgamma = p.gamma_dot_max * (actual_step / p.v_desire);
            double diff = target_gamma - cgamma;
            double dgamma = 0.0;
            if (std::abs(diff) > max_dgamma) {
                dgamma = (diff > 0) ? max_dgamma : -max_dgamma;
            } else {
                dgamma = diff;
            }
            double next_gamma = cgamma + dgamma;
            
            // Forward kinematics (Paper Eq 2-10)
            double d_theta = (ds * std::sin(cgamma) + p.L_r * dgamma) 
                           / (p.L_f * std::cos(cgamma) + p.L_r);
            double next_th = normalizeAngleA(cth + d_theta);
            
            double next_x = cx + ds * std::cos(cth + d_theta * 0.5);
            double next_y = cy + ds * std::sin(cth + d_theta * 0.5);
            
            // Collision Check
            if (!checker.isCollisionFree(next_x, next_y, next_th, next_gamma)) {
                return {}; // Path hits obstacle
            }
            
            simulated_path.push_back({next_x, next_y, next_th, next_gamma});
            cx = next_x;
            cy = next_y;
            cth = next_th;
            cgamma = next_gamma;
        }
    }

    // Check if the kinematically simulated trajectory actually reached the goal
    // Because of gamma rate limits, it will drift from the ideal RS curve.
    double dist_err = std::hypot(cx - goal_state[0], cy - goal_state[1]);
    double th_err   = std::abs(normalizeAngleA(cth - goal_state[2]));
    
    if (dist_err > p.goal_tolerance_xy || th_err > p.goal_tolerance_heading) {
        return {}; // Non-holonomic drift too large, failed to reach goal
    }

    return simulated_path;
}



bool articulated_hybrid_astar_plan(
    const MapData& map_data,
    std::array<double,4> start_state,
    std::array<double,4> goal_state,
    const ArticulatedHybridAStarParams& params,
    std::vector<Point>& out_points,
    std::vector<std::array<double,4>>* out_states)
{
    std::cout << "[ArtHA*] Building RDP obstacle segments...\n";
    auto env_segments = vectorizeMapRDP(map_data, 3.0);
    std::cout << "[ArtHA*] " << env_segments.size() << " segments extracted.\n";

    ArticulatedVehicleChecker checker(params, env_segments);

    std::cout << "[ArtHA*] Computing Dijkstra heuristic map...\n";
    auto dijkstra_map = computeArticulatedDijkstraMap(
        map_data, goal_state[0], goal_state[1]);
    std::cout << "[ArtHA*] Done.\n";

    // Lambda: look up Dijkstra distance safely
    auto h_obs = [&](double x, double y) -> double {
        int gx = (int)std::floor((x - map_data.origin[0]) / map_data.resolution);
        int gy = (int)std::floor((y - map_data.origin[1]) / map_data.resolution);
        if (gx < 0 || gx >= map_data.width || gy < 0 || gy >= map_data.height)
            return std::numeric_limits<double>::infinity();
        return dijkstra_map[gy][gx];
    };

    // Lambda: combined heuristic (Dijkstra + Reeds-Shepp)
    double R_min = params.L_f / std::tan(params.gamma_max / 2.0);
    auto heuristic = [&](double x, double y, double theta) -> double {
        auto rs = reeds_shepp_ompl::getPath(x, y, theta, goal_state[0], goal_state[1], goal_state[2], R_min);
        double h_k = rs.totalLength_ * R_min; // Un-normalize length
        double h_d = h_obs(x, y);
        return params.heuristic_weight * std::max(h_k, h_d);
    };

    // Open list (min-heap by f), open_set for O(1) lookup, closed_set
    using NodePtr = std::shared_ptr<ArticulatedNode>;
    auto cmp = [](const NodePtr& a, const NodePtr& b){ return a->f > b->f; };
    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(cmp)> open_list(cmp);
    std::unordered_map<NodeIndex4D, NodePtr, NodeIndex4DHash> open_set;
    std::unordered_set<NodeIndex4D, NodeIndex4DHash> closed_set;

    // Create start node
    auto start_node = std::make_shared<ArticulatedNode>();
    start_node->x = start_state[0]; start_node->y = start_state[1];
    start_node->theta = start_state[2]; start_node->gamma = start_state[3];
    start_node->g = 0.0;
    start_node->f = heuristic(start_state[0], start_state[1], start_state[2]);
    start_node->is_forward = true;
    start_node->dgamma = 0.0;
    start_node->parent = nullptr;

    auto start_idx = makeIndex(start_state[0], start_state[1],
                               start_state[2], start_state[3], map_data, params);
    open_list.push(start_node);
    open_set[start_idx] = start_node;

    // Pre-compute delta-gamma samples
    double delta_gamma_max = params.gamma_dot_max * (params.move_step / params.v_desire);
    // num_gamma_angles linspace from -max to +max
    std::vector<double> dgammas;
    {
        int ng = params.num_gamma_angles;
        for (int i = 0; i < ng; ++i) {
            dgammas.push_back(-delta_gamma_max + i * (2.0 * delta_gamma_max / (ng - 1)));
        }
    }

    // Number of interpolated collision-check steps per move_step
    auto num_interp_checks = [&](double step) {
        return std::max(2, (int)std::ceil(std::abs(step) / params.grid_resolution));
    };

    NodePtr goal_node = nullptr;
    int iterations = 0;

    while (!open_list.empty() && iterations < params.max_iterations) {
        NodePtr current = open_list.top();
        open_list.pop();

        auto idx = makeIndex(current->x, current->y,
                             current->theta, current->gamma, map_data, params);
        if (closed_set.count(idx)) continue;
        closed_set.insert(idx);
        open_set.erase(idx);

        // Goal check
        if (std::hypot(current->x - goal_state[0], current->y - goal_state[1])
                < params.goal_tolerance_xy &&
            std::abs(normalizeAngleA(current->theta - goal_state[2]))
                < params.goal_tolerance_heading)
        {
            goal_node = current;
            std::cout << "[ArtHA*] Path found after " << iterations << " iterations.\n";
            break;
        }

        // RS Analytic Expansion (Paper Section 3.2.4)
        // Try to connect to goal directly using Reeds-Shepp curve
        // Execute closer to goal or periodically to save time
        double dist_to_goal = std::hypot(current->x - goal_state[0], current->y - goal_state[1]);
        if (iterations % 5 == 0 || dist_to_goal < 25.0) {
            auto rs_path = tryRSConnect(*current, goal_state, params, checker);
            if (!rs_path.empty()) {
                std::cout << "[ArtHA*] Analytic RS expansion connected at iteration " << iterations << "!\n";
                
                // Backtrack current path
                std::vector<std::array<double,4>> raw_path;
                for (auto n = current; n; n = n->parent)
                    raw_path.push_back({n->x, n->y, n->theta, n->gamma});
                std::reverse(raw_path.begin(), raw_path.end());
                
                // Append RS path
                for (const auto& pt : rs_path)
                    raw_path.push_back(pt);

                // Output
                out_points.clear();
                out_points.reserve(raw_path.size());
                for (const auto& s : raw_path)
                    out_points.emplace_back(s[0], s[1], s[2]);
                if (out_states) *out_states = raw_path;
                std::cout << "[ArtHA*] Output " << out_points.size() << " waypoints.\n";
                return true;
            }
        }



        // 4D state expansion
        std::vector<std::pair<double,bool>> directions;
        directions.push_back({params.move_step, true});
        if (params.allow_reverse)
            directions.push_back({-params.move_step_backwards, false});

        for (auto [step, is_fwd] : directions) {
            for (double dg : dgammas) {
                double next_gamma = current->gamma + dg;
                if (std::abs(next_gamma) > params.gamma_max) continue;

                // Kinematic forward integration — paper eq. 2-10
                // dθ = (D·sin(γ) + L_r·dγ) / (L_f·cos(γ) + L_r)
                double d_theta = (step * std::sin(current->gamma) + params.L_r * dg)
                               / (params.L_f * std::cos(current->gamma) + params.L_r);
                double next_theta = normalizeAngleA(current->theta + d_theta);
                double next_x = current->x + step * std::cos(current->theta + d_theta * 0.5);
                double next_y = current->y + step * std::sin(current->theta + d_theta * 0.5);

                // Multi-point interpolated collision check (anti-tunneling)
                int nc = num_interp_checks(step);
                bool safe = true;
                for (int k = 1; k <= nc && safe; ++k) {
                    double r = (double)k / nc;
                    double it = normalizeAngleA(current->theta + d_theta * r);
                    double ix = current->x + step * r * std::cos(current->theta + d_theta * r * 0.5);
                    double iy = current->y + step * r * std::sin(current->theta + d_theta * r * 0.5);
                    double ig = current->gamma + dg * r;
                    if (!checker.isCollisionFree(ix, iy, it, ig))
                        safe = false;
                }
                if (!safe) continue;

                // Cost g
                double g_cost = current->g + std::abs(step) *
                                (is_fwd ? 1.0 : params.backwards_penalty);
                g_cost += params.steering_penalty * std::abs(next_gamma);
                g_cost += params.steering_change_penalty * std::abs(dg);
                if (is_fwd != current->is_forward)
                    g_cost += params.direction_change_penalty;

                double f_cost = g_cost + heuristic(next_x, next_y, next_theta);

                auto neighbor = std::make_shared<ArticulatedNode>();
                neighbor->x = next_x; neighbor->y = next_y;
                neighbor->theta = next_theta; neighbor->gamma = next_gamma;
                neighbor->g = g_cost; neighbor->f = f_cost;
                neighbor->is_forward = is_fwd;
                neighbor->dgamma = dg;
                neighbor->parent = current;

                auto n_idx = makeIndex(next_x, next_y, next_theta, next_gamma,
                                       map_data, params);
                if (closed_set.count(n_idx)) continue;

                auto it2 = open_set.find(n_idx);
                if (it2 == open_set.end() || g_cost < it2->second->g) {
                    open_set[n_idx] = neighbor;
                    open_list.push(neighbor);
                }
            }
        }

        ++iterations;
        if (iterations % 5000 == 0)
            std::cout << "[ArtHA*] iter=" << iterations
                      << " open=" << open_set.size() << "\n";
    }

    if (!goal_node) {
        std::cerr << "[ArtHA*] No path found after " << iterations << " iterations.\n";
        return false;
    }

    // Backtrack path
    std::vector<std::array<double,4>> raw_path;
    for (auto n = goal_node; n; n = n->parent)
        raw_path.push_back({n->x, n->y, n->theta, n->gamma});
    std::reverse(raw_path.begin(), raw_path.end());

    // Convert to Point
    out_points.clear();
    out_points.reserve(raw_path.size());
    for (const auto& s : raw_path)
        out_points.emplace_back(s[0], s[1], s[2]);

    // Also fill 4D states if requested
    if (out_states) {
        *out_states = raw_path;
    }

    std::cout << "[ArtHA*] Output " << out_points.size() << " waypoints.\n";
    return true;
}
