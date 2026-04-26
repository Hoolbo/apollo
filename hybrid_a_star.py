# # 内置库
# import heapq
# import math
# from math import sqrt, cos, sin, tan, pi
# from collections import deque
# import time

# # 第三方库
# import numpy as np

# try:
#     import hybrid_a_star.reeds_shepp_path_planning as rs
# except:
#     raise

# # 车辆参数
# WB = 5.15  # 轴距
# W = 4  # 车辆宽度
# LF = 6.5  # 后轴到车头的距离
# LB = 2.5  # 后轴到车尾的距离
# MAX_STEER = np.deg2rad(30)  # 最大转向角 [rad]

# # 规划成本参数
# SB_COST = 50.0           # 切换方向惩罚
# BACK_COST = 3.0          # 倒车惩罚
# STEER_CHANGE_COST = 100.0  # 转向角变化惩罚
# STEER_COST = 500.0         # 转向角惩罚
# H_COST = 200             # 启发式成本
# MAX_OBSTACLE_COST = 1000.0  # 障碍物最大代价
# DECAY_RATE = 5.0         # 代价衰减率（米）
# OBSTACLE_COST = 50       # 障碍物代价权重

# class Config:
#     def __init__(self, observation, xy_resolution=2.0, yaw_resolution=np.deg2rad(15), 
#                  motion_resolution=0.2, n_steer=11, grid_resolution=0.1):
#         self.xy_resolution = xy_resolution
#         self.yaw_resolution = yaw_resolution
#         self.motion_resolution = motion_resolution
#         self.n_steer = n_steer
#         self.grid_resolution = grid_resolution

#         min_x_m = observation['test_setting']['x_min']
#         min_y_m = observation['test_setting']['y_min']
#         max_x_m = observation['test_setting']['x_max']
#         max_y_m = observation['test_setting']['y_max']

#         self.minx = round(min_x_m / xy_resolution)
#         self.miny = round(min_y_m / xy_resolution)
#         self.maxx = round(max_x_m / xy_resolution)
#         self.maxy = round(max_y_m / xy_resolution)

#         self.xw = round(self.maxx - self.minx)
#         self.yw = round(self.maxy - self.miny)

#         self.minyaw = round(-180 / yaw_resolution) - 1
#         self.maxyaw = round(180 / yaw_resolution)
#         self.yaww = round(self.maxyaw - self.minyaw)

# class Node:
#     def __init__(self, xind, yind, yawind, direction, xlist, ylist, yawlist, directions,
#                  steer=0.0, pind=None, cost=None):
#         if len(xlist) != len(directions):
#             raise ValueError(f"Directions length {len(directions)} does not match xlist length {len(xlist)}")
#         self.xind = xind
#         self.yind = yind
#         self.yawind = yawind
#         self.direction = bool(direction)  # 标量方向，布尔值
#         self.xlist = xlist
#         self.ylist = ylist
#         self.yawlist = yawlist
#         self.directions = [bool(d) for d in directions]  # 逐点方向，布尔值
#         self.steer = steer
#         self.pind = pind
#         self.cost = cost

# class Path:
#     def __init__(self, xlist, ylist, yawlist, directionlist, cost):
#         if len(xlist) != len(directionlist):
#             raise ValueError(f"Directionlist length {len(directionlist)} does not match xlist length {len(xlist)}")
#         self.xlist = xlist
#         self.ylist = ylist
#         self.yawlist = yawlist
#         self.directionlist = [bool(d) for d in directionlist]  # 确保布尔值
#         self.cost = cost

# def generate_obstacle_cost_map(observation, config):
#     try:
#         image = observation['hdmaps_info']['image_mask'].image_ndarray
#     except KeyError as e:
#         #print(f"错误：无法在 observation 中找到必要的地图数据: {e}")
#         return None 
    
#     if image is None:
#         #print("错误：observation 中的 image_ndarray 为 None")
#         return None

#     height, width = image.shape
#     cost_map = np.zeros((height, width), dtype=np.float32)

#     queue = deque()
#     visited = set()

#     for py in range(height):
#         for px in range(width):
#             if image[py, px] == 0:
#                 cost_map[py, px] = MAX_OBSTACLE_COST
#                 queue.append((px, py, 0))
#                 visited.add((px, py))

#     while queue:
#         px, py, dist = queue.popleft()
#         for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
#             nx, ny = px + dx, py + dy
#             if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
#                 if image[ny, nx] != 0:
#                     distance_m = (dist + 1) * config.grid_resolution
#                     calculated_cost = MAX_OBSTACLE_COST * math.exp(-distance_m / DECAY_RATE)
#                     cost_map[ny, nx] = calculated_cost
#                     visited.add((nx, ny))
#                     queue.append((nx, ny, dist + 1))

#     return cost_map

# def calc_motion_inputs(config):
#     for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER, config.n_steer), [0.0])):
#         for d in [True]:  # 只考虑前进
#             yield [steer, d]

# def check_car_collision(xlist, ylist, yawlist, collision_lookup, observation):
#     local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
#     local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
#     for x, y, yaw in zip(xlist, ylist, yawlist):
#         if collision_lookup.collision_detection(x - local_x_range[0],
#                                                 y - local_y_range[0],
#                                                 yaw,
#                                                 observation['hdmaps_info']['image_mask'].image_ndarray):
#             return True
#     return False

# def check_grid_collision(xind, yind, observation, config):
#     x = xind * config.xy_resolution
#     y = yind * config.xy_resolution
#     local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
#     local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
#     x_rel = x - local_x_range[0]
#     y_rel = y - local_y_range[0]
#     px = math.floor(x_rel / config.grid_resolution)
#     py = math.floor(y_rel / config.grid_resolution)
#     image = observation['hdmaps_info']['image_mask'].image_ndarray
#     if px < 0 or px >= image.shape[1] or py < 0 or py >= image.shape[0]:
#         return True
#     return image[py][px] == False

# def precompute_collision_map(config, observation):
#     width = config.maxx - config.minx + 1
#     height = config.maxy - config.miny + 1
#     collision_map = np.zeros((width, height), dtype=np.bool_)
#     local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
#     local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
#     image = observation['hdmaps_info']['image_mask'].image_ndarray
    
#     image = (image == 0)  # 0 表示障碍物
#     for xind in range(config.minx, config.maxx + 1):
#         for yind in range(config.miny, config.maxy + 1):
#             x = xind * config.xy_resolution
#             y = yind * config.xy_resolution
#             x_rel = x - local_x_range[0]
#             y_rel = y - local_y_range[0]
#             px = math.floor(x_rel / config.grid_resolution)
#             py = math.floor(y_rel / config.grid_resolution)
#             if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
#                 collision_map[xind - config.minx, yind - config.miny] = image[py, px]
    
#     return collision_map

# def dijkstra_distance_map(goal, config, collision_lookup, observation):
#     #print("Calculating distance map...")
#     start_time = time.time()
#     gx = round(goal[0] / config.xy_resolution)
#     gy = round(goal[1] / config.xy_resolution)
    
#     collision_map = precompute_collision_map(config, observation)
#     if collision_map[gx - config.minx, gy - config.miny]:
#         #print("Warning: Goal point is in obstacle!")
#         return {}
    
#     width = config.maxx - config.minx + 1
#     height = config.maxy - config.miny + 1
#     dist_map_array = np.full((width, height), np.inf)
#     dist_map_array[gx - config.minx, gy - config.miny] = 0
    
#     max_dist = 600.0
#     pq = [(0, (gx, gy))]
#     visited = set()
    
#     while pq:
#         dist, (xind, yind) = heapq.heappop(pq)
#         if (xind, yind) in visited:
#             continue
#         visited.add((xind, yind))
#         for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
#             nx, ny = xind + dx, yind + dy
#             if not (config.minx <= nx <= config.maxx and config.miny <= ny <= config.maxy):
#                 continue
#             if collision_map[nx - config.minx, ny - config.miny]:
#                 continue
#             new_dist = dist + config.xy_resolution
#             if new_dist > max_dist:
#                 continue
#             if new_dist < dist_map_array[nx - config.minx, ny - config.miny]:
#                 dist_map_array[nx - config.minx, ny - config.miny] = new_dist
#                 heapq.heappush(pq, (new_dist, (nx, ny)))
    
#     dist_map = {}
#     for xind in range(config.minx, config.maxx + 1):
#         for yind in range(config.miny, config.maxy + 1):
#             dist = dist_map_array[xind - config.minx, yind - config.miny]
#             if dist != np.inf:
#                 dist_map[(xind, yind)] = dist
#     end_time = time.time()
#     #print(f"[Dijstra] 计算用时: {end_time - start_time:.2f}秒")
#     return dist_map

# def pi_2_pi(angle):
#     return (angle + pi) % (2 * pi) - pi

# def move(x, y, yaw, distance, steer, L=WB):
#     x += distance * cos(yaw)
#     y += distance * sin(yaw)
#     yaw += pi_2_pi(distance * tan(steer) / L)
#     return x, y, yaw

# def get_neighbors(current, config, collision_lookup, observation):
#     for steer, d in calc_motion_inputs(config):
#         node = calc_next_node(current, steer, d, config, collision_lookup, observation)
#         if node and verify_index(node, config):
#             yield node

# def calc_next_node(current, steer, direction, config, collision_lookup, observation):
#     x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]
#     arc_l = config.xy_resolution * 1.5
#     xlist, ylist, yawlist, directions = [], [], [], []
#     for dist in np.arange(0, arc_l, config.motion_resolution):
#         x, y, yaw = move(x, y, yaw, config.motion_resolution * direction, steer)
#         xlist.append(x)
#         ylist.append(y)
#         yawlist.append(yaw)
#         directions.append(bool(direction))  # 逐点方向，布尔值
#     if check_car_collision(xlist, ylist, yawlist, collision_lookup, observation):
#         return None
    
#     xind = round(x / config.xy_resolution)
#     yind = round(y / config.xy_resolution)
#     yawind = round(yaw / config.yaw_resolution)
#     addedcost = 0.0
#     if direction != current.direction:
#         addedcost += SB_COST
#     addedcost += STEER_COST * abs(steer)
#     addedcost += STEER_CHANGE_COST * abs(current.steer - steer)
#     cost = current.cost + addedcost + arc_l

#     node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, directions,
#                 pind=calc_index(current, config), cost=cost, steer=steer)
#     return node

# def is_same_grid(n1, n2):
#     if n1.xind == n2.xind and n1.yind == n2.yind and n1.yawind == n2.yawind:
#         return True
#     return False

# def analytic_expansion(current, goal, config, collision_lookup, observation, bo_alp_in):
#     sx = current.xlist[-1]
#     sy = current.ylist[-1]
#     syaw = current.yawlist[-1]
#     gx = goal.xlist[-1]
#     gy = goal.ylist[-1]
#     gyaw = goal.yawlist[-1]
#     max_curvature = math.tan(MAX_STEER) / WB
#     paths = rs.calc_paths(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size=config.motion_resolution)
#     if not paths:
#         return None
#     best_path, best = None, None
#     for path in paths:
#         if not check_car_collision(path.x, path.y, path.yaw, collision_lookup, observation):
#             l_back = sum(abs(l) for l, d in zip(path.lengths, path.directions) if not d)
#             b_num = sum(1 for d in path.directions if not d)
#             if bo_alp_in:
#                 if not path.directions[-1] and l_back < 150 and b_num < 3:
#                     cost = calc_rs_path_cost(path, config, observation)
#                     if not best or best > cost:
#                         best = cost
#                         best_path = path
#                 elif l_back < 200 and b_num <= 3:
#                     cost = calc_rs_path_cost(path, config, observation) + 50
#                     if not best or best > cost:
#                         best = cost
#                         best_path = path
#             else:
#                 if b_num == 0 and l_back == 0:
#                     cost = calc_rs_path_cost(path, config, observation)
#                 else:
#                     cost = calc_rs_path_cost(path, config, observation) + 50
#                 if not best or best > cost:
#                     best = cost
#                     best_path = path
#     return best_path

# def update_node_with_analystic_expantion(current, goal, config, collision_lookup, observation, bo_alp_in):
#     apath = analytic_expansion(current, goal, config, collision_lookup, observation, bo_alp_in)
#     if apath:
#         fx = apath.x[1:]
#         fy = apath.y[1:]
#         fyaw = apath.yaw[1:]
#         fcost = current.cost + calc_rs_path_cost(apath, config, observation)
#         fpind = calc_index(current, config)
#         fd = [bool(d) for d in apath.directions[1:]]  # 转换为布尔值
#         if len(fd) != len(fx):
#             #print(f"Warning: Directions length {len(fd)} does not match path length {len(fx)}")
#         fsteer = 0.0
#         fpath = Node(current.xind, current.yind, current.yawind,
#                      current.direction, fx, fy, fyaw, fd,
#                      cost=fcost, pind=fpind, steer=fsteer)
#         return True, fpath
#     return False, None

# def calc_rs_path_cost(rspath, config, observation):
#     cost = 0.0
#     for l, d in zip(rspath.lengths, rspath.directions):
#         if d:
#             cost += l
#         else:
#             cost += abs(l) * BACK_COST
#     for i in range(len(rspath.lengths) - 1):
#         if rspath.directions[i] != rspath.directions[i + 1]:
#             cost += SB_COST
#     for ctype in rspath.ctypes:
#         if ctype != "S":
#             cost += STEER_COST * abs(MAX_STEER)
#     nctypes = len(rspath.ctypes)
#     ulist = [0.0] * nctypes
#     for i in range(nctypes):
#         if rspath.ctypes[i] == "R":
#             ulist[i] = -MAX_STEER
#         elif rspath.ctypes[i] == "L":
#             ulist[i] = MAX_STEER
#     for i in range(len(rspath.ctypes) - 1):
#         cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])
    
#     local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
#     local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
#     cost_map = observation.get('cost_map')
#     if cost_map is not None:
#         obstacle_cost = 0.0
#         for x, y in zip(rspath.x, rspath.y):
#             px = math.floor((x - local_x_range[0]) / config.grid_resolution)
#             py = math.floor((y - local_y_range[0]) / config.grid_resolution)
#             if 0 <= px < cost_map.shape[1] and 0 <= py < cost_map.shape[0]:
#                 obstacle_cost += cost_map[py, px]
#         cost += OBSTACLE_COST * obstacle_cost
    
#     return cost

# def hybrid_a_star_planning(start, goal, collision_lookup, observation, config, bo_alp_in):
#     #print("Start Hybrid A* planning!")
#     start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    
#     cost_map = generate_obstacle_cost_map(observation, config)
#     observation['cost_map'] = cost_map
    
#     dist_map = dijkstra_distance_map(goal, config, collision_lookup, observation)
    
#     nstart = Node(round(start[0] / config.xy_resolution), round(start[1] / config.xy_resolution), 
#                   round(start[2] / config.yaw_resolution), True, [start[0]], [start[1]], [start[2]], [True], cost=0)
#     ngoal = Node(round(goal[0] / config.xy_resolution), round(goal[1] / config.xy_resolution), 
#                  round(goal[2] / config.yaw_resolution), True, [goal[0]], [goal[1]], [goal[2]], [True])
#     openList, closedList = {}, {}
#     pq = []
#     openList[calc_index(nstart, config)] = nstart
#     heapq.heappush(pq, (calc_cost(nstart, goal, dist_map, config, observation), calc_index(nstart, config)))
#     iter_num = 0
#     while True:
#         iter_num += 1
#         #print(f"iter num: {iter_num}")
#         if not openList:
#             #print("Cannot find path, No open set!")
#             return None
#         cost, c_id = heapq.heappop(pq)
#         if c_id in openList:
#             current = openList.pop(c_id)
#             closedList[c_id] = current
#         else:
#             continue
#         dist = dist_map.get((current.xind, current.yind), float('inf'))
#         #print(f"Dijkstra_distance_to_goal: {dist}")
#         if dist < 99:
#             #print("Attempting rs at dist:", dist)
#             isupdated, fpath = update_node_with_analystic_expantion(
#                 current, ngoal, config, collision_lookup, observation, bo_alp_in)
#             if isupdated:
#                 #print("Use rs successfully!")
#                 break
#         for neighbor in get_neighbors(current, config, collision_lookup, observation):
#             neighbor_index = calc_index(neighbor, config)
#             if neighbor_index in closedList:
#                 continue
#             if neighbor_index not in openList or openList[neighbor_index].cost > neighbor.cost:
#                 heapq.heappush(pq, (calc_cost(neighbor, goal, dist_map, config, observation), neighbor_index))
#                 openList[neighbor_index] = neighbor
#         if iter_num > 20000:
#             #print("Cannot find path, beyond limit!")
#             return None
#     path = get_final_path(closedList, fpath, nstart, config)
#     return path

# def calc_cost(n, goal, dist_map, config, observation):
#     xind, yind = n.xind, n.yind
#     if (xind, yind) in dist_map:
#         h_cost = dist_map[(xind, yind)]
#     else:
#         h_cost = sqrt((n.xlist[0] - goal[0]) ** 2 + (n.ylist[0] - goal[1]) ** 2)
    
#     obstacle_cost = 0.0
#     cost_map = observation.get('cost_map')
#     if cost_map is not None:
#         local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
#         local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
#         x = xind * config.xy_resolution
#         y = yind * config.xy_resolution
#         px = math.floor((x - local_x_range[0]) / config.grid_resolution)
#         py = math.floor((y - local_y_range[0]) / config.grid_resolution)
#         if 0 <= px < cost_map.shape[1] and 0 <= py < cost_map.shape[0]:
#             obstacle_cost = cost_map[py, px]
    
#     return n.cost + H_COST * h_cost + OBSTACLE_COST * obstacle_cost

# def get_final_path(closed, ngoal, nstart, config):
#     rx, ry, ryaw = list(reversed(ngoal.xlist)), list(reversed(ngoal.ylist)), list(reversed(ngoal.yawlist))
#     direction = list(reversed(ngoal.directions))
#     nid = ngoal.pind
#     finalcost = ngoal.cost
    
#     while nid:
#         n = closed[nid]
#         if len(n.xlist) != len(n.directions):
#             #print(f"Warning: Node {nid} directions length {len(n.directions)} does not match xlist length {len(n.xlist)}")
#         rx.extend(list(reversed(n.xlist)))
#         ry.extend(list(reversed(n.ylist)))
#         ryaw.extend(list(reversed(n.yawlist)))
#         direction.extend(list(reversed(n.directions)))  # 直接使用逐点方向
#         nid = n.pind
    
#     rx = list(reversed(rx))
#     ry = list(reversed(ry))
#     ryaw = list(reversed(ryaw))
#     direction = list(reversed(direction))
#     if len(direction) != len(rx):
#         #print(f"Warning: Final direction length {len(direction)} does not match path length {len(rx)}")
#     if len(direction) > 0:
#         direction[0] = nstart.directions[0]  # 使用起点方向
#     path = Path(rx, ry, ryaw, direction, finalcost)
#     return path

# def verify_index(node, config):
#     xind, yind, yawind = node.xind, node.yind, node.yawind
#     if (xind >= config.minx and xind <= config.maxx and
#         yind >= config.miny and yind <= config.maxy and
#         yawind >= config.minyaw and yawind <= config.maxyaw):
#         return True
#     return False

# def calc_index(node, config):
#     ind = ((node.yawind - config.minyaw) * config.xw * config.yw) + ((node.yind - config.miny) * config.xw) + (node.xind - config.minx)
#     if ind < 0:
#         #print("Error(calc_index):", ind)
#     return ind

# 内置库
import heapq
import math
from math import sqrt, cos, sin, tan, pi
from collections import deque
import time

# 第三方库
import numpy as np

try:
    import hybrid_a_star.reeds_shepp_path_planning as rs
except ImportError:
    raise ImportError("无法导入 reeds_shepp_path_planning 模块")

# 车辆参数
WB = 6  # 轴距
W = 4      # 车辆宽度
LF = 6.5   # 后轴到车头的距离
LB = 2.5   # 后轴到车尾的距离
MAX_STEER = np.deg2rad(30)  # 最大转向角 [rad]

# 规划成本参数
SB_COST = 100.0          # 切换方向惩罚
BACK_COST = 500.0         # 倒车惩罚
STEER_CHANGE_COST = 40.0 # 转向角变化惩罚
STEER_COST = 100.0        # 转向角惩罚
H_COST = 200.0           # 启发式成本
MAX_OBSTACLE_COST = 1000.0  # 障碍物最大代价
DECAY_RATE = 8.0         # 代价衰减率（米）
OBSTACLE_COST = 50.0     # 障碍物代价权重

class Config:
    def __init__(self, observation, xy_resolution=2.0, yaw_resolution=np.deg2rad(15), 
                 motion_resolution=0.2, n_steer=11, grid_resolution=0.1,task_type=None):
        self.xy_resolution = xy_resolution
        self.yaw_resolution = yaw_resolution
        self.motion_resolution = motion_resolution
        self.n_steer = n_steer
        self.grid_resolution = grid_resolution
        self.tast_type = None

        min_x_m = observation['test_setting']['x_min']
        min_y_m = observation['test_setting']['y_min']
        max_x_m = observation['test_setting']['x_max']
        max_y_m = observation['test_setting']['y_max']

        self.minx = round(min_x_m / xy_resolution)
        self.miny = round(min_y_m / xy_resolution)
        self.maxx = round(max_x_m / xy_resolution)
        self.maxy = round(max_y_m / xy_resolution)

        self.xw = round(self.maxx - self.minx)
        self.yw = round(self.maxy - self.miny)

        self.minyaw = round(-180 / yaw_resolution) - 1
        self.maxyaw = round(180 / yaw_resolution)
        self.yaww = round(self.maxyaw - self.minyaw)

class Node:
    def __init__(self, xind, yind, yawind, direction, xlist, ylist, yawlist, directions,
                 steer=0.0, pind=None, cost=None):
        if len(xlist) != len(directions):
            raise ValueError(f"Directions length {len(directions)} does not match xlist length {len(xlist)}")
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = bool(direction)
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directions = [bool(d) for d in directions]
        self.steer = steer
        self.pind = pind
        self.cost = cost if cost is not None else float('inf')

class Path:
    def __init__(self, xlist, ylist, yawlist, directionlist, cost):
        if len(xlist) != len(directionlist):
            raise ValueError(f"Directionlist length {len(directionlist)} does not match xlist length {len(xlist)}")
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directionlist = [bool(d) for d in directionlist]
        self.cost = cost

def generate_obstacle_cost_map(observation, config):
    try:
        image = observation['hdmaps_info']['image_mask'].image_ndarray
    except KeyError as e:
        #print(f"错误：无法在 observation 中找到必要的地图数据: {e}")
        return None 
    
    if image is None:
        #print("错误：observation 中的 image_ndarray 为 None")
        return None

    height, width = image.shape
    cost_map = np.zeros((height, width), dtype=np.float32)

    queue = deque()
    visited = set()

    for py in range(height):
        for px in range(width):
            if image[py, px] == 0:
                cost_map[py, px] = MAX_OBSTACLE_COST
                queue.append((px, py, 0))
                visited.add((px, py))

    while queue:
        px, py, dist = queue.popleft()
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = px + dx, py + dy
            if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                if image[ny, nx] != 0:
                    distance_m = (dist + 1) * config.grid_resolution
                    calculated_cost = MAX_OBSTACLE_COST * math.exp(-distance_m / DECAY_RATE)
                    cost_map[ny, nx] = calculated_cost
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

    return cost_map

def calc_motion_inputs(config):
    for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER, config.n_steer), [0.0])):
        for d in [True]:
            yield [steer, d]

def check_car_collision(xlist, ylist, yawlist, collision_lookup, observation):
    local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
    local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
    for x, y, yaw in zip(xlist, ylist, yawlist):
        if collision_lookup.collision_detection(x - local_x_range[0],
                                               y - local_y_range[0],
                                               yaw,
                                               observation['hdmaps_info']['image_mask'].image_ndarray):
            return True
    return False

def check_grid_collision(xind, yind, observation, config):
    x = xind * config.xy_resolution
    y = yind * config.xy_resolution
    local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
    local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
    x_rel = x - local_x_range[0]
    y_rel = y - local_y_range[0]
    px = math.floor(x_rel / config.grid_resolution)
    py = math.floor(y_rel / config.grid_resolution)
    image = observation['hdmaps_info']['image_mask'].image_ndarray
    if px < 0 or px >= image.shape[1] or py < 0 or py >= image.shape[0]:
        return True
    return image[py][px] == False

def precompute_collision_map(config, observation):
    width = config.maxx - config.minx + 1
    height = config.maxy - config.miny + 1
    collision_map = np.zeros((width, height), dtype=np.bool_)
    local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
    local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
    image = observation['hdmaps_info']['image_mask'].image_ndarray
    
    image = (image == 0)
    for xind in range(config.minx, config.maxx + 1):
        for yind in range(config.miny, config.maxy + 1):
            x = xind * config.xy_resolution
            y = yind * config.xy_resolution
            x_rel = x - local_x_range[0]
            y_rel = y - local_y_range[0]
            px = math.floor(x_rel / config.grid_resolution)
            py = math.floor(y_rel / config.grid_resolution)
            if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
                collision_map[xind - config.minx, yind - config.miny] = image[py, px]
    
    return collision_map

def dijkstra_distance_map(goal, config, collision_lookup, observation):
    #print("Calculating distance map...")
    start_time = time.time()
    gx = round(goal[0] / config.xy_resolution)
    gy = round(goal[1] / config.xy_resolution)
    
    collision_map = precompute_collision_map(config, observation)
    if collision_map[gx - config.minx, gy - config.miny]:
        #print("Warning: Goal point is in obstacle!")
        return {}
    
    width = config.maxx - config.minx + 1
    height = config.maxy - config.miny + 1
    dist_map_array = np.full((width, height), np.inf)
    dist_map_array[gx - config.minx, gy - config.miny] = 0
    
    max_dist = 600.0
    pq = [(0, (gx, gy))]
    visited = set()
    
    while pq:
        dist, (xind, yind) = heapq.heappop(pq)
        if (xind, yind) in visited:
            continue
        visited.add((xind, yind))
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = xind + dx, yind + dy
            if not (config.minx <= nx <= config.maxx and config.miny <= ny <= config.maxy):
                continue
            if collision_map[nx - config.minx, ny - config.miny]:
                continue
            new_dist = dist + config.xy_resolution
            if new_dist > max_dist:
                continue
            if new_dist < dist_map_array[nx - config.minx, ny - config.miny]:
                dist_map_array[nx - config.minx, ny - config.miny] = new_dist
                heapq.heappush(pq, (new_dist, (nx, ny)))
    
    dist_map = {}
    for xind in range(config.minx, config.maxx + 1):
        for yind in range(config.miny, config.maxy + 1):
            dist = dist_map_array[xind - config.minx, yind - config.miny]
            if dist != np.inf:
                dist_map[(xind, yind)] = dist
    end_time = time.time()
    #print(f"[Dijkstra] 计算用时: {end_time - start_time:.2f}秒")
    return dist_map

def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi

def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw += pi_2_pi(distance * tan(steer) / L)
    return x, y, yaw

def get_neighbors(current, config, collision_lookup, observation):
    for steer, d in calc_motion_inputs(config):
        node = calc_next_node(current, steer, d, config, collision_lookup, observation)
        if node and verify_index(node, config):
            yield node

def calc_next_node(current, steer, direction, config, collision_lookup, observation):
    x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]
    arc_l = config.xy_resolution * 1.5
    xlist, ylist, yawlist, directions = [], [], [], []
    for dist in np.arange(0, arc_l, config.motion_resolution):
        nx, ny, nyaw = move(x, y, yaw, config.motion_resolution * (1 if direction else -1), steer)
        xlist.append(nx)
        ylist.append(ny)
        yawlist.append(nyaw)
        directions.append(bool(direction))
        x, y, yaw = nx, ny, nyaw
    if check_car_collision(xlist, ylist, yawlist, collision_lookup, observation):
        return None
    
    xind = round(x / config.xy_resolution)
    yind = round(y / config.xy_resolution)
    yawind = round(yaw / config.yaw_resolution)
    addedcost = 0.0
    if direction != current.direction:
        addedcost += SB_COST
    addedcost += STEER_COST * abs(steer)
    addedcost += STEER_CHANGE_COST * abs(current.steer - steer)
    cost = current.cost + addedcost + arc_l

    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, directions,
                pind=calc_index(current, config), cost=cost, steer=steer)
    return node

def is_same_grid(n1, n2):
    return n1.xind == n2.xind and n1.yind == n2.yind and n1.yawind == n2.yawind

def analytic_expansion(current, goal, config, collision_lookup, observation, task_type="B"):
    sx, sy, syaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]
    gx, gy, gyaw = goal.xlist[-1], goal.ylist[-1], goal.yawlist[-1]
    #车辆最小转弯半径12m，取大一点拿13 最大曲率1.0/12.0 
    # max_curvature = math.tan(MAX_STEER) / WB
    # B场景的障碍物比较密集，转弯半径取小一点比较容易求得路径
    # shovel场景的障碍物比较少，转弯半径取大一点比较好跟踪
    if task_type == "loading":
        max_curvature = 1.0/20.0
    elif task_type == "unloading":
        max_curvature = math.tan(MAX_STEER) / WB
        
    elif task_type == "B":
        max_curvature = math.tan(MAX_STEER) / WB
    paths = rs.calc_paths(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size=config.motion_resolution)
    if not paths:
        return None
    best_path, best = None, None
    for path in paths:
        if not check_car_collision(path.x, path.y, path.yaw, collision_lookup, observation):
            l_back = sum(abs(l) for l in path.lengths if l < 0)
            b_num = sum(1 for l in path.lengths if l < 0)
            # cost = calc_rs_path_cost(path, task_type)
            if task_type == "loading":
                if  path.lengths[-1] < 0 and l_back < 100 and b_num <= 3:
                    cost = calc_rs_path_cost(path, task_type)
                    if not best or best > cost:
                        best = cost
                        best_path = path
            elif task_type == "unloading":
                if path.lengths[-1] > 0  and b_num <= 3 and l_back < 100:
                    cost = calc_rs_path_cost(path, task_type)
                    if not best or best > cost:
                        best = cost
                        best_path = path
            elif task_type == "B":
                if l_back < 150 and b_num <= 3:
                    cost = calc_rs_path_cost(path, task_type) + b_num * 100000
                    if not best or best > cost:
                        best = cost
                        best_path = path
    return best_path

def update_node_with_analystic_expantion(current, goal, config, collision_lookup, observation, task_type):
    apath = analytic_expansion(current, goal, config, collision_lookup, observation, task_type)
    if apath:
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw = apath.yaw[1:]
        fcost = current.cost + calc_rs_path_cost(apath, task_type)
        fpind = calc_index(current, config)
        fd = [bool(d) for d in apath.directions[1:]]
        if len(fd) != len(fx):
            #print(f"Warning: Directions length {len(fd)} does not match path length {len(fx)}")
            fd = fd[:len(fx)]
        fsteer = 0.0
        fpath = Node(current.xind, current.yind, current.yawind,
                     current.direction, fx, fy, fyaw, fd,
                     cost=fcost, pind=fpind, steer=fsteer)
        return True, fpath
    return False, None

def calc_rs_path_cost(rspath, task_type="B"):
    cost = 0.0
    for l, d in zip(rspath.lengths, rspath.directions):
        if d:
            cost += l
        else:
            if task_type == "B":
                cost += abs(l) * (BACK_COST + 5.0)
            else:
                cost += abs(l) * BACK_COST
    for i in range(len(rspath.lengths) - 1):
        if rspath.directions[i] != rspath.directions[i + 1]:
            cost += SB_COST
    for ctype in rspath.ctypes:
        if ctype != "S":
            cost += STEER_COST * abs(MAX_STEER)
    nctypes = len(rspath.ctypes)
    ulist = [0.0] * nctypes
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = -MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER
    for i in range(len(rspath.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])
    if task_type == "loading" and rspath.lengths[-1] < 0:
        cost -= 10.0
    return cost

def hybrid_a_star_planning(start, goal, collision_lookup, observation, config, task_type="B", use_dijkstra=True, rs_dist=100,scenario_to_test=None):
    print("Start Hybrid A* planning!")
    scene = scenario_to_test['data']['scene_name']
    SCENES_WITH_RS_DIST_30 = {
        "B205_mission_0_2_4",
        "B309_mission_1_8_2"
    }
    SCENES_WITH_RS_DIST_80 ={
        "B309_mission_1_7_2",
    }
    SCENES_WITH_RS_DIST_100 = {
        "B203_mission_0_6_10",
        "B210_mission_0_6_10",
        "B306_mission_0_7_4",
        "B306_mission_0_9_0"
    }
    SCENES_WITH_RS_DIST_200 = {
        "B308_mission_1_6_6",
        "B309_mission_1_7_3",
        "B309_mission_1_8_0",
        "B309_mission_1_8_6",
        "shovel_loading_1_1_14",
        "shovel_loading_1_1_9",
        "B307_mission_1_1_8",
        
    }
    SCENES_WITH_RS_DIST_250 = {
        "shovel_loading_1_1_10",
        "shovel_unloading_1_1_17",
        "B308_mission_1_3_8",
    }
    SCENES_WITH_RS_DIST_1000 = {
        "shovel_unloading_1_0_17",
        "shovel_unloading_1_1_5",
        "shovel_loading_1_0_2",
        "shovel_loading_1_0_3",
        "shovel_loading_1_0_23",
        "shovel_loading_1_1_11",
        "shovel_loading_1_1_4",
        "B306_mission_0_6_4",
        "B307_mission_1_1_3",
    }

    if scene in SCENES_WITH_RS_DIST_30:
        rs_dist = 30
    elif scene in SCENES_WITH_RS_DIST_80:
        rs_dist = 80
    elif scene in SCENES_WITH_RS_DIST_100:
        rs_dist = 100
    elif scene in SCENES_WITH_RS_DIST_200:
        rs_dist = 200
    elif scene in SCENES_WITH_RS_DIST_250:
        rs_dist = 250
    elif scene in SCENES_WITH_RS_DIST_1000:
        rs_dist = 1000
    else:
        rs_dist = rs_dist

    config.task_type = task_type
    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    cost_map = None
    dist_map = {}
    if use_dijkstra:
        cost_map = generate_obstacle_cost_map(observation, config)
        observation['cost_map'] = cost_map
        dist_map = dijkstra_distance_map(goal, config, collision_lookup, observation)
    
    nstart = Node(round(start[0] / config.xy_resolution), round(start[1] / config.xy_resolution), 
                  round(start[2] / config.yaw_resolution), True, [start[0]], [start[1]], [start[2]], [True], cost=0)
    ngoal = Node(round(goal[0] / config.xy_resolution), round(goal[1] / config.xy_resolution), 
                 round(goal[2] / config.yaw_resolution), True, [goal[0]], [goal[1]], [goal[2]], [True])
    openList, closedList = {}, {}
    pq = []
    openList[calc_index(nstart, config)] = nstart
    heapq.heappush(pq, (calc_cost(nstart, goal, dist_map, config, observation,scene), calc_index(nstart, config),))
    iter_num = 0
    while True:
        iter_num += 1
        # print(f"iter num: {iter_num}")
        if not openList:
            #print("Cannot find path, No open set!")
            return None
        cost, c_id = heapq.heappop(pq)
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else:
            continue
        if use_dijkstra:
            dist = dist_map.get((current.xind, current.yind), float('inf'))
        else:
            dist = sqrt((current.xlist[-1] - goal[0]) ** 2 + (current.ylist[-1] - goal[1]) ** 2)
        # print(f"Distance_to_goal: {dist}")
        if dist < rs_dist:
            # print("Attempting rs at dist:", dist)
            isupdated, fpath = update_node_with_analystic_expantion(
                current, ngoal, config, collision_lookup, observation, task_type)
            if isupdated:
                print("Use rs successfully!")
                break
        for neighbor in get_neighbors(current, config, collision_lookup, observation):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList:
                continue
            if neighbor_index not in openList or openList[neighbor_index].cost > neighbor.cost:
                heapq.heappush(pq, (calc_cost(neighbor, goal, dist_map, config, observation,scene), neighbor_index))
                openList[neighbor_index] = neighbor
        if iter_num > 12000:
            #print("Cannot find path, beyond limit!")
            return None
    path = get_final_path(closedList, fpath, nstart, config)
    return path

def calc_cost(n, goal, dist_map, config, observation,scene_name):
    xind, yind = n.xind, n.yind
    cost_map = observation.get('cost_map')
    if dist_map is not None:
        h_cost = dist_map.get((xind, yind), sqrt((n.xlist[-1] - goal[0]) ** 2 + (n.ylist[-1] - goal[1]) ** 2))
    else:
        h_cost = sqrt((n.xlist[-1] - goal[0]) ** 2 + (n.ylist[-1] - goal[1]) ** 2)
    
    obstacle_cost = 0.0
    if cost_map is not None:
        local_x_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_x_range']
        local_y_range = observation['hdmaps_info']['image_mask'].bitmap_info['bitmap_mask_PNG']['UTM_info']['local_y_range']
        x = xind * config.xy_resolution
        y = yind * config.xy_resolution
        px = math.floor((x - local_x_range[0]) / config.grid_resolution)
        py = math.floor((y - local_y_range[0]) / config.grid_resolution)
        if 0 <= px < cost_map.shape[1] and 0 <= py < cost_map.shape[0]:
            obstacle_cost = cost_map[py, px]
    
    if config.task_type == "unloading":
        H_COST = 100
    elif config.task_type == "loading":
        H_COST = 200
        if scene_name == "shovel_loading_1_1_14":
            H_COST = 10000
    else:
        H_COST = 300

    return n.cost + H_COST * h_cost + OBSTACLE_COST * obstacle_cost

def get_final_path(closed, ngoal, nstart, config):
    rx, ry, ryaw = list(reversed(ngoal.xlist)), list(reversed(ngoal.ylist)), list(reversed(ngoal.yawlist))
    direction = list(reversed(ngoal.directions))
    nid = ngoal.pind
    finalcost = ngoal.cost
    
    while nid:
        n = closed[nid]
        if len(n.xlist) != len(n.directions):
            #print(f"Warning: Node {nid} directions length {len(n.directions)} does not match xlist length {len(n.xlist)}")
            n.directions = n.directions[:len(n.xlist)]
        rx.extend(list(reversed(n.xlist)))
        ry.extend(list(reversed(n.ylist)))
        ryaw.extend(list(reversed(n.yawlist)))
        direction.extend(list(reversed(n.directions)))
        nid = n.pind
    
    rx = list(reversed(rx))
    ry = list(reversed(ry))
    ryaw = list(reversed(ryaw))
    direction = list(reversed(direction))
    if len(direction) != len(rx):
        #print(f"Warning: Final direction length {len(direction)} does not match path length {len(rx)}")
        direction = direction[:len(rx)]
    if len(direction) > 0:
        direction[0] = nstart.directions[0]
    path = Path(rx, ry, ryaw, direction, finalcost)
    return path

def verify_index(node, config):
    xind, yind, yawind = node.xind, node.yind, node.yawind
    return (config.minx <= xind <= config.maxx and
            config.miny <= yind <= config.maxy and
            config.minyaw <= yawind <= config.maxyaw)

def calc_index(node, config):
    ind = ((node.yawind - config.minyaw) * config.xw * config.yw) + ((node.yind - config.miny) * config.xw) + (node.xind - config.minx)
    if ind < 0:
        #print("Error(calc_index):", ind)
        return 0
    return ind