import numpy as np

from globals import *

# 定义变道函数
def lane_change(vehicle, lane_id, style = 1, time_lane_change = 5, state_desire = None):
    # lane_id 是变道后的车道
    t = int(time_lane_change / dt) + 1  # 变道的时间步数
    state = np.zeros((t, 4))  # 车辆状态临时矩阵，想不到合适的办法了只能这样绕一下

    # 变道前车辆状态
    state[0][0] = vehicle.position[0]
    state[0][1] = vehicle.position[1]
    state[0][2] = vehicle.velocity[0]
    state[0][3] = vehicle.velocity[1]

    # 变道后车辆状态，如果不特意指定为按期望速度行驶变道时间后的状态
    if state_desire:
        state[t - 1] = state_desire
    elif style == 1:
        state[t - 1][0] = vehicle.position[0] + velocity_desire * dt * (t - 1)
        state[t - 1][1] = lane_id * 4 - 2
        state[t - 1][2] = velocity_desire
        state[t - 1][3] = 0
    elif style == 2:
        state[t - 1][0] = vehicle.position[0] + velocity_desire * dt * (t - 1)
        state[t - 1][1] = lane_id * 4 - 2
        state[t - 1][2] = velocity_desire
        state[t - 1][3] = 0
        vehicle.delta[1] += 4

    # B1是横向的矩阵
    B1 = np.array([[state[t - 1][0] - state[0][0] - time_lane_change * state[0][2]],
               [state[t - 1][2] - state[0][2]],
               [0]])

    # B2是纵向的矩阵
    B2 = np.array([[state[t - 1][1] - state[0][1]],
               [0],
               [0]])

    x_parameter = np.zeros(6)  # 五次函数参数拟合之横坐标
    y_parameter = np.zeros(6)  # 五次函数参数拟合之纵坐标
    x_value = np.zeros(t)  # 五次函数值之横坐标
    y_value = np.zeros(t)  # 五次函数值之纵坐标

    x_parameter[0] = state[0][0]
    x_parameter[1] = state[0][2]
    y_parameter[0] = state[0][1]
    [x_parameter[3], x_parameter[4], x_parameter[5]] = np.dot(A_pinv, B1)
    [y_parameter[3], y_parameter[4], y_parameter[5]] = np.dot(A_pinv, B2)

    for i in range(t):
        for j in range(6):
            x_value[i] += x_parameter[j] * (i * dt) ** j
            y_value[i] += y_parameter[j] * (i * dt) ** j

    for i in range(t - 1):
        state[i + 1][0] = x_value[i + 1]
        state[i + 1][1] = y_value[i + 1]
        state[i + 1][2] = (x_value[i + 1] - x_value[i]) / dt
        state[i + 1][3] = (y_value[i + 1] - y_value[i]) / dt

    return state

# 定义变道后的道路选择函数
def lane_evaluate(vehicle, neighbors, obstacles):
    # current_lane = vehicle.lane
    #
    # # candidates = []
    # # # 可用车道检查
    # # if current_lane > 1:
    # #     candidates.append(current_lane - 1)  # 右侧车道
    # # if current_lane < len(lane_centers):
    # #     candidates.append(current_lane + 1)  # 左侧车道
    #
    # candidates = list(range(1, len(lane_centers) + 1))
    #
    # best_lane = current_lane
    # best_score = -float('inf')
    #
    # for lane in candidates:
    #     score = 0
    #
    #     # 计算车道中心坐标
    #     lane_y = lane_centers[lane - 1]
    #
    #     # 1.障碍物检查
    #     obs_front = [o for o in obstacles if abs(o.position[1] - lane_y) < 2]
    #     if any(0 <= obs.position[0] - vehicle.position[0] < 50 for obs in obs_front):
    #         score -= 500
    #     if any(50 <= obs.position[0] - vehicle.position[0] < 100 for obs in obs_front):
    #         score -= 200
    #
    #     # 2.车辆密度检查
    #     vehicles_front = [v for v in neighbors
    #                         if abs(v.position[1] - lane_y) < 2
    #                         and v.position[0] > vehicle.position[0]
    #                         and v.status == 0]
    #     density = len(vehicles_front)
    #     score -= density * 25
    #
    #     # 3.速度优势检查
    #     closest_vehicle = None
    #     min_distance = float('inf')
    #
    #     for v in neighbors:
    #         if abs(v.position[1] - lane_y) < 2 and v.status == 0:  # 判断是否在目标车道
    #             distance = v.position[0] - vehicle.position[0]  # 计算纵向距离
    #             if distance > 0 and distance < min_distance:  # 只考虑前方车辆
    #                 min_distance = distance
    #                 closest_vehicle = v
    #
    #     if closest_vehicle:
    #         closest_speed = closest_vehicle.velocity[0]  # 最近车辆的速度
    #     else:
    #         closest_speed = velocity_desire  # 如果目标车道没有车辆，使用期望速度
    #
    #     speed_advantage = closest_speed - vehicle.velocity[0]  # 速度差值
    #     score += speed_advantage * 50  # 速度优势评分
    #
    #     # 4. 跨车道惩罚
    #     lane_change_distance = abs(lane - current_lane)  # 跨车道的数量
    #     score -= lane_change_distance * 50  # 跨车道越多，惩罚越大
    #
    #     # 5. 编队距离惩罚
    #     platoon_vehicles = [v for v in neighbors if v.status == 1]  # 编队中的其他车辆
    #     if platoon_vehicles:
    #         # 计算变道后与编队车辆的平均距离
    #         avg_distance = np.mean([np.linalg.norm([vehicle.position[0], lane_y] - v.position)
    #                                 for v in platoon_vehicles])
    #         score -= avg_distance * 0.5  # 距离越远，惩罚越大
    #
    #     if score > best_score:
    #         best_score = score
    #         best_lane = lane
    #
    # return best_lane

    return vehicle.lane + 1

# def lane_allocate(vehicles, obstacles):
#     # 记录每个车道的占用情况
#     lane_occupancy = {lane: False for lane in lane_centers}
#
#     # 按优先级排序
#     priority_list = []
#     for vehicle in vehicles:
#         if vehicle.flag == 1:  # 需要变道的车辆
#             # 计算优先级（距离障碍物越近，优先级越高）
#             closest_obstacle_distance = min(
#                 [obs.position[0] - vehicle.position[0] for obs in obstacles if obs.lane == vehicle.lane],
#                 default=float('inf')
#             priority = -closest_obstacle_distance  # 距离越近，优先级越高
#             priority_list.append((priority, vehicle))
#
#     # 按优先级排序
#     priority_list.sort(key=lambda x: x[0], reverse=True)
#
#     # 分配车道
#     for _, vehicle in priority_list:
#         target_lane = None
#         # 获取可用车道
#         available_lanes = []
#         if vehicle.lane > 1:
#             available_lanes.append(vehicle.lane - 1)  # 左侧车道
#         if vehicle.lane < len(lane_centers):
#             available_lanes.append(vehicle.lane + 1)  # 右侧车道
#
#         # 选择未被占用的车道
#         for lane in available_lanes:
#             if not lane_occupancy[lane_centers[lane - 1]]:
#                 target_lane = lane
#                 lane_occupancy[lane_centers[lane - 1]] = True  # 标记车道为占用
#                 break
#
#         # 如果没有可用车道，选择次优车道（可能需要等待）
#         if target_lane is None:
#             target_lane = available_lanes[0]  # 默认选择第一个可用车道
#             vehicle.wait_counter += 1  # 增加等待计数器
#             if vehicle.wait_counter > 10:  # 等待超过一定时间后强制变道
#                 lane_occupancy[lane_centers[target_lane - 1]] = True
#                 vehicle.wait_counter = 0
#
#         # 分配目标车道
#         vehicle.target_lane = target_lane