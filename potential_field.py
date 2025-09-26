import numpy as np

from globals import *

def potential_field(vehicle, neighbors, obstacles):
    # 采用改进的势场法，取公式n=1
    k_att = 5  # 引力系数
    k_lane = 0.01  # 道路中心的吸引力系数
    k_ob = 20  # 障碍物斥力系数
    k_edge = 50  # 道路边界斥力系数
    potential_field_range = 2  # 车与障碍物的临界距离，即小于这个距离势场法才会生效
    potential_field_range_road = 0.5  # 车与道路边界的临界距离，即小于这个距离势场法才会生效
    dis_APF = 2  # 这个本来是2，先减0看看效果
    F_rep = np.array([0.0, 0.0])

    relative_goal = vehicle.goal - vehicle.position
    distance_goal = np.linalg.norm(relative_goal) - vehicle_radius * 2  # 车与目标点的距离，目前暂时设置成固定目标点

    # 车辆避撞
    if neighbors:
        for neighbor in neighbors:
            relative_position = neighbor.position - vehicle.position
            distance = np.linalg.norm(relative_position) - vehicle_radius * 2 - 0  # 车与车的距离，后续可能需要考虑实际不同转角下真实距离
            if distance < potential_field_range:
                # 障碍物的斥力1,方向由障碍物指向车辆
                F_rep_ob1_abs = k_ob * (1 / distance - 1 / potential_field_range) * distance_goal / distance ** 2  # 回看公式设定n=1
                F_rep_ob1 = - F_rep_ob1_abs * relative_position / np.linalg.norm(relative_position)

                # # 障碍物的斥力2，方向由车辆指向目标点
                # F_rep_ob2_abs = 0.5 * k_ob * (1 / distance - 1 / potential_field_range) ** 2
                # F_rep_ob2 = F_rep_ob2_abs * relative_goal / np.linalg.norm(relative_goal)
                # # 改进后的障碍物和斥力计算
                # F_rep += F_rep_ob1 + F_rep_ob2

                F_rep += F_rep_ob1

    # # 障碍物避撞
    # if obstacles:
    #     for obstacle in obstacles:
    #         relative_position = obstacle.position - vehicle.position
    #         distance = np.linalg.norm(relative_position) - vehicle_radius - obstacle.radius - dis_APF  # 车与障碍物的距离
    #         if distance < potential_field_range:
    #             # 障碍物的斥力1,方向由障碍物指向车辆
    #             F_rep_ob1_abs = k_ob * (1 / distance - 1 / potential_field_range) * distance_goal / distance ** 2  # 回看公式设定n=1
    #             F_rep_ob1 = - F_rep_ob1_abs * relative_position / np.linalg.norm(relative_position)
    #
    #             # # 障碍物的斥力2，方向由车辆指向目标点
    #             # F_rep_ob2_abs = 0.5 * k_ob * (1 / distance - 1 / potential_field_range) ** 2
    #             # F_rep_ob2 = F_rep_ob2_abs * relative_goal / np.linalg.norm(relative_goal)
    #             #
    #             # # 改进后的障碍物和斥力计算
    #             # F_rep += F_rep_ob1 + F_rep_ob2
    #
    #             F_rep += F_rep_ob1

    # 吸引力，这一块我先不加了，因为原先的势场法需要靠吸引力让汽车不断前进，但是由于有一致性算法在驱动车辆前进所以应该不需要吸引力，后续如果不对再加上

    # 车道避撞，不仅要考虑车道距离，还要考虑当前纵向车速！！
    closest_lane_center = min(lane_centers, key = lambda center: abs(center - vehicle.position[1]))  # 最近的车道中心
    relative_position = np.array([0, closest_lane_center - vehicle.position[1]])
    relative_velocity = np.array([0, 0 - vehicle.velocity[1]])
    F_rep += k_lane * np.linalg.norm(relative_position) ** 2 * relative_position + k_lane * relative_velocity * 2

    # # 道路边界避撞，还要改
    # relative_position = vehicle.position - np.array([vehicle.position[0], lane_max])
    # disObsTmp = np.linalg.norm(relative_position) - vehicle_radius  # 车与道路边界的距离
    # if disObsTmp < potential_field_range_road:
    #     F1Tmp = k_edge * (1 / disObsTmp - 1 / potential_field_range_road) / disObsTmp / disObsTmp
    #     F_rep += F1Tmp * np.array([0, -1])
    #
    # relative_position = vehicle.position - np.array([vehicle.position[0], lane_min])
    # disObsTmp = np.linalg.norm(relative_position) - vehicle_radius  # 车与道路边界的距离
    # if disObsTmp < potential_field_range_road:
    #     F1Tmp = k_edge * (1 / disObsTmp - 1 / potential_field_range_road) / disObsTmp / disObsTmp
    #     F_rep += F1Tmp * np.array([0, 1])

    return F_rep