import numpy as np
import matplotlib.pyplot as plt
import math
import random

from globals import *
from consensus import consensus
from potential_field import potential_field

# 定义车辆类
class Vehicle:
    def __init__(self, position, velocity, delta, goal, auto = 1, status = 1):
        self.position = np.array(position, dtype = np.float64)  # 车辆位置
        self.velocity = np.array(velocity, dtype = np.float64)  # 车辆速度
        self.accel = np.array([0,0], dtype = np.float64)
        self.delta = np.array(delta, dtype = np.float64)  # 车辆在编队中的相对位置，我感觉这个是需要计算的，但还是先人为给定一下
        self.goal = np.array(goal, dtype = np.float64)  #车辆的目标终点，我不知道需不需要这个
        self.radius = vehicle_radius # 车辆半径
        self.auto = auto  # 是否是自动驾驶车辆，1表示是自动驾驶车，0表示是人工车
        self.status = status  # 是否在编队中，0表示不在编队中，1表示在编队中，方便编队的split和merge
        self.number = 0  # 编队中的序号，可能能用于后续编队队形改变时确定位置？
        self.lane = int(self.position[1] // 4 + 1)
        self.flag = 0  # 车辆行为标签，1为障碍物变道，2为超车变道，需要注意的是，如果车辆需要变道，会退出编队，所以status也要改变
        self.target_lane = None  # 目标车道
        self.wait_counter = 0  # 等待计数器

    def find_neighbors(self, all_vehicles, communication_range):
        """
        查找在通信范围内的邻居车辆，对于非编队车辆也要查找，只是不一起运行一致性算法
        """
        neighbors = []
        for vehicle in all_vehicles:
            if vehicle != self:
                distance = np.linalg.norm(self.position - vehicle.position)
                if distance <= communication_range:
                    neighbors.append(vehicle)
        return neighbors

    def find_obstacles(self, all_obstacles, communication_range):
        """
        查找在通信范围内的障碍物
        """
        near_obstacles = []
        for obstacle in all_obstacles:
            distance = np.linalg.norm(self.position - obstacle.position)
            if distance <= communication_range:
                near_obstacles.append(obstacle)
        return near_obstacles

    # 状态更新函数
    def update_state(self, neighbors, near_obstacles, leader):
        delta_v = np.array([0.0, 0.0])

        if self.status == 1:
            F_con = consensus(self, neighbors, leader)
            F_rep = potential_field(self, neighbors, near_obstacles)
            delta_v = F_con + F_rep
            # delta_v = consensus(self, neighbors, leader)

        if np.linalg.norm(delta_v) > 2:
            delta_v = delta_v * 2 / np.linalg.norm(delta_v)

        self.accel = delta_v

        self.velocity += delta_v

        if np.linalg.norm(self.velocity) > velocity_max:
            self.velocity = self.velocity * velocity_max / np.linalg.norm(self.velocity)

        self.position += self.velocity * dt
        self.lane = int(self.position[1] // 4 + 1)

        # 第一种变道情况：前面有障碍物
        if near_obstacles:
            for obstacle in near_obstacles:
                if obstacle.lane == self.lane:
                    if 0 <= obstacle.position[0] - self.position[0] <= 50:
                        self.flag = 1
                        self.status = 0

        # 第二种变道情况：前车速度过慢，找一条快速路通过
        if neighbors:
            for neighbor in neighbors:
                if neighbor.lane == self.lane and neighbor.auto == 0:
                    if 0 <= neighbor.position[0] - self.position[0] <= 20 and 1 <= self.velocity[0] - neighbor.velocity[0]:
                        self.flag = 2
                        self.status = 0

    def plot(self):
        theta = np.arctan2(self.velocity[1], self.velocity[0])
        # 旋转矩阵
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        # 尺寸矩阵
        V = np.array([[-vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, -vehicle_radius],
                      [-vehicle_length / 2, -vehicle_radius]])
        global_vertices = np.dot(R, V.T).T + self.position
        # 绘制矩形
        px = np.append(global_vertices[:, 0], global_vertices[0, 0])
        py = np.append(global_vertices[:, 1], global_vertices[0, 1])
        plt.plot(px, py)

# 虚拟车辆
class VirtualVehicle(Vehicle):
    def __init__(self, vehicles, offset = 2.5):
        x_max = np.max([vehicle.position[0] for vehicle in vehicles])
        y_avg = np.mean([vehicle.position[1] for vehicle in vehicles])
        y_des = min(lane_centers, key = lambda center: abs(center - y_avg))
        position = np.array([x_max + offset, y_des])
        velocity = np.array([velocity_desire, 0])
        delta = np.array([0, 0])
        super().__init__(position, velocity, delta, vehicle_radius)

    def update_state(self, vehicles):
        self.position += self.velocity * dt

    def plot(self):
        theta = np.arctan2(self.velocity[1], self.velocity[0])
        # 旋转矩阵
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        # 尺寸矩阵
        V = np.array([[-vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, -vehicle_radius],
                      [-vehicle_length / 2, -vehicle_radius]])
        global_vertices = np.dot(R, V.T).T + self.position
        # 绘制矩形
        px = np.append(global_vertices[:, 0], global_vertices[0, 0])
        py = np.append(global_vertices[:, 1], global_vertices[0, 1])
        plt.plot(px, py, color = 'grey', linestyle = '-.')

# 定义人工车辆
class HDV(Vehicle):
    def __init__(self, position, velocity, auto = 0, status = 0, state = None):
        # state是一个可以传入的车辆整体轨迹数据，是一个n*4的矩阵，表示总共n个时刻每个时刻的位置（2维）和速度（2维）
        self.position = np.array(position, dtype = np.float64)  # 车辆位置
        self.velocity = np.array(velocity, dtype = np.float64)  # 车辆速度
        self.radius = vehicle_radius # 车辆半径
        self.auto = auto  # 是否是自动驾驶车辆，1表示是自动驾驶车，0表示是人工车
        self.status = status  # 是否在编队中，0表示不在编队中，1表示在编队中，方便编队的split和merge
        self.lane = int(self.position[1] // 4 + 1)
        self.state = state

    def update_state(self, t = None):
        if self.state:
            self.position = self.state[t][0:2]
            self.velocity = self.state[t][2:4]
        else:
            self.position += self.velocity * dt

    def random_state(self):
        vx, vy = self.velocity
        max_perturbation = 0.01

        # 生成随机的偏移量，范围在 [-max_perturbation, max_perturbation] 之间
        delta_vx = random.uniform(-max_perturbation, max_perturbation)
        delta_vy = random.uniform(-max_perturbation, max_perturbation)

        # 对速度向量进行偏移
        vx_new = vx + delta_vx
        vy_new = vy + delta_vy
        self.velocity = np.array([vx_new, vy_new], dtype = np.float64)  # 车辆位置

    def plot(self):
        theta = np.arctan2(self.velocity[1], self.velocity[0])
        # 旋转矩阵
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        # 尺寸矩阵
        V = np.array([[-vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, vehicle_radius],
                      [vehicle_length / 2, -vehicle_radius],
                      [-vehicle_length / 2, -vehicle_radius]])
        global_vertices = np.dot(R, V.T).T + self.position
        # 绘制矩形
        px = np.append(global_vertices[:, 0], global_vertices[0, 0])
        py = np.append(global_vertices[:, 1], global_vertices[0, 1])
        plt.plot(px, py, 'r')


# 定义障碍物类
class Obstacle:
    def __init__(self, position, radius):
        self.position = np.array(position, dtype = np.float64)  # 障碍物位置
        self.radius = radius  # 障碍物的半径
        self.lane = int(self.position[1] // 4 + 1)

    def plot(self):
        theta = np.linspace(0, 2 * math.pi, 10)
        px = self.position[0] + self.radius * np.cos(theta)
        py = self.position[1] + self.radius * np.sin(theta)
        plt.plot(px, py, markersize = 6, color = 'green')