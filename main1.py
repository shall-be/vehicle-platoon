import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
from matplotlib import font_manager as fm

from globals import *
from consensus import consensus
from lane_change import lane_change, lane_evaluate
from potential_field import potential_field
from vehicles import Vehicle, VirtualVehicle, Obstacle, HDV
# from scipy.signal import savgol_filter

colors = plt.get_cmap('cool', 5)
plt.close('all')

matrix = np.zeros((1200,5,6))

# 初始化车辆和虚拟车
vehicles = [
    Vehicle([6, 6], [14, 0.0], [-15, -4], [1000, 6]),
    Vehicle([0.5, 10], [15, 0.0], [-30, 4], [985, 14]),
    Vehicle([15, 13], [13, 0.0], [-30, -4], [985, 6]),
    Vehicle([6, 17], [16, 0.0], [-15, 4], [1000, 14]),
    Vehicle([25, 2], [14, 0.0], [0, 0], [1015, 40]),
    HDV([80, 10], [13, 0]),
    # HDV([30, 18], [16, 0.0]),
    # HDV([25, 13.5], [14, 0.1])
]

platoon = []

for vehicle in vehicles:
    if vehicle.auto == 1:
        platoon.append(vehicle)

virtual_vehicle = VirtualVehicle(platoon)

# # 初始化障碍物
# obstacles = [
#     # Obstacle([300.0, 10.0], 1.5),
#     Obstacle([400.0, 14.0], 1.5),
#     Obstacle([650.0, 6.0], 1.5),
#     Obstacle([650.0, 14.0], 1.5)
# ]
#
obstacles = []



# 执行一致性算法
def simulation(vehicles, communication_range, steps = 1200):
    fig, ax = plt.subplots(figsize=(10, 8))

    # 变道队列：存储正在变道的车辆及其变道状态
    lane_change_queue = []

    for step in range(steps):
        ax.clear()  # 清除上一步的绘图
        # 获取所有车辆的最小和最大 x, y 值，以调整坐标轴
        all_x = [vehicle.position[0] for vehicle in vehicles]
        all_y = [vehicle.position[1] for vehicle in vehicles]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        # 设置动态的坐标轴范围，留出一定的边距
        ax.set_xlim(min_x - 10, max_x + 10)
        ax.set_ylim(-2, 22)  # 固定纵坐标范围

        # 绘制每条车道的边界
        for lane_center in lane_centers:
            road_y_min = lane_center - lane_width / 2
            road_y_max = lane_center + lane_width / 2
            ax.plot([min_x - 10, max_x + 10], [road_y_min, road_y_min], 'k--')  # 道路下边界
            ax.plot([min_x - 10, max_x + 10], [road_y_max, road_y_max], 'k--')  # 道路上边界

        for vehicle in vehicles:
            if vehicle.auto == 1:
                if vehicle.flag == 1:
                    neighbors = vehicle.find_neighbors(vehicles, communication_range)
                    near_obstacles = vehicle.find_obstacles(obstacles, communication_range)
                    target_lane = lane_evaluate(vehicle, neighbors, near_obstacles)
                    lane_change_state = lane_change(vehicle, target_lane)
                    # 将车辆及其变道状态加入变道队列
                    lane_change_queue.append({
                        'vehicle': vehicle,
                        'state': lane_change_state,
                        'step': 0  # 当前变道步数
                    })
                    vehicle.flag = 0

                if vehicle.flag == 2:
                    neighbors = vehicle.find_neighbors(vehicles, communication_range)
                    near_obstacles = vehicle.find_obstacles(obstacles, communication_range)
                    target_lane = lane_evaluate(vehicle, neighbors, near_obstacles)
                    lane_change_state = lane_change(vehicle, target_lane, style = 2)
                    # 将车辆及其变道状态加入变道队列
                    lane_change_queue.append({
                        'vehicle': vehicle,
                        'state': lane_change_state,
                        'step': 0  # 当前变道步数
                    })
                    vehicle.flag = 0

        for vehicle in vehicles:
            if vehicle.auto == 1:
                # 如果车辆不在变道队列中，则正常更新状态
                if not any(item['vehicle'] == vehicle for item in lane_change_queue):
                    if step < 120:
                        vehicle.update_state(None, None, None)
                    else:
                        neighbors = vehicle.find_neighbors(vehicles, communication_range)
                        near_obstacles = vehicle.find_obstacles(obstacles, communication_range)
                        vehicle.update_state(neighbors, near_obstacles, virtual_vehicle)
            if vehicle.auto == 0:
                # vehicle.random_state()
                vehicle.update_state()

        for item in lane_change_queue:
            # 更新变道步数
            item['step'] += 1
            vehicle = item['vehicle']
            state = item['state']
            current_step = item['step']

            # 应用变道状态
            vehicle.position = state[current_step][0:2]
            vehicle.velocity = state[current_step][2:4]

            # 如果变道完成，从队列中移除
            if current_step + 1 >= len(state):
                vehicle.status = 1
                lane_change_queue.remove(item)
                # lane_change_queue.pop(i)  # 使用索引移除
                # break  # 退出循环，避免索引越界

        virtual_vehicle.update_state(vehicles)

        # 绘制车辆
        for i, vehicle in enumerate(vehicles):
            vehicle.plot()

        # virtual_vehicle.plot()

        # 绘制障碍物
        for i, obstacle in enumerate(obstacles):
            obstacle.plot()

        ax = plt.gca()
        ax.set_aspect(1)
        plt.pause(0.01)

        for i, vehicle in enumerate(platoon):
            matrix[step][i][0] = vehicle.position[0]
            matrix[step][i][1] = vehicle.position[1]
            matrix[step][i][2] = vehicle.velocity[0]
            matrix[step][i][3] = vehicle.velocity[1]
            matrix[step][i][4] = np.linalg.norm(vehicle.velocity)
            matrix[step][i][5] = np.linalg.norm(vehicle.accel)

    plt.show()

# 运行一致性算法
simulation(vehicles, communication_range, steps = 1200)

speed_data = matrix[:, :, 4]
accel_data = matrix[:, :, 5]
x_data = matrix[:, :, 0]
y_data = matrix[:, :, 1]

xx = matrix[:, 4, 0]
yy = matrix[:, 4, 1]

df = pd.DataFrame({"X": xx, "Y": yy})
df.to_excel("trajectory_data.xlsx", index=False, engine="openpyxl")

print(xx)


window_length = 15  # 窗口长度（必须为奇数）
polyorder = 5       # 多项式阶数


#
# plt.figure(figsize=(10, 6))  # 设置图像大小
# for vehicle_index in range(5):
#     plt.plot(np.arange(1200), speed_data[:, vehicle_index], label=f'vehicle {vehicle_index + 1}')
#
# plt.xlabel('time')
# plt.ylabel('speed')
# plt.title('speed curve')
# plt.legend()  # 添加图例
# plt.grid(True)  # 添加网格
#
# plt.savefig('vehicle_speed_curve_with_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# # plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG
#
# plt.show()  # 显示图像
#
#
#
# plt.figure(figsize=(10, 6))  # 设置图像大小
# for vehicle_index in range(5):
#     plt.plot(np.arange(1200), accel_data[:, vehicle_index], label=f'vehicle {vehicle_index + 1}')
#
# plt.xlabel('time')
# plt.ylabel('accel')
# plt.title('accel curve')
# plt.legend()  # 添加图例
# plt.grid(True)  # 添加网格
#
# plt.savefig('vehicle_accel_curve_with_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# # plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG
#
# plt.show()  # 显示图像






my_font = fm.FontProperties(fname="C:/Windows/Fonts/simsun.ttc")

plt.rcParams["font.family"] = my_font.get_name()  # 全局生效
plt.rcParams["axes.unicode_minus"] = False        # 解决负号显示问题


plt.figure(figsize=(10, 6))  # 设置图像大小
# 设置字体：英文字体 Times New Roman，中文字体 SimSun（宋体）
# plt.rcParams['font.family'] = ['Times New Roman', 'SimHei']
for vehicle_index in range(5):
    plt.plot(np.arange(1200), speed_data[:, vehicle_index], label=f'车辆 {vehicle_index + 1}')

plt.xlabel('时间')
plt.ylabel('速度')
plt.legend()  # 添加图例
plt.grid(True)  # 添加网格

plt.savefig('vehicle_speed_curve_without_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG



plt.figure(figsize=(10, 6))  # 设置图像大小
# 设置字体：英文字体 Times New Roman，中文字体 SimSun（宋体）
for vehicle_index in range(5):
    plt.plot(np.arange(1200), accel_data[:, vehicle_index], label=f'车辆 {vehicle_index + 1}')

plt.xlabel('时间')
plt.ylabel('加速度')
plt.legend()  # 添加图例
plt.grid(True)  # 添加网格

plt.savefig('vehicle_accel_curve_without_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG



plt.figure(figsize=(10, 6))  # 设置图像大小
# 设置字体：英文字体 Times New Roman，中文字体 SimSun（宋体）
for vehicle_index in range(5):
    plt.plot(np.arange(1200), speed_data[:, vehicle_index] - velocity_desire, label=f'车辆 {vehicle_index + 1}')

plt.xlabel('时间')
plt.ylabel('速度误差')
plt.legend()  # 添加图例
plt.grid(True)  # 添加网格

plt.savefig('vehicle_speed_error_without_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG





plt.figure(figsize=(10, 6))  # 设置图像大小
# 设置字体：英文字体 Times New Roman，中文字体 SimSun（宋体）
for vehicle_index in range(5):
    plt.plot(np.arange(1200), x_data[:, vehicle_index], label=f'车辆 {vehicle_index + 1}')

plt.xlabel('时间')
plt.ylabel('距离')
plt.legend()  # 添加图例
plt.grid(True)  # 添加网格

plt.savefig('vehicle_distance_without_obstacle.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
# plt.savefig('vehicle_speed_curve.jpg', dpi=300, bbox_inches='tight')  # 保存为 JPG





# === 轨迹图：y(x) + 不同标记 ===
n_veh = x_data.shape[1]
T = len(x_data)
half = T   # 只画前半段

marker_list = ['o','s','D','^','v','>','<','P','X','*','h','H','d','8']
color_map = plt.cm.tab20
markevery = max(1, half // 18)  # 半段之后重新估计稀疏度

fig, ax = plt.subplots(figsize=(8, 6))  # 比之前更“高”

for i in range(n_veh):
    m = marker_list[i % len(marker_list)]
    c = color_map(i % color_map.N)
    ax.plot(
        x_data[:half, i], y_data[:half, i],
        linestyle='-', linewidth=1.8,
        marker=m, markersize=6, markevery=markevery,
        mfc='none', mec=c, color=c,
        label=f'车辆 {i+1}'
    )

ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.grid(True, linestyle='--', alpha=0.5)

# —— 关键：图例放更下面，并给底部留出足够空间 —— #
leg = ax.legend(
    loc='upper center',
    bbox_to_anchor=(0.5, -0.22),   # 往下挪得更远
    ncol=min(n_veh, 5),
    frameon=False,
    handlelength=2.8
)
# 增大底部留白，避免遮挡
plt.subplots_adjust(bottom=0.28)   # 或者用 tight_layout(rect=[0,0.18,1,1])

# 不强制等比例；想更“高”就别用 set_aspect('equal')
ax.set_aspect('auto')

plt.savefig('vehicle_trajectories_xy_half_without_obstacle.png', dpi=300, bbox_inches='tight')
plt.show()








