import numpy as np

dt = 0.05  # 时间步长
lane_width = 4  # 每条车道的宽度
lane_centers = [2, 6, 10, 14, 18]  # 多车道中心位置
lane_max = 20.0  # 道路上边界
lane_min = 0.0  # 道路下边界
vehicle_length = 4.2  # 车辆长度
vehicle_width = 1.8  # 车辆宽度
vehicle_radius = 0.9  # 车辆半径（把车辆当做圆来进行势场法建模）
velocity_desire = 15  # 期望车速
velocity_max = 20  # 最大车速
communication_range = 100.0  # 编队的通信范围
time_lane_change = 5  # 默认的变道时间，可以根据具体情况修改

A = np.array([[time_lane_change ** 3, time_lane_change ** 4, time_lane_change ** 5],
              [3 * time_lane_change ** 2, 4 * time_lane_change ** 3, 5 * time_lane_change ** 4],
              [6 * time_lane_change, 12 * time_lane_change ** 2, 20 * time_lane_change **3]])
A_pinv = np.linalg.pinv(A)  # 五次函数参数拟合的矩阵