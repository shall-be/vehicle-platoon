import numpy as np

# 定义一致性算法
def consensus(vehicle, neighbors, leader, alpha = 0.05, beta = 0.1, gamma = 2):
    F_con = np.array([0.0, 0.0])
    if neighbors:
        for neighbor in neighbors:
            if neighbor.status == 1:
                F_con -= alpha * (vehicle.position - neighbor.position - vehicle.delta + neighbor.delta)
                F_con -= beta * (vehicle.velocity - neighbor.velocity)

    if leader:
        # 这里乘以2是想表示虚拟领头车的影响权重更大，这个可以设置成别的参数进行调整
        F_con -= alpha * (vehicle.position - leader.position - vehicle.delta) * gamma
        F_con -= beta * (vehicle.velocity - leader.velocity) * gamma

    return F_con