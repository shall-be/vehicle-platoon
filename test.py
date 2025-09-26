import time
import math
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional

# 定义车辆类
class Vehicle:
    def __init__(self, id: int, position: Tuple[float, float], velocity: Tuple[float, float], lane: int, is_autonomous: bool = True):
        self.id = id
        self.position = position  # 车辆位置 (x, y)
        self.velocity = velocity  # 车辆速度 (vx, vy)
        self.lane = lane  # 车辆所在车道
        self.is_autonomous = is_autonomous  # 是否为自动驾驶车辆
        self.acceleration = (0.0, 0.0)  # 车辆加速度 (ax, ay)

    def update_state(self, dt: float):
        """更新车辆状态"""
        self.position = (
            self.position[0] + self.velocity[0] * dt,
            self.position[1] + self.velocity[1] * dt,
        )
        self.velocity = (
            self.velocity[0] + self.acceleration[0] * dt,
            self.velocity[1] + self.acceleration[1] * dt,
        )

    def set_acceleration(self, acceleration: Tuple[float, float]):
        """设置车辆加速度"""
        self.acceleration = acceleration

    def distance_to(self, other: 'Vehicle') -> float:
        """计算与另一辆车的距离"""
        return math.sqrt(
            (self.position[0] - other.position[0]) ** 2 + (self.position[1] - other.position[1]) ** 2
        )


# 定义编队类
class Formation:
    def __init__(self, leader: Vehicle, followers: List[Vehicle], lane_width: float = 3.0, num_lanes: int = 3):
        self.leader = leader  # 编队领头车
        self.followers = followers  # 编队跟随车辆
        self.lane_width = lane_width  # 车道宽度
        self.num_lanes = num_lanes  # 车道总数
        self.safe_distance = 5.0  # 安全距离
        self.formation_distance = 10.0  # 编队内车辆间的目标距离

    def update_formation(self, vehicles: List[Vehicle]):
        """更新编队状态"""
        if not self.can_form_formation(vehicles):
            print("Formation cannot be formed due to road conditions.")
            return

        # 动态调整车道
        self.adjust_lanes(vehicles)

        # 调整跟随车辆的状态
        for follower in self.followers:
            self.adjust_follower(follower, vehicles)

    def adjust_follower(self, follower: Vehicle, vehicles: List[Vehicle]):
        """调整跟随车辆的状态"""
        # 目标位置：领头车后方一定距离，保持在同一车道或相邻车道
        target_position = (
            self.leader.position[0] - self.formation_distance,
            self.leader.position[1] + (follower.lane - self.leader.lane) * self.lane_width,
        )

        # 计算目标速度和加速度
        target_velocity = self.leader.velocity
        acceleration = (
            (target_velocity[0] - follower.velocity[0]) * 0.5,
            (target_velocity[1] - follower.velocity[1]) * 0.5,
        )

        # 避免碰撞
        for vehicle in vehicles:
            if vehicle.id != follower.id and follower.distance_to(vehicle) < self.safe_distance:
                # 如果距离太近，减速
                acceleration = (-1.0, -1.0)

        follower.set_acceleration(acceleration)

    def can_form_formation(self, vehicles: List[Vehicle]) -> bool:
        """判断是否可以形成编队"""
        # 检查前方是否有非自动驾驶车辆阻碍
        for vehicle in vehicles:
            if not vehicle.is_autonomous and self.is_vehicle_ahead(vehicle):
                return False

        # 检查目标车道是否被占用
        for follower in self.followers:
            if self.is_lane_occupied(follower.lane, vehicles):
                return False

        return True

    def is_vehicle_ahead(self, vehicle: Vehicle) -> bool:
        """判断某辆车是否在领头车前方"""
        return vehicle.position[0] > self.leader.position[0] and abs(
            vehicle.position[1] - self.leader.position[1]
        ) < self.lane_width

    def is_lane_occupied(self, lane: int, vehicles: List[Vehicle]) -> bool:
        """判断目标车道是否被占用"""
        for vehicle in vehicles:
            if vehicle.lane == lane and self.is_vehicle_ahead(vehicle):
                return True
        return False

    def adjust_lanes(self, vehicles: List[Vehicle]):
        """动态调整车道"""
        for follower in self.followers:
            # 如果当前车道被占用，尝试切换到相邻车道
            if self.is_lane_occupied(follower.lane, vehicles):
                new_lane = self.find_available_lane(follower, vehicles)
                if new_lane is not None:
                    follower.lane = new_lane

    def find_available_lane(self, follower: Vehicle, vehicles: List[Vehicle]) -> Optional[int]:
        """寻找可用的相邻车道"""
        for delta in [-1, 1]:  # 尝试左右相邻车道
            new_lane = follower.lane + delta
            if 0 <= new_lane < self.num_lanes and not self.is_lane_occupied(new_lane, vehicles):
                return new_lane
        return None

    def should_break_formation(self, vehicles: List[Vehicle]) -> bool:
        """判断是否应该解除编队"""
        # 如果有车辆离开通信范围或前方有阻碍，则解除编队
        for follower in self.followers:
            if self.leader.distance_to(follower) > 50.0:  # 通信范围假设为50米
                return True
        return not self.can_form_formation(vehicles)


# 碰撞检测函数
def detect_collisions(vehicles: List[Vehicle], safe_distance: float) -> List[Tuple[int, int]]:
    """检测所有车辆之间的碰撞"""
    collisions = []
    for i in range(len(vehicles)):
        for j in range(i + 1, len(vehicles)):
            if vehicles[i].distance_to(vehicles[j]) < safe_distance:
                collisions.append((vehicles[i].id, vehicles[j].id))
    return collisions


# 可视化函数
def visualize(vehicles: List[Vehicle], num_lanes: int, lane_width: float):
    """可视化车辆状态"""
    plt.clf()  # 清空当前画布
    plt.xlim(-50, 50)  # X轴范围
    plt.ylim(-5, num_lanes * lane_width + 5)  # Y轴范围
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Vehicle Formation Visualization")

    # 绘制车道
    for lane in range(num_lanes):
        plt.axhline(y=lane * lane_width, color="gray", linestyle="--", linewidth=1)

    # 绘制车辆
    for vehicle in vehicles:
        color = "blue" if vehicle.is_autonomous else "red"
        plt.scatter(vehicle.position[0], vehicle.position[1], color=color, label=f"Vehicle {vehicle.id}")
        plt.text(vehicle.position[0], vehicle.position[1], f"{vehicle.id}", fontsize=12, ha="right")

    plt.legend(loc="upper right")
    plt.pause(0.1)  # 暂停0.1秒以更新图像


# 主函数
def main():
    # 初始化车辆
    leader = Vehicle(1, (0.0, 0.0), (10.0, 0.0), lane=0)
    follower1 = Vehicle(2, (-10.0, 0.0), (10.0, 0.0), lane=0)
    follower2 = Vehicle(3, (-20.0, 3.0), (10.0, 0.0), lane=1)  # 在相邻车道
    human_vehicle = Vehicle(4, (30.0, 0.0), (8.0, 0.0), lane=0, is_autonomous=False)  # 前方人工驾驶车辆
    vehicles = [leader, follower1, follower2, human_vehicle]

    # 初始化编队
    formation = Formation(leader, [follower1, follower2], num_lanes=3)

    # 初始化可视化
    plt.ion()  # 开启交互模式
    plt.figure(figsize=(10, 6))

    # 主循环
    dt = 0.1  # 时间步长
    while True:
        # 更新车辆状态
        for vehicle in vehicles:
            vehicle.update_state(dt)

        # 更新编队
        if formation.can_form_formation(vehicles):
            formation.update_formation(vehicles)
        elif formation.should_break_formation(vehicles):
            print("Formation broken due to road conditions.")
            break

        # 碰撞检测
        collisions = detect_collisions(vehicles, safe_distance=5.0)
        if collisions:
            print(f"Collisions detected: {collisions}")
            # 处理碰撞（例如减速或改变车道）

        # 可视化
        visualize(vehicles, num_lanes=3, lane_width=3.0)

        # 模拟时间步进
        time.sleep(dt)


if __name__ == "__main__":
    main()