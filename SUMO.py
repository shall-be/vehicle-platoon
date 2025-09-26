import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import traci
import sumolib
import time
import os
import sys
import optparse
from sumolib import checkBinary

from globals import *
from consensus import consensus
from lane_change import lane_change, lane_evaluate
from potential_field import potential_field
from vehicles import Vehicle, VirtualVehicle, Obstacle, HDV

CAVs = ['CAV0', 'CAV1', 'CAV2', 'CAV3', 'CAV4']

# 读取轨迹数据
traj_data = pd.read_csv("data\Highway-merge-in\Trajectory.csv")
track_stats = pd.read_csv("data\Highway-merge-in\TrackIDstate.csv")



# 检测是否已经添加环境变量
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

if_show_gui = True

if __name__ == "__main__":
    if not if_show_gui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    sumo_config = "E:\\学习\\毕设\\SUMO\\test1.sumocfg"
    traci.start([sumoBinary, '-c', sumo_config])

    # try:
    #     traci.start(["sumo-gui", "-c", "your_config.sumocfg"])
    #     while traci.simulation.getMinExpectedNumber() > 0:
    #         try:
    #             traci.simulationStep()  # 执行仿真步进
    #         except traci.TraCIException as e:
    #             print(f"Error at time {traci.simulation.getTime()}: {e}", file=sys.stderr)
    #             break  # 或根据需求处理错误
    #     traci.close()
    # except Exception as e:
    #     print(f"Failed to start SUMO: {e}", file=sys.stderr)

    for step in range(0, 2000):  # 仿真时间
        traci.simulationStep()  # 一步一步（一帧一帧）进行仿真
        simulation_time = traci.simulation.getTime()  # 获得仿真时间

        all_vehicle_id = traci.vehicle.getIDList()

        current_frame_data = traj_data[traj_data["frameId"] == step]

        for _, row in current_frame_data.iterrows():
            veh_id = f"veh_{row['trackId']}"
            if not traci.vehicle.getIDList().count(veh_id):
                traci.vehicle.add(
                    vehID=veh_id,
                    routeID="straight",
                    typeID="HDV",
                    depart=step,
                    departLane="free"
                )

            # 设置车辆颜色（RGB格式，取值范围0-255）
            traci.vehicle.setColor(veh_id, (255, 255, 255))

            # 更新位置和速度
            traci.vehicle.moveToXY(
                veh_id,
                edgeID="",  # 需替换为实际道路ID
                laneIndex=0,
                # x=row["localX"],
                # y=row["localY"],
                x = 0,
                y = 0,
                keepRoute=1
            )
            traci.vehicle.setSpeed(veh_id, row["xVelocity"])

        # 过滤出CAV
        cav_ids = [veh_id for veh_id in all_vehicle_id if veh_id.startswith("CAV")]
        if len(cav_ids) == 5:
            position = traci.vehicle.getPosition('CAV3')
            neighbors = traci.vehicle.getNeighbors('CAV3', 0)
            speed = traci.vehicle.getSpeed('CAV3')
            leader = traci.vehicle.getLeader('CAV3', dist = 25)
            # print(position)
            # print(speed)
            # if len(neighbors) > 0:
            #     print('neighbor0:', neighbors[0])
            # if len(neighbors) > 1:
            #     print('neighbor1:', neighbors[1])
            # print('leader:', leader)
            # # print('stop')
            # angle = traci.vehicle.getAngle('CAV3')
            # print(angle)




    traci.close()


