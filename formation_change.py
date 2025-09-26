import numpy as np

from globals import *

formation_templates = {
    "single_column": {
        "deltas": [
            np.array([0, 0]),    # 领队车辆
            np.array([-15, 4]),  # 第二辆车（右侧车道，纵向偏移-15）
            np.array([-15, -4]), # 第三辆车（左侧车道，纵向偏移-15）
            np.array
        ]
    },

    "double_column": {
        "deltas": [
            np.array([0, 0]),
            np.array([-10, 4]),
            np.array([-10, -4]),
            np.array([-20, 4]),
            np.array([-20, -4]),
        ]
    },

    "avoid_obstacle": {
        "deltas": [
            np.array([0, 0]),
            np.array([-15, 8]),  # 向右分散
            np.array([-15, -8]), # 向左分散
            # ... 其他车辆
        ]
    }
}

def formation_change(vehicles, new_formation = None):
    for vehicle in vehicles:
        if vehicle.auto == 1 and vehicle.status == 1:
            vehicle.delta = new_formation