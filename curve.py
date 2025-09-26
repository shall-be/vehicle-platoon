import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from matplotlib import font_manager as fm

# 从Excel读取数据
try:
    df = pd.read_excel("trajectory_data.xlsx", engine="openpyxl")
    x = df["X"].values
    y = df["Y"].values
except Exception as e:
    print(f"读取文件失败: {e}")
    exit()

# 检查数据有效性
assert len(x) == len(y), "x和y长度不一致"

print(x)

xx = x[100:200]
yy = y[100:200]

window_length = 15  # 窗口长度（必须为奇数）
polyorder = 5       # 多项式阶数

x_smooth = savgol_filter(xx, window_length, polyorder)
y_smooth = savgol_filter(yy, window_length, polyorder)

def compute_curvature(x, y):
    # 计算一阶和二阶导数
    dx = np.gradient(x)
    dy = np.gradient(y)
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)

    # 计算曲率
    numerator = dx * d2y - dy * d2x
    denominator = (dx ** 2 + dy ** 2) ** 1.5
    curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator != 0)
    return curvature

curvature_orig = compute_curvature(xx, yy)
curvature_smooth = compute_curvature(x_smooth, y_smooth)




my_font = fm.FontProperties(fname="C:/Windows/Fonts/simsun.ttc")

plt.rcParams["font.family"] = my_font.get_name()  # 全局生效
plt.rcParams["axes.unicode_minus"] = False        # 解决负号显示问题





# 绘制原始和平滑后的轨迹
plt.figure(figsize=(12, 5))


plt.plot(xx, yy, 'b-', label='原始曲线')
plt.plot(x_smooth, y_smooth, 'r--', label='平滑后曲线')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('轨迹曲线')
plt.legend()
plt.grid(True)


plt.savefig('vehicle_trajectory_curve.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
plt.close()  # 关闭当前图形，避免内存泄漏

# 绘制曲率对比
plt.plot(curvature_orig, 'b-', label='原始曲率')
plt.plot(curvature_smooth, 'r--', label='平滑后曲率')
plt.xlabel('采样点')
plt.ylabel('曲率')
plt.title('曲率曲线')
plt.legend()
plt.grid(True)

plt.tight_layout()

plt.savefig('vehicle_curvature_comparison.png', dpi=300, bbox_inches='tight')  # 保存为 PNG
plt.close()  # 关闭当前图形，避免内存泄漏