import matplotlib.pyplot as plt
import numpy as np

# 支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

def plot_experiment_scene():
    # 创建图形
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 设置坐标轴范围和刻度
    ax.set_xlim(-2.0, 2.0)
    ax.set_ylim(-1.5, 1.5)
    ax.set_xticks(np.arange(-1.5, 1.6, 0.5))  # 缩短单元格间距
    ax.set_yticks(np.arange(-1.5, 1.6, 0.5))  # 缩短单元格间距
    ax.grid(True, linestyle='--', color='gray', alpha=0.5)  # 网格线更柔和
    
    # 设置标题和轴标签
    ax.set_xlabel("X [m]", fontsize=12, labelpad=10)
    ax.set_ylabel("Y [m]", fontsize=12, labelpad=10)
    ax.set_title("Experimental scene layout", fontsize=20, fontweight='bold', pad=20)
    

    
    # UWB 基站位置与标注
    uwb_stations = [(1.35, 1.35), (1.35, -1.35), (-1.35, 1.35), (-1.35, -1.35)]
    for i, (x, y) in enumerate(uwb_stations, start=1):
        ax.scatter(x, y, c='red', s=120, edgecolor='black')  # 基站圆点
        ax.text(x, y + 0.05, f"UWB{i}", fontsize=10, color='black',  # 向上移动标注
                verticalalignment='bottom', horizontalalignment='center')

    # 小车初始位置及方向
    robots = [(0.05, -0.525), (0.45, -0.525), (-0.45, 0.525), (-0.05, 0.525)]
    for i, (x, y) in enumerate(robots, start=1):
        ax.scatter(x, y, c='orange', s=100, edgecolor='black')  # 小车圆点
        ax.text(x, y + 0.05, f"Robot{i}", fontsize=12, color='black',  # 向上移动标注
                verticalalignment='bottom', horizontalalignment='center')
        ax.arrow(x, y, 0.1, 0, head_width=0.03, head_length=0.03,  # 缩小箭头
                 fc='black', ec='black', linewidth=1.2)

    # 移除图例
    # ax.legend()  # 删除图例
    
    # 显示图形
    plt.tight_layout()
    plt.show()

plot_experiment_scene()