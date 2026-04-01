import os
from scipy.io import loadmat
import matplotlib.pyplot as plt

# 文件路径
file_path = r"D:\微信文件\WeChat Files\wxid_37t9e2tdi7r212\FileStorage\File\2025-03\untitled.fig"

# 检查文件是否存在
if not os.path.exists(file_path):
    print(f"文件路径不存在: {file_path}")
else:
    try:
        # 加载 .fig 文件
        mat = loadmat(file_path)

        # 打印文件中的键值（检查数据结构）
        print("文件内容的键值如下：")
        print(mat.keys())

        # 提取数据并处理
        # 这里的处理方式取决于 .fig 文件中保存的数据结构
        # (MATLAB .fig 文件通常保存为二进制对象，包含图形的元数据和数据)

        # 可视化（示例代码，具体需要根据文件内容调整）
        # 假设其中包含 X 和 Y 数据
        if 'XData' in mat and 'YData' in mat:
            x = mat['XData']
            y = mat['YData']
            plt.plot(x, y)
            plt.title("从 .fig 文件中加载的图形")
            plt.xlabel("X 轴")
            plt.ylabel("Y 轴")
            plt.show()
        else:
            print("无法直接提取 XData 和 YData，请根据文件内容结构手动解析。")
    except Exception as e:
        print(f"加载 .fig 文件时出错: {e}")