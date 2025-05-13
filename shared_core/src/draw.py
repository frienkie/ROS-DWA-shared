import matplotlib.pyplot as plt
import numpy as np
import ast
import os

HOME_DIR = os.path.expanduser("~")
file_path1 = os.path.join(HOME_DIR, "obstacle_coords_world1.txt")
file_path2 = os.path.join(HOME_DIR, "cood/test0.txt")


def read_obstacles(file_path):
    """ 读取 obstacles.txt 的坐标 """
    return np.loadtxt(file_path)

def read_test_data(file_path):
    """ 读取 test0.txt 的坐标 """
    with open(file_path, 'r') as f:
        data = f.read()
        return np.array(ast.literal_eval(data))  # 解析字符串形式的列表

# 解析两个文件的坐标
try:
    data1 = read_obstacles(file_path1)
    data2 = read_test_data(file_path2)
    x1, y1 = data1[:, 0], data1[:, 1]
    x2, y2 = data2[:, 0], data2[:, 1]
except Exception as e:
    print(f"Error loading files: {e}")
    x1, y1, x2, y2 = [], [], [], []

# 绘制散点图
plt.figure(figsize=(8, 6))
plt.scatter(x1, y1, c='red', marker='o',s=1, label='Obstacles 1')


# 绘制圆形
radius = 0.105
for xc, yc in zip(x2, y2):
    circle = plt.Circle((xc, yc), radius, color='blue', fill=False,linewidth=1)
    plt.gca().add_patch(circle)

plt.scatter(x2, y2, c='red', marker='x',s=1, label='robot')
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Obstacle Positions")
plt.legend()
plt.axis("equal")
plt.grid()
plt.show()
