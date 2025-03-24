import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
import random
import matplotlib.pyplot as plt
import os

HOME_DIR = os.path.expanduser("~")
OUTPUT_FILE = os.path.join(HOME_DIR, "obstacles.txt")

# SDF 模板
BOX_SDF_TEMPLATE = """
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.3</size>
          </box>
        </geometry>
        <material>
            <ambient>0.4235 0.4235 0.4235 1</ambient>
            <diffuse>0.4235 0.4235 0.4235 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.3</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>1.0</mass>
      </inertial>
    </link>
  </model>
</sdf>
"""

def generate_border_points(x, y, step=0.005, size=0.2):
    half_size = size / 2
    points = []
    
    # 生成边框点
    for i in np.arange(-half_size, half_size + step, step):
        points.append((x + i, y - half_size))  # 下边
        points.append((x + i, y + half_size))  # 上边
        points.append((x - half_size, y + i))  # 左边
        points.append((x + half_size, y + i))  # 右边
    
    return points

def save_obstacle_data(filename, obstacles):
    with open(filename, "w") as file:
        for _, x, y, _ in obstacles:
            file.write(f"{x:.3f} {y:.3f}\n")
            border_points = generate_border_points(x, y)
            for bx, by in border_points:
                file.write(f"{bx:.3f} {by:.3f}\n")

def poisson_disk_sampling(width, height, r1, r2, k=30, num_samples=80):
    cell_size = min(r1, r2) / np.sqrt(2)
    grid_width = int(np.ceil(width / cell_size))
    grid_height = int(np.ceil(height / cell_size))
    
    grid_A = [[None for _ in range(grid_height)] for _ in range(grid_width)]
    grid_B = [[None for _ in range(grid_height)] for _ in range(grid_width)]
    
    samples_A = []
    samples_B = []
    active_list = []
    
    x0, y0 = random.uniform(0, width), random.uniform(0, height)
    #x0, y0 = -3.6,3.6
    # initial_class = random.choice(['A', 'B'])
    initial_class = 'A'  ###########初始种类
    if initial_class == 'A':
        samples_A.append((x0, y0))
        grid_A[int(x0 / cell_size)][int(y0 / cell_size)] = (x0, y0)
        active_list.append((x0, y0, 'A'))
    else:
        samples_B.append((x0, y0))
        grid_B[int(x0 / cell_size)][int(y0 / cell_size)] = (x0, y0)
        active_list.append((x0, y0, 'B'))
    
    def random_point_around(x, y, min_r, max_r):
        radius = random.uniform(min_r, max_r)
        angle = random.uniform(0, 2 * np.pi)
        return x + radius * np.cos(angle), y + radius * np.sin(angle)
    
    def is_valid_point(x, y, category):
        if not (0 <= x < width and 0 <= y < height):
            return False
        
        grid_x, grid_y = int(x / cell_size), int(y / cell_size)
        if category == 'A':
            r, check_grid, other_grid = r1, grid_A, grid_B
        else:
            r, check_grid, other_grid = r2, grid_B, grid_A
        
        for i in range(max(0, grid_x - 2), min(grid_width, grid_x + 3)):
            for j in range(max(0, grid_y - 2), min(grid_height, grid_y + 3)):
                neighbor = check_grid[i][j]
                if neighbor and np.linalg.norm(np.array((x, y)) - np.array(neighbor)) < r:
                    return False
                neighbor = other_grid[i][j]
                if neighbor and np.linalg.norm(np.array((x, y)) - np.array(neighbor)) < min(r1, r2):
                    return False
        return True
    
    while active_list and len(samples_A) + len(samples_B) < num_samples:
        idx = random.randint(0, len(active_list) - 1)
        x, y, category = active_list[idx]
        found = False

        for _ in range(k):
            category = 'A' if random.random() <= 1.2 else 'B'
            nx, ny = random_point_around(x, y, r1 if category == 'A' else r2, 2.0 * (r1 if category == 'A' else r2))
            
            if is_valid_point(nx, ny, category):
                if category == 'A':
                    samples_A.append((nx, ny))
                    grid_A[int(nx / cell_size)][int(ny / cell_size)] = (nx, ny)
                    active_list.append((nx, ny, 'A'))
                else:
                    samples_B.append((nx, ny))
                    grid_B[int(nx / cell_size)][int(ny / cell_size)] = (nx, ny)
                    active_list.append((nx, ny, 'B'))
                found = True
                break
        
        if not found:
            active_list.pop(idx)
    
    return samples_A, samples_B

def spawn_box(name, x, y, z, spawn_model, obstacles):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    box_sdf = BOX_SDF_TEMPLATE.format(name=name)
    try:
        spawn_model(name, box_sdf, "", pose, "world")
        rospy.loginfo(f"成功生成: {name} ({x}, {y}, {z})")
        obstacles.append((name, x, y, z))
    except rospy.ServiceException as e:
        rospy.logerr(f"生成 {name} 失败: {e}")

if __name__ == "__main__":
    rospy.init_node('spawn_poisson_boxes')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    min_distance_A = 0.47
    min_distance_B = 1.0#0.41
    num_boxes = 200
    width, height = 3.82, 9.2
    start_x, start_y = -5.6, -1.4
    start_z = 0.15
    

    samples_A, samples_B = poisson_disk_sampling(width, height, min_distance_A, min_distance_B, num_samples=num_boxes)
    obstacles = []
    for i, (x, y) in enumerate(samples_A + samples_B):
        spawn_box(f"box_{i+1}", start_x + x, start_y + y, start_z, spawn_model, obstacles)
    
    #save_obstacle_data(OUTPUT_FILE, obstacles)
    
    plt.figure(figsize=(12, 12))
    if samples_A:
        plt.scatter(*zip(*samples_A), s=10, c='blue', label="Class A")
    if samples_B:
        plt.scatter(*zip(*samples_B), s=10, c='red', label="Class B")
    plt.legend()
    plt.axis('equal')
    plt.show()
