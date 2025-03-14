import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
import random

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

def poisson_disk_sampling(width, height, r, k, num_samples):
    """
    使用泊松盘采样生成点，确保最小间距为 r。
    width, height: 采样区域大小
    r: 最小距离
    k: 每个点的最大尝试数
    num_samples: 需要生成的点数
    """
    cell_size = r / np.sqrt(2)
    grid_width = int(width / cell_size) + 1
    grid_height = int(height / cell_size) + 1
    grid = [[-1 for _ in range(grid_width)] for _ in range(grid_height)]
    
    def in_bounds(pt):
        return 0 <= pt[0] < width and 0 <= pt[1] < height
    
    def distance(pt1, pt2):
        return np.linalg.norm(np.array(pt1) - np.array(pt2))
    
    def is_valid(pt):
        gx, gy = int(pt[0] / cell_size), int(pt[1] / cell_size)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < grid_width and 0 <= ny < grid_height:
                    index = grid[ny][nx]
                    if index != -1 and distance(pts[index], pt) < r:
                        return False
        return True
    
    pts = []
    process_list = []
    first_pt = (random.uniform(0, width), random.uniform(0, height))
    pts.append(first_pt)
    process_list.append(first_pt)
    grid[int(first_pt[1] / cell_size)][int(first_pt[0] / cell_size)] = 0
    
    while process_list and len(pts) < num_samples:
        idx = random.randint(0, len(process_list) - 1)
        base_pt = process_list[idx]
        found = False
        
        for _ in range(k):
            angle = random.uniform(0, 2 * np.pi)
            radius = random.uniform(r, 2 * r)
            new_pt = (base_pt[0] + radius * np.cos(angle), base_pt[1] + radius * np.sin(angle))
            
            if in_bounds(new_pt) and is_valid(new_pt):
                pts.append(new_pt)
                process_list.append(new_pt)
                gx, gy = int(new_pt[0] / cell_size), int(new_pt[1] / cell_size)
                grid[gy][gx] = len(pts) - 1
                found = True
                break
        
        if not found:
            process_list.pop(idx)
    print(len(pts))
    return pts[:num_samples]

def spawn_box(name, x, y, z, spawn_model):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    box_sdf = BOX_SDF_TEMPLATE.format(name=name)
    try:
        spawn_model(name, box_sdf, "", pose, "world")
        rospy.loginfo(f"成功生成: {name} ({x}, {y}, {z})")
    except rospy.ServiceException as e:
        rospy.logerr(f"生成 {name} 失败: {e}")

if __name__ == "__main__":
    rospy.init_node('spawn_poisson_boxes')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    min_distance = 0.4
    num_boxes = 450
    width, height = 4, 12  # X, Y 轴范围大小
    start_x, start_y = -6.0, -3.0
    start_z = 0.15
    k=30
    positions = poisson_disk_sampling(width, height, min_distance, k , num_boxes)
    positions = [(start_x + x, start_y + y) for x, y in positions]
    
    for i, (x, y) in enumerate(positions):
        spawn_box(f"box_{i+1}", x, y, start_z, spawn_model)
