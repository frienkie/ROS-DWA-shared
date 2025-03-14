import rospy
import numpy as np
from gazebo_msgs.srv import GetWorldProperties, GetModelState
import os

HOME_DIR = os.path.expanduser("~")
OUTPUT_FILE = os.path.join(HOME_DIR, "obstacles.txt")

def get_box_positions():
    rospy.init_node('get_box_positions', anonymous=True)
    
    rospy.wait_for_service('/gazebo/get_world_properties')
    rospy.wait_for_service('/gazebo/get_model_state')
    
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        world_properties = get_world_properties()
        box_positions = {}
        
        for model_name in world_properties.model_names:
            if 'box' in model_name:
                model_state = get_model_state(model_name, 'world')
                x, y = model_state.pose.position.x, model_state.pose.position.y
                box_positions[model_name] = (x, y)
                
        return box_positions
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return {}

def generate_border_points(x, y, step=0.005, size=0.2):
    half_size = size / 2
    points = []
    
    for i in np.arange(-half_size, half_size + step, step):
        points.append((x + i, y - half_size))  # 下边
        points.append((x + i, y + half_size))  # 上边
        points.append((x - half_size, y + i))  # 左边
        points.append((x + half_size, y + i))  # 右边
    
    return points

def save_obstacle_data(filename, box_positions):
    with open(filename, "w") as file:
        for name, (x, y) in box_positions.items():
            file.write(f"{x:.3f} {y:.3f}\n")
            border_points = generate_border_points(x, y)
            for bx, by in border_points:
                file.write(f"{bx:.3f} {by:.3f}\n")

if __name__ == "__main__":
    box_positions = get_box_positions()
    save_obstacle_data(OUTPUT_FILE, box_positions)
    for name, (x, y) in box_positions.items():
        print(f"{name}: x={x}, y={y}")
