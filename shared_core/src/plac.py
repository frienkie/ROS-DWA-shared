#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
def spawn_box(name, x, y, z):
    """
    生成一个长方体模型。
    :param name: 模型名称
    :param x: X 坐标
    :param y: Y 坐标
    :param z: Z 坐标
    """
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # 定义一个简单的 SDF 模型，尺寸为 0.2 x 0.2 x 0.3
    box_sdf = f"""
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

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    # 调用 Gazebo 服务生成模型
    spawn_model(name, box_sdf, "", pose, "world")

if __name__ == "__main__":
    rospy.init_node('spawn_boxes')
    
    
    spacing = 0.45  # 间隔距离
    start_x = -5.6  # 起始 X 坐标-5.25
    start_y = -1.4  # 起始 Y 坐标-1.05
    end_x=-1.78
    end_y=7.8
    start_z = 0.15  # 起始 Z 坐标 (长方体高度一半)

    x_coords = np.arange(start_x, end_x, spacing*2)
    y_coords = np.arange(start_y, end_y, spacing*2)

    # 遍历生成所有坐标点的模型
    box_count = 1
    for x in x_coords:
        for y in y_coords:
            box_name = f"box_{box_count}"
            spawn_box(box_name, x, y, start_z)
            box_count += 1
    start_x=spacing+start_x
    start_y=spacing+start_y
    x_coords = np.arange(start_x, end_x, spacing*2)
    y_coords = np.arange(start_y, end_y, spacing*2)

    for x in x_coords:
        for y in y_coords:
            box_name = f"box_{box_count}"
            spawn_box(box_name, x, y, start_z)
            box_count += 1
