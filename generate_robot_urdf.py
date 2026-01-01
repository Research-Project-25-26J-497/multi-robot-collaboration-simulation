#!/usr/bin/env python3
"""Generate a namespaced URDF for a specific robot"""

import sys

def generate_urdf(robot_name):
    urdf = f'''<?xml version="1.0"?>
<robot name="{robot_name}">

  <!-- MACROS & MATERIALS -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <!-- BASE LINK -->
  <link name="{robot_name}/base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <origin xyz="0 0 0.1"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <origin xyz="0 0 0.1"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0.08"/>
      <inertia ixx="0.0275" ixy="0.0" ixz="0.0" iyy="0.0275" iyz="0.0" izz="0.045"/>
    </inertial>
  </link>

  <!-- DRIVE WHEELS (Right and Left) -->
  <link name="{robot_name}/left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.0025" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <link name="{robot_name}/right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.0025" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- Wheel Joints -->
  <joint name="{robot_name}/left_wheel_joint" type="continuous">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/left_wheel"/>
    <origin xyz="-0.1 0.175 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="{robot_name}/right_wheel_joint" type="continuous">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/right_wheel"/>
    <origin xyz="-0.1 -0.175 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- CASTER WHEEL (Passive support wheel) -->
  <link name="{robot_name}/caster_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="{robot_name}/caster_joint" type="fixed">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/caster_link"/>
    <origin xyz="0.08 0 0.05"/>
  </joint>

  <!-- LiDAR SENSOR (Hokuyo/Laser for SLAM) -->
  <link name="{robot_name}/laser_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="{robot_name}/laser_joint" type="fixed">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/laser_link"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- GAZEBO PHYSICS PROPERTIES -->
  <gazebo reference="{robot_name}/base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="{robot_name}/left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>10000000.0</kp>
    <kd>1.0</kd>
    <fdir1>1 0 0</fdir1>
  </gazebo>

  <gazebo reference="{robot_name}/right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>10000000.0</kp>
    <kd>1.0</kd>
    <fdir1>1 0 0</fdir1>
  </gazebo>

  <gazebo reference="{robot_name}/caster_link">
    <material>Gazebo/Green</material>
    <mu1>0.01</mu1>
    <mu2>0.01</mu2>
  </gazebo>

  <!-- GAZEBO SENSOR PLUGIN (LiDAR) with namespaced frame -->
  <gazebo reference="{robot_name}/laser_link">
    <material>Gazebo/Black</material>
    <sensor name="laser_sensor" type="ray">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14</min_angle>
            <max_angle>3.14</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>5.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/{robot_name}</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>{robot_name}/laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- GAZEBO CONTROL PLUGIN (Differential Drive) with namespaced frames -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive_controller">
      <ros>
        <namespace>/{robot_name}</namespace>
      </ros>
      <left_joint>{robot_name}/left_wheel_joint</left_joint>
      <right_joint>{robot_name}/right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>0.5</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>{robot_name}/odom</odometry_frame>
      <robot_base_frame>{robot_name}/base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

</robot>
'''
    return urdf

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./generate_robot_urdf.py <robot_name>")
        print("Example: ./generate_robot_urdf.py robot1")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    print(generate_urdf(robot_name))
