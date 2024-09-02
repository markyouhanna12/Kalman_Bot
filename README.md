# Kalman_Bot

### 1. create a workspace
1.1 Create the root workspace directory (we’lluse catkin_ws)

`$ cd ~/`
`$ mkdir -p ∼/catkin ws/src`
`$ cd ∼/catkin ws/`

1.2 Initialize the catkin workspace

`$ catkin_make`

1.3 Check the crearted folders to make sure the workspace created

`$ cd ∼/catkin ws/`
`$ ls`

**build**         **devel**         **src**

1.4 Sourcing the setup.bash files

`$ source devel/setup.bash`

### 2. Gazebo Installation

2.1 Before attempting to install the gazebo_ros_pkgs, make sure the stand alone Gazebo works by running in terminal:

`$ gazebo`

2.2 If Gazebo is not installed, you can install it using the following command:

  `$ curl -sSL http://get.gazebosim.org | sh`

    Run

`$ gazebo`

2.3 Install gazebo_ros_pkgs

Install Pre-Built Debian\Ubuntu binary packages:

`$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control`

enter the directory of the workspace you have created:

`$ cd ∼/catkin ws/`

Noetic is using the gazebo 11.x series, start by installing it:

`$ sudo apt-get install -y libgazebo11-dev`

Download the source code from the gazebo_ros_pkgs github repository:

`$ cd ~/catkin_ws/src`

`$ git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel`

Check for any missing dependencies using rosdep:

`$ rosdep update`

`$ rosdep check --from-paths . --ignore-src --rosdistro noetic`

To build the Gazebo ROS integration packages, run the following commands:

`$ cd ~/catkin_ws/`

`$ catkin_make`

2.4 Testing Gazebo with ROS Integration
Source the catkin setup.bash if it's not already in your .bashrc:

   `source ~/catkin_ws/devel/setup.bash`

Run:

   `roscore & rosrun gazebo_ros gazebo`

### 3. TurtleBot3 Installation

3.1 Install TurtleBot3 Packages:

`$ sudo apt remove ros-noetic-dynamixel-sdk`

`$ sudo apt remove ros-noetic-turtlebot3-msgs`

`$ sudo apt remove ros-noetic-turtlebot3`

`$ cd ~/catkin_ws/src/`

`$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git`

`$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`

`$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git`


`$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`$ cd ~/catkin_ws && catkin_make`

`$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`

### 4. run the Gazebo world
 Launch Simulation World

**Note>>** you must do this step every time before lunching to give the TURTLEBOT3 MODEL an argument between {burger, waffle, waffle_pi } if you want to know the Specifications between them check the following link:[Turtlebot Types](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications)

 okay we will continue lunching:

`$ export TURTLEBOT3_MODEL=burger`

**world 1**

`$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

**world 2**

`$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`

**world 3**

`$ roslaunch turtlebot3_gazebo turtlebot3_house.launch`

### 5. Control the TurtleBot3 using keyboard

we will open a new terminal :

`$ cd ~/catkin_ws/`

`$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

### 6. RViz Installation

### Install rviz Using apt-get

 `$ sudo apt-get update`

After updating apt database, We can install rviz using apt-get by running the following command:

`$ sudo apt-get -y install rviz`

### 7. Read IMU data from the turtlebot3

first we will open a terminal and enter the workspace directory and export one of the 3 models (burger,waffle,waffle_pi) using:

`$ cd ~/catkin_ws/`

`$ export TURTLEBOT3_MODEL=waffle`

then we will lunch the gazebo and choose one of the world that mentioned above :

`$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`

then we will open a new terminal and will see what topics there using:

`$ cd ~/catkin_ws/`

`$ rostopic list`

**the output be like this:**
```/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf```

as you can see there is topic named /imu if we want to see the topic:

`$ rostopic echo /imu`

### 8. Visualize the LiDAR data on RVIZ

we will open a new terminal:

`$ cd ~/catkin_ws/`

we need to export the same turtle model we use to run gazebo world in this case (waffle):

`$ export TURTLEBOT3_MODEL=waffle`

and we will run raviz using the LiDAR topic data (/scan):

`$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`

### 9. Convert the IMU readings from Quaternion to Degree and publish them on a new topic:

9.1 Create a new package in the workspace named (**imu_converter**)
`$ cd ~/catkin_ws/`

`$ catkin_create_pkg imu_converter rospy std_msgs sensor_msgs`

9.2 Create a scripts directory inside the package directory and place the Python script there:

`$ cd ~/catkin_ws/src/imu_converter`

`$ mkdir scripts`

9.3 We will open vs code and choose the folder **scripts** of the **imu_converter** and make  a python file in it and named it (**imu_converter**)

**the python script:**
```#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from math import atan2, asin, degrees

def quaternion_to_euler(quat):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Roll is rotation around x-axis in degrees (forward)
    Pitch is rotation around y-axis in degrees (sideways)
    Yaw is rotation around z-axis in degrees (heading)
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    # Convert radians to degrees
    roll = degrees(roll)
    pitch = degrees(pitch)
    yaw = degrees(yaw)

    return roll, pitch, yaw

def imu_callback(data):
    # Convert quaternion to Euler angles
    roll, pitch, yaw = quaternion_to_euler(data.orientation)

    # Log or print the converted data
    rospy.loginfo(f"Roll: {roll:.2f} Pitch: {pitch:.2f} Yaw: {yaw:.2f}")

    # Publish the converted data
    roll_pub.publish(roll)
    pitch_pub.publish(pitch)
    yaw_pub.publish(yaw)

if __name__ == '__main__':
    rospy.init_node('imu_converter', anonymous=True)

    # Publishers for roll, pitch, and yaw
    roll_pub = rospy.Publisher('/imu_euler/roll', Float32, queue_size=10)
    pitch_pub = rospy.Publisher('/imu_euler/pitch', Float32, queue_size=10)
    yaw_pub = rospy.Publisher('/imu_euler/yaw', Float32, queue_size=10)

    # Subscriber to the IMU topic
    rospy.Subscriber('/imu', Imu, imu_callback)

    # Keep the node running
    rospy.spin()

```

9.4 Make the script executable

`$ chmod +x scripts/imu_converter.py`

9.5 Build the Package

`$ cd ~/catkin_ws`

`$ catkin_make`

9.6 Run the Node

`$ source devel/setup.bash`

`$ rosrun imu_converter imu_converter.py`

9.7 Check the newly created topics by running:

open a new terminal :

`$ rostopic list`

**the output be like this:**
```/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/imu_euler/pitch
/imu_euler/roll
/imu_euler/yaw
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf```

**as we can see there are the three topics of the directions: **
/imu_euler/pitch
/imu_euler/roll
/imu_euler/yaw

