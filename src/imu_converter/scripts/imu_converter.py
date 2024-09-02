#!/usr/bin/env python

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
