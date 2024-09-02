#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu  
from std_msgs.msg import Float64  
import tf.transformations  

# Initializing global variables for the Kalman Filter
previous_yaw = 0.0  
estimate_error = 1.0  
measurement_error = 0.1
kalman_gain = 0.0  


# Kalman Filter function to estimate the YAW angle
def kalman_filter(yaw, previous_yaw, estimate_error, measurement_error):

    kalman_gain = estimate_error / (estimate_error + measurement_error)
    current_estimate = previous_yaw + kalman_gain * (yaw - previous_yaw)
    estimate_error = (1 - kalman_gain) * estimate_error
    
    return current_estimate, estimate_error


def imu_callback(data):
    global previous_yaw, estimate_error 
    
    # Extract the quaternion representing the robot's orientation from the IMU data
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )
    
    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]  # in radians
    
    # Convert YAW angle from radians to degrees for easier interpretation
    yaw_deg = yaw * 180.0 / 3.14159
    
    # Apply the Kalman Filter to get the filtered YAW angle
    filtered_yaw, estimate_error = kalman_filter(yaw_deg, previous_yaw, estimate_error, measurement_error)
    
    # Update the previous YAW with the current filtered estimate
    previous_yaw = filtered_yaw
    
    # Publish the filtered YAW angle on a new ROS topic
    yaw_pub.publish(filtered_yaw)



def main():
    rospy.init_node('kalman_filter_node')  
    
    global yaw_pub
    yaw_pub = rospy.Publisher('/filtered_yaw', Float64, queue_size=10) 
  
    rospy.Subscriber('/imu', Imu, imu_callback)  
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass  # Handle any exceptions that may occur