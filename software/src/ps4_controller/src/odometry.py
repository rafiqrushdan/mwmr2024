#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from math import cos, sin


class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_node')

        # Parameters for odometry calculation
        self.wheel_radius = 0.05  # Radius of the robot's wheels in meters
        self.base_width = 0.4     # Width of the robot's base (distance between wheels) in meters

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # IMU variables
        self.current_yaw = 0.0

        # Subscribers
        rospy.Subscriber('elephant/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)

        # Publisher
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.last_time = rospy.Time.now()

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    def cmd_vel_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate robot's linear and angular velocities
        vx = linear_velocity
        vy = 0.0
        vtheta = angular_velocity

        # Calculate change in position and orientation
        delta_x = (vx * cos(self.theta) - vy * sin(self.theta)) * dt
        delta_y = (vx * sin(self.theta) + vy * cos(self.theta)) * dt
        delta_theta = vtheta * dt

        # Update robot's pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom)

        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_calculator = OdometryCalculator()
        odom_calculator.run()
    except rospy.ROSInterruptException:
        pass
