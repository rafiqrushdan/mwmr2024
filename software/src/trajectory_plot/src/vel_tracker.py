#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

# Data lists
x_data = []
y_data = []
linear_velocity_data = []
angular_velocity_data = []
length_traveled = 0.0

def odom_callback(msg):
    global length_traveled
    
    # Extract position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Calculate distance traveled
    if len(x_data) > 0:
        distance = math.sqrt((x - x_data[-1]) ** 2 + (y - y_data[-1]) ** 2)
        length_traveled += distance

    x_data.append(x)
    y_data.append(y)

    # Extract velocities
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z
    linear_velocity_data.append(linear_velocity)
    angular_velocity_data.append(angular_velocity)

    # Clear the current axes
    ax1.cla()
    ax2.cla()
    ax3.cla()

    # Plot trajectory
    ax1.plot(x_data, y_data, label='Trajectory')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Robot Trajectory')
    ax1.legend()
    ax1.annotate(f'Length Traveled: {length_traveled:.2f} m', xy=(0.05, 0.95), xycoords='axes fraction', fontsize=10, color='red')

    # Plot linear velocity
    ax2.plot(linear_velocity_data, label='Linear Velocity', color='blue')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Linear Velocity (m/s)')
    ax2.set_title('Linear Velocity Over Time')
    ax2.legend()

    # Plot angular velocity
    ax3.plot(angular_velocity_data, label='Angular Velocity', color='green')
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Angular Velocity (rad/s)')
    ax3.set_title('Angular Velocity Over Time')
    ax3.legend()

    plt.pause(0.001)  # Pause to update the plot

def plot_position():
    rospy.init_node('trajectory_plotter', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    plt.ion()  # Turn on interactive mode

    global fig, ax1, ax2, ax3
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        plot_position()
    except rospy.ROSInterruptException:
        pass

