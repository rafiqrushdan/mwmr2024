#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

x_data = []
y_data = []

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_data.append(x)
    y_data.append(y)

    plt.cla()  # Clear the current axes
    plt.plot(x_data, y_data, label='Trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory')
    plt.legend()
    plt.pause(0.001)  # Pause to update the plot

def plot_position():
    rospy.init_node('trajectory_plotter', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    plt.ion()  # Turn on interactive mode
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        plot_position()
    except rospy.ROSInterruptException:
        pass
