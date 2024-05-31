#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

# Initialize lists to store x and y positions
x_data = []
y_data = []

def odom_callback(msg):
    # Extract x and y positions from the Odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_data.append(x)
    y_data.append(y)

def shutdown_hook():
    # Function to run on shutdown
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot

def plot_position():
    # Initialize the ROS node
    rospy.init_node('trajectory_plotter', anonymous=True)

    # Subscribe to the /odom topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Set up the plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Trajectory')

    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)

    rate = rospy.Rate(20)  # Set the loop rate to 20 Hz

    while not rospy.is_shutdown():
        if x_data and y_data:  # Only plot if data is available
            ax.plot(x_data, y_data, label='Trajectory')
            ax.legend()
            plt.draw()
            plt.pause(0.001)  # Pause to update the plot

        plt.cla()  # Clear the current axes for the next plot
        rate.sleep()

if __name__ == '__main__':
    try:
        plot_position()
    except rospy.ROSInterruptException:
        pass

