#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, UInt16
from sensor_msgs.msg import LaserScan
import time
from itertools import count
import numpy as np

index = count(step=1)  # Time counter 

pub_laser = rospy.Publisher("sonar_laser", LaserScan, queue_size=100)
pub_servo = rospy.Publisher("servo", UInt16, queue_size=10)

a = [0] * 360
w1 = 0  # Define w1 as a global variable with initial value 0

def callback1(data):
    global w1
    q = data.data
    w = q / 10
    if w > 4:
        w = 4
    elif w == 0:
        w = 0.01
    w1 = round(w, 3)
    print(w1)

scan_update_time = 0.22222  # Time interval for each data update
publish_interval = 1  # Publish complete array every second

def laser():
    global w1, a  # Ensure w1 and a are accessible within the function
    r = rospy.Rate(1 / scan_update_time)  # Adjust the rate according to update time

    last_publish_time = rospy.Time.now()
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - last_publish_time).to_sec()

        for i in range(360):
            pub_servo.publish(i)
            time.sleep(scan_update_time)
            a[i] = w1  # Update array element with w1

            if elapsed_time >= publish_interval:
                laser = LaserScan()
                Time = next(index)
                laser.header.seq = Time
                laser.header.stamp = current_time
                laser.header.frame_id = 'sonar'
                laser.angle_min = 0
                laser.angle_max = 2 * np.pi
                laser.angle_increment = np.pi / 180
                laser.time_increment = scan_update_time
                laser.scan_time = publish_interval
                laser.range_min = 0.01
                laser.range_max = 4
                laser.ranges = a
                laser.intensities = []
                pub_laser.publish(laser)
                last_publish_time = current_time  # Reset the publish time

        r.sleep()

rospy.Subscriber('/sonar', Float64, callback1)

rospy.init_node('laser', anonymous=True)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass
