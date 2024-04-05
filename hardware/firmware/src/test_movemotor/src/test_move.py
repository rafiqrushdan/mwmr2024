#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import sys
import tty
import termios

motor1_pub = rospy.Publisher('motor1', Int16, queue_size=10)
motor2_pub = rospy.Publisher('motor2', Int16, queue_size=10)
motor3_pub = rospy.Publisher('motor3', Int16, queue_size=10)
motor4_pub = rospy.Publisher('motor4', Int16, queue_size=10)
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def move_motors():
    rospy.init_node('motor_controller', anonymous=True)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'q':
            motor1_pub.publish(100)
            motor2_pub.publish(100)
            motor3_pub.publish(100)
            motor4_pub.publish(100)
        elif key == 'e':
            motor1_pub.publish(0)
            motor2_pub.publish(0)
            motor3_pub.publish(0)
            motor4_pub.publish(0)
        rate.sleep()

if __name__ == '__main__':
    def shutdown_hook():
        motor1_pub.publish(0)
        motor2_pub.publish(0)
        motor3_pub.publish(0)
        motor4_pub.publish(0)
        rospy.on_shutdown(shutdown_hook)
        rate = rospy.Rate(50)
    while not rospy.is_shutdown():
          move_motors()
