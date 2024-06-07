#!/usr/bin/env python3

import sys
sys.path.append('/home/mwmr/.platformio/penv/lib/python3.8/site-packages')

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from pynput import keyboard
import threading

class MecanumAutonomousMover:
    def __init__(self):
        rospy.init_node('mecanum_autonomous_mover', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber('/ultrasonic_front', Range, self.front_sensor_callback)
        rospy.Subscriber('/ultrasonic_back', Range, self.back_sensor_callback)
        rospy.Subscriber('/ultrasonic_left', Range, self.left_sensor_callback)
        rospy.Subscriber('/ultrasonic_right', Range, self.right_sensor_callback)

        # Sensor distances
        self.front_distance = float('inf')
        self.back_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Safe distance threshold
        self.safe_distance = 20  # 20 cm

        # Control loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

        # Movement control flags
        self.is_moving = False

        # Key listener thread
        self.listener_thread = threading.Thread(target=self.key_listener)
        self.listener_thread.start()

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def front_sensor_callback(self, msg):
        self.front_distance = msg.range
        
    def back_sensor_callback(self, msg):
        self.back_distance = msg.range

    def left_sensor_callback(self, msg):
        self.left_distance = msg.range

    def right_sensor_callback(self, msg):
        self.right_distance = msg.range

    def key_listener(self):
        def on_press(key):
            try:
                if key.char == 'q':
                    self.is_moving = True
                elif key.char == 'e':
                    self.is_moving = False
            except AttributeError:
                pass

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def move_robot(self):
        while not rospy.is_shutdown():
            twist = Twist()
            if self.is_moving:
                # Check for obstacles and decide movement
                if self.front_distance < self.safe_distance:
                    twist.linear.x = -2.4  # Move backward
                    twist.linear.y = 0
                    twist.angular.z = 0
                elif self.back_distance < self.safe_distance:
                    twist.linear.x = 2.4  # Move forward
                    twist.linear.y = 0
                    twist.angular.z = 0
                elif self.left_distance < self.safe_distance:
                    twist.linear.x = 0
                    twist.linear.y = 2.4  # Move right
                    twist.angular.z = 0
                elif self.right_distance < self.safe_distance:
                    twist.linear.x = 0
                    twist.linear.y = -2.4  # Move left
                    twist.angular.z = 0
                else:
                    twist.linear.x = 2.4  # Default forward movement
                    twist.linear.y = 0
                    twist.angular.z = 0

                self.cmd_vel_pub.publish(twist)
            else:
                # Stop the robot
                self.cmd_vel_pub.publish(Twist())

            self.rate.sleep()

    def shutdown_hook(self):
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Robot has been stopped.")

if __name__ == '__main__':
    try:
        mover = MecanumAutonomousMover()
        mover.move_robot()
    except rospy.ROSInterruptException:
        pass

