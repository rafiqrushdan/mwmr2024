#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32
import sys
sys.path.append('/home/mwmr/.platformio/penv/lib/python3.8/site-packages')
from pyPS4Controller.controller import Controller
from numpy import pi


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.x = 0
        self.y = 0
        self.z = 0

    ########## MOVEMENT TRANSLATIONAL ##########

    def on_L3_up(self, value):
        self.y = -value
        self.publish_movement()

    def on_L3_down(self, value):
        self.y = -value
        self.publish_movement()

    def on_L3_right(self, value):
        self.x = value
        self.publish_movement()

    def on_L3_left(self, value):
        self.x = value
        self.publish_movement()

    def on_L3_x_at_rest(self):
        self.x = 0
        self.publish_movement()

    def on_L3_y_at_rest(self):
        self.y = 0
        self.publish_movement()

    ########### MOVEMENT ROTATIONAL ############

    # def on_R3_left(self, value):
    #     self.z = (value/32768)*0.01
    #     self.publish_movement()

    # def on_R3_right(self, value):
    #     self.z = (value/32768)*0.01
    #     self.publish_movement()

    # def on_R3_x_at_rest(self):
    #     self.z = 0
    #     self.publish_movement()

    # def on_R3_down(self, _):
    #     self.publish_movement()

    # def on_R3_up(self, _):
    #     self.publish_movement()

    # def on_R3_y_at_rest(self):
    #     self.publish_movement()


    ############### ANGLE CHANGE ###############

    def on_left_arrow_press(self):
        self.z -= pi/2
        if self.z >= pi/2:
            self.z = pi/2
        elif self.z <= -pi/2:
            self.z = -pi/2
        self.publish_movement()

    def on_right_arrow_press(self):
        self.z += pi/2
        if self.z >= pi/2:
            self.z = pi/2
        elif self.z <= -pi/2:
            self.z = -pi/2
        self.publish_movement()

    ############# SPEED ADJUSMENTS #############

    def on_x_press(self):
        pub_rpm.publish(Float32(50))

    def on_square_press(self):
        pub_rpm.publish(Float32(80))

    def on_circle_press(self):
        pub_rpm.publish(Float32(90))

    def on_triangle_press(self):
        pub_rpm.publish(Float32(100))

    def publish_movement(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = self.y
        twist.angular.z = self.z
        pub_vel.publish(twist)
        print('\r',
              f'{self.x:6.0f}',
              f'{self.y:6.0f}', end='', flush=True)




if __name__ == '__main__':
    rospy.init_node('ps4_teleop')
    rospy.loginfo('Node started')
    pub_vel = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)
    pub_lauch = rospy.Publisher('launch', Empty, queue_size=10)
    pub_rpm = rospy.Publisher('vesc_rpm', Float32, queue_size=10)

    controller = MyController(
        interface="/dev/input/js0", connecting_using_ds4drv=False)
        # interface="/org/bluez/hci0", connecting_using_ds4drv=False)
    controller.listen()