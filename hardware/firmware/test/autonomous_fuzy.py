#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyMecanumControl:
    def __init__(self):
        rospy.init_node('fuzzy_mecanum_control')
        
        # Define ROS publishers for controlling the wheels
        self.front_left_wheel_pub = rospy.Publisher('/front_left_wheel/cmd_vel', Float64, queue_size=10)
        self.front_right_wheel_pub = rospy.Publisher('/front_right_wheel/cmd_vel', Float64, queue_size=10)
        self.rear_left_wheel_pub = rospy.Publisher('/rear_left_wheel/cmd_vel', Float64, queue_size=10)
        self.rear_right_wheel_pub = rospy.Publisher('/rear_right_wheel/cmd_vel', Float64, queue_size=10)
        
        # Define ROS subscribers for reading ultrasonic sensor data
        rospy.Subscriber('/front_ultrasonic', Range, self.front_ultrasonic_callback)
        rospy.Subscriber('/left_ultrasonic', Range, self.left_ultrasonic_callback)
        rospy.Subscriber('/right_ultrasonic', Range, self.right_ultrasonic_callback)
        rospy.Subscriber('/rear_ultrasonic', Range, self.rear_ultrasonic_callback)
        
        # Initialize variables for sensor data
        self.front_dist = 0.0
        self.left_dist = 0.0
        self.right_dist = 0.0
        self.rear_dist = 0.0
        
        # Define fuzzy variables and membership functions
        self.front_distance = ctrl.Antecedent(np.arange(0, 3501, 1), 'front_distance')
        self.left_distance = ctrl.Antecedent(np.arange(0, 3501, 1), 'left_distance')
        self.right_distance = ctrl.Antecedent(np.arange(0, 3501, 1), 'right_distance')
        self.rear_distance = ctrl.Antecedent(np.arange(0, 3501, 1), 'rear_distance')
        
        # Define output variable
        self.velocity = ctrl.Consequent(np.arange(-1, 1.1, 0.1), 'velocity')
        
        # Define membership functions for distance
        self.front_distance['near'] = fuzz.trimf(self.front_distance.universe, [0, 11.5, 11.5])
        self.front_distance['medium'] = fuzz.trimf(self.front_distance.universe, [10, 505, 1000])
        self.front_distance['far'] = fuzz.trimf(self.front_distance.universe, [900, 2200, 3500])

        self.rear_distance['near'] = fuzz.trimf(self.rear_distance.universe, [0, 11.5, 11.5])
        self.rear_distance['medium'] = fuzz.trimf(self.rear_distance.universe, [10, 505, 1000])
        self.rear_distance['far'] = fuzz.trimf(self.rear_distance.universe, [900, 2200, 3500])

        self.right_distance['near'] = fuzz.trimf(self.right_distance.universe, [0, 11.5, 11.5])
        self.right_distance['medium'] = fuzz.trimf(self.right_distance.universe, [10, 505, 1000])
        self.right_distance['far'] = fuzz.trimf(self.right_distance.universe, [900, 2200, 3500])

        self.left_distance['near'] = fuzz.trimf(self.left_distance.universe, [0, 11.5, 11.5])
        self.left_distance['medium'] = fuzz.trimf(self.left_distance.universe, [10, 505, 1000])
        self.left_distance['far'] = fuzz.trimf(self.left_distance.universe, [900, 2200, 3500])

    
        # Define output membership functions
        self.velocity['backward'] = fuzz.trimf(self.velocity.universe, [-1, -0.5, 0])
        self.velocity['stop'] = fuzz.trimf(self.velocity.universe, [-0.2, 0, 0.2])
        self.velocity['forward'] = fuzz.trimf(self.velocity.universe, [0, 0.5, 1])
        
        
# Define fuzzy rules

#Detect front obstacles ,reverse
rule1 = ctrl.Rule(self.front_distance['near'] & self.left_distance['far'] & self.right_distance['far'] & self.rear_distance['far'], 
                  (self.front_left_wheel_velocity['backward'], 
                   self.front_right_wheel_velocity['backward'],
                   self.rear_left_wheel_velocity['backward'],
                   self.rear_right_wheel_velocity['backward']))

#Detect front obstacles with medium rear, slight right left?
rule2 = ctrl.Rule(self.front_distance['near'] & self.left_distance['far'] & self.right_distance['far'] & self.rear_distance['medium'], 
                  (self.front_left_wheel_velocity['backward'], 
                   self.front_right_wheel_velocity['backward'],
                   self.rear_left_wheel_velocity['backward'],
                   self.rear_right_wheel_velocity['stop']))

#Detect front obstacles with medium right, slight right forward?
rule3 = ctrl.Rule(self.front_distance['medium'] & self.left_distance['far'] & self.right_distance['near'] & self.rear_distance['far'], 
                  (self.front_left_wheel_velocity['forward'], 
                   self.front_right_wheel_velocity['forward'],
                   self.rear_left_wheel_velocity['stop'],
                   self.rear_right_wheel_velocity['forward']))

        
        
        
    # Create fuzzy control system
self.fuzzy_ctrl_system = ctrl.ControlSystem([rule1, rule2, rule3])  # Add all rules here
self.fuzzy_ctrl = ctrl.ControlSystemSimulation(self.fuzzy_ctrl_system)
        
        # Start ROS node
rospy.spin()
    
def front_ultrasonic_callback(self, data):
        self.front_dist = data.range
    
def left_ultrasonic_callback(self, data):
        self.left_dist = data.range
    
def right_ultrasonic_callback(self, data):
        self.right_dist = data.range
    
def rear_ultrasonic_callback(self, data):
        self.rear_dist = data.range
    
def compute_wheel_velocities(self):
    # Use sensor data to compute wheel velocities using fuzzy control system
    self.fuzzy_ctrl.input['front_distance'] = self.front_dist
    self.fuzzy_ctrl.input['left_distance'] = self.left_dist
    self.fuzzy_ctrl.input['right_distance'] = self.right_dist
    self.fuzzy_ctrl.input['rear_distance'] = self.rear_dist
    self.fuzzy_ctrl.compute()
    
    # Get the computed velocities for each wheel
    front_left_velocity = self.fuzzy_ctrl.output['front_left_wheel_velocity']
    front_right_velocity = self.fuzzy_ctrl.output['front_right_wheel_velocity']
    rear_left_velocity = self.fuzzy_ctrl.output['rear_left_wheel_velocity']
    rear_right_velocity = self.fuzzy_ctrl.output['rear_right_wheel_velocity']
    
    # Publish the velocities to the respective wheel topics
    self.front_left_wheel_pub.publish(front_left_velocity)
    self.front_right_wheel_pub.publish(front_right_velocity)
    self.rear_left_wheel_pub.publish(rear_left_velocity)
    self.rear_right_wheel_pub.publish(rear_right_velocity)



if __name__ == '__main__':
    try:
        control = FuzzyMecanumControl()
    except rospy.ROSInterruptException:
        pass
