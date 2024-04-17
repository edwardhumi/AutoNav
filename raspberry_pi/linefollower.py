#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 11 16:29:25 2024

@author: john
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import cmath

from std_msgs.msg import String

import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

rotatechange = 0.7
speedchange = 0.15

#there are 8 inputs from the linefollower
#GPIO pins to be used are
#pins 13, 17, 19, 20, 21, 22, 26, 27
#black = 1; white = 0
ir1 = 17;
ir2 = 13;


#read HIGH or LOW from esp32 pins 16 and 17
esp32_pin1 = 23; #connect to pin 16 of esp
esp32_pin2 = 24; #connect to pin 17 of esp

GPIO.setup(ir1, GPIO.IN)
GPIO.setup(ir2, GPIO.IN)

GPIO.setup(esp32_pin1, GPIO.IN)
GPIO.setup(esp32_pin2, GPIO.IN)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class Mover(Node):
    def __init__(self):
        super().__init__('moverotate')

        # Create publisher to send the current stage
        # navigation, server, door, bucket, reverse, autonav
        self.publisher2_ = self.create_publisher(String, 'stage', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.stage_callback)

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.direction = "stop"

        ###########################################################################################
        # initialize stage to nav
        self.stage = "navigation"

        # Subscribe to esp server
        self.subscription2 = self.create_subscription(
                String,
                'door',
                self.server_callback,
                10)
        self.subscription2
        ################################################
        self.door = "0"
        self.doneServer = False
        self.isRotating = False

    def server_callback(self, msg):
        self.get_logger().info('Received Door: "%s"' % msg.data)
        self.door = msg.data
        print("door num", msg.data)
        
    def forward(self):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # start movement
        self.publisher_.publish(twist)
        #time.sleep(0.1)

    def backward(self):
        twist = Twist()
        twist.linear.x = -1*speedchange
        twist.angular.x = 0.0
        self.publisher_.publish(twist)
        print("mundur")

    def right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -1*rotatechange
        print('kanan')
        # start movement
        self.publisher_.publish(twist)
        #time.sleep(0.5)

    def left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = rotatechange
        # start movement
        print('kiri')
        self.publisher_.publish(twist)
        #time.sleep(0.5)

    def rightForward(self):
        # rotate right then forward
        twist = Twist()
        #twist.linear.x += speedchange
        #twist.angular.z -= rotatechange
        # start movement
        #self.publisher_.publish(twist)
        self.forward()
        time.sleep(1)
        print("rightForward")

    def leftForward(self):
        # rotate left then forward
        twist = Twist()
        #twist.linear.x += speedchange
        #twist.angular.z += rotatechange
        # start movement
        #self.publisher_.publish(twist)
        self.forward()
        time.sleep(1)
        print("leftForward")

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # start movement
        self.publisher_.publish(twist)

    def delay(self, delay_time, direction):
        curr_time = time.time()
        while(time.time() - curr_time < delay_time):
            if (direction == "forward"):
                self.forward()
            elif (direction == "backward"):
                self.backward()
            elif (direction == "left"):
                self.left()
            elif (direction == "right"):
                self.right()

    def stage_callback(self):
        msg = String()
        msg.data = self.stage
        self.publisher2_.publish(msg)
        self.get_logger().info('Publishing Stage: "%s"' % msg.data)

    # function to set the class variables using the odometry information
    def odom_callback(self, msg):
        # self.get_logger().info(msg)
        self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

        #read inputs
        s1=  1-GPIO.input(ir1)
        s2 = 1-GPIO.input(ir2)

        print(s1,s2)
        #if linefollower doesn't detect black
        #(by right at the middle of the 2 doors)
        #it will stop 
        #then go left or right depending on which door is unlocked

        #if not  (self.direction == "leftForward" or self.direction == "rightForward"):

        if self.stage != "autonav":
            if ((s1 == 0) and (s2 == 0)  and (self.direction == 'stop' or self.direction == "forward")):
                if not self.stage == "server":
                    self.direction = "stop"
                    self.stop()
    
                if self.stage == "navigation":
                    self.stage = "server"
    
                if self.stage == "door":
                    self.stage = "bucket"
    
                if self.stage == "reverse":
                    self.stage = "autonav"
    
                if self.stage == "bucket":
                    msg = String()
                    msg.data = self.stage
                    self.publisher2_.publish(msg)
                    self.get_logger().info('Publishing Stage: "%s"' % msg.data)
    
                    time.sleep(5)
                    print("AAAAAAAAAAAA")
                    #self.direction = "backward"
                    self.backward()
                    self.delay(0.5, "backward")
                    #self.direction = "right"
                    self.right()
                    self.delay(4.4, "right")
                    self.stage = "reverse"
    
                if self.stage == "server":
                    #assuming door 1 is on the left
                    if (self.door == "door1"):
                        time.sleep(2)
                        self.left()
                        self.delay(1.3, "left")
                        self.forward()
                        self.delay(0.2, "forward")
                        self.doneServer = True
                    #if rpi received that door 2 is unlocked
                    elif (self.door == "door2"):
                        time.sleep(2)
                        self.right()
                        self.delay(1.2, "right")
                        self.forward()
                        self.delay(0.1, "forward")
                        self.doneServer = True
    
            #follow black line, forward, right, left, stop
            else:
                # line follower should not work in bucket adn autonav stage
                if self.stage != "bucket" and self.stage != "autonav":
                    # robot should move forward if not in reverse stage
                    #if self.stage != "reverse":
                    if True:
                        #right
                        if s2 == 1 and s1 == 0:
                            self.direction = "right"
                            self.right()
                         #left
                        elif s1 == 1 and s2 == 0:
                            self.direction = "left"
                            self.left() 
                        elif s1 == 1 and s2 == 1:
                            self.direction = "forward"
                            self.forward()
                        else:
                            self.direction = "stop"
                            self.stop()
    
                if self.stage == "server" and self.doneServer:
                    self.stage = "door"

    def move(self):
        while (True):
            rclpy.spin_once(self)
            print(self.direction)
            try:
                if (self.direction == "stop"):
                    #self.stop()
                    print("Stop")
                elif (self.direction == "right"):
                    #self.right()
                    print("Right")
                elif (self.direction == "left"):
                    #self.left()
                    print("Left")
                elif (self.direction == "forward"):
                    #self.forward()
                    print("Forward")
                elif (self.direction == "leftForward"):
                    #self.leftForward()
                    print("leftForward")
                elif (self.direction == "rightForward"):
                    #self.rightForward()
                    print("rightForward")
                elif (self.direction == "backward"):
                    print("backward")
            except Exception as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    mover.move()
    GPIO.cleanup()
    Mover.destroy_node()
    rclpy.shutdown()
    print("Shutting down")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if __name__ == 'main':
    main()

main()

