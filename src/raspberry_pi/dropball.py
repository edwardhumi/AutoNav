#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 15:33:44 2024

@author: john
"""

"""
Created on Tue Apr  9 15:03:47 2024

@author: john
"""

#This code is for opening the cap of the ball dropper
#when near the bucket
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import cmath
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

GPIO.setmode(GPIO.BCM)

# constants
servo_pin = 12


GPIO.setup(servo_pin, GPIO.OUT)
servo = GPIO.PWM(servo_pin,50)
servo.start(5.9)


def drop(args=None):
    servo.ChangeDutyCycle(2)
    time.sleep(5)
    servo.ChangeDutyCycle(5.5)

class BallDropper(Node):
    def __init__(self):
        super().__init__('ball_dropper')
        self.subscription = self.create_subscription(
                String,
                'stage',
                self.listener_callback,
                10)
        self.subscription
        self.done = False

    def listener_callback(self, msg):
        self.get_logger().info('Received Stage: "%s":' % msg.data)
        if (msg.data == "bucket") and not self.done:
        #if not self.done:
            drop()
            self.done = True

def main(args=None):
    rclpy.init(args=args)
    ball_dropper = BallDropper()
    rclpy.spin(ball_dropper)
    ball_dropper.destroy_node()
    rclpy.shutdown()
   

if __name__ == "__main__":
    main()
