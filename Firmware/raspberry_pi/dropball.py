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
fd_pin = 18
bk_pin = 23

extendTime = 0
waitTime = 1 
retractTime = 4


GPIO.setup(fd_pin, GPIO.OUT)
GPIO.setup(bk_pin,GPIO.OUT)
fd = GPIO.PWM(fd_pin,1000)
fd.start(0)

GPIO.setup(servo_pin, GPIO.OUT)
servo = GPIO.PWM(servo_pin,50)
servo.start(5.9)

# GPIO.PWM(pin,freq)
# to be used to control the servo
#p = GPIO.PWM(servo_pin, 50)

# starting duty cycle of the servo
# from 0.0 to 100.0
#p.start(2.5)
#print("Setup done")

#p1 = GPIO.PWM(pwm_pin, 50)


def drop(args=None):
    # rclpy.init(args=args)
    # scanner = Scanner()
    # rclpy.spin(scanner)
    # p.stop()
    
    servo.ChangeDutyCycle(2)
    time.sleep(5)
    servo.ChangeDutyCycle(5.5)
    #GPIO.cleanup()
    #rclpy.shutdown()
    #print("Shutting down")

def extendAndDrop(args=None):
    # rclpy.init(args=args)
    # scanner = Scanner()
    # rclpy.spin(scanner)
    # p.stop()
    try:
        compareTime = time.time()
        while time.time() - compareTime < extendTime:
            fd.ChangeDutyCycle(50)
        compareTime = time.time()
        while time.time() - compareTime < waitTime:
            fd.ChangeDutyCycle(100)
            GPIO.output(23, True)
        drop()   
        compareTime = time.time()
        while time.time() - compareTime < retractTime:
            fd.ChangeDutyCycle(0)
        GPIO.output(23,False)
    except KeyboardInterrupt:
        fd.ChangeDutyCycle(0)
        GPIO.output(23, False)
        #time.sleep(2)
        print('aba')
        GPIO.cleanup()
    #rclpy.shutdown()
        print("Shutting down")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

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
