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
GPIO.setmode(GPIO.BCM)

#there are 8 inputs from the linefollower
#GPIO pins to be used are
#pins 13, 17, 19, 20, 21, 22, 26, 27
#black = 1; white = 0
s1 = 13;
s2 = 17;
s3 = 19;
s4 = 20;
s5 = 21;
s6 = 22;
s7 = 26;
s8 = 27;

#read HIGH or LOW from esp32 pins 16 and 17
esp32_pin1 = 23; #connect to pin 16 of esp
esp32_pin2 = 24; #connect to pin 17 of esp

GPIO.setup(s1, GPIO.IN)
GPIO.setup(s2, GPIO.IN)
GPIO.setup(s3, GPIO.IN)
GPIO.setup(s4, GPIO.IN)
GPIO.setup(s5, GPIO.IN)
GPIO.setup(s6, GPIO.IN)
GPIO.setup(s7, GPIO.IN)
GPIO.setup(s8, GPIO.IN)
GPIO.setup(esp32_pin1, GPIO.IN)
GPIO.setup(esp32_pin2, GPIO.IN)

class Mover(Node):
    def __init__(self):
        super().__init__('moverotate')
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


    # function to set the class variables using the odometry information
    def odom_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * speedchange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    # function to read keyboard input
    # function to automove
    def forward(self):
        twist = Twist()
        twist.linear.x += speedchange
        twist.angular.z = 0.0
        # start movement
        self.publisher_.publish(twist)
    
    def right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z -= rotatechange
        # start movement
        self.publisher_.publish(twist)
    
    def left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z += rotatechange
        # start movement
        self.publisher_.publish(twist)
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # start movement
        self.publisher_.publish(twist)
    
class lineFollow(Node):        
        #if linefollower doesn't detect black
        #(by right at the middle of the 2 doors)
        #it will stop 
        #then go left or right depending on which door is unlocked
        if (
            (s1!=1)and(s2!=1)and(s3!=1)and(s4!=1)and(s5!=1)and(s6!=1)and(s7!=1)and(s8!=1)
        ):
            mover = Mover()
            mover.stop()
            #assuming door 1 is on the left
            if (GPIO.input(esp32_pin1) and not(GPIO.input(esp32_pin2))):
                #run for 3 seconds
                mover.left()
                time.sleep(3)
            #if rpi received that door 2 is unlocked
            elif (GPIO.input(esp32_pin2) and not(GPIO.input(esp32_pin1))):
                mover.right()
                #run for 3 seconds
                time.sleep(3)
            else:
                mover.stop()
                time.sleep(3)
            
        #follow black line, forward, right, left, stop
        else:
            #forward
            if  ( 
                ((s1==0)and(s2==0)and(s3==0)and(s4==1)and(s5==1)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==0)) 
            ):
                mover = Mover()
                mover.forward()

            #right
            elif (
                ((s1==0)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==1)and(s6==1)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==0)and(s6==1)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==0)and(s6==0)and(s7==1)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==0)and(s6==0)and(s7==0)and(s8==1))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==0)and(s6==1)and(s7==1)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==1)and(s6==1)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==0)and(s5==1)and(s6==1)and(s7==1)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==1)and(s5==1)and(s6==1)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==0)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==0))
            ):
                mover = Mover()
                mover.right()

            #left
            elif (
                ((s1==1)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==1)and(s8==0))or
                ((s1==1)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==0)and(s8==0))or
                ((s1==1)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==1)and(s2==1)and(s3==1)and(s4==1)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==1)and(s2==1)and(s3==1)and(s4==0)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==1)and(s2==1)and(s3==0)and(s4==0)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==1)and(s2==0)and(s3==0)and(s4==0)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==1)and(s3==1)and(s4==0)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==1)and(s4==1)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==1)and(s3==1)and(s4==1)and(s5==0)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==0)and(s3==1)and(s4==1)and(s5==1)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==0)and(s7==0)and(s8==0))or
                ((s1==0)and(s2==1)and(s3==1)and(s4==1)and(s5==1)and(s6==1)and(s7==0)and(s8==0))
            ):
                mover = Mover()
                mover.left()

        
def main(args=None):
    rclpy.init(args=args)
    linefollow = lineFollow()
    rclpy.spin(scanner)
    GPIO.cleanup()
    scanner.destroy_node()
    rclpy.shutdown()
    print("Shutting down")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if name == 'main':
    main()
    
