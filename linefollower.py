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

rotatechange = 0.05
speedchange = 0.1

#there are 8 inputs from the linefollower
#GPIO pins to be used are
#pins 13, 17, 19, 20, 21, 22, 26, 27
#black = 1; white = 0
ir1 = 13;
ir2 = 17;
ir3 = 19;
ir4 = 20;
ir5 = 21;

#read HIGH or LOW from esp32 pins 16 and 17
esp32_pin1 = 23; #connect to pin 16 of esp
esp32_pin2 = 24; #connect to pin 17 of esp

GPIO.setup(ir1, GPIO.IN)
GPIO.setup(ir2, GPIO.IN)
GPIO.setup(ir3, GPIO.IN)
GPIO.setup(ir4, GPIO.IN)
GPIO.setup(ir5, GPIO.IN)
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
    
    def rightForward(self):
        # rotate right then forward
        self.rotatebot(-90)
        self.stop()
        self.direction = "forward"
    
    def leftForward(self):
        # rotate left then forward
        self.rotatebot(90)
        self.stop()
        self.direction = "forward"
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # start movement
        self.publisher_.publish(twist)
        
    # function to set the class variables using the odometry information
    def odom_callback(self, msg):
        print('a')
        # self.get_logger().info(msg)
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
        #read inputs
        s1 = GPIO.input(ir1)
        s2 = GPIO.input(ir2)
        s3 = GPIO.input(ir3)
        s4 = GPIO.input(ir4)
        s5 = GPIO.input(ir5)
        
        #if linefollower doesn't detect black
        #(by right at the middle of the 2 doors)
        #it will stop 
        #then go left or right depending on which door is unlocked
        if ((s1 == 0) and (s2 == 0) and (s3 == 0) and (s4 == 0) and (s5 == 0)):
            self.direction = "stop"
            self.stop()
            #assuming door 1 is on the left
            if (GPIO.input(esp32_pin1) and not(GPIO.input(esp32_pin2))):
                #run for 3 seconds
                self.direction = "leftForward"
                self.leftForward()
                time.sleep(3)
            #if rpi received that door 2 is unlocked
            elif (GPIO.input(esp32_pin2) and not(GPIO.input(esp32_pin1))):
                self.direction = "rightForward"
                self.rightForward()
                #run for 3 seconds
                time.sleep(3)
            else:
                self.direction = "stop"
                self.stop()
                time.sleep(3)
            
        #follow black line, forward, right, left, stop
        else:
            #forward
            if ((s1 == 1) and (s2 == 1) and (s3 == 0) and (s4 == 1) and (s5 == 1)):
                self.direction = "forward"
                self.forward()
            #right
            elif (
                    ((s1 == 1) and (s2 == 0) and (s3 == 1) and (s4 == 1) and (s5 == 1)) or
                    ((s1 == 0) and (s2 == 1) and (s3 == 1) and (s4 == 1) and (s5 == 1)) or
                    ((s1 == 1) and (s2 == 0) and (s3 == 0) and (s4 == 1) and (s5 == 1)) or
                    ((s1 == 0) and (s2 == 0) and (s3 == 0) and (s4 == 1) and (s5 == 1))
            ):
                self.direction = "right"
                self.right()
            #left
            elif (
                    ((s1 == 1) and (s2 == 1) and (s3 == 1) and (s4 == 0) and (s5 == 1)) or
                    ((s1 == 1) and (s2 == 1) and (s3 == 1) and (s4 == 1) and (s5 == 0)) or
                    ((s1 == 1) and (s2 == 1) and (s3 == 0) and (s4 == 0) and (s5 == 1)) or
                    ((s1 == 1) and (s2 == 1) and (s3 == 0) and (s4 == 0) and (s5 == 0))
            ):
                self.direction = "left"
                self.left()
        
    def move(self):
        while (True):
            rclpy.spin_once(self)
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
    
