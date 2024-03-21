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
GPIO.setmode(GPIO.BCM)

# constants
rotatechange = 0.1
speedchange = 0.05

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

servo_pin = 12
pwm_pin = 18

GPIO.setup(pin_s1, GPIO.IN)
GPIO.setup(pin_s2, GPIO.IN)
GPIO.setup(pin_s3, GPIO.IN)
GPIO.setup(pin_s4, GPIO.IN)
GPIO.setup(pin_s5, GPIO.IN)
GPIO.setup(pin_s6, GPIO.IN)
GPIO.setup(pin_s7, GPIO.IN)
GPIO.setup(pin_s8, GPIO.IN)
GPIO.setup(pwm_pin, GPIO.OUT)


#GPIO.PWM(pin,freq)
#to be used to control the servo
p = GPIO.PWM(servo_pin, 50)

#starting duty cycle of the servo
#from 0.0 to 100.0
p.start(7.5)
print("Setup done")

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
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


# function to check if keyboard input is a number as
# isnumeric does not handle negative numbers
def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False

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

class Scanner(Node):

    #lidar is scanning
    def init(self):
        super().init('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    #finding the shortest distance
    #If the distance is > threshold, keep following blackline
    #If the distance is < threshold, stop for a while, then on the servo
    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(laser_range)
        # log the info
        #self.get_logger().info('Shortest distance at %i degrees' % lr2i) 

        # ROTATE SERVO
        min_value = np.nanmin(laser_range)
        self.get_logger().info(str(min_value))
        
        #dist is smaller than threshold
        #on the servo
        if (min_value < 0.3):
            self.get_logger().info("Distance less than 1m")
            
            #need to check how long it takes to set up 
            #the ball dropper
            time.sleep(10)

            #on the servo to open the cap
            #for now, keep the cap open for 20 seconds
            p.ChangeDutyCycle(5.0)
            time.sleep(20)

            #close the cap
            p.ChangeDutyCycle(2.5)
            time.sleep(1)
            self.get_logger().info("Task done")
        
        #dist is larger than threshold
        #off the servo
        #follow black line, forward, right, left, stop
        else:
            twist = Twist()
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

            else:
                mover = Mover()
                mover.right()


def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)
    p.stop()
    GPIO.cleanup()
    scanner.destroy_node()
    rclpy.shutdown()
    print("Shutting down")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if name == 'main':
    main()
    


