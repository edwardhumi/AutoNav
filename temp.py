import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cmath
import time


# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1
threshold = 10
proximity_limit = 0.3
rotatechange = 0.2
stop_distance = 0.25
front_angle = 30
angleChange = 10
#waitTime = 60


def Adj(p,tmap):
    ans = []
    for i in range(-1,2):
        for j in range(-1,2):
            if (i==0 and j ==0) or p[0]+i < 0 or p[1] + j < 0 or p[1] + j > len(tmap[0]) -1 or p[0]+i > len(tmap) -1:
                continue
            ans.append((p[0]+i,p[1]+j))
    return ans
    

            

def median(arr):
    ind = round(len(arr)/2)
    return arr[ind]

def sortpos(arr,x,y):
    distancedic = {}
    for i in arr:
        distancedic[(i[0] - x)**2 + (i[1]-y)**2] = i
    keys = sorted(list(distancedic.keys()))
    ans =[]
    for i in keys:
        ans.append(distancedic[i])
    return ans

def ableToTravel(map_odata, curr_x_grid, curr_y_grid, target_x_grid, target_y_grid, resolution):
    margin_grid = (int)(0.1/resolution)
    #print(margin_grid)
    if (target_y_grid == curr_y_grid):
        for x in range (curr_x_grid, target_x_grid, 1):
            y = target_y_grid
            if (np.any(map_odata[math.ceil(y)-margin_grid:math.ceil(y)+margin_grid+1, x] == 3) or np.any(map_odata[math.floor(y)-margin_grid:math.floor(y)+margin_grid, x] == 3)):
                return False    # meet a wall
    elif (target_x_grid == curr_x_grid):
        for y in range (curr_y_grid, target_y_grid, 1):
            x = target_x_grid
            if (np.any(map_odata[math.ceil(y)-margin_grid:math.ceil(y)+margin_grid+1, x] == 3) or np.any(map_odata[math.floor(y)-margin_grid:math.floor(y)+margin_grid, x] == 3)):
                return False    # meet a wall
    else:
        gradient = (float)(target_y_grid - curr_y_grid)/(float)(target_x_grid - curr_x_grid)
        x_init = curr_x_grid #for iteration
        y_init = curr_y_grid
        x_dest = target_x_grid
        if curr_x_grid > target_x_grid:
            x_init = target_x_grid
            y_init = target_y_grid
            x_dest = curr_x_grid
        for x in range (x_init + 1, x_dest, 1):
            y = gradient * (x - x_init) + y_init
            if (np.any(map_odata[math.ceil(y), x] == 3) or np.any(map_odata[math.floor(y), x] == 3)):
                return False    # meet a wall
    return True

def searchCommonPoint(map_odata, curr_x_grid, curr_y_grid, target_x_grid, target_y_grid, map_res):
    queue = []
    visited = []
    markmap ={}
    initial_pos = (curr_y_grid, curr_x_grid)
    queue.append(initial_pos)
    markmap[initial_pos] = "open"

    def adj(p):
        ans = []
        for i in range(-1,2):
            for j in range(-1,2):
                if (i==0 and j ==0) or p[0]+i < 0 or p[1] + j < 0 or p[1] + j > len(map_odata[0]) -1 or p[0]+i > len(map_odata) -1:
                    continue
                ans.append((p[0]+i,p[1]+j))
        return ans
    
    def mark(p):
        return markmap.get(p,'unmarked')
    
    while queue:
        p = queue.pop(0)
        
        if (ableToTravel(map_odata, curr_x_grid, curr_y_grid, p[1], p[0], map_res) and ableToTravel(map_odata, p[1], p[0], target_x_grid, target_y_grid, map_res)):
            return p
        
        for neighbor in adj(p):
            if neighbor not in visited:
                queue.append(neighbor)
                visited.append(neighbor)
                if map_odata[p[0], p[1]] == 3:
                    markmap[p] = "close"
                else:
                    markmap[p] = "open"
    return (target_y_grid, target_x_grid)

    
def findfronteirs(tmap,posi):
    frontiers = []
    markmap ={}

    #Function to check if a position is a fronteir or not
    def isFronteir(pos):
        if tmap[pos[0],pos[1]]   == 2:
            if pos[0] ==0:
                temp = tmap[:2,:]
            elif pos[0] == len(tmap)-1:
                temp = tmap[pos[0]-1:,:]
            else:
                temp = tmap[pos[0]-1:pos[0]+2,:]

            if pos[1] == 0:
                a = temp[:,:2]
            elif pos[1] == len(tmap[0])-1:
                a = temp[:,pos[1]-1:]
            else:
                a = temp[:,pos[1]-1:pos[1]+2]
            return np.any(a==1)
        return False
    
    def mark(p):
        return markmap.get(p,'Unmarked')
    
    def adj(p):
        return Adj(p,tmap)

    #Defining basic parameters
    qm = []
    qm.append(posi)
    markmap[posi] = "Map-Open-List"

    while qm:
        p = qm.pop(0)

        if mark(p) == 'Map-Close-List':
            continue
        
        if isFronteir(p):
            qf = []
            nf = []
            qf.append(p)
            markmap[p] = "Frontier-Open-List"

            while qf:
                q = qf.pop(0)
                if mark(q) in ["Map-Close-List","Frontier-Close-List"]:
                    continue
                if isFronteir(q):
                    nf.append(q)
                    for w in adj(q):
                        if mark(w) not in ["Frontier-Open-List",'Frontier-Close-List','Map-Close-List']:
                            qf.append(w)
                            markmap[w] = "Frontier-Open-List"
                markmap[q] = 'Frontier-Close-List'
            if len(nf) > threshold:
                frontiers.append(nf)
            for i in nf:
                markmap[i] = "Map-Close-List"
                
        for v in adj(p):
            if mark(v) not in ["Map-Open-List","Map-Close-List"]:
                if any([tmap[x[0],x[1]] == 2 for x in adj(v)]):
                       qm.append(v)
                       markmap[v] = "Map-Open-List"
        markmap[p] = "Map-Close-List"
    return frontiers
    


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


class Occupy(Node):
    def __init__(self):
        super().__init__('occupy')
        
        # create subscription to track occupancy
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        
        # create publisher for moving TurtleBot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.target = []
        self.target2 = []
        self.isCrashing = False
        self.crashAngle = 0
        self.abletotravel = True
        self.isSpinning = False
        #self.startTime = time.time()
        #self.pastTargets = []
        
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        
        for i in range(front_angle):
            if (self.laser_range[i] != np.nan and self.laser_range[i] < float(stop_distance)) and (not self.isSpinning):
                self.crashAngle = i
                self.isCrashing = True
                print("CRASHHHH")
                break
            elif (self.laser_range[-1*i] != np.nan and self.laser_range[-1*i] < float(stop_distance)) and (not self.isSpinning):
                self.crashAngle = 360 - i
                self.isCrashing = True
                print("CRASHHHH")
                break
            else:
                self.isCrashing = False
        
        
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher.publish(twist)
        
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
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher.publish(twist)
        self.isSpinning = True

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
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher.publish(twist)
        self.isSpinning = False

    def listener_callback(self, msg):
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            self.stopbot()
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        self.x,self.y = cur_pos.x,cur_pos.y
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.roll,self.pitch,self.yaw = roll,pitch,yaw
        #self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # set current robot location to 0
        odata[grid_y][grid_x] = 0
        target = self.target

        if not target or abs(target[1]-cur_pos.x) + abs(target[0]-cur_pos.y) < proximity_limit:
            # if target:
            #     self.pastTargets.append(self.target)
            # should only search frontier if there is no current target or have reached the current target
            print("Frontier Positions")
            frontier_positions = findfronteirs(odata,(grid_y,grid_x))
            midpoint_positions = []
            for i in frontier_positions:
                midpoint_positions.append(median(i))
            print(midpoint_positions)
            midpoint_positions = sortpos(midpoint_positions,grid_y,grid_x)
            if midpoint_positions:
                target = self.target
                self.target = [midpoint_positions[0][0]*map_res+map_origin.y,midpoint_positions[0][1]*map_res+map_origin.x]
                #self.startTime = time.time()
            else:
                print("no frontier found")
                
        if self.target:
            target_grid = [round((self.target[0]-map_origin.y)/map_res),round((self.target[1]-map_origin.x)/map_res)]
        else:
            target_grid = []
        
        if target_grid:
            abletotravel = ableToTravel(odata, grid_x, grid_y, target_grid[1], target_grid[0], map_res)
            print("Able to travel: ", abletotravel)
            print(grid_y,grid_x)
            print(target_grid)
            print(len(odata),len(odata[0]))
            for i in range(-2,2):
                for j in range(-2,2):
                    odata[target_grid[0]+i,target_grid[1]+j] = 0
            if (not abletotravel):
                self.abletotravel = False
                p = searchCommonPoint(odata, grid_x, grid_y, target_grid[1], target_grid[0], map_res)
                self.target2 = p
                for i in range(-2,2):
                    for j in range(-2,2):
                        odata[p[0]+i,p[1]+j] = 0
            else:
                self.abletotravel = True
                    
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)
            
        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)
            
        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        
    def movetotarget(self, target):
        if target:
            # if time.time() - self.startTime > waitTime and self.pastTargets:
            #     print("TIMES UP")
            #     self.target = self.pastTargets[-2]
            #     self.startTime = time.time()
            if (self.isCrashing):
                # Handles crash avoidance
                print("Avoiding crash")
                self.stopbot()
                closestAngle = np.nanargmin(self.laser_range)
                leftFound = False
                rightFound = False
                for i in range(0,180):
                    anglePos = (closestAngle +i)%360
                    if (not rightFound) and self.laser_range[anglePos] > stop_distance + 0.2:
                        posDisplace = i
                        rightFound = True
                    angleNeg = (closestAngle - i)%360
                    if (not leftFound) and self.laser_range[angleNeg]> stop_distance + 0.2:
                        negDisplace = i
                        leftFound = True
                    if leftFound and rightFound:
                        break
                print(posDisplace, negDisplace)
                posAngle = (closestAngle + posDisplace)%360
                negAngle = (closestAngle - negDisplace)%360
                if posAngle > 180:
                    posAngle = posAngle - 360
                if negAngle > 180:
                    negAngle = negAngle - 360
                
                if posAngle + negAngle < 0:
                    destinationAngle = posAngle + 30
                else:
                    destinationAngle = negAngle - 30
                
                self.rotatebot(destinationAngle)
                twist = Twist()
                twist.linear.x = 0.3
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                time.sleep(1)
                self.stopbot()          
            else:
                # Handles normal movement to target
                print("target acquired")
                print(target)
                angle = np.arctan((target[0]-self.y)/(target[1]-self.x))-self.yaw
                if (target[1]-self.x) < 0:
                    if (target[0]-self.y) > 0:
                        angle += np.pi
                    else:
                        angle -= np.pi
                
                if abs(angle) > 0.2:
                    self.stopbot()
                    self.rotatebot(np.degrees(angle))
                
                print('Start moving')
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.publisher.publish(twist)
        rclpy.spin_once(self) #get the lidar data to prevent crash
            
    def Move(self):
        target = []
        rclpy.spin_once(self)
        print(target)
        while (True):
            rclpy.spin_once(self)
            try:
                target = self.target
                target2 = self.target2
                if not target or abs(target[1]-self.x) + abs(target[0]-self.y) < proximity_limit or abs(target2[1]-self.x) + abs(target2[0]-self.y) < proximity_limit:
                    print("Target reached")
                    self.stopbot()
                else:
                    if self.abletotravel:
                        self.movetotarget(target)
                    else:
                        self.movetotarget(target2)
            except Exception as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()

    # create matplotlib figure
    occupy.Move()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()