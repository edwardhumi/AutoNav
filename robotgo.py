#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 13 11:25:05 2024

@author: john
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 28 04:33:40 2024

@author: john
"""

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
import time
import heapq
from std_msgs.msg import String


# constants
occ_bins = [-1, 0, 55, 100]
map_bg_color = 1
threshold = 0.3
proximity_limit = 0.25
target_limit = 0.7
rotatechange = 0.25
stop_distance = 0.25
front_angle = 35
precission = 0.15
angleChange = 10
testing = True
wall_size = 0.06
    
#waitTime = 60
            
#Function to find adjacent cells in a map
def Adj(p,tmap):
    ans = []
    for i in range(-1,2):
        for j in range(-1,2):
            if (i==0 and j ==0) or p[0]+i < 0 or p[1] + j < 0 or p[1] + j > len(tmap[0]) -1 or p[0]+i > len(tmap) -1:
                continue
            ans.append((p[0]+i,p[1]+j))
    return ans

#Function to find the median points of a frontier
def median(arr):
    ind = round(len(arr)/2)
    return arr[ind]

#Function to find frontiers
def findfronteirs(tmap,posi,map_res):
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
    
    #Function to check if a cell has been marked
    def mark(p):
        return markmap.get(p,'Unmarked')
    
    def adj(p):
        return Adj(p,tmap)

    #Creating the map queue 
    qm = []
    
    #Adding robot's current position to the map queue
    qm.append(posi)
    markmap[posi] = "Map-Open-List"

    while qm:
        #Dequeueing first element of map queue
        p = qm.pop(0)

        #Skipping the element if it has already been mapped
        if mark(p) == 'Map-Close-List':
            continue
        
        #Checking if the cell is a frontier
        if isFronteir(p):
            
            #Creating a queue for finding adjacent frontiers qf
            #Creating a list of frontier points nf
            
            qf = []
            nf = []
            qf.append(p)
            
            #Marking the point as to be opened by the frontier
            markmap[p] = "Frontier-Open-List"

            while qf:
                #Dequeuing the first element of qf
                q = qf.pop(0)
                
                #Skipping the cell if it's already been mapped or if it has been searched through 
                if mark(q) in ["Map-Close-List","Frontier-Close-List"]:
                    continue
                #If the point is a frontier, add it to the nf arrray
                if isFronteir(q):
                    nf.append(q)
                    #Check for cells adjacent to current frontier point
                    for w in adj(q):
                        #If there is a neighbor that has not been added to qf, and has not been mapped or checked add it to the queue
                        if mark(w) not in ["Frontier-Open-List",'Frontier-Close-List','Map-Close-List']:
                            qf.append(w)
                            #Mark the newly added cell as frontier open list
                            markmap[w] = "Frontier-Open-List"
                #Mark the original point as being checked
                markmap[q] = 'Frontier-Close-List'
            #Adds the list of frontier points to the frontiers array
            if len(nf) > round(threshold/map_res):
                frontiers.append(nf)
            #Marking all points in the nf array as being closed by the map
            for i in nf:
                markmap[i] = "Map-Close-List"
        #Generating neighbors for the cells in the map queue
        for v in adj(p):
            #Adding any neighbors that have not been opened or checked
            if mark(v) not in ["Map-Open-List","Map-Close-List"]:
                if any([tmap[x[0],x[1]] == 2 for x in adj(v)]):
                       qm.append(v)
                       markmap[v] = "Map-Open-List"
        #Marking the point as checked in the map
        markmap[p] = "Map-Close-List"
    #returning the final list of frontiers 
    return frontiers
    

#Separate function to check if a point is a frontier or not, used in navigaiton
def isFronteir(tmap,pos):
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

#Function to check if a wall is between two points in a map
def is_wall_between(occupancy_array, point1, point2, map_res):
    print('mapRes = ',map_res)
    width = round(0.1/map_res)                                                                                                                            
    width = max(3,width)
    y1, x1 = point1
    y2, x2 = point2

    # Calculate the differences in y and x coordinates
    dy = y2 - y1
    dx = x2 - x1

    def is_wall(occupancy_array, y, x):
        # Check if the coordinates are within the occupancy array bounds
        if 0 <= y < len(occupancy_array) and 0 <= x < len(occupancy_array[0]):
            # Check if the cell contains a wall
            return occupancy_array[y, x] == 3
        else:
            # If coordinates are out of bounds, consider it a wall
            return True

    # Calculate step direction
    # Calculate absolute differences
    dy = abs(dy)
    dx = abs(dx)

    # Check for steepness of the line
    steep = dy > dx

    if steep:
        # If steep, swap coordinates
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    if x1 > x2:
        # If x1 > x2, swap coordinates
        x1, x2 = x2, x1
        y1, y2 = y2, y1

    # Recalculate differences
    dx = x2 - x1
    dy = abs(y2 - y1)

    # Initialize error
    error = dx // 2
    y = y1

    # Determine y step direction
    y_step = 1 if y1 < y2 else -1

    # Iterate over the line
    for x in range(x1, x2 + 1):
        # Check the points around the line within the width
        for i in range(-width, width + 1):
            if steep:
                if is_wall(occupancy_array, x + i, y):
                    return True
            else:
                if is_wall(occupancy_array, y + i, x):
                    return True

        # Update error
        error -= dy
        if error < 0:
            # Move to next y coordinate
            y += y_step
            error += dx

    # If no walls found, return False
    return False


#Heuristics for the A* algorithm, main heuristic is Euclidian distance 
#Additional cost of being too close to a wall added
def heuristic(node, target, occupancy_data, map_res):
    nrows = len(occupancy_data)
    ncols = len(occupancy_data[0])
    max_penalty_distance = round(0.25/map_res)
    # Euclidean distance between current node and target
    euclidean_distance = ((node[0] - target[0])**2 + (node[1] - target[1])**2) ** 0.5
    
    # Penalties for proximity to walls
    wall_penalty = 0 # You can adjust this distance as needed
    
    for dx in range(-max_penalty_distance, max_penalty_distance + 1):
        for dy in range(-max_penalty_distance, max_penalty_distance + 1):
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < nrows and 0 <= ny < ncols and occupancy_data[nx, ny] == 3:  # Wall detected
                distance_to_wall = ((dx**2 + dy**2) ** 0.5)
                wall_penalty += max_penalty_distance - distance_to_wall

    # Combine Euclidean distance and wall penalty
    total_heuristic = euclidean_distance + wall_penalty
    
    return total_heuristic

#Generating neighbors for A*
def find_neighbors(node, occupancy_data, nrows, ncols):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1),(1,1),(1,-1),(-1,1),(-1,-1)]
    for dir in directions:
        neighbor = (node[0] + dir[0], node[1] + dir[1])
        if 0 <= neighbor[0] < nrows and 0 <= neighbor[1] < ncols:
            if occupancy_data[neighbor[0], neighbor[1]] not in  (1,3):  # Check if it is not wall
                neighbors.append(neighbor)
    return neighbors

#Function to return a nearest path from one point to another
def a_star(start, target, occupancy_data, nrows, ncols, map_res):
    open_set = []
    closed_set = set()
    heapq.heappush(open_set, (0, start))
    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, target,occupancy_data, map_res)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == target:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current)

        for neighbor in find_neighbors(current, occupancy_data, nrows, ncols):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + 1  # Assuming cost of moving from one cell to another is 1

            if neighbor not in [x[1] for x in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target, occupancy_data,map_res)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

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
        
        # create subscription to track current stage
        self.subscription2 = self.create_subscription(
                String,
                'stage',
                self.stage_callback,
                10)
        self.subscription2
        
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
        self.target = [] #Current immediate target of the robot
        self.isCrashing = False #If the robot is inbound for a crash
        self.isSpinning = False #If the robot is rotating towards its target
        self.path = [] #Current path of the robot
        self.currentFrontier = [] #Current frontier the robot is moving towards
        self.targetReached = False #If the robot is closed enough to its current target
        self.avoidingCrash = False #If the robot is avoiding a wall
        
        # signal to start the program
        self.start_autonav = True

    #Callback to check the stage of the robot
    def stage_callback(self, msg):
        self.get_logger().info('Received Stage: "%s":' % msg.data)
        if (msg.data == "autonav"):
            self.start_autonav = True
    
    #Callback to get the LiDAR reading and check for crashes
    def scan_callback(self, msg):
        if self.start_autonav:
            # self.get_logger().info('In scan_callback')
            # create numpy array
            self.laser_range = np.array(msg.ranges)
            # replace 0's with nan
            self.laser_range[self.laser_range==0] = np.nan
            
            #Crashing triggers when an object is at a certain distance in front of the robot
            #Crashing does not trigger during rotation towards a target
            #Robot should stop unless is doing crash avoidance
            for i in range(front_angle):
                if (self.laser_range[i] != np.nan and self.laser_range[i] < float(stop_distance)) and (not self.isSpinning) and self.target:
                    self.crashAngle = i
                    self.isCrashing = True
                    print("CRASHHHH")
                    if not self.avoidingCrash:
                        self.stopbot()
                    break
                elif (self.laser_range[-1*i] != np.nan and self.laser_range[-1*i] < float(stop_distance)) and (not self.isSpinning) and self.target:
                    self.crashAngle = 360 - i
                    self.isCrashing = True
                    print("CRASHHHH")
                    if not self.avoidingCrash:
                        self.stopbot()
                    break
                else:
                    self.isCrashing = False

    #Stops the robots
    def stopbot(self):
        #self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher.publish(twist)
    
    #Rotates the bot a certain angle
    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        if abs(rot_angle) < 40:
            rotatechange = 0.2
        else:
            rotatechange = 0.4
        
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
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
        #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            # if self.angularspeed == 0:
            #     break\
            self.isSpinning = True
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # self.get_logger().info('Target Yaw: %f' %math.degrees(target_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher.publish(twist)
        self.isSpinning = False
        
    #Main callback function
    def listener_callback(self, msg):
        if self.start_autonav:
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
                trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.01))
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                print(e)
                self.stopbot()
                return
            
            cur_pos = trans.transform.translation
            cur_rot = trans.transform.rotation
            self.x,self.y = cur_pos.x,cur_pos.y
            # convert quaternion to Euler angles
            roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
            self.roll,self.pitch,self.yaw = roll,pitch,yaw
            #self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
    
            # get map resolution
            map_res = msg.info.resolution
            self.map_res = map_res
            # get map origin struct has fields of x, y, and z
            map_origin = msg.info.origin.position
            # get map grid positions for x, y position
            grid_x = round((cur_pos.x - map_origin.x) / map_res)
            grid_y = round(((cur_pos.y - map_origin.y) / map_res))
            # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))
            self.origin = [map_origin.y,map_origin.x]

            # binnum go from 1 to 3 so we can use uint8
            # convert into 2D array using column order
            odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
            
            #Checks if target has been reached
            if self.target:
                mapTarget = (round((self.target[0] - map_origin.y)/map_res), round((self.target[1]-map_origin.x)/map_res))
                if odata[mapTarget[0],mapTarget[1]] == 3:
                    self.target = []
                    self.path = []
                    self.targetReached = True
            if not self.target or (((self.target[1]-self.x)**2 + (self.target[0]-self.y)**2)**0.5 < proximity_limit and\
                                   not is_wall_between(odata, (grid_y,grid_x), mapTarget, map_res) and not self.isSpinning):
                self.targetReached = True
                self.isCrashing = False
                print('target Reached')
                self.stopbot()
        
                           
            odata[grid_y][grid_x] = 0
            
            if self.targetReached:
                self.stopbot()
                
                #Creating a modified occupancy map with expanded walls
                ndata = np.copy(odata)
                size = round(wall_size/map_res)
                #size = 1
                for i in range(len(odata)):
                    for j in range(len(odata[0])):
                        if odata[i,j] == 3:
                            for k in range(-size,size):
                                for l in range(-size,size):
                                    if (0 < i+k < len(odata) and 0 < j+l < len(odata[0])) and abs(i+k - grid_y) + abs(j+l - grid_x) > 0.40/map_res :
                                        ndata[i+k,j+l] = 3

                self.targetReached = False
                if self.path:
                    #Checks if current frontier is stil a frontier
                    if self.currentFrontier:
                        print("Checking Fronteir")
                        if not isFronteir(odata, (round((self.currentFrontier[0]-map_origin.y)/map_res),round((self.currentFrontier[1]-map_origin.x)/map_res))):
                            #If frontier has already been mapped clear path, target and frontier
                            self.path = []
                            self.currentFrontier = []
                            self.target = []
                            print("Frontier Removed")

                    #Removing points close to the robot
                    remove_index = []
                    goesThroughWall = False
                    for i in self.path:
                        if ((i[0] - self.y)**2 + (i[1] - self.x)**2)**0.5 < target_limit:
                            if not is_wall_between(ndata, (grid_y,grid_x),(round((i[0]-map_origin.y)/map_res),round((i[1]-map_origin.x)/map_res)), map_res):
                                remove_index.append(i)
                            elif odata[round((i[0]-map_origin.y)/map_res),round((i[1]-map_origin.x)/map_res)] == 3:
                                goesThroughWall = True
                                remove_index.append(i)
                        else:
                            break
                    for i in remove_index:
                        self.path.remove(i)
                    if goesThroughWall:
                        self.path = []
                    
                    #If the entier path is removed, reset everything and set targetReached to be true
                    if self.path:
                        print('Target Set')
                        self.target = self.path.pop(0)
                    else:
                        print('Frontier Filled')
                        self.targetReached = True
                        self.target = []
                
                #If path is empty, initialize path finding
                if not self.path:
                    self.stopbot()

                    #Generates frontier positions
                    frontier_positions = findfronteirs(odata,(grid_y,grid_x), map_res)
                    
                    #Adds the median of the frontiers to an array
                    midpoint_positions = []
                    for i in frontier_positions:
                        midpoint_positions.append(median(i))
        
                
                    if midpoint_positions:
                        # WALL THICKENING
                        odata = ndata
                        
                        
                        a_star_list = []
    
                        for i in range(len(midpoint_positions)):
                            self.stopbot()
                            print('astar running')
                            test_target = midpoint_positions[i]   
                            
                            # Searches through the array for travellable frontiers
                            a_star_list = a_star((grid_y, grid_x), (test_target[0], test_target[1]), odata, iheight, iwidth, map_res)
                            if a_star_list:
                                print(a_star_list)
                                break
                        
                        #Converts path fron A* algorithm into real coordinates in meters
                        if a_star_list:
                            path = a_star_list
                            realpath = [] #path in real coordinates
                            for point in path:
                                realpath.append([point[0] * map_res + map_origin.y, point[1] * map_res + map_origin.x])
                            self.path = realpath[1:]
                            self.isCrashing = False
                            self.currentFrontier = self.path[-1]
                            #print('path' , self.path)
                            self.target = self.path.pop(0)
                        else:
                            self.target = []
                            print('no path found')
                            self.stopbot()
            
            #Prints out image of path if is debugging
            if testing:
                for i in self.path:
                    npoint = (round((i[0]-map_origin.y)/map_res),round((i[1]-map_origin.x)/map_res))
                    odata[npoint[0],npoint[1]] = 0
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
                #new_width = iwidth + right + left
                #new_height = iheight + top + bottom
                #img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
                #img_transformed.paste(img, (left, top))
        
                # rotate by 90 degrees so that the forward direction is at the top of the image
                #rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)
                #rotated.save('Map.png',cmap = 'gray',origin = 'lower')
        
                # show the image using grayscale map
                plt.imshow(img, cmap='gray', origin='lower')
                # plt.imshow(img_transformed, cmap='gray', origin='lower')
                #plt.imshow(rotated, cmap='gray', origin='lower')
                plt.draw_all()
                # pause to make sure the plot gets created
                plt.pause(0.00000000001)
    
    #Main method to move towards a target
    def movetotarget(self, target):
        if target:
            #Locate relative yaw of the target
            angle = np.arctan((target[0]-self.y)/(target[1]-self.x))
            if (target[1]-self.x) < 0:
                if (target[0]-self.y) > 0:
                    angle += np.pi
                else:
                    angle -= np.pi
            #print(np.degrees(self.yaw))
            #print(np.degrees(angle))
            angle = angle - self.yaw
            #print(np.degrees(angle))
            
            if angle > np.pi:
                angle = angle - 2 * np.pi
            if angle < -np.pi:
                angle = angle + 2 * np.pi
            
            #Rotate towards the target        
            if abs(np.degrees(angle)) > 10 :




                self.stopbot()
                self.rotatebot(np.degrees(angle))    
            
            #Callback to check for crashing
            rclpy.spin_once(self)
        
            if (self.isCrashing):
                # Handles crash avoidance
                print("Avoiding crash")
                self.stopbot()
                closestAngle = np.nanargmin(self.laser_range)
                degreeNum = len(self.laser_range)
                
                for i in range(len(self.laser_range)):
                    print(i, self.laser_range[i])
                
                leftFound = False
                rightFound = False
                
                #Looks for open angles with 35cm clearance
                for i in range(0,round(degreeNum/2)):
                    anglePos = (closestAngle +i)%degreeNum
                    angleNeg = (closestAngle- i)%degreeNum
                    try:
                        if (not rightFound) and abs(self.laser_range[anglePos]) > 0.35:
                            posDisplace = i
                            rightFound = True
                            #print(anglePos)
                        angleNeg = (closestAngle - i)%degreeNum
                        if (not leftFound) and abs(self.laser_range[angleNeg]) > 0.35:
                            negDisplace = i
                            leftFound = True
                            #print(angleNeg)
                        if leftFound and rightFound:
                            break
                    except:
                        continue
                if not leftFound:
                    negDisplace = round(degreeNum/2)
                if not rightFound:
                    posDisplace = round(degreeNum/2)
                #print(posDisplace,negDisplace)
                posAngle = (closestAngle + posDisplace)%degreeNum
                negAngle = (closestAngle - negDisplace)%degreeNum
                #print(posAngle,negAngle)
                if posAngle > degreeNum/2:
                    posAngle = posAngle - degreeNum
                if negAngle > degreeNum/2:
                    negAngle = negAngle - degreeNum

                if posAngle + negAngle < 0:
                    print('LEFT\n\n\n\n')
                else:
                    print('RIGHT\n\n\n\n')
                
                # print("Pos Angle = {}, negAngle = {}".format(posAngle,negAngle))
                # print("destination angle: ",destinationAngle)
                while self.isCrashing:
                    rclpy.spin_once(self)
                    #Moves towards the clear space
                    self.avoidingCrash = True
                    twast = Twist()
                    twast.linear.x = 0.0
                    if posAngle + negAngle > 0:
                        angular = -0.2
                    else:
                        angular = 0.2
                    twast.angular.z = angular
                    self.publisher.publish(twast)
                    time.sleep(0.1)
                self.stopbot()
                print('crash avoided')
                rclpy.spin_once(self)
                
                #Moves forward if robot is not detecting a crash
                if not self.isCrashing:
                    twist = Twist()
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    print('moving away')
                    time.sleep(1)
                self.avoidingCrash = False
                self.stopbot()          
            else:
                # Handles normal movement to target                       
                print('Start moving')
                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.publisher.publish(twist)
        rclpy.spin_once(self) #get the lidar data to prevent crash
            
    def Move(self):
        while (True):
            rclpy.spin_once(self)
            #Only activates if stage is in autonav
            if self.start_autonav:
                try:
                    if self.targetReached: 
                        self.stopbot()
                    else:
                        self.movetotarget(self.target)
                except Exception as e:
                    print(e)
                    print('b')


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
