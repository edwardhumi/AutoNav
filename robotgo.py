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


# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1
threshold = 20
proximity_limit = 0.5
rotatechange = 0.1


#creating target as an empty list first


#class for the nodes

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
        ans = []
        for i in range(-1,2):
            for j in range(-1,2):
                if (i==0 and j ==0) or p[0]+i < 0 or p[1] + j < 0 or p[1] + j > len(tmap[0]) -1 or p[0]+i > len(tmap) -1:
                    continue
                ans.append((p[0]+i,p[1]+j))
        return ans
                
    
        
    
    

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
                if any([tmap[x[0],x[1]]== 2 for x in adj(v)]):
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
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
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

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            print(c_change_dir * c_dir_diff)
            if c_change_dir * c_dir_diff < 0.1:
                print('cycle broken')
                break
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

        print(target)
        print("Frontier Positions")
        frontier_positions =findfronteirs(odata,(grid_x,grid_y))
        midpoint_positions = []
        for i in frontier_positions:
            midpoint_positions.append(median(i))
        print(midpoint_positions)
        midpoint_positions = sortpos(midpoint_positions,grid_y,grid_x)
        


        if not target or abs(target[1]-cur_pos.x) + abs(target[0]-cur_pos.y) < proximity_limit:
            if midpoint_positions:
                self.target = [midpoint_positions[0][0]*map_res+map_origin.y,midpoint_positions[0][1]*map_res+map_origin.x]
        if target:
            target_grid = [round((target[0]-map_origin.y)/map_res),round((target[1]-map_origin.x)/map_res)]
        else:
            target_grid = []
        
        if target_grid:
            for i in range(-2,2):
                for j in range(-2,2):
                    odata[target_grid[0]+i,target_grid[1]+j] =0
                    
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
        
    def movetotarget(self):
        target = self.target
        if target:
            print("target acquired")
            angle = np.arctan((target[0]-self.y)/(target[1]-self.x))-self.yaw
            
            if angle > 0.1:
                self.rotatebot(angle)
            
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            self.publisher.publish(twist)
            print('Start moving')
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            self.publisher.publish(twist)
    def Move(self):
        target = self.target
        rclpy.spin_once(self)
        print(target)
        while (True):
            rclpy.spin_once(self)
            print(target)
            try:
                if not target or abs(target[1]-self.x) + abs(target[0]-self.y) < proximity_limit:
                    self.stopbot()
                    self.movetotarget()
            except:
                print("eeror?")
            
            
            
            

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
