import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats


# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1
threshold = 20

#class for the nodes
class Pos:
    def __init__(self,pos):
        self.pos = pos
        self.mark = "unmarked"
    def adj(self,Map,dick):
        lst = []
        for i in range(-1,min(2,len(Map)-self.pos[0])):
            for j in range(-1,min(2,len(Map[0])-self.pos[1])):
                if (i == 0 and j ==0) or self.pos[0]+i < 0 or self.pos[1]+j < 0:
                    continue
                if (self.pos[0]+i,self.pos[1]+j) not in dick:
                    dick[(self.pos[0]+i,self.pos[1]+j)] = Pos((self.pos[0]+i,self.pos[1]+j))
                lst.append((self.pos[0]+i,self.pos[1]+j))
                
        return lst
    def val(self,Map):
        return Map[self.pos[0],self.pos[1]]
def mean(lst):
    count = 0
    total = 0
    for i in lst:
        count += 1
        total += i
    return total/count

def findfronteirs(tmap,posi):
    frontiers = []
    dick ={}

    #Function to check if a position is a fronteir or not
    def isFronteir(pos):
        if pos.val(tmap) == 2:
            if pos.pos[0] ==0:
                temp = tmap[:2,:]
            elif pos.pos[0] == len(tmap)-1:
                temp = tmap[pos.pos[0]-1:,:]
            else:
                temp = tmap[pos.pos[0]-1:pos.pos[0]+2,:]

            if pos.pos[1] == 0:
                a = temp[:,:2]
            elif pos.pos[1] == len(tmap[0])-1:
                a = temp[:,pos.pos[1]-1:]
            else:
                a = temp[:,pos.pos[1]-1:pos.pos[1]+2]
            return np.any(a==1)
        return False

    #Defining basic parameters
    qm = []
    qm.append(posi)
    dick[qm[0]] = Pos(posi)
    dick[qm[0]].mark = "Map-Open-List"
       

    while qm:
        p = qm.pop(0)

        if dick[p].mark == 'Map-Close-List':
            continue
        
        if isFronteir(dick[p]):
            qf = []
            nf = []
            qf.append(p)
            dick[p].mark = "Frontier-Open-List"

            while qf:
                q = qf.pop(0)
                if dick[q].mark in ["Map-Close-List","Frontier-Close-List"]:
                    continue
                if isFronteir(dick[q]):
                    nf.append(q)
                    for w in dick[q].adj(tmap,dick):
                        if dick[w].mark not in ["Frontier-Open-List",'Frontier-Close-List','Map-Close-List']:
                            qf.append(w)
                            dick[w].mark = "Frontier-Open-List"
                dick[q].mark = 'Frontier-Close-List'
            if len(nf) > threshold:
                frontiers.append(nf)
            for i in nf:
                dick[i].mark = "Map-Close-List"
                
        for v in dick[p].adj(tmap,dick):
            if dick[v].mark not in ["Map-Open-List","Map-Close-List"]:
                if any([dick[x].val(tmap) == 2 for x in dick[v].adj(tmap,dick)]):
                       qm.append(v)
                       dick[v].mark = "Map-Open-List"
        dick[p].mark = "Map-Close-List"
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
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

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
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
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


        print("Frontier Positions")
        frontier_positions =findfronteirs(odata,(grid_x,grid_y))
        midpoint_positions = []
        for i in frontier_positions:
            midpoint_positions.append([round(mean([x[0] for x in i])),round(mean(x[1] for x in i))])
        print(midpoint_positions)
        
        #Min = abs(midpoint_positions[0][0]) + abs(midpoint_positions[0][1])
        #tem = 0
        #for i in range(len(midpoint_positions)):
        #    if abs(midpoint_positions[0][0]) + abs(midpoint_positions[0][1]) < Min:
        #        Min = abs(midpoint_positions[0][0]) + abs(midpoint_positions[0][1]) < Min
        #        tem = i
        
        #for i in range(-3,3):
        #    for j in range(-3,3):
        #        odata[midpoint_positions[tem][0]+i,midpoint_positions[tem][1]+j] = 0



                    
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


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()

    # create matplotlib figure
    plt.ion()
    plt.show()

    rclpy.spin(occupy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
