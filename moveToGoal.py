import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Pose #I added this to subscribe to map2base
import math
import cmath
import numpy as np
import json ## I added this to retrieve the coordinates from a file

# constants
rotatechange = 0.1
speedchange = 0.05

# defining the individual tables 'points' based on the wayPointsData.json file
table1 = [1,2] 
table2 = [1,3]
table3 = [1,4]
table4 = [1,5]
table5 = [1,6,7]
table6 = [1,8,9]

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


# class for moving the robot based on coordinates
class Navigate(Node):
    def __init__(self):
        super().__init__('moveToGoal')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')

        ## create subscription to get location values from map2base
        self.map2base_subscription = self.create_subscription(
            Pose,
            '/map2base',
            self.map2base_callback,
            10)
        self.map2base_subscription # prevent unused variable warning
        self.x_coordinate = 0 # x-coordinate of the bot in the map
        self.y_coodinate = 0 # y-coordinate of the bot in the map
        self.roll = 0 # roll of the bot in the map (not needed)
        self.pitch = 0 # pitch of the bot in the map (not needed)
        self.yaw = 0 # yaw of the bot in the map (needed)

    # function to set the class variables using the map2base information
    def map2base_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In map2base_callback')
        orientation_quat_map = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat_map.x, orientation_quat_map.y, orientation_quat_map.z, orientation_quat_map.w)
        position_map = msg.position
        self.x_coordinate = position_map.x
        self.y_coodinate = position_map.y

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

    # function to move to goal
    def moveToGoal(self, next_x, next_y):
        goal = Point()
        goal.x = next_x
        goal.y = next_y
        reached = False

        speed = Twist()

        rclpy.spin_once(self)
        inc_x = goal.x - self.x_coordinate
        inc_y = goal.y - self.y_coodinate
        
        self.get_logger().info('inc_x: %f inc_y: %f' %(inc_x, inc_y))

        angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
        #self.get_logger().info(('angle is: %f' %(angle_to_goal)))
            
        if abs(angle_to_goal - math.degrees(self.yaw)) > 5:
                #self.get_logger().info('in angle thing')
                self.get_logger().info('angleToGoal: %f' % (angle_to_goal - math.degrees(self.yaw)))
                self.rotatebot(angle_to_goal - math.degrees(self.yaw))
        self.get_logger().info('moving straight to goal')
        while (reached == False):
            rclpy.spin_once(self)

            if ((abs(goal.x - self.x_coordinate) > 0.05) or (abs(goal.y - self.y_coodinate) > 0.05)):        
                speed.linear.x = 0.5
                speed.angular.z = 0.0
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                reached = True       
            self.publisher_.publish(speed)
        #self.get_logger().info('reached goal')

    # function to use waypoints to navigate to individual Tables
    def moveToTable(self, table_num):
        # loads the coordinate data from the wayPointsData.json file into the variable data, as a dictionary
        with open('/home/vaibhav/colcon_ws/src/auto_nav/auto_nav/wayPointsData.json') as f:
            data = json.load(f) 

        # returns the array of the table mentioned by table_num
        current_table = globals()[f"table{table_num}"]
        #current_table = table1

        for point_number in current_table:
            x_cord = data['point' + str(point_number)]['x_cord']
            y_cord = data['point' + str(point_number)]['y_cord']
            #orientation = data['point' + point_number]['orientation']  Not using as of now

            self.moveToGoal(x_cord,y_cord)
            self.get_logger().info('Reached point %d' %(point_number))
        
        self.get_logger().info('Reached table %d' %(table_num))

    # temporary function to type in the table number
    def readKey(self):
        try:
            while True:
                # get keyboard input
                cmd_char = int(input("Please enter the table number: "))
                
                self.moveToTable(cmd_char)
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    navigation = Navigate()    
    navigation.readKey()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigation.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()