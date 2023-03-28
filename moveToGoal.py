import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Pose #I added this to subscribe to map2base
import math
import cmath
import numpy as np
import json ## I added this to retrieve the coordinates from a file

# constants
rotatechange = 0.5
speedchange = 0.05
angle_error = 5 # in degrees
x_error = 0.05 
y_error = 0.05 

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

        # create subscription to get location values from map2base
        self.map2base_subscription = self.create_subscription(
            Pose,
            '/map2base',
            self.map2base_callback,
            10)
        self.map2base_subscription # prevent unused variable warning
        # initialize variables
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
        # create Point object
        goal = Point()
        # create Twist object
        twist = Twist()

        # the x-coordinate we need to go to
        goal.x = next_x
        # the y-coordinate we need to go to
        goal.y = next_y
        # define reached variable to end the while loop when the robot reaches a point
        reached = False

        # allow the callback functions to run
        rclpy.spin_once(self)
        # inc_x is the difference in the x-coordinate between goal and current
        inc_x = goal.x - self.x_coordinate
        # inc_y is the difference in y-coordinate between goal and current
        inc_y = goal.y - self.y_coodinate
        # self.get_logger().info('inc_x: %f inc_y: %f' %(inc_x, inc_y))

        # angle_to_coordinate uses the atan2 function to compute the angle from x-axis to the coordinate in the counter-clockwise direction
        angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
        # angle_to_turn stores the angle between the robots 0 degree and the coordinate
        angle_to_turn = angle_to_goal - math.degrees(self.yaw)
        # the if statement checks if the angle to turn is > angle_error   
        if abs(angle_to_turn) > angle_error:
                self.get_logger().info('Turning: %f degrees' % (angle_to_turn))
                # calls the rotatebot function to rotate the robot angle_to_turn degrees
                self.rotatebot(angle_to_turn)
        # prints that the robot is facing the goal
        self.get_logger().info('Facing goal. Now going straight.')

        # the while loop continues until the robot reaches the goal
        while (reached == False):
            # allow the callback functions to run
            rclpy.spin_once(self)
            # the if statement checks if the difference between the goal's, x or y, coordinate and the current, x or y, coordinate is > x_error or > y_error respecitively
            if ((abs(goal.x - self.x_coordinate) > x_error) or (abs(goal.y - self.y_coodinate) > y_error)):
                # move forward
                twist.linear.x += speedchange
                twist.angular.z = 0.0
            
            else:
                # stop moving
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # changed the reached variable to True in order to exit the loop
                reached = True

            # publish the movement commands
            self.publisher_.publish(twist)

        # self.get_logger().info('reached goal')


    # function to use waypoints to navigate to individual Tables
    def moveToTable(self, table_num):
        # loads the coordinate data from the wayPointsData.json file into the variable data, as a dictionary
        with open('/home/vaibhav/colcon_ws/src/auto_nav/auto_nav/wayPointsData.json') as f:
            data = json.load(f) 
        # returns the array of the table mentioned by table_num
        current_table = globals()[f"table{table_num}"]

        # goes through each point in the array of current_table
        for point_number in current_table:
            # extracts the x-coordinate of the point in the current_table's array
            x_cord = data['point' + str(point_number)]['x_cord']
            # extracts the y-coordinate of the point in the current_table's array
            y_cord = data['point' + str(point_number)]['y_cord']
            # extracts the orientation of the robot at the point in the currect_table's array
            #orientation = data['point' + point_number]['orientation']  Not using as of now

            # calls the function to move the robot to the point
            self.moveToGoal(x_cord,y_cord)
            # prints to the terminal that the point has been reached
            self.get_logger().info('Reached point %d' %(point_number))
        #prints to the terminal that the table has been reached
        self.get_logger().info('Reached table %d' %(table_num))


    # temporary function to type in the table number
    def readKey(self):
        try:
            while True:
                # get keyboard input
                cmd_char = int(input("Please enter the table number: "))
                
                # move to table number cmd_char
                self.moveToTable(cmd_char)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            twist = Twist()
        	# stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


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