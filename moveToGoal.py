import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Pose # I added this to subscribe to map2base
import math
import cmath
import numpy as np
import json # I added this to retrieve the coordinates from a file
import paho.mqtt.client as mqtt
import socket
from std_msgs.msg import String, Bool # I added this to subscribe to ir_state and switch_state
import time

# constants
rotatechange = 0.1
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

table_num = -1 # table number Note set to -1 so that it acts as a flag


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

        # create subscription to get location values from /map2base
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
        self.yaw = 0 # yaw of the bot in the map

        # create subscription to get IR sensor data from /ir_state
        self.ir_state_subscription = self.create_subscription(
            String,
            'ir_state',
            self.ir_state_callback,
            10)
        self.ir_state_subscription # prevent unused variable warning
        # initialize variables
        self.ir_state = '' # IR sensor data from the bot: f = forward,r = right, l = left, s = stop

        # create subscription to get switch sensor data from /switch_state
        self.switch_state_subscription = self.create_subscription(
            Bool,
            'switch_state',
            self.switch_state_callback,
            10)
        self.switch_state_subscription # prevent unused variable warning
        # initialize variables
        self.switch_state = False # switch state: True = pressed, False = not pressed

        # variable to store if line is found
        self.foundLine = False
        # variable to count the number of times 's' is received from the IR sensor while docking
        self.count_stop = 0


    # function to set the class variables using the /map2base information
    def map2base_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In map2base_callback')
        orientation_quat_map = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat_map.x, orientation_quat_map.y, orientation_quat_map.z, orientation_quat_map.w)
        position_map = msg.position
        self.x_coordinate = position_map.x
        self.y_coodinate = position_map.y


    # function to set class variables using the /ir_state information
    def ir_state_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In ir_state_callback')
        self.ir_state = msg.data


    # function to set class variables using the /switch_state information
    def switch_state_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In switch_state_callback')
        self.switch_state = msg.data


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

        # twist msg to do straight
        twist.linear.x += speedchange
        twist.angular.z = 0.0
        # the while loop continues until the robot reaches the goal
        # the if statement checks if the difference between the goal's, x or y, coordinate and the current, x or y, coordinate is > x_error or > y_error respecitively
        while((abs(goal.x - self.x_coordinate) > x_error) or (abs(goal.y - self.y_coodinate) > y_error)):
            # allow the callback functions to run
            rclpy.spin_once(self)
            self.publisher_.publish(twist)
        
        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # publish the movement commands
        self.publisher_.publish(twist)


    # function to dock the robot
    def dock(self):
        # create Twist object
        twist = Twist()
        # set twist such that it rotates in point
        twist.linear.x = 0.0
        twist.angular.z += rotatechange
        # keeps rotating until line is found
        while(self.foundLine == False):
              rclpy.spin_once(self)
              if(self.ir_state == 'r'):
                    self.get_logger().info('Found line')
                    self.foundLine = True
                    twist.angular.z -= rotatechange
                    self.publisher_.publish(twist)
                    time.sleep(1)
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    break
              else:
                    self.publisher_.publish(twist)
        
        # now that we have the line in between the ir sensors, we will now start the docking
        self.get_logger().info('Docking...')
        # How the code works is that the robot will keep following the line until it reaches the end of the line.
        # It will only stop completely if the ir_state becomes 's' twice.
        # Hence, the loop will only stop if count_stop becomes 2
        while(self.count_stop != 2):
            rclpy.spin_once(self)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # if the right ir sensor detects a line, then the robot will turn right
            if(self.ir_state == 'r'):
                twist.linear.x = 0.0
                twist.angular.z -= rotatechange
                self.publisher_.publish(twist)
            # if the left ir sensor detects a line, then the robot will turn left
            elif(self.ir_state == 'l'):
                twist.linear.x = 0.0
                twist.angular.z += rotatechange
                self.publisher_.publish(twist)
            # if both ir sensors dont detect a line, then the robot will move forward
            elif(self.ir_state == 'f'):
                twist.linear.x += -(speedchange - 0.03)
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            # if both ir sensors detect a line and the count_stop = 0, then the robot will stop
            elif(self.ir_state == 's' and self.count_stop == 0):
                self.count_stop += 1
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            # if count_stop = 1 then the robot will wait for 2 seconds and check if the ir_state is still 's'
            # if it is, then the robot will stop completely
            # if it is not, then the robot will restart the docking process
            # this is to prevent the robot from stopping when it approaches the line at 90 degrees
            elif(self.count_stop == 1):
                twist.linear.x += -(speedchange - 0.03)
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info('waiting for 2 seconds')
                time.sleep(2) # sleep in seconds
                rclpy.spin_once(self)
                if(self.ir_state == 'f'):
                    self.count_stop = 0
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.get_logger().info('Robot was at 90 degrees at line. Therefore restarted docking.')
                elif(self.ir_state == 's'):
                    self.count_stop += 1

        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


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

        # wait for the can to be picked (switch_state to become false)
        while(self.switch_state):
            rclpy.spin_once(self)
        # prints to the terminal that the can has been picked
        self.get_logger().info('Can picked')

        # goes through each point in the array of current_table in reverse order
        for point_number in reversed(current_table):
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
        # prints to the terminal that robot has reached the docking point
        self.get_logger().info('Reached docking point')

        # initiates docking
        self.dock()
        # once docked, prints to the terminal that the robot has docked
        self.get_logger().info('Docked!')



# function to store the msg sent by the esp32 into the global variable table_num
def on_table_num(client, userdata, msg):
    global table_num 
    table_num = int(msg.payload.decode('utf-8'))
    print(table_num) # added cuz without this IT WONT WORK


# main function
def main(args=None):
     global table_num
     rclpy.init(args=args)
     # to get ip address of the laptop
     my_ip = socket.gethostbyname(socket.gethostname())

     # to connect to the mqtt broker
     client = mqtt.Client("Turtlebot")
     client.message_callback_add('esp32/output', on_table_num)
     client.connect(my_ip, 1883)
     client.loop_start()
     client.subscribe("esp32/output")

     # to start the navigation based on the table number esp32 sends. The code runs forever.
     navigation = Navigate()
     while True:
         #print (table_num)
         if(table_num != -1):
             # send message to esp32 to tell it that the robot has un-docked and is moving to the table
             client.publish("esp32/input", "0")
             #navigation.moveToTable(table_num)
             navigation.dock()
             table_num = -1
             # send message back to esp32 to tell it that the robot has docked
             client.publish("esp32/input", "1")
         
         pass
     
     navigation.destroy_node()
     rclpy.shutdown()
    
        

if __name__ == '__main__':
    main()