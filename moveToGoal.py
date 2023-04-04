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
ROTATE_CHANGE = 0.1
SPEED_CHANGE = 0.05
ANGLE_ERROR = 1.0
DIST_ERROR = 0.08
ANGLE_CHECK_DISTANCE = 0.5

# defining the individual tables 'points' based on the wayPointsData.json file
table1 = [1,2,3] 
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
        # IR sensor data from the bot: f = forward,r = right, l = left, s = stop
        self.ir_state = '' 
        # variable to store if line is found
        self.foundLine = False
        # variable to count the number of times 's' is received from the IR sensor while docking
        self.count_stop = 0

        # create subscription to get switch sensor data from /switch_state
        self.switch_state_subscription = self.create_subscription(
            Bool,
            'switch_state',
            self.switch_state_callback,
            10)
        self.switch_state_subscription # prevent unused variable warning
        # initialize variables
        self.switch_state = False # switch state: True = pressed, False = not pressed




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
        # distance_to_goal is the distance between the goal and the current position uses the pythagorean theorem
        distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)

        # angle_to_coordinate uses the atan2 function to compute the angle from x-axis to the coordinate in the counter-clockwise direction
        angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
        # angle_to_turn stores the angle between the robots 0 degree and the coordinate
        angle_to_turn = angle_to_goal - math.degrees(self.yaw)
        
        # the if statement checks if the absolute of angle_to_turn is > ANGLE_ERROR   
        while abs(angle_to_turn) > ANGLE_ERROR:
                # cleares the twist object to avoid adding values
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # allow the callback functions to run
                rclpy.spin_once(self)

                # updates the angle_to_turn
                angle_to_turn = angle_to_goal - math.degrees(self.yaw)
                # prints the angle to turn
                self.get_logger().info('Turning: %f degrees' % (angle_to_turn))
                
                #if the angle to turn is > 0, the robot turns left else it turns right
                if (angle_to_turn > 0):
                    twist.angular.z += ROTATE_CHANGE
                elif (angle_to_turn < 0):
                    twist.angular.z -= ROTATE_CHANGE
                # publishes the twist object
                self.publisher_.publish(twist)
        # stops the robot
        twist.linear.x = 0.0  
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        # prints that the robot is facing the goal
        self.get_logger().info('Facing goal. Now going straight.')

        # twist msg to go straight
        ##twist.linear.x += SPEED_CHANGE
        ##twist.angular.z = 0.0
        # initialize variables
        distance_traveled = 0.0 # distance traveled by the robot
        last_angle_check_distance = 0.0 # distance traveled by the robot when the last angle check was done
        distance_left = distance_to_goal # distance left to travel by the robot

        # while loop to move the robot straight until it reaches the goal
        while(distance_to_goal > DIST_ERROR):
            # cleares the twist object to avoid adding values
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # allow the callback functions to run to update the distance to goal
            rclpy.spin_once(self)

            # updates the distance to goal
            inc_x = goal.x - self.x_coordinate
            inc_y = goal.y - self.y_coodinate
            distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)
            # prints the distance to goal
            self.get_logger().info('distance: %f' %(distance_to_goal))
            # updates the distance traveled
            distance_traveled = distance_left - distance_to_goal
            # moves the robot straight
            twist.linear.x += SPEED_CHANGE
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            # if statement to check if the robot has traveled ANGLE_CHECK_DISTANCE since the last angle check
            if distance_traveled - last_angle_check_distance >= ANGLE_CHECK_DISTANCE:
                # updates the angle to goal and angle to turn
                rclpy.spin_once(self)
                angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
                angle_to_turn = angle_to_goal - math.degrees(self.yaw)

                # the if statement checks if the absolute of angle_to_turn is > ANGLE_ERROR   
                while abs(angle_to_turn) > ANGLE_ERROR:
                        # cleares the twist object to avoid adding values
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        # allow the callback functions to run
                        rclpy.spin_once(self)

                        # prints the correcting angle
                        self.get_logger().info('correcting angle')
                        # updates the angle_to_turn
                        angle_to_turn = angle_to_goal - math.degrees(self.yaw)
                        # prints the angle to turn
                        self.get_logger().info('Turning: %f degrees' % (angle_to_turn))
                
                        #if the angle to turn is > 0, the robot turns left else it turns right
                        if (angle_to_turn > 0):
                            twist.angular.z += ROTATE_CHANGE
                        elif (angle_to_turn < 0):
                            twist.angular.z -= ROTATE_CHANGE
                        # publishes the twist object
                        self.publisher_.publish(twist)
                # stops the robot
                twist.linear.x = 0.0  
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                # updates the last angle check distance
                last_angle_check_distance = distance_traveled
            # while loop ends here
        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    # function to dock the robot
    def dock(self):
        # create Twist object
        twist = Twist()
        # set twist such that it rotates in point
        twist.linear.x = 0.0
        twist.angular.z += ROTATE_CHANGE
        # keeps rotating until line is found
        while(self.foundLine == False):
              rclpy.spin_once(self)
              if(self.ir_state == 'r'):
                    self.get_logger().info('Found line')
                    self.foundLine = True
                    twist.angular.z -= ROTATE_CHANGE
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
                twist.angular.z -= ROTATE_CHANGE
                self.publisher_.publish(twist)
            # if the left ir sensor detects a line, then the robot will turn left
            elif(self.ir_state == 'l'):
                twist.linear.x = 0.0
                twist.angular.z += ROTATE_CHANGE
                self.publisher_.publish(twist)
            # if both ir sensors dont detect a line, then the robot will move forward
            elif(self.ir_state == 'f'):
                twist.linear.x += -(SPEED_CHANGE - 0.03)
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
                twist.linear.x += -(SPEED_CHANGE - 0.03)
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

        # resets foundLine and count_stop to initial values
        self.foundLine = False
        self.count_stop = 0
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
        rclpy.spin_once(self)
        while(self.switch_state == True):
            rclpy.spin_once(self)
        # prints to the terminal that the can has been picked
        self.get_logger().info('Can picked')
        time.sleep(4) # wait for some time after can is picked

        # goes through each point in the array of current_table in reverse order
        for point_number in list(reversed(current_table))[1:]:
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
             navigation.moveToTable(table_num)
             #navigation.dock()
             table_num = -1
             # send message back to esp32 to tell it that the robot has docked
             client.publish("esp32/input", "1")
         
         pass
     
     navigation.destroy_node()
     rclpy.shutdown()
    
        

if __name__ == '__main__':
    main()