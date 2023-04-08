import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math

IDEAL_ANGLE = 22
table_range =range(-IDEAL_ANGLE,IDEAL_ANGLE+1,1)

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.min_angle = 0

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array of lidar ranges
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan's
        self.laser_range[self.laser_range == 0] = np.nan

        # find the minimum range in the table range
        min_index = -1 # index of the minimum range
        min_distance = 1000 # distance of the min_index

        # the table range is the range of angles that we are interested in
        for i in table_range:
            distance = self.laser_range[i]
            # if the distance is less than the current minimum distance
            # then update the minimum distance and the index
            if distance < min_distance:
                min_distance = distance
                min_index = i
        
        # convert the index to an angle based on the turtlebot3
        self.min_angle = math.degrees(min_index * msg.angle_increment)
        self.get_logger().info('min_index: %d' % min_index)
        #self.get_logger().info('min_angle: %f' % min_angle)
        self.get_logger().info('min_distance: %f' % min_distance)

    



    
def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin_once(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()