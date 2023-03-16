#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus as ActionGoalStatus
from move_base_msgs.action import MoveBase


class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal')
        self.client = self.create_client(MoveBase, 'move_base')
        self.goal_handle = None

    def send_goal(self, x, y, theta):
        goal = MoveBase.Goal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.wait_for_service()
        self.goal_handle = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.goal_handle.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        print('Feedback received: {0}'.format(feedback_msg))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected')
            return
        print('Goal accepted')

        self.get_logger().info('Waiting for result')
        while rclpy.ok():
            status = goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                print('Goal succeeded')
                result = goal_handle.result
                if result:
                    print('Result:', result.result)
                return
            elif status == GoalStatus.STATUS_ABORTED:
                print('Goal aborted')
                return
            elif status == GoalStatus.STATUS_CANCELED:
                print('Goal canceled')
                return

            self.get_logger().info('Goal status: {0}'.format(status))
            rclpy.spin_once(self)

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            print('Goal canceled')
        else:
            print('No goal to cancel')

def main(args=None):
    rclpy.init(args=args)
    send_goal = SendGoal()
    send_goal.send_goal(0.0, 0.0, 0.0)

    try:
        while rclpy.ok():
            # Do some work here...
            send_goal.get_logger().info('Sending goal...')
            rclpy.spin_once(send_goal)
    except KeyboardInterrupt:
        pass

    send_goal.cancel_goal()
    send_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
