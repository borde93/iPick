#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from swiftpro_interfaces.action import GrabStrawberry
from geometry_msgs.msg import PointStamped
from swiftpro_interfaces.msg import ArmPointDetection

class GrabStrawberryClientNode(Node):
    def __init__(self):
        super().__init__("grab_strawberry_client")
        #self.pick_in_progress = False
        self.goal_handle_ = None
        self.grab_strawberry_client_ = ActionClient(self, GrabStrawberry, "grab_strawberry")
        self._target_sub_ = self.create_subscription(ArmPointDetection, '/yolo/strawberry_arm_coord', self.send_goal, 10)

    # Wait for action server, send a goal, and register a callback for the response
    # Also register another callback for the optional feedback



    def send_goal(self, target: PointStamped):
        self.grab_strawberry_client_.wait_for_server()
        goal = GrabStrawberry.Goal()
        goal.target = target
        self.grab_strawberry_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback)


    # Method to send a cancel request for the current goal
    def cancel_goal(self):
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()


    # Get the goal response and if accepted, register a callback for the result
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    # Get the goal result and print it
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Result: {result.move_completed}")

    # # Get the goal feedback and print it
    # def goal_feedback_callback(self, feedback_msg):
    #     number = feedback_msg.feedback.current_number
    #     self.get_logger().info("Got feedback: " + str(number))
    #     # if number >= 2:
    #     #    self.cancel_goal()


def main(args=None):
    rclpy.init(args=args)
    node = GrabStrawberryClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()