import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from std_msgs.msg import String
import py_trees
import time

class NavigateToPoseBt(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose, node: Node, status_pub: rclpy.publisher.Publisher):
        super().__init__(name)
        self.pose = pose
        self.node = node
        self._client = None
        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None
        self.cancel_requested = False
        self.pub = status_pub

        # 创建服务接口
        self.node.create_service(Trigger, 'cancel_navigation', self.cancel_callback)
        self.node.get_logger().info("Cancel navigation service 'cancel_navigation' created.")

    def setup(self, **kwargs):
        self._client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        return True

    def initialise(self):
        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None
        self.cancel_requested = False
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('NavigateToPose action server not ready')
            return
        self.send_goal()
        self.publish_status(f"navigating-{self.name}")

    def update(self):
        if self.cancel_requested:
            self.node.get_logger().warn("Navigation cancelled via service.")
            return py_trees.common.Status.FAILURE

        if self._send_goal_future is None:
            self.node.get_logger().error("Goal was not sent properly.")
            return py_trees.common.Status.FAILURE

        if self._send_goal_future.done():
            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self.node.get_logger().warn("Goal rejected by action server..")
                time.sleep(0.2)
                self.send_goal()
                return py_trees.common.Status.RUNNING

            if self._result_future is None:
                self._result_future = self._goal_handle.get_result_async()

            if self._result_future.done():
                status = self._result_future.result().status
                if status == 4:  # SUCCEEDED
                    self.node.get_logger().info("Navigation succeeded.")
                    self.publish_status(f"reached-{self.name}")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().error(f"Navigation failed with status: {status}")
                    return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self._goal_handle and not self._result_future:
            self._goal_handle.cancel_goal_async()

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = self.pose
        self._send_goal_future = self._client.send_goal_async(goal)
        self.node.get_logger().info(f"Sending navigation goal: {self.pose.pose.position.x}, {self.pose.pose.position.y}")

    def cancel_callback(self, request, response):
        self.cancel_requested = True
        self.node.get_logger().info("Cancel navigation request received.")
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            response.success = True
            response.message = "Navigation goal cancelled."
        else:
            response.success = False
            response.message = "No active goal to cancel."
        return response

    def publish_status(self, status: str):
        msg = String(data=status)
        self.pub.publish(msg)