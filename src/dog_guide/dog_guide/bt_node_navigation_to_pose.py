import py_trees
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigateToPoseBt(py_trees.behaviour.Behaviour):
    """包装 NavigateToPose 动作"""
    def __init__(self, name, pose, node: Node):
        super().__init__(name)
        self.pose = pose
        self.node = node
        self._client = None
        self._send_goal_future = None
        self._result_future = None

    def setup(self, **kwargs):
        self._client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        return True

    def initialise(self):
        self._send_goal_future = None
        self._result_future = None
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('NavigateToPose action server not ready')
        goal = NavigateToPose.Goal()
        goal.pose = self.pose
        self._send_goal_future = self._client.send_goal_async(goal)

    def update(self):
        if self._send_goal_future is None:
            return py_trees.common.Status.FAILURE
        if self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            if self._result_future is None:
                self._result_future = goal_handle.get_result_async()
            if self._result_future.done():
                status = self._result_future.result().status
                if status == 4:   # SUCCEEDED
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if (self._result_future is None and self._send_goal_future is not None and
            self._send_goal_future.done() and
            self._send_goal_future.result().accepted):
            self._send_goal_future.result().cancel_goal()