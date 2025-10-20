import py_trees
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialLocalization(py_trees.behaviour.Behaviour):
    """初始定位节点：检查定位协方差是否收敛"""
    def __init__(self, name, node: Node, default_pose, tol=0.15, max_retry=3):
        super().__init__(name)
        self.default_pose = default_pose
        self.tol = tol
        self.max_retry = max_retry
        self.node = node
        self._retry = 0
        self._sub = None
        self._pub = None
        self._latest_cov = float('inf')
        self._latest_msg = None

    # --- py_trees_ros 接口 --------------
    def setup(self, **kwargs):
        self._sub = self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._amcl_cb, 10)
        return True

    def _amcl_cb(self, msg):
        # self._latest_cov = msg.pose.covariance[0]   # σ_x²
        self._latest_msg = msg

    def initialise(self):
        self._retry = 0
        self._latest_cov = float('inf')

    def update(self):
        # 还没收到任何消息
        if self._latest_msg is None:
            self.feedback_message = "等待 /amcl_pose ..."
            self.node.get_logger().info(self.feedback_message)
            time.sleep(1.0)
            return py_trees.common.Status.RUNNING

        time.sleep(1.0)
        # 取 x 方差（covariance[0]）
        x_var = self._latest_msg.pose.covariance[0]
        self.feedback_message = f"收到 /amcl_pose, 方差 = {x_var:.3f}"

        if x_var < self.tol:
            self.feedback_message += " → 满足阈值，定位完成"
            self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message += " → 方差过大，继续等待"
            self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self._sub:
            self.node.destroy_subscription(self._sub)
            self._sub = None
