import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from tf_transformations import euler_from_quaternion

class PointCloudOdomParser(Node):
    def __init__(self):
        super().__init__('pointcloud_odom_parser')

        # 订阅点云和里程计话题
        self.create_subscription(PointCloud2, '/cloud_in_map', self.pointcloud_callback, 10)
        self.create_subscription(Odometry, '/localization', self.odom_callback, 10)

    def pointcloud_callback(self, msg: PointCloud2):
        # 解析点云中的每个点的坐标
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.get_logger().info("Received PointCloud2 data:")
        for i, point in enumerate(points):
            x, y, z = point
            self.get_logger().info(f"Point {i}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            if i >= 4:  # 只打印前5个点
                break

    def odom_callback(self, msg: Odometry):
        # 提取位置
        pos = msg.pose.pose.position
        # 提取方向（四元数）
        ori = msg.pose.pose.orientation
        # 转换为欧拉角
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.get_logger().info("Received Odometry data:")
        self.get_logger().info(f"Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        self.get_logger().info(f"Orientation (Euler): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOdomParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
