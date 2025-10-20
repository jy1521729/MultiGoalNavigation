import os, subprocess, time, math
import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

from dog_guide.bt_node_initial_loc import InitialLocalization
from dog_guide.bt_node_navigation_to_pose import NavigateToPoseBt

class PlaySpeech(py_trees.behaviour.Behaviour):
    """播放一段语音"""
    def __init__(self, name, node: Node, speech_file):
        super().__init__(name)
        self.node = node
        self.file = speech_file
    def update(self):
        voice_id = self.file['voice_id']
        play_delay = self.file['play_delay']
        self.node.get_logger().info(f"voice_id: {voice_id}, play_delay: {play_delay}s")
        time.sleep(play_delay)
        for i in range(3):
            self.node.get_logger().info(f"sound playing: {self.file['voice_content']}")
            time.sleep(1)
        self.node.get_logger().info(f"Finished playing sound {self.name}")
        return py_trees.common.Status.SUCCESS


def create_tree(waypoints, node: Node):
    """根据 waypoints 动态拼装行为树"""
    root = py_trees.composites.Sequence("TourSequence", memory=True)
    for idx, wp in enumerate(waypoints):
        seq = py_trees.composites.Sequence(f"Spot{idx}", memory=True)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
        nav = NavigateToPoseBt(f"GoTo{idx}", pose, node)
        speech = PlaySpeech(f"Speak{idx}", node, wp['voice'])

        seq.add_children([nav, speech])
        root.add_child(seq)
    return root


def create_full_tree(waypoints, node: Node):
    """返回一颗完整树：初始定位 -> 依次导航+讲解"""
    root = py_trees.composites.Sequence("FullTask", memory=True)

    # ---- 1. 初始定位节点 ---------------------------
    default_pose = PoseWithCovarianceStamped()
    default_pose.header.frame_id = "map"
    default_pose.pose.pose.position.x = 0.0
    default_pose.pose.pose.position.y = 0.0
    default_pose.pose.pose.orientation.w = 1.0
    default_pose.pose.covariance[0] = 0.5   # 初始给个大方差
    init_loc = InitialLocalization("InitialLocalization",
                                   node,
                                   default_pose,
                                   tol=0.3,
                                   max_retry=3)
    # 用 RetryNode 包一层，可再重试 2 轮
    retry_init = py_trees.decorators.Retry(name="RetryInit", num_failures=2, child=init_loc)

    # ---- 2. 原来的 waypoint 序列 -------------------
    tour_seq = create_tree(waypoints, node)   # 之前写的导航+语音序列

    # ---- 3. 拼接 ----------------------------------
    root.add_children([retry_init, tour_seq])
    return root

def pub_status(pub, status: str):
    msg = String()
    msg.data = status
    pub.publish(msg)

import yaml
def main():
    rclpy.init()
    node = rclpy.create_node('temp_for_bt')   # 临时节点，仅用来读参+创建树
    node.declare_parameter('waypoints_file', '')
    waypoints_file = node.get_parameter('waypoints_file').value
    if not waypoints_file:
        node.get_logger().error('waypoints_file 参数为空！')
        return
    
    pub = node.create_publisher(String, "/bt_status", 10)
    pub_status(pub, "running")

    # 手动读 YAML
    with open(waypoints_file, 'r', encoding='utf-8') as f:
        waypoints = yaml.safe_load(f)['waypoints']

    if not waypoints:
        node.get_logger().error('waypoints 参数为空，检查 waypoints_file 是否加载')
        return

    # 用前面写的 create_tree 拼出行为树
    tree = create_full_tree(waypoints, node)
    tree.setup_with_descendants()

    # 周期 tick
    try:
        while rclpy.ok():
            tree.tick_once()
            if tree.status == py_trees.common.Status.SUCCESS:
                node.get_logger().info('全部点位完成！')
                pub_status(pub, "finished")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
