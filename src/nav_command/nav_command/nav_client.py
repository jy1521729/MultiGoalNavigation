#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from custom_nav_interfaces.srv import NavigationExecute
from custom_nav_interfaces.msg import NavigationRequest, Waypoint


class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')

        self.declare_parameter(
            'api_id',
            1001,
            ParameterDescriptor(description='API id for navigation request')
        )

        self.cli = self.create_client(NavigationExecute, '/slam/navigation/execute')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not ready, waiting...')

        # 启动后自动发一次请求
        api_id = self.get_parameter('api_id').value
        self.get_logger().info(f'api_id={api_id}')
        self.send(api_id)

    def send(self, api_id: int):
        req = NavigationExecute.Request()
        req.request = NavigationRequest()
        req.request.request_id = 42
        req.request.api_id = api_id
        req.request.route_id = 'R001'
        req.request.route_name = 'demo'

        wp = Waypoint()
        wp.x, wp.y, wp.yaw = 3.0, 0.0, 0.0
        wp.voice.voice_id = 'v1'
        wp.voice.voice_content = '这是第 1 个点'
        wp.voice.play_delay = 3.0

        wp2 = Waypoint()
        wp2.x, wp2.y, wp2.yaw = 4.0, -3.0, 1.57
        wp2.voice.voice_id = 'v2'
        wp2.voice.voice_content = '这是第 2 个点'
        wp2.voice.play_delay = 2.0

        wp3 = Waypoint()
        wp3.x, wp3.y, wp3.yaw = 4.0, -10.0, -1.57
        wp3.voice.voice_id = 'v3'
        wp3.voice.voice_content = '这是第 3 个点'
        wp3.voice.play_delay = 4.0

        req.request.waypoints = [wp, wp2, wp3]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(
            f'响应：accepted={resp.response.success}, '
            f'status_code={resp.response.status_code}'
        )


def main():
    rclpy.init()
    NavClient()
    rclpy.shutdown()


if __name__ == '__main__':
    main()