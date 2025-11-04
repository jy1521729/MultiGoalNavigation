#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from custom_nav_interfaces.srv import NavigationExecute
from custom_nav_interfaces.msg import NavigationRequest, Waypoint
from nav_command.tools import load_waypoints
import os


class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')

        self.declare_parameter(
            'api_id',
            1001,
            ParameterDescriptor(description='API id for navigation request')
        )
        self.declare_parameter('waypoints_file', '')

        self.cli = self.create_client(NavigationExecute, '/slam/navigation/execute')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not ready, waiting...')

        # 启动后自动发一次请求
        api_id = self.get_parameter('api_id').value
        self.get_logger().info(f'api_id={api_id}')
        self.send(api_id)

    def get_waypoints(self):
        waypoints_file = self.get_parameter('waypoints_file').value
        waypoints = load_waypoints(waypoints_file, self)
        if not waypoints:
            return None
        return waypoints

    def send(self, api_id: int):
        req = NavigationExecute.Request()
        req.request = NavigationRequest()
        req.request.request_id = 42
        req.request.api_id = api_id
        req.request.route_id = 'R001'
        req.request.route_name = 'demo'

        waypoints = self.get_waypoints() if api_id in [1011] else None
        if waypoints is not None:
            for pt in waypoints:
                wp = Waypoint()
                wp.waypoint_id = pt['id']
                wp.x, wp.y, wp.yaw = pt['x'], pt['y'], pt['yaw']
                wp.voice.voice_id = pt['voice']['voice_id']
                wp.voice.voice_content = pt['voice']['voice_content']
                wp.voice.play_delay = pt['voice']['play_delay']
                if os.path.exists(pt['voice']['file_path']):
                    with open(pt['voice']['file_path'], 'rb') as f:
                        audio_data = f.read()
                    wp.voice.has_audio_file = True
                    wp.voice.audio_file_name = os.path.basename(pt['voice']['file_path'])
                    wp.voice.audio_file_data = list(audio_data)
                else:
                    wp.voice.has_audio_file = False
                req.request.waypoints.append(wp)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(
            f'响应：accepted={resp.response.success}, '
            f'message={resp.response.message}'
        )


def main():
    rclpy.init()
    NavClient()
    rclpy.shutdown()


if __name__ == '__main__':
    main()