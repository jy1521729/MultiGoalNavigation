#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_nav_interfaces.srv import NavigationExecute
from nav_command.tools import write_waypoints
import subprocess
import os, time

class NavServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        self.srv = self.create_service(NavigationExecute, '/slam/navigation/execute', self.nav_command_cb)
        self.current_working_dir= os.getcwd()
        self.get_logger().info(f"Current working directory: {self.current_working_dir}")
        self.current_status = {'status': 'idle',   # Current status: "idle", "starting", "running", "completed", "error", "emergency", "stopped"
                               'nav_status': 'none',
                               }

        self.sub_status = self.create_subscription(
            String, '/bt_status',
            self.status_cb, 10)
    
    def nav_command_cb(self, request, response):
        req = request.request
        self.get_logger().info(f'收到指令: {req.api_id}')
        self.get_logger().info(f'current_status: {self.current_status}')

        try:
            match req.api_id:
                case 1001:
                    return self.handle_start_navigation(request, response)
                case 1002:
                    self.get_logger().info('紧急停止！')
                    return self.handle_stop_navigation(request, response)
                case 1003: # TODO
                    self.get_logger().info('解除急停')
                    return self.handle_status_request(request, response)
                case 1004: # TODO
                    self.get_logger().info('继续导航')
                    return self.handle_status_request(request, response)
                case 1005:
                    self.get_logger().info('获取当前导航状态')
                    return self.handle_status_request(request, response)
                case 1011:
                    self.get_logger().info('启动所有导航节点！')
                    return self.handle_launch_all(request, response)
                case 1012:
                    self.get_logger().info('关闭所有导航节点！')
                    return self.handle_kill_all(request, response)
                case 1021: # TODO
                    return self.handle_relocalize(request, response)
                case 1022: # TODO
                    return self.handle_relocalize(request, response)
                case _:
                    self.get_logger().info(f'无效指令: 未知的 api_id {req.api_id}')
                    response.response.success = False
                    response.response.status_code = 'Error: Unknown api_id'
        
        except Exception as e:
            self.get_logger().error(f'Error handling command: {str(e)}')
            response.response.success = False
            response.response.status_code = f"Error: {str(e)}"

        return response
    
    def status_cb(self, msg):
        self.current_status['nav_status'] = msg.data
        if msg.data == "finished":
            self.current_status['status'] = "completed"

    def handle_start_navigation(self, request, response):
        """Handle start navigation command"""
        if self.current_status['status'] not in ["starting"]:
            response.response.success = False
            response.response.status_code = f"Nodes not launched OR Navigation already in progress."
            self.get_logger().info("Nodes not launched OR Navigation already in progress.")
            return response

        req = request.request
        if len(req.waypoints) > 0:
            self.get_logger().info(f'路线 {req.route_name}，共 {len(req.waypoints)} 个航点')
            waypoints_file = '/tmp/waypoints.yaml'
            write_waypoints(self, req, waypoints_file)
        else:
            response.response.success = False
            response.response.status_code = "Error: Empty waypoints"
            self.get_logger().info('Error: Empty waypoints')
            return response

        self.launch_nodes(node='btree', waypoints=waypoints_file)
        self.current_status['status'] = "running"
        
        response.response.success = True
        response.response.status_code = "Navigation started successfully"
        
        return response
    
    def handle_stop_navigation(self, request, response):
        """Handle stop navigation command"""
        self.emergency_stop()
        self.current_status['status'] = "emergency"
        
        response.response.success = True
        response.response.status_code = "Navigation stopped successfully"
        
        return response

    def handle_launch_all(self, request, response):
        """Handle launch all command"""
        if self.current_status['status'] not in ["idle", "stopped"]:
            response.response.success = False
            response.response.status_code = "Navigation nodes already launched"
            return response
        self.launch_nodes()
        self.current_status['status'] = "starting"
        
        response.response.success = True
        response.response.status_code = "Navigation nodes launched successfully"
        
        return response
    
    def handle_kill_all(self, request, response):
        """Handle kill all command"""
        self.emergency_stop()
        time.sleep(1)
        self.kill_nodes()
        self.current_status['status'] = "stopped"

        response.response.success = True
        response.response.status_code = "Navigation process stopped successfully"
        
        return response
    
    def handle_relocalize(self, request, response):
        """Handle launch all command"""
        if request.request.api_id == 1021:
            self.get_logger().info('自动定位')
        else:
            self.get_logger().info('手动定位')
        self.pub_initial_pose()
        
        response.response.success = True
        response.response.status_code = "relocalizing"
        
        return response

    def handle_status_request(self, request, response):
        """Handle status request"""
        response.response.success = True
        response.response.status_code = self.current_status['status']
        
        return response
        
    def launch_nodes(self, node='all', waypoints='') -> bool:
        try:
            params = ''
            if node == 'all':
                self.get_logger().info('Launching all nodes...')
            elif node == 'dog_control':
                self.get_logger().info('Launching node dog_control...')
                params = '1'
            elif node == 'btree':
                self.get_logger().info('Launching nodes btree...')
                params = f'2 {waypoints}'
            else:
                self.get_logger().info(f'Unknown node type: {node}')
                return False
            cmd = [
                'bash', '-c', f'{self.current_working_dir}/launch_all.sh {params}'
            ]
            subprocess.Popen(cmd)
            self.get_logger().info(f'{node} node started')

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to launch all nodes: {str(e)}')
            return False
        
    def kill_nodes(self, node='') -> bool:
        try:
            if node == '':
                self.get_logger().info('Killing all nodes...')
            else:
                self.get_logger().info(f'Killing node: {node}...')
            cmd = [
                'bash', '-c', f'{self.current_working_dir}/kill_all.sh {node}'
            ]
            subprocess.Popen(cmd)

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to kill all nodes: {str(e)}')
            return False
        
    def pub_initial_pose(self):
        try:
            self.get_logger().info('publish initial pose')
            cmd = [
                'bash', '-c', f'ros2 run fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0'
            ]
            subprocess.Popen(cmd)

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish initial pose: {str(e)}')
            return False

    def emergency_stop(self):
        try:
            self.get_logger().info('dog emergency_stop')
            cmd = [
                'bash', '-c', f'{self.current_working_dir}/stop.sh'
            ]
            subprocess.Popen(cmd)

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop dog: {str(e)}')
            return False

    def handle_navigation_error(self, error_message: str):
        """Handle navigation errors"""
        self.get_logger().error(f'Navigation error: {error_message}')
        self.current_status['status'] = "error"

def main():
    rclpy.init()
    rclpy.spin(NavServer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
