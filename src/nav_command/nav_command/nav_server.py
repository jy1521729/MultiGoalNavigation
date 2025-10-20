#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_nav_interfaces.srv import NavigationExecute
from nav_command.tools import write_waypoints
import yaml
import threading
import subprocess
import os

class NavServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        self.srv = self.create_service(NavigationExecute, '/slam/navigation/execute', self.nav_command_cb)
        self.current_working_dir= os.getcwd()
        self.get_logger().info(f"Current working directory: {self.current_working_dir}")
        self.current_status = "init"

        self.sub_status = self.create_subscription(
            String, '/bt_status',
            self.status_cb, 10)
    
    def nav_command_cb(self, request, response):
        req = request.request
        self.get_logger().info(f'收到指令: {req.api_id}')

        try:
            if req.api_id == 1001 and len(req.waypoints) > 0:
                self.get_logger().info(f'路线 {req.route_name}，共 {len(req.waypoints)} 个航点')
                waypoints_file = '/tmp/waypoints.yaml'
                write_waypoints(self, req, waypoints_file)
                return self.handle_start_navigation(waypoints_file, response)
            elif req.api_id == 1002:
                self.get_logger().info('紧急停止！')
                return self.handle_stop_navigation(request, response)
            elif req.api_id == 1003:
                return self.handle_status_request(request, response)
            else:
                self.get_logger().info('无效指令')
                response.response.success = False
                response.response.status_code = '000'
        
        except Exception as e:
            self.get_logger().error(f'Error handling command: {str(e)}')
            response.response.success = False
            response.response.status_code = f"Error: {str(e)}"

        return response
    
    def status_cb(self, msg):
        self.current_status = msg.data

    def handle_start_navigation(self, waypoints_file: str, response):
        """Handle start navigation command"""
        # Start navigation in separate thread
        self.navigation_thread = threading.Thread(
            target=self.start_navigation_process,
            args=(waypoints_file,)
        )
        self.navigation_thread.start()
        
        response.response.success = True
        response.response.status_code = "Navigation started successfully"
        
        return response
    
    def handle_stop_navigation(self, request, response):
        """Handle stop navigation command"""
        # Stop navigation
        # self.stop_navigation_process()
        self.emergency_stop()
        
        response.response.success = True
        response.response.status_code = "Navigation stopped successfully"
        
        return response
    
    def handle_status_request(self, request, response):
        """Handle status request"""
        response.response.success = True
        response.response.status_code = self.current_status
        
        return response

    def start_navigation_process(self, waypoints_file: str):
        """Start the navigation process"""
        try:
            self.get_logger().info('Starting navigation process...')
            success = self.start_tasks_in_terminal(waypoints_file)
            
            if not success:
                self.handle_navigation_error("Failed to start navigation")
                return
            
            self.get_logger().info('Navigation process started successfully')
            
        except Exception as e:
            self.handle_navigation_error(f"Error starting navigation: {str(e)}")
    
    def start_tasks_in_terminal(self, waypoints_file: str) -> bool:
        try:
            self.get_logger().info('Starting all tasks in terminal...')

            cmd = [
                'gnome-terminal', '--', 'bash', '-c', f'{self.current_working_dir}/run_all.sh {waypoints_file}; exec bash'
            ]
            subprocess.Popen(cmd)
                        
            self.get_logger().info('gnome-terminal started')

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start all tasks: {str(e)}')
            return False

    def emergency_stop(self):
        try:
            self.get_logger().info('dog emergency_stop')
            cmd = [
                'bash', '-c', f'{self.current_working_dir}/stop.sh; exec bash'
            ]
            subprocess.Popen(cmd)

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start all tasks: {str(e)}')
            return False

    def handle_navigation_error(self, error_message: str):
        """Handle navigation errors"""
        self.get_logger().error(f'Navigation error: {error_message}')
        self.current_status = "error"

def main():
    rclpy.init()
    rclpy.spin(NavServer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
