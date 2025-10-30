#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_nav_interfaces.msg import NavigationStatus
from custom_nav_interfaces.srv import NavigationExecute
from nav_command.tools import write_waypoints, write_waypoints_status
import subprocess
import os, time

class NavServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        self.srv = self.create_service(NavigationExecute, '/slam/navigation/execute', self.nav_command_cb)
        self.current_working_dir= os.getcwd()
        self.get_logger().info(f"Current working directory: {self.current_working_dir}")

        self.waypoints_status = {}
        self.waypoints_file = '/tmp/waypoints.yaml'

        self.current_status = NavigationStatus()
        self.current_status.status = "idle"     # "idle", "starting", "running", "completed", "error", "emergency", "stopped"
        self.current_status.phase = ""          # "localizing", "navigating", "voice_playing"
        self.current_status.error_code = ""
        self.current_status.error_message = ""
        self.current_status.device_connect = True
        self.current_status.localization_ready = False
        self.current_status.navigation_ready = False

        self.sub_status = self.create_subscription(
            String, '/bt_status',
            self.status_cb, 10)

        self.pub_status = self.create_publisher(NavigationStatus, '/navigation_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)
    
    def nav_command_cb(self, request, response):
        req = request.request
        self.get_logger().info(f'收到指令: {req.api_id}')
        self.print_status()

        try:
            match req.api_id:
                case 1001:
                    return self.handle_start_navigation(request, response, is_continue=False)
                case 1002:
                    self.get_logger().info('紧急停止！')
                    return self.handle_stop_navigation(request, response)
                case 1003:
                    self.get_logger().info('解除急停')
                    return self.handle_dog_control(request, response)
                case 1004:
                    self.get_logger().info('继续导航')
                    return self.handle_start_navigation(request, response, is_continue=True)
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
                    response.response.success = False
                    response.response.message = f'Error: Unknown api_id {req.api_id}'
                    self.print_response(response, log_level='error')
        
        except Exception as e:
            response.response.success = False
            response.response.message = f"Error handling command: {str(e)}"
            self.print_response(response, log_level='error')

        return response
    
    def status_cb(self, msg):
        self.current_status.phase = msg.data
        if msg.data == "finished":
            self.current_status.status = "completed"
        elif msg.data == "localization_success":
            self.current_status.localization_ready = True
        elif msg.data.startswith("reached-"):
            waypoint_id = msg.data.split("-")[1]
            if waypoint_id in self.waypoints_status:
                self.waypoints_status[waypoint_id] = 'completed'
                self.get_logger().info(f'Waypoint {waypoint_id} reached. Updated waypoints status: {self.waypoints_status}')
            else:
                self.get_logger().warn(f'Received reached status for unknown waypoint id: {waypoint_id}')

    def handle_start_navigation(self, request, response, is_continue=False):
        """Handle start navigation command"""
        if self.current_status.navigation_ready == False:
            response.response.success = False
            response.response.message = f"Navigation nodes not launched."
            self.print_response(response, log_level='warn')
            return response
        if self.current_status.status in ["running", 'error']:
            response.response.success = False
            response.response.message = f"Navigation already in progress."
            self.print_response(response, log_level='warn')
            return response

        if not is_continue:
            self.init_waypoints_status(None)
            self.get_logger().info(f'开始新导航, waypoints status: {self.waypoints_status}')
        else:
           self.get_logger().info(f'继续导航, waypoints status: {self.waypoints_status}')

        write_waypoints_status(self.waypoints_file, self.waypoints_status)
        time.sleep(1)
        self.launch_nodes(node='btree', waypoints=self.waypoints_file)
        self.current_status.status = "running"
        
        response.response.success = True
        response.response.message = "Navigation started successfully"
        self.print_response(response)
        return response
    
    def handle_stop_navigation(self, request, response):
        """Handle stop navigation command"""
        self.emergency_stop()
        time.sleep(0.1)
        self.kill_nodes(node='dog_control')
        if self.current_status.navigation_ready and self.current_status.status == "running":
            self.cancel_goal()
        self.current_status.status = "emergency"
        
        response.response.success = True
        response.response.message = "Navigation stopped successfully"
        self.print_response(response)
        return response
    
    def handle_dog_control(self, request, response):
        if self.current_status.status != "emergency":
            response.response.success = False
            response.response.message = "Dog is not in emergency state"
            self.print_response(response, log_level='warn')
            return response
        self.launch_nodes(node='dog_control')
        self.current_status.status = "starting"
        
        response.response.success = True
        response.response.message = "Dog ready for navigation"
        self.print_response(response)
        return response

    def handle_launch_all(self, request, response):
        """Handle launch all command"""
        if self.current_status.navigation_ready:
            response.response.success = False
            response.response.message = "Navigation nodes already launched"
            self.print_response(response, log_level='warn')
            return response
        
        req = request.request
        if len(req.waypoints) > 0:
            self.get_logger().info(f'路线 {req.route_name}，共 {len(req.waypoints)} 个航点')
            write_waypoints(self, req, self.waypoints_file)
            self.init_waypoints_status(req.waypoints)
        else:
            response.response.success = False
            response.response.message = "Error: Empty waypoints"
            self.print_response(response, log_level='error')
            return response

        self.launch_nodes()
        self.current_status.status = "starting"
        self.current_status.navigation_ready = True
        
        response.response.success = True
        response.response.message = "Navigation nodes launched successfully"
        self.print_response(response)
        return response
    
    def handle_kill_all(self, request, response):
        """Handle kill all command"""
        self.emergency_stop()
        time.sleep(1)
        self.kill_nodes()
        self.current_status.status = "stopped"
        self.current_status.navigation_ready = False
        self.current_status.localization_ready = False

        response.response.success = True
        response.response.message = "Navigation process stopped successfully"
        self.print_response(response)
        return response
    
    def handle_relocalize(self, request, response):
        """Handle launch all command"""
        if request.request.api_id == 1021:
            self.get_logger().info('自动定位')
        else:
            self.get_logger().info('手动定位')
        self.pub_initial_pose()
        
        response.response.success = True
        response.response.message = "relocalizing"
        self.print_response(response)
        return response

    def handle_status_request(self, request, response):
        """Handle status request"""
        response.response.success = True
        response.response.message = self.current_status.status
        self.print_response(response)
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
                self.get_logger().warn(f'Unknown node type: {node}')
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
        
    def cancel_goal(self):
        try:
            self.get_logger().info('Cancel current navigation goal...')
            cmd = [
                'bash', '-c', f'ros2 service call /cancel_navigation std_srvs/srv/Trigger'
            ]
            subprocess.Popen(cmd)

            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to cancel goal: {str(e)}')
            return False

    def handle_navigation_error(self, error_message: str):
        """Handle navigation errors"""
        self.get_logger().error(f'Navigation error: {error_message}')
        self.current_status.status = "error"

    def init_waypoints_status(self, waypoints):
        self.get_logger().info('Initializing waypoints status...')
        if waypoints is None or len(waypoints) == 0:
            for key in self.waypoints_status.keys():
                self.waypoints_status[key] = 'pending'
        else:
            self.waypoints_status = {wp.waypoint_id: 'pending' for wp in waypoints}
    
    def print_status(self):
        self.get_logger().info(f'Current: status={self.current_status.status},'
                               f'phase={self.current_status.phase},'
                               f'navigation_ready={self.current_status.navigation_ready},'
                               f'localization_ready={self.current_status.localization_ready}')

    def publish_status(self):
        self.pub_status.publish(self.current_status)

    def print_response(self, response, log_level='info'):
        if log_level == 'info':
            self.get_logger().info(f"Response: {response.response.success}, {response.response.message}")
        elif log_level == 'warn':
            self.get_logger().warn(f"Response: {response.response.success}, {response.response.message}")
        elif log_level == 'error':
            self.get_logger().error(f"Response: {response.response.success}, {response.response.message}")

def main():
    rclpy.init()
    rclpy.spin(NavServer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
