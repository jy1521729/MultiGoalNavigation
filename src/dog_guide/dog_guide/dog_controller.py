import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import mc_sdk_py 
import time

app=mc_sdk_py.HighLevel()
v_limit = 1.5
a_limit = 1.0


class ZslDogControllerNode(Node):
    def __init__(self):
        super().__init__('zsl_dog_controller_node')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)

    def velocity_callback(self, msg):
        self.get_logger().info(f'Raw velocity: Vx={msg.linear.x:.5f}, Vy={msg.linear.y:.5f}, VA={msg.angular.z:.5f}')
        
        Vx = self.value_limit(msg.linear.x, v_limit, 0.41, 0.2)
        Vy = self.value_limit(msg.linear.y, v_limit, 0.41, 0.2)
        Va = self.value_limit(msg.angular.z, a_limit, 0.35, 0.1)

        self.get_logger().info(f'接收到速度指令: Vx={Vx:.5f}, Vy={Vy:.5f}, VA={Va:.5f}')
        app.move(Vx,Vy,Va)
        pass


    def value_limit(self, vel, vel_max_limit, vel_min_value, zero_limit):
        if vel < -vel_max_limit:
            return -vel_max_limit
        elif vel > vel_max_limit:
            return vel_max_limit
        
        if(abs(vel) < zero_limit):
            return 0.0
        
        if vel < 0 and vel > -vel_min_value:
            return -vel_min_value
        if vel > 0 and vel < vel_min_value:
            return vel_min_value
        return vel


def main(args=None):
    print("Initializing...")
    app.initRobot("192.168.168.99",43988, "192.168.168.168") #local_ip, local_port, dog_ip
    time.sleep(1)
    app.standUp()
    time.sleep(4)
    print("Initialization completed")
    rclpy.init(args=args)
    controller_node = ZslDogControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except:
        app.passive()
        time.sleep(2)