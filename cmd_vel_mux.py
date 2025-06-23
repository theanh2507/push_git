#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.nav2_sub = self.create_subscription(Twist, '/cmd_vel', self.nav2_callback, 10)
        self.pause_sub = self.create_subscription(Bool, '/pause_cmd', self.pause_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)

        self.paused = False                         # false: resume, true: pause
    
    def pause_callback(self, msg: Bool):
        self.paused = msg.data
        print(self.paused)

    def nav2_callback(self, msg: Twist):
        vel_mux = Twist()
        if(self.paused == True):
            vel_mux.linear.x = 0.0
            vel_mux.linear.y = 0.0
            vel_mux.linear.z = 0.0
            vel_mux.angular.x = 0.0
            vel_mux.angular.y = 0.0
            vel_mux.angular.z = 0.0
        else:
            vel_mux.linear.x = msg.linear.x
            vel_mux.linear.y = msg.linear.y
            vel_mux.linear.z = msg.linear.z
            vel_mux.angular.x = msg.angular.x
            vel_mux.angular.y = msg.angular.y
            vel_mux.angular.z = msg.angular.z

        self.cmd_vel_pub.publish(vel_mux)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
