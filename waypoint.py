#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String 
from nav2_msgs.msg import BehaviorTreeLog


class WaypointFollow(Node):
    def __init__(self):
        super().__init__("waypoint_follow")

        # Publish at 1 Hz
        self.timer_period = 1.0  # seconds

        self.check_goal = 0

        self.goal_result = False

        self.msg = String()

        self.goal_index = 0

        self.commands = None

        # publisher vi tri cho robot di chuyen
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # publisher trang thai result khi da den dich cua robot len gui
        self.publisher_result = self.create_publisher(String, "/gui_node", 10)

        # nhan ten cua table tu Gui
        self.subscriptions_ = self.create_subscription(String, '/selected_table', self.receive_data_callback, 10)            

        # nhan result robot (xem robot den noi chua)
        self.subscriptions_goal_result = self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self.goal_result_callback, 10)

        # nhan trang thai khi an nut "finish" tu ben Gui de di chuyen den diem tiep theo
        self.finish_subscriptions = self.create_subscription(String, "/finish_node", self.update_commands, 10)

        self.timer_result = self.create_timer(self.timer_period, self.send_result_robot)

        self.declare_parameter("default_frame_id", "map")

    def declare_table_params(self, table_name):
        prefix = f"poses.{table_name}"
        fields = ["position.x", "position.y", "position.z", "orientation.x", "orientation.y", "orientation.z", "orientation.w"]

        for field in fields:
            param_name = f"{prefix}.{field}"
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, 0.0)

    def receive_data_callback(self, msg:String):
        self.data = msg.data.split(",")
        print(self.data)

        self.timer = self.create_timer(self.timer_period, self.send_goal_from_name(self.data[self.goal_index]))
        self.goal_index += 1
        self.timer.cancel()

    def send_goal_from_name(self, table_name: str):
        self.declare_table_params(table_name)

        frame_id = self.get_parameter("default_frame_id").get_parameter_value().string_value

        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.header.stamp = self.get_clock().now().to_msg()

        prefix = f"poses.{table_name}"
        try:
            target_pose.pose.position.x = self.get_parameter(f"{prefix}.position.x").value
            target_pose.pose.position.y = self.get_parameter(f"{prefix}.position.y").value
            target_pose.pose.position.z = self.get_parameter(f"{prefix}.position.z").value

            target_pose.pose.orientation.x = self.get_parameter(f"{prefix}.orientation.x").value
            target_pose.pose.orientation.y = self.get_parameter(f"{prefix}.orientation.y").value
            target_pose.pose.orientation.z = self.get_parameter(f"{prefix}.orientation.z").value
            target_pose.pose.orientation.w = self.get_parameter(f"{prefix}.orientation.w").value

            self.publisher_.publish(target_pose)
            self.get_logger().info(f"Published goal for {table_name}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to read parameters for {table_name}: {e}")
    
    def goal_result_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == "NavigateWithReplanning" and event.current_status == "SUCCESS":
                    if not self.goal_result:
                        self.goal_result = True
                        self.get_logger().info('Goal reached!')
            
    def send_result_robot(self):
        if(self.goal_result == True):
            self.msg.data = "ok"
        else:
            self.msg.data = "running"
        
        self.publisher_result.publish(self.msg)

    def update_commands(self, msg: String):
        self.commands = msg.data
        print(f"Received command: {self.commands}")

        if self.commands == "go_to_next_point" and self.goal_result:
            if self.goal_index < len(self.data):
                table_name = self.data[self.goal_index]
                self.goal_index += 1
                self.send_goal_from_name(table_name)
                self.get_logger().info(f"Sending next goal to: {table_name}")
                self.goal_result = False
            else:
                self.send_goal_from_name("home")
                self.get_logger().info("No more goals to send (go home).")
                self.goal_index = 0



def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
