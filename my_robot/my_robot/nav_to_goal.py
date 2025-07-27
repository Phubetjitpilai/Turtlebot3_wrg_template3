#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf_transformations

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # --- Init navigator
        self.navigator = BasicNavigator()

        self.servo_done = False
        self.camera_done = False
        self.waypoints = 0

        # --- Set initial pose
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        # --- Publisher/Subscriber
        self.waypoint_reached_pub = self.create_publisher(Empty, '/waypoint_reached', 10)
        self.end_process_pub = self.create_publisher(Empty, '/task_completed', 10)
        self.servo_sub = self.create_subscription(Empty, '/servo_completed', self.wait_for_servo, 10)
        self.camera_sub = self.create_subscription(Empty, '/person_no_detected', self.wait_for_camera, 10)
    
        self.goal_pose = [
            [3.74, 0.50, 0.0], #index 0 waypoint 1
            [2.01, 2.33, 0.0], #index 1 waypoint 2
            [0.02, 0.01, 0.0]  #index 2 waypoint 3
        ]

        self.start_navigation()

    def wait_for_servo(self, msg):
        self.get_logger().info("Servo completed signal received.")
        self.servo_done = True

    def wait_for_camera(self, msg):
        self.get_logger().info("Camera (no person) signal received.")
        self.camera_done = True

    def wait_for_signal(self):
        # รอจนกว่า flag ใด flag หนึ่งจะเป็น True
        while not (self.servo_done or self.camera_done):
            rclpy.spin_once(self, timeout_sec=0.1)
        # รีเซ็ต flag สำหรับรอบถัดไป
        self.servo_done = False
        self.camera_done = False

    def start_navigation(self):
        for i, pose in enumerate(self.goal_pose):
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose[0]
            goal_pose.pose.position.y = pose[1]
            goal_pose.pose.position.z = pose[2]
            goal_pose.pose.orientation.x = q_x
            goal_pose.pose.orientation.y = q_y
            goal_pose.pose.orientation.z = q_z
            goal_pose.pose.orientation.w = q_w

            self.get_logger().info(f"Navigating to waypoint {i}")
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # self.get_logger().info(str(feedback))  # Optional

            if i in [1, 2]:
                self.waypoint_reached_pub.publish(Empty())
                self.waypoints+=1
                self.get_logger().info(f"Reached waypoint {self.waypoints}, waiting for servo or camera")
                self.wait_for_signal()
                

        self.end_process_pub.publish(Empty())
        self.get_logger().info("All waypoints completed. Task finished.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

