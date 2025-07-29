#!/usr/bin/env python3
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from rclpy.node import Node 
import rclpy


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        # รอรับการแจ้งจาก Node 2 ว่ามีการตรวจจับคน
        self.servo_sub_reached_points = self.create_subscription(Empty, 'waypoint_reached', self.on_waypoints, 10)
        self.task_done_sub = self.create_subscription(Empty,  '/task_completed', self.on_task_completed, 10)
        self.servo_pub_completed = self.create_publisher(Empty, '/servo_completed',10)

    def on_waypoints(self, msg):
        self.release_box()
        self.servo_pub_completed.publish(Empty())


    def on_task_completed(self, msg): #ปิด node
        self.get_logger().info("shutdown node servoControl...")
        self.destroy_node()  
        rclpy.shutdown()
    

    def release_box(self): #ปล่อย ลูกบาศก์
        self.get_logger().info("box_released")



def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
 
