#!/usr/bin/env python3
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from rclpy.node import Node 
import rclpy


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        # รอรับการแจ้งจาก Node 2 ว่ามีการตรวจจับคน
        self.detected_sub = self.create_subscription(Bool, '/person_detected', self.on_detected, 10)
        self.task_done_sub = self.create_subscription(Empty,  '/taske_completed', self.on_task_completed, 10)

    def on_detected(self, msg):
        if msg.data:
            self.open_servo()
            self.close_servo()

    def on_task_completed(self, msg):
        self.get_logger().info("shutdown node servoControl...")
        self.destroy_node()  
        rclpy.shutdown()
    

    def open_servo(self):
        self.get_logger().info("Servo opened!")

    def close_servo(self):
       self.get_logger().info("Servo closed!")


def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
 