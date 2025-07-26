# node_person_detection.py
from std_msgs.msg import Bool
from std_msgs.msg import Empty
import rclpy

class DetectionNode:
    def __init__(self):
        self.node = rclpy.create_node('detection_node')
        # รอรับสัญญาณจาก Node 1 ว่าไปถึง waypoint แล้ว
        self.subscriber = self.node.create_subscription(Empty, '/waypoint_reached', self.callback, 10)
        self.publisher = self.node.create_publisher(Bool, '/person_detected', 10)
        self.person_detected = False

    def callback(self, msg):
        # เริ่มทำการตรวจจับคนเมื่อได้รับสัญญาณจาก Node 1
        print("Waypoint reached, starting person detection...")
        self.person_detected = self.detect_person()  # จำลองการตรวจจับ
        detection_msg = Bool()
        detection_msg.data = self.person_detected
        self.publisher.publish(detection_msg)
        print("Published person detection status:", self.person_detected)

    def detect_person(self):
        # การตรวจจับคน (ตัวอย่างใช้การตรวจจับใบหน้า)
        return True  # สมมติว่าเจอคน

    def spin(self):
        rclpy.spin(self.node)

# Run the detection node
detection_node = DetectionNode()
detection_node.spin()
