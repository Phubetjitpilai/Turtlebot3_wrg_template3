#!/usr/bin/env python3
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def main():
    # --- Init ROS2 node
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose (เริ่มที่ map origin)
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 stack ready
    nav.waitUntilNav2Active()

    # --- Waypoints list (ทั้งหมด 16 จุด)
   # goal_pose_ = [
       # [0.880975604057312, -0.19757458567619324, 0.0],  # point 1
       # [1.228120756149292, -0.0006982684135437012, 0.0],  # point 2
      #  [2.6014227867126465, -0.9997363686561584, 0.0],  # point 3
        #[2.4714226722717285, -1.4532400369644165, 0.0],  # point 4
       # [2.2938787937164307, -1.4585250616073608, 0.0],  # point 5
        #[2.6225697994232178, -1.4484708309173584, 0.0],  # point 6
        #[2.4886255264282227, -1.8592928647994995, 0.0],  # point 7
        #[2.304171085357666, -1.8214411735534668, 0.0],  # point 8
        #[2.6414542198181152, -1.8917107582092285, 0.0],  # point 9
        #[2.2938787937164307, -2.295990467071533, 0.0],  # point 10
        #[1.058830976486206, -1.8837921619415283, 0.0],  # point 11
       # [1.0529485940933228, -2.0051002502441406, 0.0],  # point 12
        #[0.9099054336547852, -2.1257221698760986, 0.0],  # point 13
       # [0.4419511556625366, -1.8121237754821777, 0.0],  # point 14
       # [0.06046581268310547, -0.03235733509063721, 0.0], # point 15
      #  [0.0, 0.0, 0.0]                                   # point 16 กลับจุดเริ่มต้น
   # ]
   
    # --- Waypoints list (x, y, z, qx, qy, qz, qw)
    goal_pose_ = [
        [0.8059219121932983, -0.17383164167404175, 0.0, 0.0, 0.0, 0.004246642976211978, 0.9999909829710629], #point1 drop 
        [2.5960423946380615, -0.15482759475708008, 0.0, 0.0, 0.0,-0.7070412982237968, 0.707172258085686], #point2 ways

        [2.455105781555176, -1.4594608640670776, 0.0, 0.0, 0.0, -0.7085511291426619, 0.705659476936758], # point 3 drop
        [2.4662322998046875, -2.0100863952636719, 0.0, 0.0, 0.0, -0.7128670455634724, 0.7012992052965744],  # point 4 drop
        
        [2.4487218856811523, -2.2100892066955566, 0.0, 0.0, 0.0, -0.9999983781670552, 0.0018010172845805761 ], # point 5 ways
        [1.0779128074645996, -1.9279274940490723, 0.0, 0.0, 0.0, -0.9014558827851237,  0.43287098700662924],  # point 6 drop
        
                
        [0.40123450756073, -1.932122826576233, 0.0, 0.0, 0.0, 0.912893420940346,0.40819799363033654], # point 7 drop
        [0.43536388874053955,-1.9075031280517578, 0.0, 0.0, 0.0, 0.9089338852665702, 0.4169402741571237],  # point 8 ways
    
        
        [0.0,        0.0,       0.0, 0.0, 0.0, 0.0, 0.0]  # กลับจุดเริ่ม
    ]

    # --- Loop all waypoints
    for i, pose in enumerate(goal_pose_):
        print(f"\n=== Going to waypoint {i+1}/{len(goal_pose_)} : {pose} ===")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()

        # position
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]

        # orientation (ใช้ quaternion ตรงๆ)
        goal_pose.pose.orientation.x = pose[3]
        goal_pose.pose.orientation.y = pose[4]
        goal_pose.pose.orientation.z = pose[5]
        goal_pose.pose.orientation.w = pose[6]

        # --- Send goal
        nav.goToPose(goal_pose)

        # --- Wait until task complete
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
                               
        # --- getResult() returns a STRING: 'SUCCEEDED', 'FAILED', 'CANCELED'
        result = nav.getResult()
        if result == "SUCCEEDED":
            print(f"✅ Waypoint {i+1} reached!")

        elif result == "FAILED":
            print(f"❌ Failed to reach waypoint {i+1}, retrying once...")

        print(1)            
        time.sleep(10)

    # --- Shutdown ROS2
    print("\n🎉 All waypoints processed!")
    rclpy.shutdown()


if __name__ == '__main__':
    main()



