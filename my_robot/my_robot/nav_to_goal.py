#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf_transformations


def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
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

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Publisher to notify when a waypoint is reached
    waypoint_reached_pub = nav.create_publisher(Empty, '/waypoint_reached', 10)
    # publish when task is completed
    end_process_pub = nav.create_publisher(Empty, '/taske_completed', 10)

    # Define multiple goal positions (waypoints)
    goal_pose_ = [
        [3.7433080673217773, 0.5050445199012756, 0.0], #point 1 ,index = 0
        [2.0179057121276855, 2.3381803035736084, 0.0], #point2 ,index = 1
        [0.020585298538208008, 0.014914095401763916, 0.0] #point3 ,index = 2
    ]
    waypoints = 0

    for i, pose in enumerate(goal_pose_):
        # --- Set goal pose
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w

        # --- Command robot to move to goal pose
        nav.goToPose(goal_pose)

        # --- Wait for task completion
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            # print(feedback)
        # --- ตรวจว่าเป็นจุดที่ต้องการปล่อยไหม
        if i in [1,2]: #เอา index มาใส่
            waypoints+=1
            waypoint_reached_pub.publish(Empty())  # Publish Empty message when waypoint is reached
            print(f"Reached waypoint {waypoints}")
            

        # --- Optional: You can insert a delay here if you want a pause before moving to next waypoint
        # rclpy.sleep(1.0)

    
    # publish massge to other node to shutdown when task is completed
    end_process_pub.publish(Empty())
    # --- Shutdown and destroy
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
