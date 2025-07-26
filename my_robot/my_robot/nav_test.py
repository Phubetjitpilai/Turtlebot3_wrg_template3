#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
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



    goal_pose_ =   [[0.9817022085189819,  -2.0868029594421387, 0.2],
                    [2.0179057121276855, 2.3381803035736084, 0.2],
                    [0.020585298538208008, 0.014914095401763916, 0.2]
    ]
    
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
        nav.goToPose(goal_pose)

        # --- Wait for task completion
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            #print(feedback)

        i+=1
        print(f"Reached waypoint {i}")

    # --- Shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
