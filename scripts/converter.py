#!/usr/bin/env python

import rospy
import copy

from geometry_msgs.msg import PoseStamped
from unity_transfer.msg import TrackedPoses

class ConverterNode:
    def __init__(self):
        rospy.init_node("converter_node")
        rospy.loginfo("Initialised ConverterNode")

        unity_poses_topic = "/tracked_poses"
        left_ee_topic = "/whole_body_kinematic_controller/arm_left_tool_link_goal"
        right_ee_topic = "/whole_body_kinematic_controller/arm_right_tool_link_goal"
        head_topic = "/whole_body_kinematic_controller/gaze_objective_xtion_optical_frame_goal"

        # TODO: maybe add namespace

        self.unity_poses_sub = rospy.Subscriber(unity_poses_topic, TrackedPoses , self.unity_poses_cb)

        self.left_ee_pub = rospy.Publisher(left_ee_topic, PoseStamped, queue_size=1000)
        self.right_ee_pub = rospy.Publisher(right_ee_topic, PoseStamped, queue_size=1000)
        self.head_pub = rospy.Publisher(head_topic, PoseStamped, queue_size=1000)


    def unity_poses_cb(self, msg: TrackedPoses): 
        left_ee_pose = PoseStamped()
        left_ee_pose.header.stamp = rospy.Time.now()
        # left_ee_pose.header.frame_id = "base_footprint"
        
        left_ee_pose.pose.position.x = msg.lcon_pos_z
        left_ee_pose.pose.position.y = - msg.lcon_pos_x
        left_ee_pose.pose.position.z = msg.lcon_pos_y
        left_ee_pose.pose.orientation.x = msg.lcon_rot_x
        left_ee_pose.pose.orientation.y = msg.lcon_rot_y
        left_ee_pose.pose.orientation.z = msg.lcon_rot_z
        left_ee_pose.pose.orientation.w = msg.lcon_rot_w

        self.left_ee_pub.publish(left_ee_pose)

        right_ee_pose = PoseStamped()
        right_ee_pose.header.stamp = rospy.Time.now()
        # right_ee_pose.header.frame_id = "base_footprint"
        
        right_ee_pose.pose.position.x = msg.rcon_pos_z
        right_ee_pose.pose.position.y = - msg.rcon_pos_x
        right_ee_pose.pose.position.z = msg.rcon_pos_y
        right_ee_pose.pose.orientation.x = msg.rcon_rot_x
        right_ee_pose.pose.orientation.y = msg.rcon_rot_y
        right_ee_pose.pose.orientation.z = msg.rcon_rot_z
        right_ee_pose.pose.orientation.w = msg.rcon_rot_w

        self.right_ee_pub.publish(right_ee_pose)

        head_pose = PoseStamped()
        head_pose.header.stamp = rospy.Time.now()
        head_pose.header.frame_id = "base_footprint"
        
        head_pose.pose.position.x = msg.hs_pos_z
        head_pose.pose.position.y = - msg.hs_pos_x
        head_pose.pose.position.z = msg.hs_pos_y
        head_pose.pose.orientation.x = msg.hs_rot_x
        head_pose.pose.orientation.y = msg.hs_rot_y
        head_pose.pose.orientation.z = msg.hs_rot_z
        head_pose.pose.orientation.w = msg.hs_rot_w

        self.head_pub.publish(head_pose)



if __name__ == "__main__":
    converter = ConverterNode()

    rospy.spin()


    
