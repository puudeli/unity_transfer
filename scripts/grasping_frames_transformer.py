#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import Transform

class GraspingFrameTransformer:
    def __init__(self):
        rospy.init_node("grasping_frames_transformer")
        rospy.loginfo("Initialised grasping_frames_transformer")
        
        self.left_gripper_pose = rospy.Publisher("/left_grasping_frame_transform", Transform, queue_size=1)
        self.right_gripper_pose = rospy.Publisher("/right_grasping_frame_transform", Transform, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

if __name__ == "__main__":
    transformer = GraspingFrameTransformer()

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        try:
            left_gripper_transform = transformer.tfBuffer.lookup_transform("base_footprint", "gripper_left_grasping_frame", rospy.Time())
            right_gripper_transform = transformer.tfBuffer.lookup_transform("base_footprint", "gripper_right_grasping_frame", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        left_gripper_transform_message = Transform()
        right_gripper_transform_message = Transform()
        
        left_gripper_transform_message = left_gripper_transform.transform
        right_gripper_transform_message = right_gripper_transform.transform

        transformer.left_gripper_pose.publish(left_gripper_transform_message)
        transformer.right_gripper_pose.publish(right_gripper_transform_message)
        rate.sleep()
