#!/usr/bin/env python3

import rospy
import pickle
import geometry_msgs.msg
import tf

rospy.init_node('visualize_poses')
br = tf.TransformBroadcaster()
rate = rospy.Rate(10)  # 10 Hz

# Load the pickle file
with open('end_effector_poses.pkl', 'rb') as f:
    end_effector_poses = pickle.load(f)

for pose in end_effector_poses:
    br.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        rospy.Time.now(),
        "end_effector",
        "base_link"  # Adjust this frame as needed
    )
    rate.sleep()

rospy.spin()