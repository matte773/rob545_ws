#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from urdf_parser_py.urdf import URDF
from PyKDL import Chain, JntArray, Frame, ChainFkSolverPos_recursive
import numpy as np
import tf
import os
import geometry_msgs.msg
import pickle
import kdl_parser_py.urdf

rospy.init_node('nao_smooth_wave_motion')
joint_state_publisher = rospy.Publisher('/cmd_joint_states', JointState, queue_size=10)
rospy.sleep(1)  # Give RViz time to initialize

# Load the URDF directly from the file
#urdf_path = "/home/magraz/rob545_ws/src/nao_robot/nao_description/urdf/naoV40_generated_urdf/nao.urdf"
#robot = URDF.from_xml_file(urdf_path)

# Load the converted URDF
robot = URDF.from_xml_file("/home/magraz/rob545_ws/src/jamie_description/urdf/jamie.xacro")

# Create a KDL tree and chain from the URDF
ok, tree = kdl_parser_py.urdf.treeFromUrdfModel(robot)
if not ok:
    rospy.logerr("Failed to construct KDL tree from URDF")
    exit(1)

chain = tree.getChain("chest_link_1", "forearm_right_v1_1")

# Get joint names and limits from the URDF
joint_names = [joint.name for joint in robot.joints if joint.type != 'fixed']
joint_limits = {joint.name: (joint.limit.lower, joint.limit.upper) for joint in robot.joints if joint.type != 'fixed' and joint.limit is not None}

# Define joint angle ranges for the wave motion (radians)
wave_range = {
    'Revolute 3': [-2.51, -2.51],
    'Revolute 4': [-0.44, 0.44],
    'Revolute 21': [-0.65, 0.65],
    'Revolute 22': [-1.0, 0.0],
}

fixed_joint_angles = {}

# Joint names for NAO's right arm
joint_names = list(wave_range.keys())
print(f"Joint Names: {list(wave_range.keys())}")

# Initialize a KDL JntArray to hold joint angles
num_joints = chain.getNrOfJoints()
joint_angles = JntArray(num_joints)

# Initialize a TF broadcaster to publish the end effector pose
br = tf.TransformBroadcaster()

def generate_smooth_trajectory(wave_range, duration=10.0, num_points=100):
    trajectory = {}
    t = np.linspace(0, duration, num_points)
    wave_frequency = 2.0  # Number of waves per duration

    for joint, (min_angle, max_angle) in wave_range.items():
        amplitude = (max_angle - min_angle) / 2
        offset = (max_angle + min_angle) / 2
        trajectory[joint] = offset + (amplitude * 0.2) * np.sin(2 * np.pi * wave_frequency * t / duration)

    return trajectory

# Forward kinematics function
def forward_kinematics(joint_angles, chain):
    end_effector_frame = Frame()
    fk_solver = ChainFkSolverPos_recursive(chain)
    fk_solver.JntToCart(joint_angles, end_effector_frame)

    end_effector_pose = geometry_msgs.msg.Pose()
    end_effector_pose.position.x = end_effector_frame.p[0]
    end_effector_pose.position.y = end_effector_frame.p[1]
    end_effector_pose.position.z = end_effector_frame.p[2]
    q = end_effector_frame.M.GetQuaternion()
    end_effector_pose.orientation.x = q[0]
    end_effector_pose.orientation.y = q[1]
    end_effector_pose.orientation.z = q[2]
    end_effector_pose.orientation.w = q[3]

    return end_effector_pose

# Function to publish end effector pose
def publish_end_effector_pose(end_effector_pose):
    br.sendTransform(
        (end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z),
        (end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w),
        rospy.Time.now(),
        "forearm_right_v1_1",
        "chest_link_1 "
    )

# Main loop to execute the smooth wave motion
end_effector_poses = []

while not rospy.is_shutdown():
    smooth_trajectory = generate_smooth_trajectory(wave_range)

    num_points = len(next(iter(smooth_trajectory.values())))
    for i in range(num_points):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joint_names
        joint_state.position = [smooth_trajectory[joint][i] for joint in joint_names]

        for j in range(num_joints):
            if joint_names[j] in smooth_trajectory:
                joint_angles[j] = smooth_trajectory[joint_names[j]][i]

        end_effector_pose = forward_kinematics(joint_angles, chain)
        # publish_end_effector_pose(end_effector_pose)
        end_effector_poses.append(end_effector_pose)

        joint_state_publisher.publish(joint_state)
        print(f"Joint State: {joint_state.position} End Effector Pose: {end_effector_pose.position}")
        rospy.sleep(0.05)

# Save end effector poses to a file (e.g., pickle)
current_dir = os.path.dirname(os.path.realpath(__file__))
file_path = os.path.join(current_dir, 'end_effector_poses_jamie.pkl')

with open(file_path, 'wb') as f:
    pickle.dump(end_effector_poses, f)
