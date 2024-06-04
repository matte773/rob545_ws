#!/usr/bin/env python3

''' When running with NAO urdf use the following command:
rosrun nao_ik inverse_nao_move.py --start_link base_link --end_link r_gripper

When running with Jamie urdf use the following command:
rosrun nao_ik inverse_nao_move.py --start_link chest_link_1 --end_link forearm_right_v1_1
'''

import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from PyKDL import Chain, JntArray, Frame, Rotation, Vector, ChainFkSolverPos_recursive, ChainIkSolverVel_pinv, ChainIkSolverPos_NR
import kdl_parser_py.urdf
import numpy as np
import tf
import geometry_msgs.msg
import pickle
import os
import subprocess
import argparse

# Function to parse arguments
def parse_args():
    parser = argparse.ArgumentParser(description='Nao robot behavior script')
    parser.add_argument('--start_link', type=str, required=True, help='Start link of the chain')
    parser.add_argument('--end_link', type=str, required=True, help='End link of the chain')
    return parser.parse_args()

args = parse_args()

rospy.init_node('nao_wave_replication')
joint_state_publisher = rospy.Publisher('/cmd_joint_states', JointState, queue_size=10)
rospy.sleep(1)

# Convert xacro to URDF
# robot2_urdf_path = "/home/matt/rob545_ws/src/nao_robot/nao_description/urdf/naoV40_generated_urdf/nao.urdf"
robot2_urdf_path = "/home/matt/rob545_ws/src/jamie_description/urdf/jamie.xacro"


urdf_output_path = "/tmp/jamie.urdf"
subprocess.run(["rosrun", "xacro", "xacro", robot2_urdf_path, "-o", urdf_output_path], check=True)

# Load the converted URDF
robot2 = URDF.from_xml_file("/home/matt/rob545_ws/src/jamie_description/urdf/jamie.xacro")
# robot2 = URDF.from_xml_file("/home/matt/rob545_ws/src/nao_robot/nao_description/urdf/naoV40_generated_urdf/nao.urdf")


# Create a KDL tree and chain for the specified part of the robot
ok, tree2 = kdl_parser_py.urdf.treeFromUrdfModel(robot2)
if not ok:
    rospy.logerr("Failed to construct KDL tree from URDF")
    exit(1)
chain2 = tree2.getChain(args.start_link, args.end_link)

# Get joint names and limits for the specified part of the robot
joint_limits = {}
chain_segments = [chain2.getSegment(i) for i in range(chain2.getNrOfSegments())]
for joint in robot2.joints:
    if joint.type != 'fixed' and any(joint.name == seg.getJoint().getName() for seg in chain_segments):
        if joint.limit is not None:
            joint_limits[joint.name] = (joint.limit.lower, joint.limit.upper)
        else:
            rospy.logwarn(f"No limits defined for joint {joint.name}")

joint_names2 = list(joint_limits.keys())
print(f"Joint names for the specified chain: {joint_names2}")

# Ensure the path to the pickle file is correct
pickle_file_path = os.path.join(os.path.dirname(__file__), 'end_effector_poses_jamie.pkl')

# Load the saved end effector poses
try:
    with open(pickle_file_path, 'rb') as f:
        end_effector_poses = pickle.load(f)
        print(f"Loaded {len(end_effector_poses)} poses from {pickle_file_path}")
except FileNotFoundError:
    rospy.logerr(f"File not found: {pickle_file_path}")
    exit(1)

# Initialize IK solver (KDL)
fk_solver = ChainFkSolverPos_recursive(chain2)
ik_solver_vel = ChainIkSolverVel_pinv(chain2)
ik_solver_pos = ChainIkSolverPos_NR(chain2, fk_solver, ik_solver_vel, 100, 1e-6)

# Initial joint angles for the specified part of the robot
q_init = JntArray(chain2.getNrOfJoints())
for i in range(chain2.getNrOfJoints()):
    q_init[i] = 0.0  # Start with all joints at 0

# Dictionary to keep track of previous joint positions
previous_joint_positions = {name: 0.0 for name in joint_names2}

# Helper function to print KDL frames for debugging
def print_kdl_frame(frame):
    pos = frame.p
    rot = frame.M
    rot_matrix = [[rot[0, 0], rot[0, 1], rot[0, 2]], [rot[1, 0], rot[1, 1], rot[1, 2]], [rot[2, 0], rot[2, 1], rot[2, 2]]]
    print(f"Position: [{pos[0]}, {pos[1]}, {pos[2]}]")
    print(f"Rotation: {rot_matrix}")

# Main loop to replicate the wave motion
for idx, pose in enumerate(end_effector_poses):  # Iterate directly over the list of poses
    print(f"Processing pose {idx + 1}/{len(end_effector_poses)}")
    print(f"Pose details: Position({pose.position.x}, {pose.position.y}, {pose.position.z}), Orientation({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")

    # Set desired end-effector pose
    desired_pose = Frame(
        Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        Vector(pose.position.x, pose.position.y, pose.position.z)
    )
    print(f"Desired pose for pose {idx + 1}:")
    print_kdl_frame(desired_pose)

    # Solve IK for the current pose
    q_out = JntArray(chain2.getNrOfJoints())
    result = ik_solver_pos.CartToJnt(q_init, desired_pose, q_out)
    print(f"IK result for pose {idx + 1}: {result}")

    # Check if a valid IK solution was found
    if result >= 0:
        print(f"IK solution found for pose {idx + 1}/{len(end_effector_poses)}")
        joint_positions = [q_out[i] for i in range(chain2.getNrOfJoints())]
        print(f"Joint positions: {joint_positions}")

        # Publish the joint angles to the specified part of the robot
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = []
        joint_state.position = []

        affected_joints = []
        for i, pos in enumerate(joint_positions):
            joint_name = joint_names2[i]
            joint_state.name.append(joint_name)
            joint_state.position.append(pos)
            affected_joints.append(joint_name)

        # Publish the joint states
        joint_state_publisher.publish(joint_state)
        print(f"Published joint state for pose {idx + 1}")
        print(f"Affected joints: {affected_joints}")

        # Update the guess for the next iteration
        for i in range(chain2.getNrOfJoints()):
            q_init[i] = q_out[i]
    else:
        print(f"IK solution not found for pose {idx + 1}/{len(end_effector_poses)}")

    rospy.sleep(0.1)  # Adjust timing to match the original wave motion
