#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from PyKDL import Chain, JntArray, Frame, ChainFkSolverPos_recursive
import numpy as np
import tf
import os
import geometry_msgs.msg
import pickle
import kdl_parser_py.urdf

rospy.init_node('nao_smooth_wave_motion')
joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
rospy.sleep(1)  # Give RViz time to initialize

# Load the URDF directly from the file
urdf_path = "/home/matt/rob545_ws/src/nao_robot/nao_description/urdf/naoV40_generated_urdf/nao.urdf"
robot = URDF.from_xml_file(urdf_path)

# Create a KDL tree and chain from the URDF
ok, tree = kdl_parser_py.urdf.treeFromUrdfModel(robot)
if not ok:
    rospy.logerr("Failed to construct KDL tree from URDF")
    exit(1)
chain = tree.getChain("torso", "r_gripper")

# Get joint names and limits from the URDF
joint_names = [joint.name for joint in robot.joints if joint.type != 'fixed']
print("this are the joints:", joint_names)
joint_limits = {joint.name: (joint.limit.lower, joint.limit.upper) for joint in robot.joints if joint.type != 'fixed' and joint.limit is not None}

# Define joint angle ranges for the wave motion (radians)
wave_range = {
    'RShoulderPitch': [-1.0, 0.0],
    # 'RShoulderRoll': [0.0, 0.5],
    'RElbowYaw': [0.0, 1.5],
    # 'RElbowRoll': [-0.6, 0.0],
    'RWristYaw': [-1.0, 1.0],  
    'RHand': [0.0, 1.0],  
}
fixed_joint_angles = {
    'RShoulderRoll': 0,
    'RElbowRoll': 0
}

# Joint names for NAO's right arm
joint_names = list(wave_range.keys())

# Initialize a KDL JntArray to hold joint angles
num_joints = chain.getNrOfJoints()
joint_angles = JntArray(num_joints)

# Initialize a TF broadcaster to publish the end effector pose
br = tf.TransformBroadcaster()

def generate_smooth_trajectory(wave_range, duration=5.0, num_points=50):
    trajectory = {}
    t = np.linspace(0, duration, num_points)
    wave_frequency = 2.0  # Number of waves per duration

    for joint, (min_angle, max_angle) in wave_range.items():
        amplitude = (max_angle - min_angle) / 2
        offset = (max_angle + min_angle) / 2
        # Create a wave pattern for the elbow and wrist joints
        if joint in fixed_joint_angles:
            trajectory[joint] = fixed_joint_angles[joint]
        elif joint == 'RHand':
            trajectory[joint] = offset + (amplitude * 0.2) * np.sin(2 * np.pi * wave_frequency * t / duration)
        else:
            trajectory[joint] = offset
        print(f"Joint {joint}: {trajectory[joint]}")  # Debug statement

    return trajectory

# Forward kinematics function
def forward_kinematics(joint_angles, chain, end_link_name="r_gripper"):
    # Create a KDL frame to store the end effector pose
    end_effector_frame = Frame()

    # Create a KDL FK solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # Solve forward kinematics
    fk_solver.JntToCart(joint_angles, end_effector_frame)

    # Print the frame for debugging
    print(f"Joint angles: {joint_angles}")
    print(f"End effector frame: {end_effector_frame}")

    # Convert the KDL frame to a Pose message
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
        "r_gripper",
        "base_link"  # Reference frame
    )

# Main loop to execute the smooth wave motion
end_effector_poses = []  # List to store end effector poses

while not rospy.is_shutdown():
    smooth_trajectory = generate_smooth_trajectory(wave_range)

    for i in range(len(smooth_trajectory['RShoulderPitch'])):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joint_names
        joint_state.position = [smooth_trajectory[joint][i] for joint in joint_names if joint in smooth_trajectory]

        # Update joint angles in the KDL JntArray
        for j in range(num_joints):
            if joint_names[j] in smooth_trajectory:
                joint_angles[j] = smooth_trajectory[joint_names[j]][i]

        # Calculate and publish the end effector pose
        end_effector_pose = forward_kinematics(joint_angles, chain)
        publish_end_effector_pose(end_effector_pose)
        end_effector_poses.append(end_effector_pose)

        joint_state_publisher.publish(joint_state)
        rospy.sleep(0.1)  # Adjust timing for smoothness (10 Hz)

# Save end effector poses to a file (e.g., pickle)
current_dir = os.path.dirname(os.path.realpath(__file__))

# Create the file path
file_path = os.path.join(current_dir, 'end_effector_poses.pkl')

# Save end effector poses to a file (e.g., pickle)
with open(file_path, 'wb') as f:
    pickle.dump(end_effector_poses, f)
