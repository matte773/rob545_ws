#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import os

def main():
    rospy.init_node('nao_xar_behavior_node')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Get xar file path from ROS parameter (default to "behavior.xar")
    xar_file_path = rospy.get_param('~xar_file', 'behavior.xar')
    script_path = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_path)
    xar_file_path = os.path.join(script_dir, xar_file_path)

    rate = rospy.Rate(1)  # 30 Hz

    # Joint Name Mapping 
    joint_name_mapping = {
        "HeadYaw": "HeadYaw",
        "HeadPitch": "HeadPitch",
        "LShoulderPitch": "LShoulderPitch",
        "LShoulderRoll": "LShoulderRoll",
        "LElbowYaw": "LElbowYaw",
        "LElbowRoll": "LElbowRoll",
        "LWristYaw": "LWristYaw",
        "LHand": "LHand",  
        "RShoulderPitch": "RShoulderPitch",
        "RShoulderRoll": "RShoulderRoll",
        "RElbowYaw": "RElbowYaw",
        "RElbowRoll": "RElbowRoll",
        "RWristYaw": "RWristYaw",
        "RHand": "RHand",
        "LHipYawPitch": "LHipYawPitch",
        "LHipRoll": "LHipRoll",
        "LHipPitch": "LHipPitch",
        "LKneePitch": "LKneePitch",
        "LAnklePitch": "LAnklePitch",
        "LAnkleRoll": "LAnkleRoll",
        "RHipYawPitch": "RHipYawPitch",
        "RHipRoll": "RHipRoll",
        "RHipPitch": "RHipPitch",
        "RKneePitch": "RKneePitch",
        "RAnklePitch": "RAnklePitch",
        "RAnkleRoll": "RAnkleRoll",
    }

    # Parse the .xar file (extracting frame durations as well)
    joint_names, joint_angles, frame_durations = parse_xar_file(xar_file_path)

    if not joint_names or not joint_angles:
        rospy.logerr("Failed to parse joint names or angles from .xar file.")
        return

    # Start from the frame specified in the parameter (default to 1)
    current_frame = rospy.get_param('~current_frame', 1)

    while not rospy.is_shutdown():
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [joint_name_mapping.get(name, name) for name in joint_names]  # Map names

        # Get the joint angles for the current frame (with interpolation for missing values)
        if 0 < current_frame <= len(joint_angles):
            joint_state_msg.position = [angle for joint, angle in zip(joint_names, joint_angles[current_frame - 1]) if joint in joint_name_mapping]
        else:
            rospy.logwarn("Invalid frame number. Using first frame angles.")
            joint_state_msg.position = [angle for joint, angle in zip(joint_names, joint_angles[0]) if joint in joint_name_mapping]
            current_frame = 1

        pub.publish(joint_state_msg)

        # Increment frame and sleep
        current_frame += 1
        if current_frame > len(joint_angles):
            current_frame = 1  # Loop back to the first frame
        if current_frame - 1 < len(frame_durations):  # Avoid index out of range
            rospy.sleep(frame_durations[current_frame - 1] / 60.0)

    rate.sleep()

def parse_xar_file(file_path):
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        ns = {'': 'http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd'}
        timeline = root.find('.//Timeline', ns)

        if timeline is None:
            rospy.logerr("Timeline element not found in .xar file.")
            return None, None, None

        # Extract joint names
        joint_names = []
        joint_angles_dict = {}
        frame_durations = []

        for actuator in timeline.findall('.//ActuatorCurve', ns):
            joint_name = actuator.get('actuator')
            joint_names.append(joint_name)
            joint_angles_dict[joint_name] = []

            keys = actuator.findall('Key', ns)
            for i in range(len(keys) - 1):
                frame_durations.append(int(keys[i + 1].get('frame')) - int(keys[i].get('frame')))

            for key in keys:
                frame = int(key.get('frame'))
                value = float(key.get('value'))
                joint_angles_dict[joint_name].append((frame, value))

        # Combine joint angles by frame with interpolation for missing angles
        all_frames = sorted(set(frame for angles in joint_angles_dict.values() for frame, _ in angles))
        joint_angles = []
        for frame in all_frames:
            frame_angles = []
            for joint_name in joint_names:
                angles = dict(joint_angles_dict[joint_name])
                frame_angles.append(angles.get(frame, frame_angles[-1] if frame_angles else 0.0))  # Default to 0.0 or last known value
            joint_angles.append(frame_angles)

        return joint_names, joint_angles, frame_durations

    except Exception as e:
        rospy.logerr(f"Error parsing .xar file: {e}")
        return None, None, None

if __name__ == '__main__':
    main()