#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import os

def main():
    rospy.init_node('nao_xar_behavior_node')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Assuming the script and .xar file are in the same folder
    script_path = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_path)
    xar_file_path = os.path.join(script_dir, "behavior.xar")

    # Parse the .xar file
    joint_names, joint_angles = parse_xar_file(xar_file_path)
    print(f"Joint names: {joint_names} and Joint angles: {joint_angles}")

    if not joint_names or not joint_angles:
        rospy.logerr("Failed to parse joint names or angles from .xar file.")
        return

    rate = rospy.Rate(10)  # Adjust loop rate as needed

    while not rospy.is_shutdown():
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = joint_names

        # Extract joint angles for the current frame (assuming frame data starts from index 1)
        current_frame = rospy.get_param('~current_frame', 1)  # Use a private ROS parameter to specify the frame
        if current_frame <= len(joint_angles):
            joint_state_msg.position = joint_angles[current_frame - 1]
        else:
            rospy.logwarn("Invalid frame number. Using first frame angles.")
            joint_state_msg.position = joint_angles[0]

        pub.publish(joint_state_msg)
        rate.sleep()

def parse_xar_file(file_path):
    """
    Parses the given .xar file and extracts joint names and angles.

    Args:
        file_path (str): Path to the .xar file.

    Returns:
        tuple: A tuple containing two lists - joint names and joint angles (for all frames).
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        ns = {'': 'http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd'}
        timeline = root.find('.//{http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd}Timeline')

        if timeline is None:
            rospy.logerr("Timeline element not found in .xar file.")
            return None, None

        # Extract joint names
        joint_names = []
        joint_angles_dict = {}

        for actuator in timeline.findall('.//{http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd}ActuatorCurve'):
            joint_name = actuator.get('actuator')
            joint_names.append(joint_name)
            joint_angles_dict[joint_name] = []

            for key in actuator.findall('{http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd}Key'):
                frame = int(key.get('frame'))
                value = float(key.get('value'))
                joint_angles_dict[joint_name].append((frame, value))

        pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Combine joint angles by frame
        all_frames = sorted(set(frame for angles in joint_angles_dict.values() for frame, _ in angles))
        joint_angles = []

        for i in range(len(all_frames)):
            frame = all_frames[i]
            frame_angles = []
            for joint_name in joint_names:
                angles = dict(joint_angles_dict[joint_name])
                frame_angles.append(angles.get(frame, 0.0))  # Default to 0.0 if frame is missing
            joint_angles.append(frame_angles)

            # Create a JointState message
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = joint_names
            joint_state.position = frame_angles

            # Publish the JointState message
            pub.publish(joint_state)

            # Calculate delay based on frame difference
            if i < len(all_frames) - 1:  # Avoid index out of range
                next_frame = all_frames[i + 1]
                frame_diff = abs(next_frame - frame)
                delay = frame_diff * 0.1  # Adjust delay as needed

                # Sleep for the calculated delay
                rospy.sleep(delay)

        return joint_names, joint_angles


    except Exception as e:
        rospy.logerr(f"Error parsing .xar file: {e}")
    return None, None

if __name__ == '__main__':
    main()
