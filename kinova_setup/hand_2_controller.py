#!/usr/bin/env python3.8

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse

from robot_controller import RobotController
from webcam_controller import HAND_LANDMARKS


class HandController(RobotController):
    def __init__(self) -> None:
        super().__init__()
        self.prefix = None
        self.nb_joints = None
        self.nb_fingers = None

    def set_required_landmarks(self):
        self.required_landmarks = [
            HAND_LANDMARKS.INDEX_FINGER_MCP,
            HAND_LANDMARKS.INDEX_FINGER_PIP,
            HAND_LANDMARKS.INDEX_FINGER_DIP,
            HAND_LANDMARKS.THUMB_MCP,
            HAND_LANDMARKS.THUMB_IP,
            HAND_LANDMARKS.THUMB_TIP,
            HAND_LANDMARKS.WRIST,
        ]

    def argument_parser(self):
        """Argument parser"""
        parser = argparse.ArgumentParser(
            description="Drive robot joint to command position"
        )
        parser.add_argument(
            "kinova_robotType",
            metavar="kinova_robotType",
            type=str,
            default="j2n6a300",
            help="kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.",
        )
        # args_ = parser.parse_args(argument)
        argv = rospy.myargv()
        args_ = parser.parse_args(argv[1:])
        self.prefix = args_.kinova_robotType
        self.nb_joints = int(args_.kinova_robotType[3])
        self.nb_fingers = int(args_.kinova_robotType[5])

    def move_joint(self, jointcmds):
        topic_name = "/" + self.prefix + "/effort_joint_trajectory_controller/command"
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, self.nb_joints):
            jointCmd.joint_names.append(self.prefix + "_joint_" + str(i + 1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        jointCmd.points.append(point)
        rate = rospy.Rate(100)
        count = 0
        while count < 50:
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep()

    def move_fingers(self, jointcmds):
        topic_name = "/" + self.prefix + "/effort_finger_trajectory_controller/command"
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, self.nb_fingers):
            jointCmd.joint_names.append(self.prefix + "_joint_finger_" + str(i + 1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)

        jointCmd.points.append(point)
        # rate = rospy.Rate(100)
        # count = 0
        # while count < 500:
        pub.publish(jointCmd)
        # count = count + 1
        # rate.sleep()

    def move_to_start_pose(self):
        self.argument_parser()

        rospy.wait_for_service("/gazebo/unpause_physics")
        unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        resp = unpause_gazebo()

        if self.nb_joints == 6:
            self.move_joint([0.0, 2.9, 1.3, 4.2, 1.4, 3.0])
        else:
            self.move_joint([0.0, 2.9, 0.0, 1.3, 4.2, 1.4, 0.0])

        rospy.sleep(5)

        self.move_fingers([0, 0])

    def calculate_angles(self, joints):
        high_mid_vec = [
            joints["x"][2] - joints["x"][1],
            joints["y"][2] - joints["y"][1],
            joints["z"][2] - joints["z"][1],
        ]

        low_mid_vec = [
            joints["x"][0] - joints["x"][1],
            joints["y"][0] - joints["y"][1],
            joints["z"][0] - joints["z"][1],
        ]

        # Calculate the magnitudes of those vectors to normalise them
        high_mid_vec_mag = np.sqrt(
            high_mid_vec[0] ** 2 + high_mid_vec[1] ** 2 + high_mid_vec[2] ** 2
        )
        high_mid_vec_normalised = [
            high_mid_vec[0] / high_mid_vec_mag,
            high_mid_vec[1] / high_mid_vec_mag,
            high_mid_vec[2] / high_mid_vec_mag,
        ]

        low_mid_vec_mag = np.sqrt(
            low_mid_vec[0] ** 2 + low_mid_vec[1] ** 2 + low_mid_vec[2] ** 2
        )
        low_mid_vec_normalised = [
            low_mid_vec[0] / low_mid_vec_mag,
            low_mid_vec[1] / low_mid_vec_mag,
            low_mid_vec[2] / low_mid_vec_mag,
        ]

        # Get the dot product between the two vectors
        # Equivalent to the cosine of the angle between the two vectors
        dot = (
            high_mid_vec_normalised[0] * low_mid_vec_normalised[0]
            + high_mid_vec_normalised[1] * low_mid_vec_normalised[1]
            + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]
        )

        # Calculate the actual angle
        angle = 180 - np.degrees(np.arccos(dot))

        # If the angle is calculated for a lower joint, clip it between 0 and 90
        # Avoids out of bounds angles being published to the robot

        return angle

    def finger_angles(self, landmark_data):
        angle_dict = {}
        for finger in range(2):
            upper_joints = {
                dim: [
                    landmark_data[dim][self.required_landmarks[finger * 3 + i]]
                    for i in range(3)
                ]
                for dim in ["x", "y", "z"]
            }
            lower_joints = {
                dim: [
                    landmark_data[dim][self.required_landmarks[finger * 3 + i]]
                    if i != -1
                    else landmark_data[dim][self.required_landmarks[-1]]
                    for i in range(-1, 2)
                ]
                for dim in ["x", "y", "z"]
            }

            lower_joint = self.required_landmarks[finger * 3]
            upper_joint = self.required_landmarks[finger * 3 + 1]

            angle_dict[lower_joint] = np.clip(
                (self.calculate_angles(lower_joints) - 45) * 2 + 45, 0, 90
            )
            angle_dict[upper_joint] = self.calculate_angles(upper_joints)

        return angle_dict

    def publish_move(self):
        while True:
            angle_dict = self.data_queue.get()

            if angle_dict == "END":
                return

            if angle_dict is not None:
                flex = {
                    "rh_FFJ2": np.radians(angle_dict[HAND_LANDMARKS.INDEX_FINGER_PIP]),
                    "rh_FFJ3": np.radians(angle_dict[HAND_LANDMARKS.INDEX_FINGER_MCP]),
                    "rh_THJ2": np.radians(angle_dict[HAND_LANDMARKS.THUMB_IP]),
                    "rh_THJ3": np.radians(angle_dict[HAND_LANDMARKS.THUMB_MCP]),
                }

                joint_commands = [
                    flex["rh_THJ2"] + flex["rh_THJ3"],
                    flex["rh_FFJ2"] + flex["rh_FFJ3"],
                ]

                for i in range(2):
                    joint_commands[i] = min(joint_commands[i], 1.2)

                self.move_fingers(joint_commands)
