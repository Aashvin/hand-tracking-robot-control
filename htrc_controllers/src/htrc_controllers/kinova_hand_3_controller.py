#!/usr/bin/env python3.8

import numpy as np
import rospy
from std_srvs.srv import Empty
import sys
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from typing import List

from htrc_framework.base_robot_controllers import BaseHandController
from htrc_framework.webcam_controller import HAND_LANDMARKS


class HandController(BaseHandController):
    def __init__(self, nb_fingers: int) -> None:
        super().__init__(nb_fingers)
        self.prefix: str = None
        self.nb_joints: int = None

    def set_required_landmarks(self) -> None:
        self.required_landmarks = [
            HAND_LANDMARKS.INDEX_FINGER_MCP,
            HAND_LANDMARKS.INDEX_FINGER_PIP,
            HAND_LANDMARKS.INDEX_FINGER_DIP,
            HAND_LANDMARKS.MIDDLE_FINGER_MCP,
            HAND_LANDMARKS.MIDDLE_FINGER_PIP,
            HAND_LANDMARKS.MIDDLE_FINGER_DIP,
            HAND_LANDMARKS.THUMB_MCP,
            HAND_LANDMARKS.THUMB_IP,
            HAND_LANDMARKS.THUMB_TIP,
            HAND_LANDMARKS.WRIST,
        ]

    def argument_parser(self) -> None:
        """
        Set the prefix and number of joints from the robot type argument when running the file.
        """

        self.prefix = sys.argv[1]
        self.nb_joints = int(self.prefix[3])

    def move_joint(self, jointcmds: List[int]) -> None:
        """
        Move the robot arm joints.

        Code has been adapted from:
        URL: https://github.com/Kinovarobotics/kinova-ros/blob/kinetic-devel/kinova_control/src/move_robot.py
        Publisher: Kinova Robotics
        Date accessed: 20/04/2023
        """

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

    def move_fingers(self, jointcmds: List[int]) -> None:
        """
        Move the robot finger joints.

        Code has been adapted from:
        URL: https://github.com/Kinovarobotics/kinova-ros/blob/kinetic-devel/kinova_control/src/move_robot.py
        Publisher: Kinova Robotics
        Date accessed: 20/04/2023
        """
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

        pub.publish(jointCmd)

    def move_to_start_pose(self) -> None:
        """
        Moves the robot to the starting pose at the beginning of the program.
        Code has been adapted from:
        https://github.com/Kinovarobotics/kinova-ros/blob/melodic-devel/kinova_control/src/move_robot.py
        """

        self.argument_parser()

        # Handle Gazebo robot model physics
        rospy.wait_for_service("/gazebo/unpause_physics")
        unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        resp = unpause_gazebo()

        # Move robot arm to starting pose
        if self.nb_joints == 6:
            self.move_joint([0.0, 2.9, 1.3, 4.2, 1.4, 3.0])
        else:
            self.move_joint([0.0, 2.9, 0.0, 1.3, 4.2, 1.4, 0.0])

        rospy.sleep(5)

        # Move fingers to starting pose
        self.move_fingers([0, 0, 0])

    def publish_move(self) -> None:
        while True:
            # Get data from the queue
            angle_dict = self.data_queue.get()

            # Exit thread if end signal received
            if angle_dict == "END":
                return

            # If there is data in the queue
            if angle_dict is not None:
                # Convert the relevant data to radians
                flex = {
                    "rh_FFJ2": np.radians(angle_dict[HAND_LANDMARKS.INDEX_FINGER_PIP]),
                    "rh_FFJ3": np.radians(angle_dict[HAND_LANDMARKS.INDEX_FINGER_MCP]),
                    "rh_MFJ2": np.radians(angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_PIP]),
                    "rh_MFJ3": np.radians(angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_MCP]),
                    "rh_THJ2": np.radians(angle_dict[HAND_LANDMARKS.THUMB_IP]),
                    "rh_THJ3": np.radians(angle_dict[HAND_LANDMARKS.THUMB_MCP]),
                }

                # Combine the PIP and MCP joints for the robot
                joint_commands = [
                    flex["rh_THJ2"] + flex["rh_THJ3"],
                    flex["rh_FFJ2"] + flex["rh_FFJ3"],
                    flex["rh_MFJ2"] + flex["rh_MFJ3"],
                ]

                # Set the max angle to avoid self-collision
                for i in range(3):
                    joint_commands[i] = min(joint_commands[i], 1.35)

                # Move the robot
                self.move_fingers(joint_commands)
