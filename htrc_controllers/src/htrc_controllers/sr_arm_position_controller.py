#!/usr/bin/env python3.8

import numpy as np

import geometry_msgs
import moveit_msgs.msg
from sr_robot_commander.sr_arm_commander import SrArmCommander
import tf
from typing import Dict

from htrc_framework.base_robot_controllers import BaseArmController
from htrc_framework.webcam_controller import HAND_LANDMARKS


class ArmController(BaseArmController):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.commander: SrArmCommander = SrArmCommander(name=name)
        self.orientation: geometry_msgs.msg.PoseStamped.orientation = None
        self.constraints: moveit_msgs.msg.Constraints = None

    def set_required_landmarks(self) -> None:
        self.required_landmarks = [
            HAND_LANDMARKS.MIDDLE_FINGER_MCP,
            HAND_LANDMARKS.WRIST,
        ]

    def set_constraints(self) -> None:
        """
        Set the joint constriants for the robot arm.
        """

        self.constraints = moveit_msgs.msg.Constraints()
        self.constraints.name = "all_constraints"

        # Constrain the shoulder lift joint movement
        # Avoids the robot taking indirect routes to goals
        shoulder_lift_constraint = moveit_msgs.msg.JointConstraint()
        shoulder_lift_constraint.joint_name = "ra_shoulder_lift_joint"
        shoulder_lift_constraint.position = np.radians(-45)
        shoulder_lift_constraint.tolerance_below = np.radians(45)
        shoulder_lift_constraint.tolerance_above = np.radians(100)

        # Set the list of the constraints for every movement
        self.constraints.joint_constraints = [shoulder_lift_constraint]

    def move_to_start_pose(self) -> None:
        # Move to the predefined starting position
        self.commander.set_pose_reference_frame("ra_base")
        self.commander.move_to_named_target("ra_start")

        # Rotate the wrist so the hand is facing the ground
        self.commander.move_to_joint_value_target_unsafe(
            {"ra_wrist_3_joint": -180}, wait=True, angle_degrees=True
        )

        current_pose = self.commander.get_current_pose("ra_base")

        starting_pose = geometry_msgs.msg.PoseStamped()

        # Set the custom starting position for this robot
        starting_pose.pose.position.x = 0.5
        starting_pose.pose.position.y = -0.5
        starting_pose.pose.position.z = current_pose.position.z

        # Transform the orientation to Euler from quaternion
        starting_orientation_euler = tf.transformations.euler_from_quaternion(
            (
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            )
        )

        # Set the new custom starting orientation and transform it to quaternion
        starting_orientation = tf.transformations.quaternion_from_euler(
            starting_orientation_euler[0], starting_orientation_euler[1], 0
        )

        # Add the custom starting orientation to the pose object
        starting_pose.pose.orientation.x = starting_orientation[0]
        starting_pose.pose.orientation.y = starting_orientation[1]
        starting_pose.pose.orientation.z = starting_orientation[2]
        starting_pose.pose.orientation.w = starting_orientation[3]

        self.orientation = starting_pose.pose.orientation

        self.set_constraints()

        # Move the robot to the custom starting position
        self.commander.move_to_pose_value_target_unsafe(starting_pose)

    def process_landmark_data(
        self, landmark_data: Dict[str, Dict[HAND_LANDMARKS, float]]
    ) -> Dict[str, float]:
        """
        Calculate the end effector goal coordinates from landmark data.
        """

        # Extract the middle finger MCP data
        mid_finger_x = landmark_data["x"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        mid_finger_y = landmark_data["y"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]

        # Get the normalised distance from the wrist to the middle finger MCP
        squared_x_dist = (
            landmark_data["x"][HAND_LANDMARKS.WRIST]
            - landmark_data["x"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        ) ** 2
        squared_y_dist = (
            landmark_data["y"][HAND_LANDMARKS.WRIST]
            - landmark_data["y"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        ) ** 2
        norm_dist = np.sqrt(squared_x_dist + squared_y_dist)

        # Map the data to coordinates within the arm end effector's desired range
        arm_position_dict = {
            "x": -mid_finger_y + 1,
            "y": -mid_finger_x * 2 + 1,
            "z": max(0.1, 0.7 - norm_dist),
        }

        return arm_position_dict

    def publish_move(self) -> None:
        while True:
            # Get data from the queue
            landmark_data = self.data_queue.get()

            # Exit thread if end signal received
            if landmark_data == "END":
                return

            # If there is data in the queue
            if landmark_data is not None:
                arm_position_dict = self.process_landmark_data(landmark_data)

                # Construct the geometry message to send to the robot
                target = geometry_msgs.msg.PoseStamped()
                target.pose.orientation = self.orientation

                target.pose.position.x = arm_position_dict["x"]
                target.pose.position.y = arm_position_dict["y"]
                target.pose.position.z = arm_position_dict["z"]

                # Move the robot
                self.commander.move_to_pose_value_target_unsafe(
                    target, wait=False, ik_constraints=self.constraints
                )
