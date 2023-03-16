#!/usr/bin/env python3.8

import numpy as np

import geometry_msgs
import moveit_msgs.msg
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
import tf

from robot_controller import RobotController
from webcam_controller import HAND_LANDMARKS


class ArmController(RobotController):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.commander = SrArmCommander(name=name)
        self.orientation = None
        self.constraints = None

    def set_required_landmarks(self):
        self.required_landmarks = [
            HAND_LANDMARKS.MIDDLE_FINGER_MCP,
            HAND_LANDMARKS.WRIST,
        ]

    def set_constraints(self):
        self.constraints = moveit_msgs.msg.Constraints()
        self.constraints.name = "all_constraints"

        shoulder_lift_constraint = moveit_msgs.msg.JointConstraint()
        shoulder_lift_constraint.joint_name = "ra_shoulder_lift_joint"
        shoulder_lift_constraint.position = np.radians(-45)
        shoulder_lift_constraint.tolerance_below = np.radians(45)
        shoulder_lift_constraint.tolerance_above = np.radians(100)

        self.constraints.joint_constraints = [shoulder_lift_constraint]

    def move_to_start_pose(self):
        self.commander.set_pose_reference_frame("ba_base")
        self.commander.move_to_named_target("ra_start")
        self.commander.move_to_joint_value_target_unsafe(
            {"ra_wrist_3_joint": -180}, wait=True, angle_degrees=True
        )

        current_pose = self.commander.get_current_pose("ra_base")

        starting_pose = geometry_msgs.msg.PoseStamped()

        starting_pose.pose.position.x = 0.5
        starting_pose.pose.position.y = -0.5
        starting_pose.pose.position.z = current_pose.position.z

        starting_orientation_euler = tf.transformations.euler_from_quaternion(
            (
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            )
        )

        starting_orientation = tf.transformations.quaternion_from_euler(
            starting_orientation_euler[0], starting_orientation_euler[1], 0
        )

        starting_pose.pose.orientation.x = starting_orientation[0]
        starting_pose.pose.orientation.y = starting_orientation[1]
        starting_pose.pose.orientation.z = starting_orientation[2]
        starting_pose.pose.orientation.w = starting_orientation[3]

        self.orientation = starting_pose.pose.orientation

        self.set_constraints()

        self.commander.move_to_pose_value_target_unsafe(starting_pose)

    def process_landmark_data(self, landmark_data):
        mid_finger_x = landmark_data["x"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        mid_finger_y = landmark_data["y"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]

        squared_x_dist = (
            landmark_data["x"][HAND_LANDMARKS.WRIST]
            - landmark_data["x"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        ) ** 2
        squared_y_dist = (
            landmark_data["y"][HAND_LANDMARKS.WRIST]
            - landmark_data["y"][HAND_LANDMARKS.MIDDLE_FINGER_MCP]
        ) ** 2
        norm_dist = np.sqrt(squared_x_dist + squared_y_dist)

        arm_position_dict = {
            "x": -mid_finger_y + 1,
            "y": -mid_finger_x * 2 + 1,
            "z": max(0.2, 0.4 - norm_dist),
        }

        return arm_position_dict

    def publish_move(self):
        while True:
            arm_position_dict = self.data_queue.get()

            if arm_position_dict == "END":
                return

            if arm_position_dict is not None:
                with self.data_queue.mutex:
                    self.data_queue.queue.clear()

                target = geometry_msgs.msg.PoseStamped()
                target.pose.orientation = self.orientation

                target.pose.position.x = arm_position_dict["x"]
                target.pose.position.y = arm_position_dict["y"]
                target.pose.position.z = arm_position_dict["z"]

                self.commander.move_to_pose_value_target_unsafe(
                    target, wait=False, ik_constraints=self.constraints
                )
