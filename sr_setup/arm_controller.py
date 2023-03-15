#!/usr/bin/env python3.8

import numpy as np

import geometry_msgs
import moveit_msgs.msg
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
import tf

from robot_controller import RobotController

    #         # Get the position of the base of the middle finger for the x, y of the robot
    #         mid_finger_x = hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
    #         mid_finger_y = hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
    #         mid_finger_pos = (mid_finger_x, mid_finger_y)

    #         # Get the normalised distance between the base of the middle finger and the wrist for the z of the robot
    #         squared_y_dist = (
    #             hand.landmark[self.mp_hands.HandLandmark.WRIST].y
    #             - hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
    #         ) ** 2
    #         squared_x_dist = (
    #             hand.landmark[self.mp_hands.HandLandmark.WRIST].x
    #             - hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
    #         ) ** 2
    #         norm_dist = np.sqrt(squared_y_dist + squared_x_dist)


class ArmController(RobotController):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.controller = SrArmCommander(name=name)

    def set_required_data(self):
        self.required_data = 

    def move_to_start_pose(self):
        starting_pose = self.controller.get_current_pose("ra_base")

        desired_pose = geometry_msgs.msg.PoseStamped()

        desired_pose.pose.position.x = 0.5
        desired_pose.pose.position.y = -0.5
        desired_pose.pose.position.z = starting_pose.position.z

        starting_orientation_euler = tf.transformations.euler_from_quaternion(
            (
                starting_pose.orientation.x,
                starting_pose.orientation.y,
                starting_pose.orientation.z,
                starting_pose.orientation.w,
            )
        )

        desired_orientation = tf.transformations.quaternion_from_euler(
            starting_orientation_euler[0], starting_orientation_euler[1], 0
        )

        desired_pose.pose.orientation.x = desired_orientation[0]
        desired_pose.pose.orientation.y = desired_orientation[1]
        desired_pose.pose.orientation.z = desired_orientation[2]
        desired_pose.pose.orientation.w = desired_orientation[3]
        self.controller.move_to_pose_value_target_unsafe(desired_pose)

        return desired_pose

    def publish_move(self):
        constraint = moveit_msgs.msg.Constraints()
        constraint.name = "all_constraints"
        arm_constraint = moveit_msgs.msg.JointConstraint()
        arm_constraint.joint_name = "ra_shoulder_lift_joint"
        arm_constraint.position = np.radians(-45)
        arm_constraint.tolerance_below = np.radians(45)
        arm_constraint.tolerance_above = np.radians(100)

        constraint.joint_constraints = [arm_constraint]

        while True:
            move = self.pub_queue.get()

            if move == "END":
                return

            if move is not None:
                self.controller.move_to_pose_value_target_unsafe(
                    move, wait=False, ik_constraints=constraint
                )
