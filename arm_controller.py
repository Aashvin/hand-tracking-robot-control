import numpy as np
from queue import Queue

import geometry_msgs
import moveit_msgs.msg
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
import tf


class ArmController:
    def __init__(self, name: str) -> None:
        self.controller = SrArmCommander(name=name)
        self.pub_queue = Queue()

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
