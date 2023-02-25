from typing import Optional
from queue import Queue

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


class HandController:
    def __init__(self, name: str) -> None:
        self.controller = SrHandCommander(name=name)
        self.pub_queue = Queue()

    def publish_joint(self):
        while True:
            flex = self.pub_queue.get()

            if flex == "END":
                return

            if flex is not None:
                self.controller.move_to_joint_value_target_unsafe(
                    flex, wait=False, angle_degrees=True
                )
