#!/usr/bin/env python3.8

from queue import Queue


class RobotController:
    def __init__(self) -> None:
        self.required_landmarks = []
        self.data_queue = Queue()

    def set_required_data(self):
        raise NotImplementedError(
            "The robot controller does not have a method to set the required data."
        )

    def move_to_start_pose(self):
        raise NotImplementedError(
            "The robot controller does not have a starting pose movement method."
        )

    def publish_move(self):
        raise NotImplementedError(
            "The robot controller does not have a publish move method."
        )
