#!/usr/bin/env python3.8

from queue import Queue
from typing import List

from htrc_framework.webcam_controller import HAND_LANDMARKS


class RobotController:
    def __init__(self) -> None:
        self.required_landmarks: List[HAND_LANDMARKS]
        self.data_queue: Queue = Queue()

    def set_required_landmarks(self) -> None:
        """
        Sets the required_landmarks attribute to a list of required MediaPipe Hand landmarks.
        If the controller is for a hand, the list should following the form:
        Joints of required fingers in the order MCP, PIP, DIP. Wrist should be the last landmark.
        An example is found in the file 'sr_hand_5_finger_controller.py' in the controllers package.
        """

        raise NotImplementedError(
            "The robot controller does not have a method to set the required landmarks."
        )

    def move_to_start_pose(self):
        """
        Moves the robot to the starting pose at the beginning of the program.
        """

        raise NotImplementedError(
            "The robot controller does not have a starting pose movement method."
        )

    def publish_move(self):
        """
        Transforms the data given by the main program into the form necessary for the particular robot.
        Custom methods can be called from here if any additional transformations are needed.
        Should take the format defined below.
        """

        raise NotImplementedError(
            "The robot controller does not have a publish move method."
        )

        while True:
            # Get data from the queue
            variable = self.data_queue.get()

            # Exit thread if end signal received
            if variable == "END":
                return

            # If there is data in the queue
            if variable is not None:
                # Any data processing and publish to the robot
                pass


class BaseHandController(RobotController):
    def __init__(self, nb_fingers: int) -> None:
        super().__init__()
        self.nb_fingers: int = nb_fingers


class BaseArmController(RobotController):
    def __init__(self) -> None:
        super().__init__()
