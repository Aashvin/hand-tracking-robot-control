#!/usr/bin/env python3.8

import cv2
import threading
import time
from typing import Optional

from finger_angle_utilities import finger_angles
from robot_controller import RobotController
from webcam_controller import WebcamController


REFRESH_RATE = 0.5


class Controller:
    def __init__(
        self,
        webcam_controller: WebcamController,
        hand_controller: RobotController,
        arm_controller: Optional[RobotController] = None,
    ) -> None:
        self.hand_controller: RobotController = hand_controller
        self.arm_controller: Optional[RobotController] = arm_controller

        self.webcam_controller: WebcamController = webcam_controller

    def run_hand(self) -> None:
        """
        Runs the program if there is only a hand.
        """

        # Set the required MediaPipe Hands landmarks and move the hand to its starting pose
        self.hand_controller.set_required_landmarks()
        self.hand_controller.move_to_start_pose()

        time.sleep(2)

        # Start the hand thread with its target
        hand_thread = threading.Thread(target=self.hand_controller.publish_move)
        hand_thread.start()

        # Start the time for refresh rate implementation
        time1 = time.time()

        # Set the detection and tracking confidences of MediaPipe Hands
        with self.webcam_controller.mp_hands.Hands(
            min_detection_confidence=0.7, min_tracking_confidence=0.7
        ) as hands:
            while self.webcam_controller.cap.isOpened():
                # Get the image and hand tracking results from the webcam and MediaPipe Hands
                image, results = self.webcam_controller.read_capture(hands)

                # If a hand is found in the webcam frame
                if results.multi_hand_landmarks:
                    # Process x, y, z coordinates for the required landmarks
                    landmark_data = self.webcam_controller.get_landmark_data(
                        results, self.hand_controller.required_landmarks
                    )

                    # Calculate the angles of the required relevant joints
                    angle_dict = finger_angles(
                        self.hand_controller.nb_fingers,
                        self.hand_controller.required_landmarks,
                        landmark_data,
                    )

                    # Render hand tracking landmark visuals and angles
                    self.webcam_controller.draw_landmark_results(results, image)
                    self.webcam_controller.display_angle(
                        image, landmark_data, angle_dict
                    )

                    # If refresh rate time has been hit
                    time2 = time.time()
                    if time2 - time1 >= REFRESH_RATE:
                        time1 = time2

                        # Add relevant angles to the data queue for the hand thread
                        self.hand_controller.data_queue.put(angle_dict)

                # Display the camera and hand tracking data if present
                cv2.imshow("Finger Angles", image)

                # Quit if the 'q' key is pressed
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        hand_thread.join()

    def run_arm_hand(self) -> None:
        """
        Runs the program if there is both a hand and an arm.
        """

        # Set the required MediaPipe Hands landmarks
        self.hand_controller.set_required_landmarks()
        self.arm_controller.set_required_landmarks()

        all_required_landmarks = (
            self.hand_controller.required_landmarks
            + self.arm_controller.required_landmarks
        )

        # Move the hand and arm to their starting poses
        self.arm_controller.move_to_start_pose()
        self.hand_controller.move_to_start_pose()

        time.sleep(2)

        # Start the hand and arm threads with their targets
        hand_thread = threading.Thread(target=self.hand_controller.publish_move)
        hand_thread.start()

        arm_thread = threading.Thread(target=self.arm_controller.publish_move)
        arm_thread.start()

        time1 = time.time()

        # Set the detection and tracking confidences of MediaPipe Hands
        with self.webcam_controller.mp_hands.Hands(
            min_detection_confidence=0.7, min_tracking_confidence=0.7
        ) as hands:
            while self.webcam_controller.cap.isOpened():
                # Get the image and hand tracking results from the webcam and MediaPipe Hands
                image, results = self.webcam_controller.read_capture(hands)

                # If a hand is found in the webcam frame
                if results.multi_hand_landmarks:
                    # Process x, y, z coordinates for the required landmarks
                    landmark_data = self.webcam_controller.get_landmark_data(
                        results, all_required_landmarks
                    )

                    # Calculate the angles of the required relevant joints
                    # This processing is required here so the angles can be displayed
                    angle_dict = finger_angles(
                        self.hand_controller.nb_fingers,
                        self.hand_controller.required_landmarks,
                        landmark_data,
                    )

                    # Render hand tracking landmark visuals and angles
                    self.webcam_controller.draw_landmark_results(results, image)
                    self.webcam_controller.display_angle(
                        image, landmark_data, angle_dict
                    )

                    # If refresh rate time has been hit
                    time2 = time.time()
                    if time2 - time1 > 0.5:
                        time1 = time2

                        # Add relevant data to the data queues for the hand and arm
                        self.hand_controller.data_queue.put(angle_dict)
                        self.arm_controller.data_queue.put(landmark_data)

                # Display the camera and hand tracking data if present
                cv2.imshow("Finger Angles", image)

                # Quit if the 'q' key is pressed
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        self.arm_controller.data_queue.put("END")
        hand_thread.join()
        arm_thread.join()
