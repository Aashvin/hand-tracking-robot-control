#!/usr/bin/env python3.8

import cv2
import threading
import time
from typing import Optional

from finger_angle_utilities import finger_angles
from robot_controller import RobotController
from webcam_controller import WebcamController


REFRESH_RATE = 0.2


class Controller:
    def __init__(
        self,
        webcam_controller: WebcamController,
        hand_controller: RobotController,
        hand2_controller: Optional[RobotController] = None,
        arm_controller: Optional[RobotController] = None,
    ) -> None:
        self.hand_controller: RobotController = hand_controller
        self.hand2_controller: Optional[RobotController] = hand2_controller
        self.arm_controller: Optional[RobotController] = arm_controller

        self.webcam_controller: WebcamController = webcam_controller

        

    def run_hand(self) -> None:
        """
        Runs the program if there is only one hand.
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
        with self.webcam_controller.detector:
            while self.webcam_controller.cap.isOpened():
                # Get the image and hand tracking results from the webcam and MediaPipe Hands
                mp_image, results = self.webcam_controller.read_capture()
                image = mp_image.numpy_view()

                # If the right hand has been found in the frame
                if results.hand_landmarks:

                    # Process x, y, z coordinates for the required landmarks
                    landmark_data = self.webcam_controller.get_landmark_data(
                        results.hand_landmarks[0], self.hand_controller.required_landmarks
                    )

                    # Calculate the angles of the required relevant joints
                    angle_dict = finger_angles(
                        self.hand_controller.nb_fingers,
                        self.hand_controller.required_landmarks,
                        landmark_data,
                    )

                    # Render hand tracking landmark visuals and angles
                    image = self.webcam_controller.draw_landmark_results(results.hand_landmarks[0], image)
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
                if cv2.waitKey(10) & 0xFF == 27:
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        hand_thread.join()

    def run_2_hands(self) -> None:
        """
        Runs the program if there are two hands.
        """

        try:
            assert self.hand2_controller
        except:
            raise AttributeError("The hand2_controller attribute needs to be set.")

        # Set the required MediaPipe Hands landmarks and move the hand to its starting pose
        self.hand_controller.set_required_landmarks()
        self.hand2_controller.set_required_landmarks()
        self.hand_controller.move_to_start_pose()
        self.hand2_controller.move_to_start_pose()

        time.sleep(2)

        # Start the hand thread with its target
        hand_thread = threading.Thread(target=self.hand_controller.publish_move)
        hand_thread.start()

        hand2_thread = threading.Thread(target=self.hand2_controller.publish_move)
        hand2_thread.start()

        # Start the time for refresh rate implementation
        time1 = time.time()

        # Set the detection and tracking confidences of MediaPipe Hands
        with self.webcam_controller.detector:
            while self.webcam_controller.cap.isOpened():
                # Get the image and hand tracking results from the webcam and MediaPipe Hands
                mp_image, results = self.webcam_controller.read_capture()
                image = mp_image.numpy_view()

                second_hand = None

                if results.hand_landmarks:
                    first_hand = self.hand_controller if results.handedness[0][0].category_name == "Right" else self.hand2_controller

                    first_hand_landmark_data = self.webcam_controller.get_landmark_data(
                        results.hand_landmarks[0], first_hand.required_landmarks
                    )

                    # Calculate the angles of the required relevant joints
                    first_hand_angle_dict = finger_angles(
                        first_hand.nb_fingers,
                        first_hand.required_landmarks,
                        first_hand_landmark_data,
                    )

                    # Render hand tracking landmark visuals and angles
                    image = self.webcam_controller.draw_landmark_results(results.hand_landmarks[0], image)
                    self.webcam_controller.display_angle(
                        image, first_hand_landmark_data, first_hand_angle_dict
                    )

                    if len(results.handedness) == 2:
                        second_hand = self.hand2_controller if results.handedness[0][0].category_name == "Right" else self.hand_controller

                        second_hand_landmark_data = self.webcam_controller.get_landmark_data(
                            results.hand_landmarks[1], second_hand.required_landmarks
                        )

                        second_hand_angle_dict = finger_angles(
                            second_hand.nb_fingers,
                            second_hand.required_landmarks,
                            second_hand_landmark_data,
                        )

                        image = self.webcam_controller.draw_landmark_results(results.hand_landmarks[1], image)
                        self.webcam_controller.display_angle(
                            image, second_hand_landmark_data, second_hand_angle_dict
                        )

                    # If refresh rate time has been hit
                    time2 = time.time()
                    if time2 - time1 >= REFRESH_RATE:
                        time1 = time2

                        # Add relevant angles to the data queue for the hand thread
                        first_hand.data_queue.put(first_hand_angle_dict)

                        if second_hand:
                            second_hand.data_queue.put(second_hand_angle_dict)

                # Display the camera and hand tracking data if present
                cv2.imshow("Finger Angles", image)

                # Quit if the 'q' key is pressed
                if cv2.waitKey(10) & 0xFF == 27:
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        self.hand2_controller.data_queue.put("END")
        hand_thread.join()
        hand2_thread.join()

    def run_arm_hand(self) -> None:
        """
        Runs the program if there is both a hand and an arm.
        """

        try:
            assert self.arm_controller
        except:
            raise AttributeError("The arm_controller attribute needs to be set.")

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
        with self.webcam_controller.detector:
            while self.webcam_controller.cap.isOpened():
                # Get the image and hand tracking results from the webcam and MediaPipe Hands
                mp_image, results = self.webcam_controller.read_capture()
                image = mp_image.numpy_view()

                # If the right hand has been found in the frame
                if results.hand_landmarks:

                    # Process x, y, z coordinates for the required landmarks
                    landmark_data = self.webcam_controller.get_landmark_data(
                        results.hand_landmarks[0], all_required_landmarks
                    )

                    # Calculate the angles of the required relevant joints
                    angle_dict = finger_angles(
                        self.hand_controller.nb_fingers,
                        self.hand_controller.required_landmarks,
                        landmark_data,
                    )

                    # Render hand tracking landmark visuals and angles
                    image = self.webcam_controller.draw_landmark_results(results.hand_landmarks[0], image)
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
                if cv2.waitKey(10) & 0xFF == 27:
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        self.arm_controller.data_queue.put("END")
        hand_thread.join()
        arm_thread.join()
