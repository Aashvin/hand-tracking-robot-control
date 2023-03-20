#!/usr/bin/env python3.8

import mediapipe as mp
import cv2
import rospy
import time
import threading
from typing import Optional

from finger_angle_utilities import finger_angles
from webcam_controller import WebcamController
from robot_controller import RobotController


class Controller:
    def __init__(
        self,
        webcam_controller: WebcamController,
        hand_controller: RobotController,
        arm_controller: Optional[RobotController] = None,
    ) -> None:
        self.hand_controller = hand_controller
        self.arm_controller = arm_controller

        self.webcam_controller = webcam_controller

    def run_hand(self):
        self.hand_controller.set_required_landmarks()
        self.hand_controller.move_to_start_pose()

        time.sleep(2)

        hand_thread = threading.Thread(target=self.hand_controller.publish_move)
        hand_thread.start()

        time1 = time.time()

        # Set the detection confidence and tracking confidence for better result
        with self.webcam_controller.mp_hands.Hands(
            min_detection_confidence=0.7, min_tracking_confidence=0.7
        ) as hands:
            while self.webcam_controller.cap.isOpened():
                image, results = self.webcam_controller.read_capture(hands)

                # Rendering results
                if results.multi_hand_landmarks:
                    self.webcam_controller.draw_landmark_results(results, image)

                    landmark_data = self.webcam_controller.get_landmark_data(
                        results, self.hand_controller.required_landmarks
                    )

                    angle_dict = finger_angles(
                        self.hand_controller.nb_fingers,
                        self.hand_controller.required_landmarks,
                        landmark_data,
                    )

                    self.webcam_controller.display_angle(
                        image, landmark_data, angle_dict
                    )

                    time2 = time.time()
                    if time2 - time1 > 0.5:
                        time1 = time2

                        self.hand_controller.data_queue.put(angle_dict)

                # Showing the camera
                cv2.imshow("Finger Angles", image)
                # exxit the program
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        hand_thread.join()

    def run_arm_hand(self):
        self.hand_controller.set_required_landmarks()
        self.arm_controller.set_required_landmarks()

        self.arm_controller.move_to_start_pose()
        # rospy.sleep(3.0)
        self.hand_controller.move_to_start_pose()

        # rospy.sleep(2)

        hand_thread = threading.Thread(target=self.hand_controller.publish_move)
        hand_thread.start()

        arm_thread = threading.Thread(target=self.arm_controller.publish_move)
        arm_thread.start()

        time1 = time.time()

        # Set the detection confidence and tracking confidence for better result
        with self.webcam_controller.mp_hands.Hands(
            min_detection_confidence=0.7, min_tracking_confidence=0.7
        ) as hands:
            while self.webcam_controller.cap.isOpened():
                image, results = self.webcam_controller.read_capture(hands)

                # Rendering results
                if results.multi_hand_landmarks:
                    self.webcam_controller.draw_landmark_results(results, image)

                    all_required_landmarks = (
                        self.hand_controller.required_landmarks
                        + self.arm_controller.required_landmarks
                    )

                    landmark_data = self.webcam_controller.get_landmark_data(
                        results, all_required_landmarks
                    )

                    angle_dict = self.hand_controller.finger_angles(landmark_data)

                    self.webcam_controller.display_angle(
                        image, landmark_data, angle_dict
                    )

                    time2 = time.time()
                    if time2 - time1 > 0.5:
                        time1 = time2

                        arm_position_dict = self.arm_controller.process_landmark_data(
                            landmark_data
                        )

                        self.hand_controller.data_queue.put(angle_dict)
                        self.arm_controller.data_queue.put(arm_position_dict)

                # Showing the camera
                cv2.imshow("Finger Angles", image)
                # exxit the program
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        self.arm_controller.data_queue.put("END")
        hand_thread.join()
        arm_thread.join()
