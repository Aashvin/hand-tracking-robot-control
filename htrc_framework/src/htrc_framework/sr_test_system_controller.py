#!/usr/bin/env python3.8

import cv2
import os
import pandas as pd
import threading
import time
from typing import Optional

from htrc_framework.finger_angle_utilities import finger_angles
from htrc_framework.base_robot_controllers import BaseHandController, BaseArmController
from htrc_framework.webcam_controller import WebcamController


REFRESH_RATE = 0.2


class Controller:
    def __init__(
        self,
        webcam_controller: WebcamController,
        hand_controller: BaseHandController,
        arm_controller: Optional[BaseArmController] = None,
    ) -> None:
        self.hand_controller: BaseHandController = hand_controller
        self.arm_controller: Optional[BaseArmController] = arm_controller

        self.webcam_controller: WebcamController = webcam_controller

    def run_hand(self, pose, angle) -> None:
        """
        Runs the program if there is only a hand.
        """

        test_data = {
            "rh_FFJ2": [],
            "rh_FFJ3": [],
            "rh_MFJ2": [],
            "rh_MFJ3": [],
            "rh_RFJ2": [],
            "rh_RFJ3": [],
            "rh_LFJ2": [],
            "rh_LFJ3": [],
            "rh_THJ1": [],
            "rh_THJ2": [],
        }

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

                        current_pose = (
                            self.hand_controller.commander.get_current_state_bounded()
                        )
                        for joint in test_data.keys():
                            test_data[joint].append(current_pose[joint])

                        # Add relevant angles to the data queue for the hand thread
                        with self.hand_controller.data_queue.mutex:
                            self.hand_controller.data_queue.queue.clear()
                        self.hand_controller.data_queue.put(angle_dict)

                # Display the camera and hand tracking data if present
                cv2.imshow("Finger Angles", image)

                # Quit if the 'ESC' key is pressed
                if cv2.waitKey(10) & 0xFF == 27:
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        hand_thread.join()

        for joint in test_data.keys():
            test_data[joint] = test_data[joint][-150:]
        test_data_df = pd.DataFrame(test_data)

        file_path = os.path.realpath(os.path.dirname(__file__)).replace("\\", "/")
        slash_indices = [i for i, c in enumerate(file_path) if c == "/"]
        file_path = file_path[:slash_indices[-3]]
        if not os.path.exists(f"{file_path}/test_data"):
            os.mkdir(f"{file_path}/test_data")

        test_data_df.to_csv(
            f"{file_path}/test_data/{self.hand_controller.nb_fingers}-fingers-pose{pose}-angle{angle}.csv", index=False
        )


    def run_arm_hand(self, refresh_rate) -> None:
        """
        Runs the program if there is both a hand and an arm.
        """

        test_data = {
            "end_effector_x": [],
            "end_effector_y": [],
            "end_effector_z": [],
        }

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

        print("Click ENTER when you're ready to start the test. Click ESC when the test has finished.")
        input()
        test_started_time = time.time()

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
                    if time2 - time1 >= refresh_rate:
                        time1 = time2

                        current_pose = (
                            self.arm_controller.commander.get_current_pose("ra_base")
                        )

                        test_data["end_effector_x"].append(current_pose.position.x)
                        test_data["end_effector_y"].append(current_pose.position.y)
                        test_data["end_effector_z"].append(current_pose.position.z)

                        # Add relevant data to the data queues for the hand and arm
                        with self.hand_controller.data_queue.mutex:
                            self.hand_controller.data_queue.queue.clear()
                        self.hand_controller.data_queue.put(angle_dict)
                        with self.arm_controller.data_queue.mutex:
                            self.arm_controller.data_queue.queue.clear()
                        self.arm_controller.data_queue.put(landmark_data)

                # Display the camera and hand tracking data if present
                cv2.imshow("Finger Angles", image)

                # Quit if the 'q' key is pressed
                if cv2.waitKey(10) & 0xFF == 27:
                    total_test_time = time.time() - test_started_time
                    break

        # Safely end the program
        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.data_queue.put("END")
        self.arm_controller.data_queue.put("END")
        hand_thread.join()
        arm_thread.join()

        for axis in test_data.keys():
            test_data[axis] = test_data[axis][-150:]
            test_data[axis].append(total_test_time)
        test_data_df = pd.DataFrame(test_data)

        file_path = os.path.realpath(os.path.dirname(__file__)).replace("\\", "/")
        slash_indices = [i for i, c in enumerate(file_path) if c == "/"]
        file_path = file_path[:slash_indices[-3]]
        if not os.path.exists(f"{file_path}/test_data"):
            os.mkdir(f"{file_path}/test_data")

        file_num = 0
        file_path = f"{file_path}/test_data/arm-hand-refresh-rate-{refresh_rate}"
        while os.path.isfile(f"{file_path}-trial-{file_num}.csv"):
            file_num += 1

        test_data_df.to_csv(
            f"{file_path}-trial-{file_num}.csv", index=False
        )