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
                            self.hand_controller.controller.get_current_state_bounded()
                        )
                        for joint in test_data.keys():
                            test_data[joint].append(current_pose[joint])

                        # Add relevant angles to the data queue for the hand thread
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

        if not os.path.exists("/home/user/test_data"):
            os.mkdir("/home/user/test_data")

        test_data_df.to_csv(
            f"/home/user/test_data/{self.hand_controller.nb_fingers}-fingers-pose{pose}-angle{angle}.csv", index=False
        )