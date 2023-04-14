#!/usr/bin/env python3.8

import control_msgs.msg
import cv2
import os
import pandas as pd
import rospy
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

        test_data = {"finger1": [], "finger2": []}

        if self.hand_controller.nb_fingers == 2:
            finger_positions = [0.0, 0.0]
        elif self.hand_controller.nb_fingers == 3:
            finger_positions = [0.0, 0.0, 0.0]
            test_data["finger3"] = []

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

                        finger_topic = "/" + self.hand_controller.prefix + "/effort_finger_trajectory_controller/state"
                        finger_positions = rospy.wait_for_message(finger_topic, control_msgs.msg.JointTrajectoryControllerState).actual.positions

                        for joint in test_data.keys():
                            test_data[joint].append(finger_positions[int(joint[-1]) - 1])

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