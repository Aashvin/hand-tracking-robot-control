#!/usr/bin/env python3.8

import mediapipe as mp
import cv2
import numpy as np


HAND_LANDMARKS = mp.solutions.hands.HandLandmark


class WebcamController:
    def __init__(self, source: str = "/dev/video0") -> None:
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

        self.cap_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.cap_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def read_capture(self, hands):
        ret, frame = self.cap.read()

        # Convert from BGR to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Flip on the horizontal axis for hand tracking and display
        image = cv2.flip(image, 1)

        # Set flag
        image.flags.writeable = False

        # Hand tracking for this frame
        results = hands.process(image)

        # Set flag to true
        image.flags.writeable = True

        # RGB 2 BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        return image, results

    def draw_landmark_results(self, results, image):
        for num, hand in enumerate(results.multi_hand_landmarks):
            self.mp_drawing.draw_landmarks(
                image,
                hand,
                self.mp_hands.HAND_CONNECTIONS,
            )

    def get_landmark_data(self, results, required_landmarks):
        landmark_data = {}
        for hand in results.multi_hand_landmarks:
            landmark_data["x"] = {
                landmark: hand.landmark[landmark].x for landmark in required_landmarks
            }

            landmark_data["y"] = {
                landmark: hand.landmark[landmark].y for landmark in required_landmarks
            }

            landmark_data["z"] = {
                landmark: hand.landmark[landmark].z for landmark in required_landmarks
            }

        return landmark_data

    def display_angle(self, image, landmark_data, angle_dict):
        for joint, angle in angle_dict.items():
            coordinates = np.array(
                [landmark_data["x"][joint], landmark_data["y"][joint]]
            )

            cv2.putText(
                image,
                str(round(angle, 2)),
                tuple(
                    np.multiply(coordinates, [self.cap_width, self.cap_height]).astype(
                        int
                    )
                ),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    # def finger_angles(self, image, results):
    #     counter = 1
    #     angle_dict = {}

    #     # For each hand in the image - currently robot control only supports one hand
    #     for hand in results.multi_hand_landmarks:
    #         # Get the position of the base of the middle finger for the x, y of the robot
    #         mid_finger_x = hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
    #         mid_finger_y = hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
    #         mid_finger_pos = (mid_finger_x, mid_finger_y)

    #         # Get the normalised distance between the base of the middle finger and the wrist for the z of the robot
    #         squared_y_dist = (
    #             hand.landmark[self.mp_hands.HandLandmark.WRIST].y
    #             - hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
    #         ) ** 2
    #         squared_x_dist = (
    #             hand.landmark[self.mp_hands.HandLandmark.WRIST].x
    #             - hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
    #         ) ** 2
    #         norm_dist = np.sqrt(squared_y_dist + squared_x_dist)

    #         # Loop through each finger in the joint list
    #         for upper_joint, lower_joint in self.joint_list:
    #             # Get upper and lower angles to publish to robot
    #             upper_angle = self.get_angle(hand, upper_joint)
    #             lower_angle = self.get_angle(hand, lower_joint)
    #             angle_dict[counter + 1] = lower_angle
    #             angle_dict[counter] = upper_angle
    #             counter += 2

    #             # Display the angle values at the correct place on the image
    #             upper_display = np.array(
    #                 [hand.landmark[upper_joint[1]].x, hand.landmark[upper_joint[1]].y]
    #             )
    #             lower_display = np.array(
    #                 [hand.landmark[lower_joint[1]].x, hand.landmark[lower_joint[1]].y]
    #             )
    #             self.display_angle(image, upper_angle, upper_display)
    #             self.display_angle(image, lower_angle, lower_display)

    #     return angle_dict, image, mid_finger_pos, norm_dist
