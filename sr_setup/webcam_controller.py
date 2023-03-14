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

        # self.joint_list = [
        #     ([7, 6, 5], [6, 5, 0]),
        #     ([11, 10, 9], [10, 9, 0]),
        #     ([15, 14, 13], [14, 13, 0]),
        #     ([19, 18, 17], [18, 17, 0]),
        #     ([4, 3, 2], [3, 2, 0]),
        # ]

    # def get_angle(self, hand, joint):
    #     # Get each joint position
    #     high_joint_x = hand.landmark[joint[0]].x
    #     mid_joint_x = hand.landmark[joint[1]].x
    #     low_joint_x = hand.landmark[joint[2]].x

    #     high_joint_y = hand.landmark[joint[0]].y
    #     mid_joint_y = hand.landmark[joint[1]].y
    #     low_joint_y = hand.landmark[joint[2]].y

    #     high_joint_z = hand.landmark[joint[0]].z
    #     mid_joint_z = hand.landmark[joint[1]].z
    #     low_joint_z = hand.landmark[joint[2]].z

    #     # if joint[2] == 0:
    #     #     low_joint_z = mid_joint_z
    #     # else:
    #     #     low_joint_z = hand.landmark[joint[2]].z

    #     # Calculate the vectors from the top to the middle and the bottom to the middle
    #     high_mid_vec = [
    #         high_joint_x - mid_joint_x,
    #         high_joint_y - mid_joint_y,
    #         high_joint_z - mid_joint_z,
    #     ]
    #     low_mid_vec = [
    #         low_joint_x - mid_joint_x,
    #         low_joint_y - mid_joint_y,
    #         low_joint_z - mid_joint_z,
    #     ]

    #     # Calculate the magnitudes of those vectors to normalise them
    #     high_mid_vec_mag = np.sqrt(
    #         high_mid_vec[0] ** 2 + high_mid_vec[1] ** 2 + high_mid_vec[2] ** 2
    #     )
    #     high_mid_vec_normalised = [
    #         high_mid_vec[0] / high_mid_vec_mag,
    #         high_mid_vec[1] / high_mid_vec_mag,
    #         high_mid_vec[2] / high_mid_vec_mag,
    #     ]

    #     low_mid_vec_mag = np.sqrt(
    #         low_mid_vec[0] ** 2 + low_mid_vec[1] ** 2 + low_mid_vec[2] ** 2
    #     )
    #     low_mid_vec_normalised = [
    #         low_mid_vec[0] / low_mid_vec_mag,
    #         low_mid_vec[1] / low_mid_vec_mag,
    #         low_mid_vec[2] / low_mid_vec_mag,
    #     ]

    #     # Get the dot product between the two vectors
    #     # Equivalent to the cosine of the angle between the two vectors
    #     dot = (
    #         high_mid_vec_normalised[0] * low_mid_vec_normalised[0]
    #         + high_mid_vec_normalised[1] * low_mid_vec_normalised[1]
    #         + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]
    #     )

    #     # Calculate the actual angle
    #     angle = 180 - np.degrees(np.arccos(dot))

    #     # If the angle is calculated for a lower joint, clip it between 0 and 90
    #     # Avoids out of bounds angles being published to the robot
    #     if joint[2] == 0:
    #         angle = np.clip((angle - 45) * 2 + 45, 0, 90)

    #     return angle

    # def display_angle(self, image, angle, xy):
    #     # Display the angle at the correct place on the image.
    #     cv2.putText(
    #         image,
    #         str(round(angle, 2)),
    #         tuple(np.multiply(xy, [self.cap_width, self.cap_height]).astype(int)),
    #         cv2.FONT_HERSHEY_TRIPLEX,
    #         0.5,
    #         (255, 255, 255),
    #         2,
    #         cv2.LINE_AA,
    #     )

    def read_capture(self):
        ret, frame = self.webcam_controller.cap.read()

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

        return image

    def draw_landmark_results(self, results, image):
        for num, hand in enumerate(results.multi_hand_landmarks):
            self.webcam_controller.mp_drawing.draw_landmarks(
                image,
                hand,
                self.webcam_controller.mp_hands.HAND_CONNECTIONS,
            )

    def get_landmark_data(self, image, results, required_landmarks):
        landmark_data = {"x": [], "y": [], "z": []}
        for hand in results.multi_hand_landmarks:
            landmark_data["x"].append(
                [hand.landmark[landmark].x for landmark in required_landmarks]
            )
            landmark_data["y"].append(
                [hand.landmark[landmark].y for landmark in required_landmarks]
            )
            landmark_data["z"].append(
                [hand.landmark[landmark].z for landmark in required_landmarks]
            )

        return landmark_data

    def display_angle(self, image, landmark_data, angle_dict):
        for joint, angle in angle_dict.items:
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
