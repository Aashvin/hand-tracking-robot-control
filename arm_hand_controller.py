#!/usr/bin/env python3.8

# General Python imports
import cv2
import mediapipe as mp
import numpy as np
from queue import Queue
import threading
import time

# ROS imports
import geometry_msgs
import moveit_msgs.msg
import rospy
import tf

# Shadow Robot imports
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
print("DONE")
cap_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cap_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

joint_list = [
    ([7, 6, 5], [6, 5, 0]),
    ([11, 10, 9], [10, 9, 0]),
    ([15, 14, 13], [14, 13, 0]),
    ([19, 18, 17], [18, 17, 0]),
    ([4, 3, 2], [3, 2, 0]),
]


def publish_joint(flex_queue, hand_commander):
    while True:
        flex = flex_queue.get()

        if flex == "END":
            return

        if flex is not None:
            hand_commander.move_to_joint_value_target_unsafe(
                flex, wait=False, angle_degrees=True
            )


def publish_move(move_queue, arm_commander):
    constraint = moveit_msgs.msg.Constraints()
    constraint.name = "all_constraints"
    arm_constraint = moveit_msgs.msg.JointConstraint()
    arm_constraint.joint_name = "ra_shoulder_lift_joint"
    arm_constraint.position = np.radians(-45)
    arm_constraint.tolerance_below = np.radians(45)
    arm_constraint.tolerance_above = np.radians(100)

    constraint.joint_constraints = [arm_constraint]

    while True:
        move = move_queue.get()

        if move == "END":
            return

        if move is not None:
            arm_commander.move_to_pose_value_target_unsafe(
                move, wait=False, ik_constraints=constraint
            )


def get_angle(hand, joint):
    # Get each joint position
    high_joint_x = hand.landmark[joint[0]].x
    mid_joint_x = hand.landmark[joint[1]].x
    low_joint_x = hand.landmark[joint[2]].x

    high_joint_y = hand.landmark[joint[0]].y
    mid_joint_y = hand.landmark[joint[1]].y
    low_joint_y = hand.landmark[joint[2]].y

    high_joint_z = hand.landmark[joint[0]].z
    mid_joint_z = hand.landmark[joint[1]].z
    low_joint_z = hand.landmark[joint[2]].z

    # if joint[2] == 0:
    #     low_joint_z = mid_joint_z
    # else:
    #     low_joint_z = hand.landmark[joint[2]].z

    # Calculate the vectors from the top to the middle and the bottom to the middle
    high_mid_vec = [
        high_joint_x - mid_joint_x,
        high_joint_y - mid_joint_y,
        high_joint_z - mid_joint_z,
    ]
    low_mid_vec = [
        low_joint_x - mid_joint_x,
        low_joint_y - mid_joint_y,
        low_joint_z - mid_joint_z,
    ]

    # Calculate the magnitudes of those vectors to normalise them
    high_mid_vec_mag = np.sqrt(
        high_mid_vec[0] ** 2 + high_mid_vec[1] ** 2 + high_mid_vec[2] ** 2
    )
    high_mid_vec_normalised = [
        high_mid_vec[0] / high_mid_vec_mag,
        high_mid_vec[1] / high_mid_vec_mag,
        high_mid_vec[2] / high_mid_vec_mag,
    ]

    low_mid_vec_mag = np.sqrt(
        low_mid_vec[0] ** 2 + low_mid_vec[1] ** 2 + low_mid_vec[2] ** 2
    )
    low_mid_vec_normalised = [
        low_mid_vec[0] / low_mid_vec_mag,
        low_mid_vec[1] / low_mid_vec_mag,
        low_mid_vec[2] / low_mid_vec_mag,
    ]

    # Get the dot product between the two vectors
    # Equivalent to the cosine of the angle between the two vectors
    dot = (
        high_mid_vec_normalised[0] * low_mid_vec_normalised[0]
        + high_mid_vec_normalised[1] * low_mid_vec_normalised[1]
        + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]
    )

    # Calculate the actual angle
    angle = 180 - np.degrees(np.arccos(dot))

    # If the angle is calculated for a lower joint, clip it between 0 and 90
    # Avoids out of bounds angles being published to the robot
    if joint[2] == 0:
        angle = np.clip((angle - 45) * 2 + 45, 0, 90)

    return angle


def display_angle(image, angle, xy):
    # Display the angle at the correct place on the image.
    cv2.putText(
        image,
        str(round(angle, 2)),
        tuple(np.multiply(xy, [cap_width, cap_height]).astype(int)),
        cv2.FONT_HERSHEY_TRIPLEX,
        0.5,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def finger_angles(image, results, joint_list):
    counter = 1
    angle_dict = {}

    # For each hand in the image - currently robot control only supports one hand
    for hand in results.multi_hand_landmarks:
        # Get the position of the base of the middle finger for the x, y of the robot
        mid_finger_x = hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
        mid_finger_y = hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
        mid_finger_pos = (mid_finger_x, mid_finger_y)

        # Get the normalised distance between the base of the middle finger and the wrist for the z of the robot
        squared_y_dist = (
            hand.landmark[mp_hands.HandLandmark.WRIST].y
            - hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
        ) ** 2
        squared_x_dist = (
            hand.landmark[mp_hands.HandLandmark.WRIST].x
            - hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
        ) ** 2
        norm_dist = np.sqrt(squared_y_dist + squared_x_dist)

        # Loop through each finger in the joint list
        for upper_joint, lower_joint in joint_list:
            # Get upper and lower angles to publish to robot
            upper_angle = get_angle(hand, upper_joint)
            lower_angle = get_angle(hand, lower_joint)
            angle_dict[counter + 1] = lower_angle
            angle_dict[counter] = upper_angle
            counter += 2

            # Display the angle values at the correct place on the image
            upper_display = np.array(
                [hand.landmark[upper_joint[1]].x, hand.landmark[upper_joint[1]].y]
            )
            lower_display = np.array(
                [hand.landmark[lower_joint[1]].x, hand.landmark[lower_joint[1]].y]
            )
            display_angle(image, upper_angle, upper_display)
            display_angle(image, lower_angle, lower_display)

    return angle_dict, image, mid_finger_pos, norm_dist


def run():
    rospy.init_node("right_hand_demo", anonymous=True)
    hand_commander = SrHandCommander(name="right_hand")

    arm_commander = SrArmCommander(name="right_arm")
    arm_commander.set_pose_reference_frame("ra_base")

    arm_commander.move_to_named_target("ra_start")
    arm_commander.move_to_joint_value_target_unsafe(
        {"ra_wrist_3_joint": -180}, wait=True, angle_degrees=True
    )
    rospy.sleep(3.0)
    hand_commander.move_to_named_target("open")

    # print(arm_commander.get_current_pose("ra_base"))

    starting_pose = arm_commander.get_current_pose("ra_base")

    desired_pose = geometry_msgs.msg.PoseStamped()

    desired_pose.pose.position.x = 0.5
    desired_pose.pose.position.y = -0.5
    desired_pose.pose.position.z = starting_pose.position.z

    starting_orientation_euler = tf.transformations.euler_from_quaternion(
        (
            starting_pose.orientation.x,
            starting_pose.orientation.y,
            starting_pose.orientation.z,
            starting_pose.orientation.w,
        )
    )

    desired_orientation = tf.transformations.quaternion_from_euler(
        starting_orientation_euler[0], starting_orientation_euler[1], 0
    )

    desired_pose.pose.orientation.x = desired_orientation[0]
    desired_pose.pose.orientation.y = desired_orientation[1]
    desired_pose.pose.orientation.z = desired_orientation[2]
    desired_pose.pose.orientation.w = desired_orientation[3]
    arm_commander.move_to_pose_value_target_unsafe(desired_pose)

    time.sleep(2)

    flex_queue = Queue()
    move_queue = Queue()

    t1 = threading.Thread(target=publish_joint, args=[flex_queue, hand_commander])
    t1.start()

    t2 = threading.Thread(target=publish_move, args=[move_queue, arm_commander])
    t2.start()

    time1 = time.time()

    # Set the detection confidence and tracking confidence for better result
    with mp_hands.Hands(
        min_detection_confidence=0.7, min_tracking_confidence=0.7
    ) as hands:
        while cap.isOpened():
            ret, frame = cap.read()

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

            # Rendering results
            if results.multi_hand_landmarks:
                for num, hand in enumerate(results.multi_hand_landmarks):
                    mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS)

                # Draw angles to image from joint list
                angle_list, image, mid_finger_pos, norm_dist = finger_angles(
                    image, results, joint_list
                )

                time2 = time.time()
                if time2 - time1 > 0.5:
                    time1 = time2

                    flex = {
                        "rh_FFJ2": angle_list[1],
                        "rh_FFJ3": angle_list[2],
                        "rh_MFJ2": angle_list[3],
                        "rh_MFJ3": angle_list[4],
                        "rh_RFJ2": angle_list[5],
                        "rh_RFJ3": angle_list[6],
                        "rh_LFJ2": angle_list[7],
                        "rh_LFJ3": angle_list[8],
                        "rh_THJ2": angle_list[9],
                        "rh_THJ3": angle_list[10],
                    }

                    flex_queue.put(flex)

                    desired_pose.pose.position.x = -mid_finger_pos[1] + 1
                    desired_pose.pose.position.y = -mid_finger_pos[0] * 2 + 1
                    desired_pose.pose.position.z = 0.8 - norm_dist
                    desired_pose.pose.position.z = max(
                        0.2, desired_pose.pose.position.z
                    )
                    move_queue.put(desired_pose)

            # Showing the camera
            cv2.imshow("Finger Angles", image)
            # exxit the program
            if cv2.waitKey(10) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    flex_queue.put("END")
    move_queue.put("END")
    t1.join()
    t2.join()


if __name__ == "__main__":
    run()
