import mediapipe as mp
import cv2
import numpy as np
import time
import threading
from queue import Queue
import tf
import geometry_msgs

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
mp_hands2 = mp.solutions.hands

cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
print("DONE")
cap_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cap_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

joint_list = [[7, 6, 5], [11, 10, 9], [15, 14, 13], [19, 18, 17], [2, 3, 4]]


def argumentParser(argument):
    """Argument parser"""
    parser = argparse.ArgumentParser(
        description="Drive robot joint to command position"
    )
    parser.add_argument(
        "kinova_robotType",
        metavar="kinova_robotType",
        type=str,
        default="j2n6a300",
        help="kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.",
    )
    # args_ = parser.parse_args(argument)
    argv = rospy.myargv()
    args_ = parser.parse_args(argv[1:])
    prefix = args_.kinova_robotType
    nbJoints = int(args_.kinova_robotType[3])
    nbfingers = int(args_.kinova_robotType[5])
    return prefix, nbJoints, nbfingers


def moveFingers(jointcmds, prefix, nbJoints):
    topic_name = "/" + prefix + "/effort_finger_trajectory_controller/command"
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, nbJoints):
        jointCmd.joint_names.append(prefix + "_joint_finger_" + str(i + 1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    # rate = rospy.Rate(100)
    # count = 0
    # while count < 500:
    pub.publish(jointCmd)
    # count = count + 1
    # rate.sleep()


def publish_joint(flex_queue, prefix, nbfingers):
    while True:
        flex = flex_queue.get()

        if flex == "END":
            return

        if flex is not None:
            moveFingers(flex, prefix, nbfingers)


def fingerAngles(image, results, joint_list):
    thing = []

    # Loop through hands
    for hand in results.multi_hand_landmarks:
        # Loop through joint sets
        for joint in joint_list:
            high_joint_x = hand.landmark[joint[0]].x
            mid_joint_x = hand.landmark[joint[1]].x
            low_joint_x = hand.landmark[joint[2]].x

            high_joint_y = hand.landmark[joint[0]].y
            mid_joint_y = hand.landmark[joint[1]].y
            low_joint_y = hand.landmark[joint[2]].y

            high_joint_z = hand.landmark[joint[0]].z
            mid_joint_z = hand.landmark[joint[1]].z
            low_joint_z = hand.landmark[joint[2]].z

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

            dot = (
                high_mid_vec_normalised[0] * low_mid_vec_normalised[0]
                + high_mid_vec_normalised[1] * low_mid_vec_normalised[1]
                + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]
            )

            angle = np.pi - np.arccos(dot)

            b = np.array([hand.landmark[joint[1]].x, hand.landmark[joint[1]].y])
            cv2.putText(
                image,
                str(round(angle, 2)),
                tuple(np.multiply(b, [cap_width, cap_height]).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            thing.append(angle)

    return thing, image


def run():
    rospy.init_node("move_robot_using_trajectory_msg")
    prefix, nbJoints, nbfingers = argumentParser(None)

    rospy.wait_for_service("/gazebo/unpause_physics")
    unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    resp = unpause_gazebo()

    time1 = time.time()

    flex_queue = Queue()

    t1 = threading.Thread(target=publish_joint, args=[flex_queue, prefix, nbfingers])
    t1.start()

    # Set the detection confidence and tracking confidence for better result
    with mp_hands.Hands(
        min_detection_confidence=0.5, min_tracking_confidence=0.5
    ) as hands:
        while cap.isOpened():
            ret, frame = cap.read()

            # BGR 2 RGB
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Flip on horizontal
            image = cv2.flip(image, 1)

            # Set flag
            image.flags.writeable = False

            # Detections
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
                angle_list, image = fingerAngles(image, results, joint_list)

                time2 = time.time()
                if time2 - time1 > 1:
                    time1 = time2

                    if angle_list[4] + angle_list[0] > 2.7:
                        angle_list[4], angle_list[0] = 1.35, 1.35

                    flex = [angle_list[4], angle_list[0], angle_list[1]]

                    flex_queue.put(flex)

            # Showing the camera
            cv2.imshow("Finger Angles", image)
            # exxit the program
            if cv2.waitKey(10) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    flex_queue.put("END")
    t1.join()


if __name__ == "__main__":
    run()
