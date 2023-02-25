import mediapipe as mp
import cv2
import rospy
import time
import threading
from typing import List, Optional

from webcam_controller import WebcamController
from hand_controller import HandController
from arm_controller import ArmController


class Controller:
    def __init__(
        self,
        hand_controller: HandController,
        arm_controller: Optional[ArmController],
        webcam_controller: WebcamController,
    ) -> None:
        self.hand_controller = hand_controller
        self.arm_controller = arm_controller

        self.webcam_controller = webcam_controller

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

    def run(self):
        rospy.init_node("robot_control", anonymous=True)

        self.arm_commander.set_pose_reference_frame("ra_base")
        self.arm_commander.move_to_named_target("ra_start")
        self.arm_commander.move_to_joint_value_target_unsafe(
            {"ra_wrist_3_joint": -180}, wait=True, angle_degrees=True
        )
        rospy.sleep(3.0)
        self.hand_commander.move_to_named_target("open")

        desired_pose = self.arm_controller.move_to_start_pose()

        time.sleep(2)

        t1 = threading.Thread(target=self.hand_controller.publish_joint())
        t1.start()

        t2 = threading.Thread(target=self.arm_controller.publish_move())
        t2.start()

        time1 = time.time()

        # Set the detection confidence and tracking confidence for better result
        with self.webcam_controller.mp_hands.Hands(
            min_detection_confidence=0.7, min_tracking_confidence=0.7
        ) as hands:
            while self.webcam_controller.cap.isOpened():
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

                # Rendering results
                if results.multi_hand_landmarks:
                    for num, hand in enumerate(results.multi_hand_landmarks):
                        self.webcam_controllermp_drawing.draw_landmarks(
                            image, hand, self.webcam_controllermp_hands.HAND_CONNECTIONS
                        )

                    # Draw angles to image from joint list
                    (
                        angle_list,
                        image,
                        mid_finger_pos,
                        norm_dist,
                    ) = self.webcam_controller.finger_angles(image, results)

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

                        self.hand_controller.pub_queue.put(flex)

                        desired_pose.pose.position.x = -mid_finger_pos[1] + 1
                        desired_pose.pose.position.y = -mid_finger_pos[0] * 2 + 1
                        desired_pose.pose.position.z = 0.4 - norm_dist
                        desired_pose.pose.position.z = max(
                            0.2, desired_pose.pose.position.z
                        )
                        self.arm_controller.pub_queue.put(desired_pose)

                # Showing the camera
                cv2.imshow("Finger Angles", image)
                # exxit the program
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

        self.webcam_controller.cap.release()
        cv2.destroyAllWindows()
        self.hand_controller.pub_queue.put("END")
        self.arm_controller.pub_queue.put("END")
        t1.join()
        t2.join()
