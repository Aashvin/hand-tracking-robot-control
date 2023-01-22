import mediapipe as mp
import cv2
import numpy as np
#import uuid
import os
#import pandas as pd
import time
import threading
from queue import Queue
import tf
import geometry_msgs

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
mp_hands2 = mp.solutions.hands

cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
print("DONE")
cap_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cap_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

joint_list = [[7,6,5], [11,10,9], [15,14,13], [19,18,17],[2,3,4]]

def publish_joint(flex_queue, hand_commander):
    while True:
        flex = flex_queue.get()
        
        if flex == "END":
            return
            
        if flex is not None:
            hand_commander.move_to_joint_value_target_unsafe(flex, wait=False, angle_degrees=True)


def fingerAngles(image, results, joint_list):

    counter = 1
    thing = {}

    # Loop through hands
    for hand in results.multi_hand_landmarks:
        #Loop through joint sets 
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

            high_mid_vec = [high_joint_x - mid_joint_x, high_joint_y - mid_joint_y, high_joint_z - mid_joint_z]
            low_mid_vec = [low_joint_x - mid_joint_x, low_joint_y - mid_joint_y, low_joint_z - mid_joint_z]

            high_mid_vec_mag = np.sqrt(high_mid_vec[0] ** 2 + high_mid_vec[1] ** 2 + high_mid_vec[2] ** 2)
            high_mid_vec_normalised = [high_mid_vec[0] / high_mid_vec_mag, high_mid_vec[1] / high_mid_vec_mag, high_mid_vec[2] / high_mid_vec_mag]
            
            low_mid_vec_mag = np.sqrt(low_mid_vec[0] ** 2 + low_mid_vec[1] ** 2 + low_mid_vec[2] ** 2)
            low_mid_vec_normalised = [low_mid_vec[0] / low_mid_vec_mag, low_mid_vec[1] / low_mid_vec_mag, low_mid_vec[2] / low_mid_vec_mag]

            dot = high_mid_vec_normalised[0] * low_mid_vec_normalised[0] + high_mid_vec_normalised[1] * low_mid_vec_normalised[1] + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]

            angle = 180 - np.degrees(np.arccos(dot))
                
            b = np.array([hand.landmark[joint[1]].x, hand.landmark[joint[1]].y])
            cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(b, [cap_width, cap_height]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

            
            thing[counter] = angle
            counter += 1

    return thing, image


def run():
    rospy.init_node("right_hand_demo", anonymous=True)
    hand_commander = SrHandCommander(name="right_hand")
    
    arm_commander = SrArmCommander(name="right_arm")
    arm_commander.set_pose_reference_frame("ra_base")
    
    # arm_commander.move_to_pose_target([1, 0, 0.5, np.pi, 0, 0])
    # arm_commander.move_to_named_target("ra_start")
    arm_commander.move_to_joint_value_target_unsafe({'ra_wrist_3_joint': -180}, wait=True, angle_degrees=True)
    rospy.sleep(3.0)
    hand_commander.move_to_named_target("open")
    
    print(arm_commander.get_current_pose("ra_base"))
    
    current_pose = arm_commander.get_current_pose("ra_base")
    curr_pos_x = current_pose.position.x
    curr_pos_y = current_pose.position.y
    curr_pos_z = current_pose.position.z
    curr_or_x = current_pose.orientation.x
    curr_or_y = current_pose.orientation.y
    curr_or_z = current_pose.orientation.z
    curr_or_w = current_pose.orientation.w
    
    euler = tf.transformations.euler_from_quaternion((curr_or_x, curr_or_y, curr_or_z, curr_or_w))
    print(euler)
    
    #goal = [-1, -0.5, curr_pos_z, curr_or_x, curr_or_y, curr_or_z, curr_or_w]
    #arm_commander.move_to_pose_target(goal)
    
    goal = [0, -0.7, curr_pos_z, euler[0], euler[1], 1.5 * np.pi]
    arm_commander.move_to_pose_target(goal)
    
    goal = [0.5, -0.7, curr_pos_z, euler[0], euler[1], 1.5 * np.pi]
    arm_commander.move_to_pose_target(goal)
    
    goal = [-0.5, -0.7, curr_pos_z, euler[0], euler[1], 1.5 * np.pi]
    arm_commander.move_to_pose_target(goal)

    time1 = time.time()
    
    flex_queue = Queue()
    
    t1 = threading.Thread(target=publish_joint, args=[flex_queue, hand_commander])
    t1.start()

    #Set the detection confidence and tracking confidence for better result
    with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
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

                    flex = {
                        'rh_FFJ2': angle_list[1],
                        'rh_MFJ2': angle_list[2],
                        'rh_RFJ2': angle_list[3],
                        'rh_LFJ2': angle_list[4]
                    }

                    
                    flex_queue.put(flex)

            #Showing the camera
            cv2.imshow('Finger Angles', image)
            #exxit the program
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
            
    cap.release()
    cv2.destroyAllWindows()
    flex_queue.put("END")
    t1.join()

if __name__ == "__main__":

    run()