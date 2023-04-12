import os
import time

print("Please don't close the pop-up window.")
time.sleep(3)

os.system("docker start dexterous_hand_simulated")

os.system("docker cp ../../hand-tracking-robot-control dexterous_hand_simulated:/home/user/projects/shadow_robot/base/src/hand-tracking-robot-control")

os.system("docker cp htrc_setup.sh dexterous_hand_simulated:/home/user/htrc_setup.sh")

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x htrc_setup.sh"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "./htrc_setup.sh"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "rm /home/user/htrc_setup.sh"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "python3.8 -m pip install mediapipe pandas"'
)

os.system("docker stop dexterous_hand_simulated")

os.system(
    'chmod -R +x ../htrc_control/src/scripts'
)

os.system(
    'cp kinova_2_finger_models/j2n6s200_control.yaml $(rospack find kinova_control)/config'
)

os.system(
    'cp kinova_2_finger_models/j2n7s200_control.yaml $(rospack find kinova_control)/config'
)

print("Hand Tracking Robot Control setup done!")