import os

print("Please don't close the pop-up window.")

os.system("docker start dexterous_hand_simulated")

os.system("docker cp sr_setup.sh dexterous_hand_simulated:/home/user/sr_setup.sh")

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x sr_setup.sh"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "./sr_setup.sh"'
)

os.system(
    "docker cp sr_setup/. dexterous_hand_simulated:/home/user/projects/shadow_robot/base/src/robot_control/src/scripts"
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x /home/user/projects/shadow_robot/base/src/robot_control/src/scripts/hand.py"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x /home/user/projects/shadow_robot/base/src/robot_control/src/scripts/arm_hand.py"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x /home/user/projects/shadow_robot/base/src/robot_control/src/scripts/test_hand.py"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "python3.8 -m pip install mediapipe pandas"'
)

os.system(
    'docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "rm /home/user/sr_setup.sh"'
)

os.system("docker stop dexterous_hand_simulated")

print("Shadow Robot setup done!")
