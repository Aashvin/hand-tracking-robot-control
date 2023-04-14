docker start dexterous_hand_simulated
if docker exec dexterous_hand_simulated test -d /home/user/projects/shadow_robot/base/src/hand-tracking-robot-control; then
    echo "HTRC already exists, reinstalling."
    docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "sudo rm -r /home/user/projects/shadow_robot/base/src/hand-tracking-robot-control"
fi
docker cp ../../hand-tracking-robot-control dexterous_hand_simulated:/home/user/projects/shadow_robot/base/src/hand-tracking-robot-control
docker cp sr_setup.sh dexterous_hand_simulated:/home/user/sr_setup.sh
docker cp ../sr_arm_hand_test/test_launch.launch dexterous_hand_simulated:home/user/projects/shadow_robot/base/src/sr_interface/sr_robot_launch/launch
docker cp ../sr_arm_hand_test/test_world_new.world dexterous_hand_simulated:home/user/projects/shadow_robot/base/src/common_resources/sr_description_common/worlds
docker cp ../sr_arm_hand_test/red_unit_sphere dexterous_hand_simulated:home/user/projects/shadow_robot/base/src/common_resources/sr_description_common/models/red_unit_sphere
docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "chmod +x sr_setup.sh"
docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "./sr_setup.sh"
docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "rm /home/user/sr_setup.sh"
docker exec -ti --user user -w /home/user dexterous_hand_simulated /bin/bash -c "python3.8 -m pip install mediapipe pandas"
docker stop dexterous_hand_simulated
chmod -R +x ../htrc_control/src/scripts
cp kinova_2_finger_models/j2n6s200_control.yaml $(rospack find kinova_control)/config
cp kinova_2_finger_models/j2n7s200_control.yaml $(rospack find kinova_control)/config