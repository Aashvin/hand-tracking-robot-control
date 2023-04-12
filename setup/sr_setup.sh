source /opt/ros/noetic/setup.bash
cd /home/user/projects/shadow_robot/base
catkin_make
source /home/user/projects/shadow_robot/base/devel/setup.bash
chmod -R +x /home/user/projects/shadow_robot/base/src/hand-tracking-robot-control/htrc_control/src/scripts
python3.8 -m pip install mediapipe pandas