cd /home/user/projects/shadow_robot/base/src
catkin_create_pkg robot_control
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
cd /home/user/projects/shadow_robot/base/src/robot_control
mkdir src && cd src
mkdir scripts && cd scripts
source /home/user/projects/shadow_robot/base/devel/setup.bash

echo "DONE"