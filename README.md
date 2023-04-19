# Hand-Tracking-Robot-Control
COMP0138 Final Year Project - Visual Hand Tracking for Teleoperation of Kinematically Unmatched Robotic Systems


## Dependencies
- Ubuntu 20.04 with ROS Noetic and Gazebo
- Docker
- Python 3.8
- Numpy
- MediaPipe Solutions Python package
- cv2 Python package
- Shadow Robot Dexterous Hand (please ensure launch_hand=false)
bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true container_name=dexterous_hand_simulated tag=noetic-release launch_hand=false nvidia_docker=false reinstall=true
- Kinova Robots to your catkin workspace

### If running the evaluation notebooks:
- Pandas
- Seaborn

## Setup
Start Docker
`docker TODO CHECK COMMAND`

Clone this repository to your catkin workspace:
`cd <path-to-catkin_ws>/src`
`git clone TODO THIS REPO`

Make and source your catkin workspace:
`cd <path-to-catkin_ws>`
`catkin_make`
`source devel/setup.bash`

Run the HTRC setup Python script:
`cd <path-to-catkin_ws>/src/hand-tracking-robot-control/setup`
`python3 htrc_setup.py`

## Running the Program

### Shadow Robots

Start Docker
`docker TODO CHECK COMMAND`

Start the Shadow Robot Docker container:
`docker start dexterous_hand_simulated`

The rest should be executed in the container window.

#### One Hand

In one tab, start the hand simulation:
`roslaunch sr_robot_launch srhand.launch sim:=true`

After Gazebo and RVIZ have loaded, in another tab run:
`rosrun htrc_control sr_5_finger_hand.py`

#### Two Hands

In one tab, start the two handed simulation:
`roslaunch sr_robot_launch sr_bimanual.launch sim:=true`

After Gazebo and RVIZ have loaded, in another tab run:
`rosrun htrc_control sr_2_hands.py`

#### Hand and Arm

In one tab, start the hand and arm simulation:
`roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true`

After Gazebo and RVIZ have loaded, in another tab run:
`rosrun htrc_control sr_arm_hand.py`

### Kinova Robots

Open two terminals or tabs and source your catkin workspace in both (include both the Kinova Robot packages and HTRC packages):
`source <path-to-catkin_ws>/devel/setup.bash`

The below robot types are supported by HTRC:
- j2n6s300
- j2n7s300
- j2n6s200
- j2n7s200

If the robot type ends in '300', it has 3 fingers. If it ends in '200', it has 2 fingers. The 6/7 refers to the number of joints the arm has. This is not relevant to the control of the hand in this project, but these versions are supported.

In the commands below, replace <robot_type> with one of the above robot types. Keet the robot consistent in both commands below.

In one terminal, start the simulation:
`roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=<robot_type>`

In the second terminal, start HTRC with one of the following commands.

If you've chosen a 3-fingered robot, run:
`rosrun htrc_control kinova_hand_3.py <robot_type>`

If you've chosen a 2-fingered robot, run:
`rosrun htrc_control kinova_hand_2.py <robot_type>`

## Ending the Program

Click on the CV2 window that displays the webcam and finger angles. Press ESC to exit the program.

## Running the Test Programs

