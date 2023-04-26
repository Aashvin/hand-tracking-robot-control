# Hand-Tracking-Robot-Control (HTRC)
COMP0138 Final Year Project - Resilient Teleoperation of Kinematically Unmatched Robotic Systems by Visual Hand Tracking

## Dependencies
- Ubuntu 20.04 with [ROS Noetic Desktop-Full Install](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Docker](https://docs.docker.com/get-docker/)
- Python 3.8 (installed with ROS Noetic)
- [NumPy](https://pypi.org/project/numpy/)
- [MediaPipe Solutions](https://pypi.org/project/mediapipe/) Python package
- [Shadow Robot Dexterous Hand Simulation](https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/latest/user_guide/sim_gazebo.html) (please ensure launch_hand=false)
- Clone [Kinova ROS](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel) to your catkin workspace

### If running the evaluation notebooks:
- [Pandas](https://pypi.org/project/pandas/)
- [Seaborn](https://pypi.org/project/seaborn/)

## Setup
Clone the HTRC repository to your catkin workspace:

```
cd <path-to-catkin_ws>/src
git clone https://github.com/Aashvin/hand-tracking-robot-control
```

Download the [MediaPipe Hands model bundle](https://developers.google.com/mediapipe/solutions/vision/hand_landmarker/index#models) and place it in the `hand-tracking-robot-control/htrc_framework/src/htrc_framework` directory.

Make and source your catkin workspace:

```
cd <path-to-catkin_ws>
catkin_make
source devel/setup.bash
```

Run the HTRC setup Python script:

```
cd <path-to-catkin_ws>/src/hand-tracking-robot-control/setup
python3 htrc_setup.py
```

## Running the Program

### Shadow Robots

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

Open two terminals or tabs and source your catkin workspace in both:

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

## Running the Experiment Programs

### Shadow Robots

Start the Shadow Robot Docker container:

`docker start dexterous_hand_simulated`

The rest should be executed in the container window.

#### Hand Rotation Experiments

In one tab, start the hand simulation:

`roslaunch sr_robot_launch srhand.launch sim:=true`

After Gazebo and RVIZ have loaded, in another tab run:

`rosrun htrc_control sr_test_hand.py <pose> <angle>`

Note that <Pose> is the integer ID of the pose (1-7 already exist), and <angle> is the rotation of the hand in degrees where negative values indicate clockwise and positive values indicate clockwise. Running this with a pose and angle combination that exists already will overwrite the current data file associated with it.

#### Refresh Rate Experiments

In one tab, start the hand and arm test environment simulation:

`roslaunch sr_robot_launch test_launch.launch sim:=true start_home:=true`

After Gazebo and RVIZ have loaded, in another tab run:

`rosrun htrc_control sr_test_arm.py <refresh_rate>`

Note that <refresh_rate> is the desired refresh rate for the experiment in Hz. Running this with a refresh rate that already exists will create another trial file for the desired refresh rate.

Once the robot has moved to its starting pose, press SPACE to start the experiment and ESC to finish the experiment.

### Kinova Robots

#### Hand Rotation Experiments

Open two terminals or tabs and source your catkin workspace in both:

`source <path-to-catkin_ws>/devel/setup.bash`

In one terminal, start the simulation:

`roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=<robot_type>`

In the second terminal, start HTRC with one of the following commands.

If you've chosen a 3-fingered robot, run:

`rosrun htrc_control kinova_test_hand_3.py <robot_type> <pose> <angle>`

If you've chosen a 2-fingered robot, run:

`rosrun htrc_control kinova_test_hand_2.py <robot_type> <pose> <angle>`

Note that <Pose> is the integer ID of the pose (1-7 already exist), and <angle> is the rotation of the hand in degrees where negative values indicate clockwise and positive values indicate clockwise. Running this with a pose and angle combination that exists already will overwrite the current data file associated with it.

## Ending the Experiment Programs

Click on the CV2 window that displays the webcam and finger angles. Press ESC to exit the program.

The relevant data is saved to the `hand-tracking-robot-control/test_data/` directory under the name of the experiment that was performed.

## Running the Analysis Notebooks

Make sure the relevant dependencies are installed. Each notebook can then be run, and the pose for the `n_finger_analysis.ipynb` can be adjusted to show different data.
