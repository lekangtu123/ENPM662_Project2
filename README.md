# ENPM662_Project2

1. Locate the workspace to ‘project2_ws’.

2. Build the workspace: colcon build.

3. Source the evironment: source install/setup.bash.

4. project2_pkg contains files to launch the simulations in Gazebo, RViz, both of them and grab the goods: 
ros2 launch project2_pkg gazebo.launch.py,

ros2 launch project2_pkg display.launch.py, 

ros2 launch project2_pkg debug.launch.py,

ros2 launch project2_pkg competition.launch.py.

6. project2_control contains the executable to run the robot using the keyboard:
ros2 run project2_control project2_teleop_control.

8. robot_arm_control package contains the executable to run the robot arm using keyboard,
another executable to control the robot arm using inverse kinematics,
and executable to active the vacuum gripper:

ros2 run robot_arm_control arm_teleop_control,

ros2 run robot_arm_control arm_inv_kine_control,

ros2 run robot_arm_control vacuum_gripper.
