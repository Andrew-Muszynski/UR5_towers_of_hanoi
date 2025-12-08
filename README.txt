I first enable the virtual environment to ensure that the python and ros2 libraries work. 

To run the code here we first need to launch the robto driver for the UR. The following command is to be run in the first terminal:

ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5   robot_ip:=192.168.0.146   rviz:=true

Then in the second terminal, we run the following command: 
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur5   launch_rviz:=true

These will both launch rviz but the first initializes the communication and the second will allow moveit commands to be sent to the robot.

In the third terminal we can run either Hanoi_3_disks.py or Hanoi_4_disks.py which will start the program. The robot will move to the home position and then it waits for the user to press enter to begin after it is ready (in the home position).
