# holistic control

clone to your workspace

From /franka_control/src/qp_controller.cpp, edit the course to your ur5 urdf file

$ catkin_make

$ gedit ~/.bashrc

add ~/{your_wokrspace}/devel/setup.bash

$source ~/.bashrc

to launch

roslaunch ur_gazebo ur5_bringup.launch

rosrun franka_control franka_control_control
