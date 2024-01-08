# holistic control

clone to your workspace

**USE ur5 controller**
From /ur_control/src/robot_node.cpp, edit the course to your ur5 urdf file
In Cmakelists.txt in ur_control package,
edit "robot_node.cpp" executable.

$ catkin_make

$ gedit ~/.bashrc

add ~/{your_wokrspace}/devel/setup.bash

$source ~/.bashrc

to launch

roslaunch ur_gazebo ur5_bringup.launch

roslaunch ur_control holi_con_node.launch

**USE franka_controller**
From /ur_control/src/panda_control.cpp, edit the course to your panda urdf file
In Cmakelists.txt in ur_control package,
edit "panda_control.cpp" executable.

$ catkin_make

$ gedit ~/.bashrc

add ~/{your_wokrspace}/devel/setup.bash

$source ~/.bashrc

to launch

roslaunch franka_moveit demo_gazebo.launch

roslaunch ur_control holi_con_node.launch
