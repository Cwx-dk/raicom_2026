rosrun teleop_twist_keyboard teleop_twist_keyboard.py 键盘
rostopic echo /odom #获取里程计
<!-- roslaunch bobac3_description gazebo.launch -->
<!-- roslaunch bobac3_navigation demo_nav_2d.launch map_file_name:=reicom -->
roslaunch bobac3_navigation demo_nav_2d.launch map_file_name:=reimap
rosrun reicoures_nav nav_goal_node 
rosrun task_scheduler task_scheduler_node