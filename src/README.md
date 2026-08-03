rosrun teleop_twist_keyboard teleop_twist_keyboard.py 键盘
rostopic echo /odom #获取里程计
rostopic pub /voice_recognition std_msgs/String "参观" -1
<!-- roslaunch bobac3_description gazebo.launch -->
<!-- roslaunch bobac3_navigation demo_nav_2d.launch map_file_name:=reicom -->
roslaunch bobac3_navigation demo_nav_2d.launch map_file_name:=reimap
roslaunch relative_move relative_move.launch   开启相对移动服务
roslaunch ar_pose ar_base_sim.launch   开启二次定位服务

rqt_image_view  使用ROS图像可视化工具查看相机

<!-- rosrun voice_pkg voice_recognition.py

rosrun yolo_face face_node.py 

rosrun reicoures_nav nav_goal_node 

rosrun reicoures_relocalization auto_charge_node

rosrun task_scheduler task_scheduler_node -->

roslaunch task_scheduler tesk.launch
