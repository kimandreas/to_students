cd /opt/ros/humble/share/turtlebot4_bringup/config
sudo cp oakd_pro.yaml oakd_pro_orig.yaml
sudo cp oakd_pro_new.yaml oakd_pro.yaml
sudo reboot

sudo systemctl status turtlebot4.service 
sudo systemctl restart turtlebot4.service 
ros2 topic list

ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot0 map:=$HOME/Documents/room/room_map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=robot0
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot0


ros2 run day4 create_path --ros-args -r __ns:=/robot0
ros2 run day4 nav_to_poses --ros-args -r __ns:=/robot0
ros2 run day4 nav_through_poses --ros-args -r __ns:=/robot0
ros2 run day4 follow_waypoints --ros-args -r __ns:=/robot0
ros2 run day4 mail_delivery --ros-args -r __ns:=/robot0
ros2 run day4 patrol_loop --ros-args -r __ns:=/robot0
