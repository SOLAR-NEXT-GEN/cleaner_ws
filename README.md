# solar cleaner ROS
โปรแกรม ROS สำหรับควบคุมหุ่นยนต์ Solar cleaner

## LIDAR
LIDAR C1 package from https://github.com/Slamtec/sllidar_ros2.git
1. Create udev rules for rplidar
```
sudo chmod 777 /dev/ttyUSB0
```
หรือ
```
cd src/rpldiar_ros/
source scripts/create_udev_rules.sh
```
2. Run Slidar ROS2
```
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```
or 
```
ros2 launch sllidar_ros2 my_c1.launch.py
```
3. Read LIDAR data
```
ros2 run solar lidar_read.py
```
