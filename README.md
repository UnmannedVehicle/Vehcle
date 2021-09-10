# UnmannedVehicle
## 设置bash
```
export LD_LIBRARY_PATH=~/mavlink/build/lib:~/ouster_client/build/lib:
```
## 依赖
```
sudo apt-get install ros-foxy-ecl-*
sudo apt install ros-foxy-nav2-*
sudo apt install ros-foxy-joint-state-publisher
```
## 打开底盘
运行
```
sudo chmod 777 /dev/ttyUSB*
ros2 launch aimibot minimal.launch.py
```
## 雷达
运行
```
ros2 launch ouster_ros ouster.launch.py
```
## 键盘
运行
```
ros2 run py talker
```
## 地图
运行
```
ros2 launch osm_plan planner_node_launch.py
```
## 导航
运行
```
cd aimi_ws/
source devel/setup.sh
ros2 run ros1_bridge dynamic_bridge
```
运行
```
cd aimi_ws/
source devel/setup.sh
roslaunch aimibot amcl.launch
```

## gazebo
```
ros2 launch navigation navigation2.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
