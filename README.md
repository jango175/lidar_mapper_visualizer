# LIDAR mapper visualiser

## Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ./install/local_setup.bash
```

## Run
```bash
ros2 launch lidar_mapper_visualiser lidar_mapper_rviz.launch.py
ros2 bag play lidar_bag_YY-MM-DD_hh:mm:ss/ --clock
```
