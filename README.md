# LIDAR mapper visualiser

## Build
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ./install/local_setup.bash
```

## Run
```bash
ros2 launch lidar_mapper_visualiser lidar_mapper_rviz.launch.py
ros2 bag play lidar_bag_YY-MM-DD_hh:mm:ss/ --clock
```

## Sources
* https://github.com/Myzhar/ldrobot-lidar-ros2
* https://github.com/mavlink/mavros
* https://github.com/OctoMap/octomap_mapping
* https://github.com/OctoMap/octomap_rviz_plugins
* https://github.com/alvinsunyixiao/vrpn_mocap
