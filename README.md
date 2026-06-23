# ecat_novanta_workshop

## Installation

### Install Etherlab EtherCAT master

See https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html

### Install this workspace

**Option A:** Install locally

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ws_ros/src/external
cd ws_ros/src

git clone https://github.com/Asterion-Robotics/ecat_novanta_workshop.git

vcs import . < ecat_novanta_workshop/ecat_novanta_workshop.repos
cd ..

rosdep install --from-paths src -i -r -y

colcon build --symlink-install
```

**Option B:** Use docker

TODO

## Bringup

```bash
source install/setup.bash

ros2 launch enwbot_bringup enwbot.launch.py
```
