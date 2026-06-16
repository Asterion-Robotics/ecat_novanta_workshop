#!/bin/bash
set -e

# setup ros environment
# export ROS_LOCALHOST_ONLY=1
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
else
	ros_setup=$(find /opt/ros -maxdepth 2 -type f -name setup.bash | head -n 1)
	if [ -n "$ros_setup" ]; then
		export ROS_DISTRO=$(basename "$(dirname "$ros_setup")")
		source "$ros_setup"
	else
		echo "ERROR: No ROS setup.bash found under /opt/ros" >&2
		exit 1
	fi
fi
source "/ros2_dev/install/setup.bash"
exec "$@"