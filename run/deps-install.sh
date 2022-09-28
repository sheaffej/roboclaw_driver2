#!/usr/bin/env bash

# This should aready have been done by entrypoint.sh in .bashrc
# but including here for good measure
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo " Installing dependencies"
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo

pushd ${ROS_WS}
apt update
rosdep update
rosdep install --from-paths src -y 

# echo
# echo "~~~~~~~~~~~~~~~~~~~~~~~~"
# echo " Building the workspace"
# echo "~~~~~~~~~~~~~~~~~~~~~~~~"
# echo

# # This also assumes that the distro's setup.bash is called
# # in entrypoint.sh via .bashrc
# bash -c "colcon build --symlink-install"

echo
