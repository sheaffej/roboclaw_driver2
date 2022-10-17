#!/usr/bin/env bash
set -e

# This should aready have been done by entrypoint.sh in .bashrc
# but including here for good measure
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo " Installing dependencies"
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo

pushd ${ROS_WS}
rm -Rf install build log
if [[ ! -f ${HOME}/.apt-rosdep-updated ]]; then
    apt update
    rosdep update
    touch ${HOME}/.apt-rosdep-updated
else
    echo "Skipping apt and rosdep update as it was previously run."
fi
rosdep install --from-paths src -y 

echo
echo "~~~~~~~~~~~~~~~~~~~~~~~~"
echo " Building the workspace"
echo "~~~~~~~~~~~~~~~~~~~~~~~~"
echo

# This also assumes that the distro's setup.bash is called
# in entrypoint.sh via .bashrc
bash -c "colcon build --packages-select roboclaw_interfaces roboclaw_driver2"
# bash -c "colcon build --packages-select roboclaw_driver2 --symlink-install"

echo
