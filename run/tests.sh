#!/bin/bash
set -e

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${ROS_WS}/install/setup.bash

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
/usr/bin/env python3 -m pytest -v --cache-clear --cov=roboclaw_driver2 ${MYDIR}/../test
