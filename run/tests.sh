#!/bin/bash
set -e

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# echo
# echo "============================"
# echo "   Building the workspace   "
# echo "============================"
# ${MYDIR}/build.sh
# source ${ROS_WS}/install/setup.bash

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
/usr/bin/env python3 -m pytest -v --cache-clear --cov=roboclaw_driver ${MYDIR}/../test/unit/

echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
/usr/bin/env python3 -m pytest -v --cache-clear ${MYDIR}/../test/node/
