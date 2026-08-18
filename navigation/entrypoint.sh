#!/usr/bin/env bash
set -euo pipefail

# ROS-generated setup files probe optional variables and are not nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/house_bot/install/setup.bash
set -u

exec "$@"
