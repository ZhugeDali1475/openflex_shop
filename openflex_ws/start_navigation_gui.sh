#!/usr/bin/env bash

set -e

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
exec ros2 run openflex_navigation_gui openflex_navigation_gui
