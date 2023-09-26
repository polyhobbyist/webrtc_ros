#!/usr/bin/env bash

set -e

# Get directory of this file
script_dir=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd $script_dir/..

config=${1:-release}

source /opt/ros/humble/setup.bash

colcon build                                  \
  --event-handlers console_cohesion+          \
  --cmake-args                                \
    -DCMAKE_BUILD_TYPE=$config                \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
