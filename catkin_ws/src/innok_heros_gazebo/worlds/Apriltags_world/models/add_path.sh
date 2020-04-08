#! /bin/bash

## This shell script is used to add path to my apriltag map.

# get the absolute path of the current folder.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export GAZEBO_MODEL_PATH=$DIR:$GAZEBO_MODEL_PATH
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"

