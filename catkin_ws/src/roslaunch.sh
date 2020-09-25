#!/bin/bash

# Call: ./gazebo_client

DIR="$( cd "$( dirname "$1" )" >/dev/null && pwd )"
echo $DIR
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=~/.gazebo/models:$DIR/sim/models/:$GAZEBO_MODEL_PATH

#export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/:$GAZEBO_PLUGIN_PATH
cd ..
source $(pwd)/devel/setup.bash
cd src
roslaunch sim my_world.launch