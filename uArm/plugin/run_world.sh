#!/bin/bash

cd "$(dirname "$0")"

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build

gazebo ./uarm.world --verbose

