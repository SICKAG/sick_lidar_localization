#!/bin/bash

pushd ../../../../
source /opt/ros/melodic/setup.bash
./src/sick_lidar_localization/test/scripts/killall.bash
killall rosmaster ; sleep 1

echo -e "\n# cleanup.bash: Deleting ros cache and logfiles and catkin folders ./build ./devel ./install"
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
catkin clean --yes --all-profiles --verbose
catkin_make clean
popd

