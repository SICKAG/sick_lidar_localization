#!/bin/bash

pushd ../../../../
source /opt/ros/melodic/setup.bash
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
killall rosmaster ; sleep 1

echo -e "\n# cleanup.bash: Deleting ros cache and logfiles and catkin folders ./build ./devel ./install"
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
catkin clean --yes --all-profiles --verbose
catkin_make clean --cmake-args -DROS_VERSION=1
popd
rm -rf ./log/*
rm -rf ../ros2_scripts/log/*

