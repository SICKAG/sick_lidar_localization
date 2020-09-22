#!/bin/bash

pushd ../../../../
source /opt/ros/eloquent/setup.bash
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros2_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros2_scripts/killall.bash ; fi

echo -e "\n# cleanup.bash: Deleting ros cache and logfiles and colcon folders ./build ./devel ./install ./log"
rm -rf ./build ./devel ./install ./log
mkdir  ./build ./devel ./install ./log ./install/sick_lidar_localization
rm -rf ~/.ros/*
popd
rm -rf ./log/*
rm -rf ../ros1_scripts/log/*

