#!/bin/bash

#
# Check/set/cleanup environment
#

pushd ../../../..
source /opt/ros/eloquent/setup.bash
export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH
rm -f ./install/sick_lidar_localization/* 2> /dev/null

#
# set build type (Debug or Release) and logfile
#
BUILDTYPE=Debug
# BUILDTYPE=Release
 
#
# Build and install sick_lidar_localization binaries.
#

#if [ ! -d ./install/sick_lidar_localization  ] ; then mkdir -p ./install/sick_lidar_localization ; fi  
if [ -d ./src/sick_lidar_localization         ] && [ ! -f ./src/sick_lidar_localization/package.xml         ] ; then pushd ./src/sick_lidar_localization         ; ln -s ./package_ros2.xml ./package.xml ; popd ; fi
if [ -d ./src/sick_lidar_localization_pretest ] && [ ! -f ./src/sick_lidar_localization_pretest/package.xml ] ; then pushd ./src/sick_lidar_localization_pretest ; ln -s ./package_ros2.xml ./package.xml ; popd ; fi
colcon build --packages-select sick_lidar_localization --event-handlers console_direct+ --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE"
colcon build --packages-select sick_lidar_localization --event-handlers console_direct+ --cmake-target RunDoxygen --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE"
source ./install/setup.bash

# print warnings and errors
echo -e "\nmake.bash finished.\n"
echo -e "colcon build warnings:"
cat ./log/latest_build/*.* ./log/latest_build/sick_lidar_localization/*.* | grep -i "warning:"
echo -e "\ncolcon build errors:"
cat ./log/latest_build/*.* ./log/latest_build/sick_lidar_localization/*.* | grep -i "error:"

# print sick_lidar_localization install files, libraries, executables
echo -e "\ninstall/sick_lidar_localization/lib/sick_lidar_localization:"
ls -al install/sick_lidar_localization/lib/sick_lidar_localization/*
popd

