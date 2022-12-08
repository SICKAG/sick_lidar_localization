#!/bin/bash
pushd ../../../..
source /opt/ros/noetic/setup.bash

#
# cleanup
#

if [ -d ./src/sick_lidar_localization2_pretest ] ; then 
  cp -f ./src/sick_lidar_localization2_pretest/package_ros1.xml ./src/sick_lidar_localization2_pretest/package.xml
else
  cp -f ./src/sick_lidar_localization/package_ros1.xml ./src/sick_lidar_localization/package.xml
fi
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
catkin_make clean
popd 

#
# make
#

./make_ros1.bash

