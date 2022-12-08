#!/bin/bash

#
# cleanup
#

printf "\033c"
pushd ../../../..
if [ -d ./src/sick_lidar_localization2_pretest ] ; then 
  cp -f ./src/sick_lidar_localization2_pretest/package_ros2.xml ./src/sick_lidar_localization2_pretest/package.xml
else
  cp -f ./src/sick_lidar_localization/package_ros2.xml ./src/sick_lidar_localization/package.xml
fi 
rm -rf ./log
rm -rf ./build
rm -rf ./install 
popd 

#
# make
#

./make_ros2.bash
