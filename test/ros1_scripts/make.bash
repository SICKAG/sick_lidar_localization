#!/bin/bash

#
# Check/set/cleanup environment
#

pushd ../../../..
rm -f build/catkin_make_install.log
source /opt/ros/melodic/setup.bash
rm -f ./devel/lib/*sick*                           2> /dev/null
rm -f ./devel/lib/sick_lidar_localization/*sick*   2> /dev/null
rm -f ./install/lib/*sick*                         2> /dev/null
rm -f ./install/lib/sick_lidar_localization/*sick* 2> /dev/null

#
# Build and install sick_lidar_localization binaries.
#

if [ -d ./src/sick_lidar_localization         ] && [ ! -f ./src/sick_lidar_localization/package.xml         ] ; then pushd ./src/sick_lidar_localization         ; ln -s ./package_ros1.xml ./package.xml ; popd ; fi
if [ -d ./src/sick_lidar_localization_pretest ] && [ ! -f ./src/sick_lidar_localization_pretest/package.xml ] ; then pushd ./src/sick_lidar_localization_pretest ; ln -s ./package_ros1.xml ./package.xml ; popd ; fi
catkin_make            --cmake-args -DROS_VERSION=1 2>&1 | tee -a build/catkin_make_install.log
catkin_make install    --cmake-args -DROS_VERSION=1 2>&1 | tee -a build/catkin_make_install.log
catkin_make RunDoxygen --cmake-args -DROS_VERSION=1 2>&1 | tee -a build/catkin_make_install.log
source ./install/setup.bash

# lint
# catkin_lint -W1 ./src/sick_lidar_localization
# if [ -d ./src/sick_lidar_localization ]         ; then catkin_lint -W1 ./src/sick_lidar_localization         ; fi
# if [ -d ./src/sick_lidar_localization_pretest ] ; then catkin_lint -W1 ./src/sick_lidar_localization_pretest ; fi

# print warnings and errors
echo -e "\nmake.bash finished.\n"
echo -e "catkin_make warnings:"
cat build/catkin_make_install.log | grep -i "warning:"
echo -e "\ncatkin_make errors:"
cat build/catkin_make_install.log | grep -i "error:"

# print sick_lidar_localization install files, libraries, executables
echo -e "\ninstall/lib/sick_lidar_localization:"
ls -al install/lib/sick_lidar_localization/*
echo -e "\ninstall/share/sick_lidar_localization:"
ls install/share/sick_lidar_localization/*.*
popd

