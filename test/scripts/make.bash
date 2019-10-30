#!/bin/bash

#
# Build and install sick_lidar_localization binaries.
#

pushd ../../../..
rm -f build/catkin_make_install.log
source /opt/ros/melodic/setup.bash
rm -f ./devel/lib/*sick*                           2> /dev/null
rm -f ./devel/lib/sick_lidar_localization/*sick*         2> /dev/null
rm -f ./devel/lib/sick_lidar_localization/*sick*   2> /dev/null
rm -f ./install/lib/*sick*                         2> /dev/null
rm -f ./install/lib/sick_lidar_localization/*sick*       2> /dev/null
rm -f ./install/lib/sick_lidar_localization/*sick* 2> /dev/null
catkin_make            2>&1 | tee -a build/catkin_make_install.log
catkin_make install    2>&1 | tee -a build/catkin_make_install.log
catkin_make RunDoxygen 2>&1 | tee -a build/catkin_make_install.log
source ./install/setup.bash

# lint, install by running
catkin_lint -W1 src/sick_lidar_localization

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
echo -e "\ninstall/lib/sick_lidar_localization:"
ls -al install/lib/sick_lidar_localization/*
echo -e "\ninstall/share/sick_lidar_localization:"
ls install/share/sick_lidar_localization/*.*
popd

