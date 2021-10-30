#!/bin/bash
pushd ../../../..
source /opt/ros/melodic/setup.bash
rm -f ./build/catkin_make_install.log

#
# build and install
#

catkin_make install --cmake-args -DROS_VERSION=1 2>&1 | tee -a ./build/catkin_make_install.log
source ./install/setup.bash

#
# print warnings and errors
#

echo -e "\nmake.bash finished.\n"
echo -e "catkin_make warnings:"
cat build/catkin_make_install.log | grep -i "warning:"
echo -e "\ncatkin_make errors:"
cat build/catkin_make_install.log | grep -i "error:"

# Check sick_lidar_localization
if [ ! -f ./install/lib/sick_lidar_localization/sick_lidar_localization ] ; then echo -e "\n## ERROR building sick_lidar_localization\n" ; else echo -e "\nbuild sick_lidar_localization finished successfully.\n" ; fi
if [ ! -f ./install/lib/sick_lidar_localization/gen_service_call        ] ; then echo -e "\n## ERROR installing gen_service_call\n"      ; else echo -e "\ninstall gen_service_call finished successfully.\n"      ; fi
echo -e "\ninstall/lib/sick_lidar_localization:"
ls -al ./install/lib/sick_lidar_localization/sick_lidar_localization
ls -al ./install/lib/sick_lidar_localization/gen_service_call

popd

