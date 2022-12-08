#!/bin/bash

# 
# Build sick_lidar_localization for ROS2-Linux
# 

pushd ../../../.. 

# set build type (Debug or Release) and logfile
# BUILDTYPE=Debug
BUILDTYPE=Release

if [ -f /opt/ros/humble/setup.bash ] ; then 
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
fi
colcon build --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash

# Check sick_lidar_localization
if [ ! -f ./install/sick_lidar_localization/lib/sick_lidar_localization/sick_lidar_localization ] ; then echo -e "\n## ERROR building sick_lidar_localization\n" ; else echo -e "\nbuild sick_lidar_localization finished successfully.\n" ; fi
if [ ! -f ./install/sick_lidar_localization/lib/sick_lidar_localization/gen_service_call        ] ; then echo -e "\n## ERROR installing gen_service_call\n"      ; else echo -e "\ninstall gen_service_call finished successfully.\n"      ; fi
echo -e "\ninstall/lib/sick_lidar_localization:"
ls -al ./install/sick_lidar_localization/lib/sick_lidar_localization/sick_lidar_localization
ls -al ./install/sick_lidar_localization/lib/sick_lidar_localization/gen_service_call
echo -e "\n"

popd

