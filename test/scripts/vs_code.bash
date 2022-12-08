#!/bin/bash

if [ -f /opt/ros/humble/setup.bash ] ; then 
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ] ; then 
    source /opt/ros/noetic/setup.bash
fi
pushd ../../../..
if [ -f ./install/setup.bash ] ; then source ./install/setup.bash ; fi
code ./sick_lidar_localization_vscode.code-workspace
popd
 
