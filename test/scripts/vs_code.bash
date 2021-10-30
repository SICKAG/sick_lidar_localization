#!/bin/bash


if [ -f /opt/ros/melodic/setup.bash  ] ; then source /opt/ros/melodic/setup.bash  ; fi
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
pushd ../../../..
if [ -f ./install/setup.bash ] ; then source ./install/setup.bash ; fi
code ./sick_lidar_localization_vscode.code-workspace
popd 
