#!/bin/bash

# 
# Start Visual Studio Code
# 

echo -e "\nvs_code.bash: Starting visual studio code."  
echo -e "set BUILDTYPE=Debug in make.bash for debugging and development."  
echo -e "set BUILDTYPE=Release in make.bash for profiling and benchmarks.\n"  
gedit ./make.bash ./run_demo_simu.bash ./run_simu.bash ./run_error_simu.bash ./run_cola_examples.bash ./run_odom_simu.bash &

pushd ../../../..
source /opt/ros/eloquent/setup.bash
source ./install/setup.bash
code ./lidar_localization_vscode.code-workspace 
popd
