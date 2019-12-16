#!/bin/bash

# init ros environment
source /opt/ros/melodic/setup.bash
if [ -f ../../../../devel/setup.bash   ] ; then source ../../../../devel/setup.bash   ; fi
if [ -f ../../../../install/setup.bash ] ; then source ../../../../install/setup.bash ; fi

# start edit resource-files
gedit ./run_simu.bash ./run_cola_examples.bash ./send_cola_examples.bash run_demo_simu.bash ./run_error_simu.bash ./run.bash &

# start clion
echo -e "Starting clion...\nNote in case of clion/cmake errors:"
echo -e "  Click 'File' -> 'Reload Cmake Project'"
echo -e "  cmake/clion: Project 'XXX' tried to find library '-lpthread' -> delete 'thread' from find_package(Boost REQUIRED COMPONENTS ...) in CMakeLists.txt"
echo -e "  rm -rf ../../../.idea # removes all clion settings"
echo -e "  rm -f ~/CMakeCache.txt"
echo -e "  'File' -> 'Settings' -> 'CMake' -> 'CMake options' : -DCATKIN_DEVEL_PREFIX=~/TASK013_PA0160_SIM_Localization/catkin_ws/devel"
echo -e "  'File' -> 'Settings' -> 'CMake' -> 'Generation path' : ../cmake-build-debug"
echo -e "Note: Do NOT install Hatchery plugin for launch.file support. It doesn't work but crashes clion. If done by accident:"
echo -e "  'File' -> 'Settings' -> 'Plugins' -> 'Installed' -> Deactivate Hatchery"

pushd ../../../..
~/Public/clion-2018.3.3/bin/clion.sh ./src &
popd

