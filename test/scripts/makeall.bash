#!/bin/bash
echo -e "makeall.bash: build and install sick_lidar_localization binaries"
source /opt/ros/melodic/setup.bash
./cleanup.bash
./make.bash
echo -e "makeall.bash finished."

