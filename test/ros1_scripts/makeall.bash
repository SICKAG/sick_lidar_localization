#!/bin/bash
printf "\033c"
echo -e "makeall.bash: build and install sick_lidar_localization binaries"
source /opt/ros/melodic/setup.bash
./cleanup.bash
rm -f ../../package.xml
./make.bash
echo -e "makeall.bash finished."

