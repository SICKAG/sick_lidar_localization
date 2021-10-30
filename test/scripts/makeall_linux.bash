#!/bin/bash

# 
# Clean and rebuild sick_lidar_localization on Linux
# 

if [ -d ../../build ] ; then rm -rf ../../build ; fi

./make_linux.bash

