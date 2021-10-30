#!/bin/bash

# 
# Build settings
# 

BUILDLOGFILE=sick_lidar_localization_build.log
ERRORLOGFILE=sick_lidar_localization_build_errors.log
USECORES=4

# 
# Build sick_lidar_localization on Linux
# 

pushd ../..
if [ ! -d ./build ] ; then mkdir -p ./build ; fi

cd ./build
rm -f $BUILDLOGFILE
rm -f $ERRORLOGFILE
cmake -DROS_VERSION=0 -G "Unix Makefiles" .. 2>&1 | tee -a $BUILDLOGFILE
make -j$USECORES                             2>&1 | tee -a $BUILDLOGFILE

# Check build errors and warnings
grep "warning:" $BUILDLOGFILE   2>&1 | tee -a $ERRORLOGFILE
grep "undefined:" $BUILDLOGFILE 2>&1 | tee -a $ERRORLOGFILE
grep "error:" $BUILDLOGFILE     2>&1 | tee -a $ERRORLOGFILE
echo -e "---" >> $ERRORLOGFILE
echo -e "\nbuild warnings and errors:"
cat $ERRORLOGFILE

echo -e "\n" 
if [ ! -f ./gen_service_call        ] ; then echo -e "\n## ERROR building gen_service_call\n"        ; else echo -e "make gen_service_call finished."        ; fi
if [ ! -f ./sick_lidar_localization ] ; then echo -e "\n## ERROR building sick_lidar_localization\n" ; else echo -e "make sick_lidar_localization finished." ; fi
echo -e "\n" 
ls -al ./gen_service_call ./sick_lidar_localization

popd 

