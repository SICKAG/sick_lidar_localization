#!/bin/bash

# 
# Unittest for gen_sick_caller on native linux
# 

# 
# Start rest server
# 

echo -e "Starting sudo sick_rest_server.py ...\n" 
sudo pkill -f sick_rest_server.py
sudo python3 ../rest_server/python/sick_rest_server.py &
sleep 3

# 
# Send REST commands
# 

pushd ../../build

./gen_service_call --help
./gen_service_call LocGetErrorLevel POST "{}" --hostname=localhost --verbose=1 --dump=1 --output=linux_unittest_gen_sick_caller.log
./gen_service_call LocIsSystemReady POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocAutoStartSavePose POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocClearMapCache POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocGetMap POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocGetSystemState POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocInitializeAtPose POST "{x: 1000, y: 1000, yaw: 1000, searchRadius: 1000}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocLoadMapToCache POST "{mapPath: \"test.vmap\"}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocRequestTimestamp POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocResumeAtPose POST "{x: 1000, y: 1000, yaw: 1000}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSaveRingBufferRecording POST "{reason: \"YYYY-MM-DD_HH-MM-SS pose quality low\"}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetKinematicVehicleModelActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetLinesForSupportActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetMap POST "{mappath: \"test.vmap\"}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetMappingActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetOdometryActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetRecordingActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSetRingBufferRecordingActive POST "{active: 1}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocStartLocalizing POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocStop POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocSwitchMap POST "{subMapName: \"test.vmap\"}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocGetLocalizationStatus POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocGetSoftwareVersion POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log
./gen_service_call LocLoadPersistentConfig POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=linux_unittest_gen_sick_caller.log

popd 
sudo pkill -f sick_rest_server.py

# 
# Compare responses with unittest reference in ../data/simu_unittests/linux_unittest_gen_sick_caller.txt
# 

echo -e "\nunittest for gen_sick_caller finished.\n" 
cat ../../build/linux_unittest_gen_sick_caller.log
diff ../../build/linux_unittest_gen_sick_caller.log ../data/simu_unittests/linux_unittest_gen_sick_caller.txt
if [ $? != 0 ] ; then
  echo -e "\n## ERROR unittest for gen_sick_caller: files ../../build/linux_unittest_gen_sick_caller.log ../data/simu_unittests/linux_unittest_gen_sick_caller.txt different, should be identical\n" 
  echo -e "## ERROR: unittest for gen_sick_caller failed. Press any key to continue..."
  read -n1 -s key 
else
  echo -e "\nunittest for gen_sick_caller passed, no errors.\n" 
fi

