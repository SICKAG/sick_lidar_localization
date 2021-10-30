REM 
REM Unittest for gen_sick_caller on native Windows 64
REM 

REM 
REM Start rest server
REM 

start "rest server" /min .\simu_run_sim_rest_server.cmd
@timeout /t 3

REM 
REM Send REST commands
REM 

pushd ..\..\build
.\Debug\gen_service_call --help
.\Debug\gen_service_call LocGetErrorLevel POST "{}" --hostname=localhost --verbose=1 --dump=1 --output=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocIsSystemReady POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocAutoStartSavePose POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocClearMapCache POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocGetMap POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocGetSystemState POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocInitializeAtPose POST "{\"data\":{\"x\":1000,\"y\":1000,\"yaw\":1000,\"searchRadius\":1000}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocLoadMapToCache POST "{\"data\":{\"mapPath\":\"test.vmap\"}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocRequestTimestamp POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocResumeAtPose POST "{\"data\":{\"x\":1000,\"y\":1000,\"yaw\":1000}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSaveRingBufferRecording POST "{\"data\":{\"reason\":\"YYYY-MM-DD_HH-MM-SS pose quality low\"}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetKinematicVehicleModelActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetLinesForSupportActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetMap POST "{\"data\":{\"mappath\":\"test.vmap\"}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetMappingActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetOdometryActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetRecordingActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSetRingBufferRecordingActive POST "{\"data\":{\"active\":1}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocStartLocalizing POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocStop POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocSwitchMap POST "{\"data\":{\"subMapName\":\"test.vmap\"}}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocGetLocalizationStatus POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocGetSoftwareVersion POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
.\Debug\gen_service_call LocLoadPersistentConfig POST "{}" --hostname=localhost --verbose=1 --dump=1 --append=win64_unittest_gen_sick_caller.log
popd

REM 
REM Compare responses with unittest reference in ..\data\simu_unittests\win64_unittest_gen_sick_caller.txt
REM 

comp /a/l/m ..\..\build\win64_unittest_gen_sick_caller.log ..\data\simu_unittests\win64_unittest_gen_sick_caller.txt
@if not "%errorlevel%"=="0" ( 
  @echo ## ERROR: files ..\..\build\win64_unittest_gen_sick_caller.log and ..\data\simu_unittests\win64_unittest_gen_sick_caller.txt different, should be identical & @pause 
) else (
  @echo unittest for gen_sick_caller passed
)
rem @pause
