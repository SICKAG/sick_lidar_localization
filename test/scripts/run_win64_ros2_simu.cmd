REM 
REM Run sick_lidar_localization simu on ROS-2 Windows
REM 

rem if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
rem if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
call C:\dev\ros2_foxy\local_setup.bat
call c:\opt\ros\foxy\x64\setup.bat
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
set SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization
if exist ./src/sick_lidar_localization2_pretest set SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization2_pretest
@echo SICK_LIDAR_LOCALIZATION_ROOT=%SICK_LIDAR_LOCALIZATION_ROOT%

REM 
REM Start rest server
REM 

python --version
rem Required for python rest server and pcapng player:
rem pip install flask
rem pip install pcapng
rem pip install scapy
rem pip install pypcapfile
rem pip install python-pcapng
start "rest server" python %SICK_LIDAR_LOCALIZATION_ROOT%/test/rest_server/python/sick_rest_server.py
@timeout /t 3

REM 
REM Start sick_lidar_localization on ROS-2 Windows
REM 

start "sick_lidar_localization" ros2 run sick_lidar_localization sick_lidar_localization %SICK_LIDAR_LOCALIZATION_ROOT%/launch/sick_lidar_localization.launch hostname:=localhost verbose:=1
@timeout /t 3

REM 
REM Run services
REM  

ros2 service list
ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}" 
ros2 service call LocGetErrorLevel sick_lidar_localization/srv/LocGetErrorLevelSrv "{}"
ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}"
ros2 service call LocAutoStartSavePose sick_lidar_localization/srv/LocAutoStartSavePoseSrv "{}"
ros2 service call LocClearMapCache sick_lidar_localization/srv/LocClearMapCacheSrv "{}"
ros2 service call LocGetMap sick_lidar_localization/srv/LocGetMapSrv "{}"
ros2 service call LocGetSystemState sick_lidar_localization/srv/LocGetSystemStateSrv "{}"
ros2 service call LocInitializeAtPose sick_lidar_localization/srv/LocInitializeAtPoseSrv "{\"x\":1000,\"y\":1000,\"yaw\":1000,\"searchradius\":1000}"
ros2 service call LocLoadMapToCache sick_lidar_localization/srv/LocLoadMapToCacheSrv "{\"mappath\":\"test.vmap\"}"
ros2 service call LocRequestTimestamp sick_lidar_localization/srv/LocRequestTimestampSrv "{}"
ros2 service call LocResumeAtPose sick_lidar_localization/srv/LocResumeAtPoseSrv "{\"x\":1000,\"y\":1000,\"yaw\":1000}"
ros2 service call LocSaveRingBufferRecording sick_lidar_localization/srv/LocSaveRingBufferRecordingSrv "{\"reason\":\"YYYY-MM-DD_HH-MM-SS pose quality low\"}"
ros2 service call LocSetKinematicVehicleModelActive sick_lidar_localization/srv/LocSetKinematicVehicleModelActiveSrv "{\"active\":1}"
ros2 service call LocSetLinesForSupportActive sick_lidar_localization/srv/LocSetLinesForSupportActiveSrv "{\"active\":1}"
ros2 service call LocSetMap sick_lidar_localization/srv/LocSetMapSrv "{\"mappath\":\"test.vmap\"}"
ros2 service call LocSetMappingActive sick_lidar_localization/srv/LocSetMappingActiveSrv "{\"active\":1}"
ros2 service call LocSetOdometryActive sick_lidar_localization/srv/LocSetOdometryActiveSrv "{\"active\":1}"
ros2 service call LocSetRecordingActive sick_lidar_localization/srv/LocSetRecordingActiveSrv "{\"active\":1}"
ros2 service call LocSetRingBufferRecordingActive sick_lidar_localization/srv/LocSetRingBufferRecordingActiveSrv "{\"active\":1}"
ros2 service call LocStart sick_lidar_localization/srv/LocStartSrv "{}"
ros2 service call LocStop sick_lidar_localization/srv/LocStopSrv "{}"
ros2 service call LocSwitchMap sick_lidar_localization/srv/LocSwitchMapSrv "{\"submapname\":\"test.vmap\"}"
ros2 service call LocGetLocalizationStatus sick_lidar_localization/srv/LocGetLocalizationStatusSrv "{}"
ros2 service call LocGetSoftwareVersion sick_lidar_localization/srv/LocGetSoftwareVersionSrv "{}"
ros2 service call LocLoadPersistentConfig sick_lidar_localization/srv/LocLoadPersistentConfigSrv "{}"
@timeout /t 3

REM 
REM Start lls_transform and visualize TF messages by rviz
REM rviz -> Add by display type TF
REM 

start "lls_transform" /min ros2 run sick_lidar_localization lls_transform %SICK_LIDAR_LOCALIZATION_ROOT%/launch/lls_transform.launch
start "rviz2" ros2 run rviz2 rviz2 -d %SICK_LIDAR_LOCALIZATION_ROOT%/test/config/rviz2_win64_sick_lidar_localization_pointcloud.rviz
start "ros2 topic echo /localizationcontroller/out" cmd /k .\src\sick_lidar_localization2_pretest\test\scripts\simu_echo_localizationcontroller_out_topics.cmd

REM 
REM Start udp sender resp. pcapng player to send UDP output messages
REM 

rem python %SICK_LIDAR_LOCALIZATION_ROOT%/test/rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --max_message_count=300
python %SICK_LIDAR_LOCALIZATION_ROOT%/test/rest_server/python/lls_pcapng_player.py --pcap_filename %SICK_LIDAR_LOCALIZATION_ROOT%/test/data/wireshark/20210803_moving_lidarloc2.pcapng

REM 
REM Start odometry sender
REM 

python %SICK_LIDAR_LOCALIZATION_ROOT%/test/rest_server/python/send_ros2_odom_messages.py
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           "{telegram_count: 1000001, timestamp: 123456789, source_id: 0, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}"
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           "{telegram_count: 1000001, timestamp: 123456789, source_id: 2, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}"
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           "{telegram_count: 1000002, timestamp: 123456780, source_id: 0, x_position: -1234, y_position: -1234, heading: 1234}"
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           "{telegram_count: 1000002, timestamp: 123456780, source_id: 3, x_position: -1234, y_position: -1234, heading: 1234}"
ros2 topic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 "{telegram_count: 1000003, timestamp: 123456781, source_id: 0, encoder_value: 123456789}"
ros2 topic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 "{telegram_count: 1000003, timestamp: 123456781, source_id: 4, encoder_value: 123456789}"
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    "{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: 1234}"
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    "{telegram_count: 1000004, timestamp: 123456782, source_id: 5, code: 1234}"
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    "{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}"
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    "{telegram_count: 1000004, timestamp: 123456782, source_id: 6, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}"
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    "{telegram_count: 1000005, timestamp: 123456783, source_id: 0, num_lanes: 1, lanes: [1234]}"
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    "{telegram_count: 1000005, timestamp: 123456783, source_id: 7, num_lanes: 1, lanes: [1234]}"
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    "{telegram_count: 1000006, timestamp: 123456784, source_id: 0, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}"
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    "{telegram_count: 1000006, timestamp: 123456784, source_id: 8, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}"

popd
@pause
