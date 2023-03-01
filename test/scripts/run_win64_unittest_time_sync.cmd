REM 
REM Unittest for sick_lidar_localization time sync on native Windows 64
REM 

REM 
REM Start rest server for LocTimestampRequests
REM 

start "rest server" /min .\simu_run_lls_rest_server_time_sync.cmd
@timeout /t 3

REM 
REM Run sick_lidar_localization
REM 

pushd ..\..\build
start "sick_lidar_localization" .\Debug\sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost verbose:=1
@timeout /t 3
popd

REM 
REM Wait 7 seconds until Software-PLL is initialized and start udp sender
REM 

@timeout /t 7
python ../rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --udp_output_logfile=..\..\build\udp_sender.log --max_message_count=1000

REM 
REM Check the timestamps calculated from sensor tics by Software-PLL:
REM - Software-PLL initialized after 7 LocTimestampRequests
REM - Calculated timestamps in output messages close to current time
REM 

@pause
