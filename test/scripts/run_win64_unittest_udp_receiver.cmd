REM 
REM Run sick_lidar_localization unittest for udp messages on native Windows 64
REM 

REM 
REM Start rest server
REM 

@if exist "c:\Anaconda3\python.exe" set PATH=c:\Anaconda3;%PATH%
@echo PATH=%PATH%
start "rest server" /min python ../rest_server/python/sick_rest_server.py
@timeout /t 3

REM 
REM Start sick_lidar_localization
REM 

pushd ..\..\build
start "sick_lidar_localization" .\Debug\sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost verbose:=1 udp_lls_output_logfile:=udp_lls_output.log
@timeout /t 10
popd

REM 
REM Start udp sender
REM 

python ../rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --udp_output_logfile=..\..\build\udp_sender.log --max_message_count=300

REM 
REM Compare received udp_lls_output.log with udp_sender.log, should be identical
REM 

comp /a/l/m ..\..\build\udp_lls_output.log ..\..\build\udp_sender.log
@if not "%errorlevel%"=="0" ( 
  @echo ## ERROR: files ..\..\build\udp_lls_output.log and ..\..\build\udp_sender.log different, should be identical & @pause 
) else (
  @echo unittest for sick_lidar_localization udp messages passed
)
@pause
