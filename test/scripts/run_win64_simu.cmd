REM 
REM Run sick_lidar_localization simu on native Windows 64
REM 

REM 
REM Start rest server and run unittest for services
REM 

call .\run_win64_unittest_gen_sick_caller.cmd
@timeout /t 10

REM 
REM Run sick_lidar_localization
REM 

pushd ..\..\build
start "sick_lidar_localization" .\Debug\sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost verbose:=1 udp_sim_output_logfile:=udp_sim_output.log
@timeout /t 3
popd

REM 
REM Start udp sender and run unittest for udp messages
REM 

python --version
python ../rest_server/python/sim_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --udp_output_logfile=..\..\build\udp_sender.log --max_message_count=300

REM 
REM Compare received udp_sim_output.log with udp_sender.log, should be identical
REM 

comp /a/l/m ..\..\build\udp_sim_output.log ..\..\build\udp_sender.log
@if not "%errorlevel%"=="0" ( 
  @echo ## ERROR: files ..\..\build\udp_sim_output.log and ..\..\build\udp_sender.log different, should be identical & @pause 
) else (
  @echo unittest for sick_lidar_localization udp messages passed
)

REM 
REM Start pcapng player sending recorded UDP messages
REM 

python --version
python ../rest_server/python/sim_pcapng_player.py  --pcap_filename ../data/wireshark/20210816_lidarloc2_2.0.0.14R_moving.pcapng

@pause
