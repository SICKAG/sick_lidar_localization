REM 
REM Play and verify received UDP input message from file 20210804_UDP_Port_5009_RecvData.pcapng
REM 

REM
REM Run sim udp receiver
REM

start "lls_udp_receiver" python ../rest_server/python/lls_udp_receiver.py --udp_port=5009

REM
REM Run sim udp sender
REM

python ../rest_server/python/lls_pcapng_player.py --pcap_filename ../data/wireshark/20210804_UDP_Port_5009_RecvData.pcapng --udp_port=5009
@pause
