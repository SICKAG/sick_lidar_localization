#!/bin/bash

echo -e "#\n# killall.bash: Stopping all rosnodes...\n# rosnode kill -a ; sleep 1\n#"
rosnode kill -a ; sleep 1 ; killall -9 sim_loc_test_server ; sleep 1

