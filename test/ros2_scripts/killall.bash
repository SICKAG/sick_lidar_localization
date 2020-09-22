#!/bin/bash

killall -s SIGINT sim_loc_driver sim_loc_driver_check pointcloud_converter verify_sim_loc_driver ; sleep 3
killall -s SIGINT sim_loc_test_server ; sleep 3
killall    sim_loc_driver sim_loc_driver_check pointcloud_converter verify_sim_loc_driver ; sleep 1
killall    sim_loc_test_server ; sleep 1
killall -9 sim_loc_test_server ; sleep 1
killall -9 sim_loc_driver sim_loc_driver_check pointcloud_converter verify_sim_loc_driver ; sleep 1

