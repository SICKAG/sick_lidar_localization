# sick_lidar_localization FAQ

## Setup failed under Linux 

:question: Setup using SOPASair failed under Linux

:white_check_mark: To setup and configure LiDAR-LOC with SOPASair, use of Chrome-browser under Windows is highly recommended.
See [Quickstart-Setup-SOPASair.md](Quickstart-Setup-SOPASair.md) for a quickstart. 
Find detailed information in the operation manuals published on https://supportportal.sick.com/Product_notes/lidar-loc-operation-instruction/. 

## Reconnect after network error

:question: Reconnect required after network changes or errors

In case of network errors, the left status LED on the SIM100FXA is blinking and the ros driver will periodically try
to reconnect to the localization controller (once a second by default). After reestablishing a network connection, 
the left status LED will switch to green light and the driver will automatically reconnect to the controller.
After ca. 15 seconds, the localization controller SIM1000FXA will start to send result port telegrams.
