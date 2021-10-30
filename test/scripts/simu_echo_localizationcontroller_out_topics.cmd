timeout /t 10
start /b ros2 topic echo /localizationcontroller/out/odometry_message_0104
start /b ros2 topic echo /localizationcontroller/out/odometry_message_0105
start /b ros2 topic echo /localizationcontroller/out/code_measurement_message_0304
start /b ros2 topic echo /localizationcontroller/out/line_measurement_message_0403
start /b ros2 topic echo /localizationcontroller/out/line_measurement_message_0404
start /b ros2 topic echo /localizationcontroller/out/localizationcontroller_result_message_0502
