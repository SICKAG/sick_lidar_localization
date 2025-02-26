# C++ API

On native Linux or Windows without ROS, tool `gen_service_call` can be used for the [REST API services](../doc/sick_localization_services.md). UDP stream messages can be processed using the C++ API.

UDP input messages can be send by calling function `sick_lidar_localization::API::sendUDPMessage()`. UDP output messages can be received by registration of a callback function, which processes the message from the localization device. The main function in [sick_lidar_localization_main.cpp](../src/sick_lidar_localization_main.cpp) shows how to send and receive UDP stream messages:

```
#include <limits>
#include <thread>
#include "sick_lidar_localization/sick_lidar_localization.h"

int main(int argc, char** argv)
{
    rosNodePtr node = 0;
    sick_lidar_localization::Config config; // default configuration, overwrite with customized settings

    // Init sick_lidar_localization API
    sick_lidar_localization::API lidar_loc_api;
    if (!lidar_loc_api.init(node, config))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::init() failed, aborting... ");
        exit(EXIT_FAILURE);
    }

    // Register callbacks to receive UDP stream messages. Overwrite the callbacks in 
    // sick_lidar_localization::UDPMessage::InfoListener to handle UDP output messages
    sick_lidar_localization::UDPMessage::InfoListener udp_receiver_info_listener;
    lidar_loc_api.registerListener(&udp_receiver_info_listener);

    // Examples to send UDP messages to the LLS device
    sick_lidar_localization::UDPMessage::OdometryPayload0104 odometry0104;
    sick_lidar_localization::UDPMessage::OdometryPayload0105 odometry0105;
    sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202 encoder_measurement0202;
    sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303 code_measurement0303;
    sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701 code_measurement0701;
    sick_lidar_localization::UDPMessage::LineMeasurementPayload0403 line_measurement0403;
    sick_lidar_localization::UDPMessage::LineMeasurementPayload0404 line_measurement0404;
    odometry0104.telegram_count = 1000001;
    odometry0104.timestamp = 123456789;
    odometry0104.source_id = 0;
    odometry0104.x_velocity = -1234;
    odometry0104.y_velocity = -1234;
    odometry0104.angular_velocity = 1234;
    odometry0105.telegram_count = 1000002;
    odometry0105.timestamp = 123456780;
    odometry0105.source_id = 0;
    odometry0105.x_position = -1234;
    odometry0105.y_position = -1234;
    odometry0105.heading = 1234;
    encoder_measurement0202.telegram_count = 1000003;
    encoder_measurement0202.timestamp = 123456781;
    encoder_measurement0202.source_id = 0;
    encoder_measurement0202.encoder_value = 123456789;
    code_measurement0303.telegram_count = 1000004;
    code_measurement0303.timestamp = 123456782;
    code_measurement0303.source_id = 0;
    code_measurement0303.code = 1234;
    code_measurement0701.telegram_count = 1000004;
    code_measurement0701.timestamp = 123456782;
    code_measurement0701.source_id = 0;
    code_measurement0701.code = "1234";
    code_measurement0701.x_position = -1234;
    code_measurement0701.y_position = -1234;
    code_measurement0701.heading = 1234;
    line_measurement0403.telegram_count = 1000005;
    line_measurement0403.timestamp = 123456783;
    line_measurement0403.source_id = 0;
    line_measurement0403.num_lanes = 1;
    line_measurement0403.lanes.push_back(1234);
    line_measurement0404.telegram_count = 1000006;
    line_measurement0404.timestamp = 123456784;
    line_measurement0404.source_id = 0;
    line_measurement0404.lcp1 = 12;
    line_measurement0404.lcp2 = 34;
    line_measurement0404.lcp3 = 56;
    line_measurement0404.cnt_lpc = 78;
    line_measurement0404.reserved = 0;
    if (!lidar_loc_api.sendUDPMessage(odometry0104, false, false)
        || !lidar_loc_api.sendUDPMessage(odometry0105, false, false)
        || !lidar_loc_api.sendUDPMessage(encoder_measurement0202, false, false)
        || !lidar_loc_api.sendUDPMessage(code_measurement0303, false, false)
        || !lidar_loc_api.sendUDPMessage(code_measurement0701, false, false)
        || !lidar_loc_api.sendUDPMessage(line_measurement0403, false, false)
        || !lidar_loc_api.sendUDPMessage(line_measurement0404, false, false))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: UDPSender::sendUDPPayload() failed");
    }

    // Run some application while receiving UDP stream messages
    // ...
    ROS_INFO_STREAM("Press ENTER to exit...");
    getchar();

    // Close sick_lidar_localization API and exit
    lidar_loc_api.close();
}
```

