
if __name__ == "__main__":
    
    ros_service_names = [ "LocAutoStartSavePoseSrv", "LocClearMapCacheSrv", "LocGetErrorLevelSrv", "LocGetMapSrv", "LocGetSystemStateSrv", "LocInitializeAtPoseSrv", 
                      "LocInitializePoseSrv", "LocIsSystemReadySrv", "LocLoadMapToCacheSrv", "LocRequestTimestampSrv", "LocResumeAtPoseSrv", 
                      "LocSaveRingBufferRecordingSrv", "LocSetKinematicVehicleModelActiveSrv", "LocSetLinesForSupportActiveSrv", "LocSetMappingActiveSrv",
                      "LocSetMapSrv", "LocSetOdometryActiveSrv", "LocSetRecordingActiveSrv", "LocSetRingBufferRecordingActiveSrv", "LocStartLocalizingSrv",
                      "LocStateSrv", "LocStopSrv", "LocSwitchMapSrv" ]
   
    rest_service_names = [ "LocAutoStartSavePose", "LocClearMapCache", "GetErrorLevel", "LocGetMap", "LocGetSystemState", "LocInitializeAtPose", 
                      "IsSystemReady", "LocLoadMapToCache", "LocRequestTimestamp", "LocResumeAtPose", "LocSaveRingBufferRecording", 
                      "LocSetKinematicVehicleModelActive", "LocSetLinesForSupportActive", "LocSetMappingActive",
                      "LocSetMap", "LocSetOdometryActive", "LocSetRecordingActive", "LocSetRingBufferRecordingActive", "LocStartLocalizing",
                      "LocStop", "LocSwitchMap" ]
    
    if False: # print ros service callback declarations
        for srv_name in ros_service_names:
            print("        bool serviceCb{}ROS1(sick_lidar_localization::{}::Request &service_request, sick_lidar_localization::{}::Response &service_response);".format(srv_name, srv_name, srv_name))
            print("        bool serviceCb{}ROS2(std::shared_ptr<sick_lidar_localization::{}::Request> service_request, std::shared_ptr<sick_lidar_localization::{}::Response> service_response);".format(srv_name, srv_name, srv_name))
            print("        rosServiceServer<sick_lidar_localization::{}> m_srv_server_{};".format(srv_name, srv_name))
            print("        ")

    if False: # print ros service callback declarations
        print("#if __ROS_VERSION == 1")
        for srv_name in ros_service_names:
            print("#define serviceCb{} sick_lidar_localization::SickServices::serviceCb{}ROS1".format(srv_name, srv_name))
        print("#elif __ROS_VERSION == 2")
        for srv_name in ros_service_names:
            print("#define serviceCb{} sick_lidar_localization::SickServices::serviceCb{}ROS2".format(srv_name, srv_name))
        print("#endif")

    if False: # print ros service advertisements
        for n, srv_name in enumerate(ros_service_names):
            rest_name = rest_service_names[n]
            print("                auto srv_{} = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::{}, \"{}\", &serviceCb{}, this);".format(srv_name, srv_name, rest_name, srv_name))
            print("                m_srv_server_{} = rosServiceServer<sick_lidar_localization::{}>(srv_{});".format(srv_name, srv_name, srv_name))

    if True: # print ros service callback function (default implementation: send http post request)
        for n, srv_name in enumerate(ros_service_names):
            rest_name = rest_service_names[n]
            print("bool sick_lidar_localization::SickServices::serviceCb{}ROS1(sick_lidar_localization::{}::Request &service_request, sick_lidar_localization::{}::Response &service_response)".format(srv_name, srv_name, srv_name))
            print("{")
            print("    std::string command = \"{}\", method = \"POST\", json_data = \"{{}}\";".format(rest_name))
            print("    std::map<std::string, sick_lidar_localization::JsonValue> response_data = sendJsonRequestGetResponse(command, method, json_data);")
            print("    service_response.success = response_data[\"/data/success\"].toBool();")
            print("    ROS_INFO_STREAM(\"SickServices::serviceCb(\\\"\" << command << \"\\\", \\\"\" << method << \"\\\", \\\"\" << json_data << \"\\\"): \" << std::to_string(service_response.success));")
            print("    return true;")
            print("}")
            print("bool sick_lidar_localization::SickServices::serviceCb{}ROS2(std::shared_ptr<sick_lidar_localization::{}::Request> service_request, std::shared_ptr<sick_lidar_localization::{}::Response> service_response)".format(srv_name, srv_name, srv_name))
            print("{")
            print("    return serviceCb{}ROS1(*service_request, *service_response);".format(srv_name))
            print("}")
            print("")
