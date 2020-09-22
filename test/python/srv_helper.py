# utility functions to support the new lidar_loc services
# Usage: python3 srv_helper.py

import os

def createDefaultSrvFile(service_name):
    print("createDefaultSrvFile({})".format(service_name))
    filename = "srv/Sick{}Srv.srv".format(service_name)
    file = open(filename , "w")
    file.write("# Definition of ROS service Sick{} for sick localization\n".format(service_name))
    file.write("# Example call (ROS1):\n")
    file.write("# rosservice call Sick{} \"{{<parameter>}}\"\n".format(service_name))
    file.write("# Example call (ROS2):\n")
    file.write("# ros2 service call Sick{} sick_lidar_localization/srv/Sick{}Srv \"{{<parameter>}}\"\n".format(service_name, service_name))
    file.write("# \n")
    file.write("\n")
    file.write("# \n")
    file.write("# Request (input)\n")
    file.write("# \n")
    file.write("\n")
    file.write("---\n")
    file.write("\n")
    file.write("# \n")
    file.write("# Response (output)\n")
    file.write("# \n")
    file.write("\n")
    file.write("bool success # true: success response received from localization controller, false: service failed (timeout or error status from controller)\n")
    file.close()

def createServerVariableDefinition(file,service_name):
    file.write("    sick_lidar_localization::Sick{}SrvServer m_Sick{}SrvServer; ///< service \"{}\", callback &sick_lidar_localization::ColaServices::serviceCb{}\n".format(service_name,service_name,service_name,service_name))

def createServerVariableDeclarationROS1(file,service_name):
    file.write("     m_Sick{}SrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::Sick{}Srv, \"Sick{}\", &sick_lidar_localization::ColaServices::serviceCb{}, this);\n".format(service_name,service_name,service_name,service_name))

def createServerVariableDeclarationROS2(file,service_name):
    file.write("     m_Sick{}SrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::Sick{}Srv, \"Sick{}\", &sick_lidar_localization::ColaServices::serviceCb{}ROS2, this);\n".format(service_name,service_name,service_name,service_name))

def createSrvCallbackDeclaration(file,service_name):
    file.write("    /*!\n")
    file.write("     * Callback for service \"Sick{}Srv\"\n".format(service_name))
    file.write("     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response\n")
    file.write("     * Uses ros service \"SickLocColaTelegramSrv\"\n")
    file.write("     * @param[in] service_request ros service request to localization controller\n")
    file.write("     * @param[out] service_response service response from localization controller\n")
    file.write("     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).\n")
    file.write("     */\n")
    file.write("    virtual bool serviceCb{}(sick_lidar_localization::Sick{}Srv::Request& service_request, sick_lidar_localization::Sick{}Srv::Response& service_response);\n".format(service_name,service_name,service_name))
    file.write("    /*! ROS2 version of function serviceCb{} */\n".format(service_name))
    file.write("    virtual bool serviceCb{}ROS2(std::shared_ptr<sick_lidar_localization::Sick{}Srv::Request> service_request, std::shared_ptr<sick_lidar_localization::Sick{}Srv::Response> service_response)\n".format(service_name,service_name,service_name))
    file.write("    {\n")
    file.write("      return serviceCb{}(*service_request, *service_response);\n".format(service_name))
    file.write("    }\n")
    file.write("\n")

def createSrvCallbackImplementation(file,service_name):
    file.write("/*!\n")
    file.write(" * Callback for service \"Sick{}Srv\"\n".format(service_name))
    file.write(" * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response\n")
    file.write(" * Uses ros service \"SickLocColaTelegramSrv\"\n")
    file.write(" * @param[in] service_request ros service request to localization controller\n")
    file.write(" * @param[out] service_response service response from localization controller\n")
    file.write(" * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).\n")
    file.write(" */\n")
    file.write("bool sick_lidar_localization::ColaServices::serviceCb{}(sick_lidar_localization::Sick{}Srv::Request& service_request, sick_lidar_localization::Sick{}Srv::Response& service_response)\n".format(service_name,service_name,service_name))
    file.write("{\n")
    file.write("  service_response.success = false;\n")
    file.write("  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);\n")
    file.write("  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);\n")
    file.write("  if (cola_response.command_name == \"{}\" && cola_response.parameter.size() > 0)\n".format(service_name))
    file.write("  {\n")
    file.write("    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)\n")
    file.write("    {\n")
    file.write("      ROS_WARN_STREAM(\"## ERROR ColaServices::sendColaTelegram(\" << cola_ascii << \") failed, invalid response: \" << sick_lidar_localization::Utils::flattenToString(cola_response) << \", ColaConverter::parseServiceResponse() failed.\");\n")
    file.write("      return false;\n")
    file.write("    }\n")
    file.write("    return true;\n")
    file.write("  }\n")
    file.write("  ROS_WARN_STREAM(\"## ERROR ColaServices::sendColaTelegram(\" << cola_ascii << \") failed, invalid response: \" << sick_lidar_localization::Utils::flattenToString(cola_response));\n")
    file.write("  return false;\n")
    file.write("}\n")
    file.write("\n")

def createColaEncoderDeclaration(file,service_name):
    file.write("    /*!\n")
    file.write("     * Converts the service request for service Sick{}Srv into a cola ascii telegram\n".format(service_name))
    file.write("     * @param[in] service_request ros request for service Sick{}Srv\n".format(service_name))
    file.write("     * @return cola ascii telegram\n")
    file.write("     */\n")
    file.write("    static std::string encodeServiceRequest(const sick_lidar_localization::Sick{}Srv::Request& service_request);\n".format(service_name))
    file.write("    \n")
    file.write("    /*!\n")
    file.write("     * Parses a cola response and converts the arguments to a service response for service Sick{}Srv\n".format(service_name))
    file.write("     * @param[in] cola_response cola ascii telegram (response from localization server)\n")
    file.write("     * @param[out] service_response converted response for service Sick{}Srv\n".format(service_name))
    file.write("     * @return true on succes or false in case of parse errors\n")
    file.write("     */\n")
    file.write("    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::Sick{}Srv::Response& service_response);\n".format(service_name))
    file.write("    \n")

def createColaEncoderImplementation(file,service_name):
    file.write("/*!\n")
    file.write(" * Converts the service request for service Sick{}Srv into a cola ascii telegram\n".format(service_name))
    file.write(" * @param[in] service_request ros request for service Sick{}Srv\n".format(service_name))
    file.write(" * @return cola ascii telegram\n")
    file.write(" */\n")
    file.write("std::string sick_lidar_localization::ColaEncoder::encodeServiceRequest(const sick_lidar_localization::Sick{}Srv::Request& service_request)\n".format(service_name))
    file.write("{\n")
    file.write("  std::stringstream cola_ascii;\n")
    file.write("  cola_ascii << \"sMN {}\"; // todo: convert the input parameter of service request Sick{}Srv to cola string\n".format(service_name, service_name))
    file.write("  return cola_ascii.str();\n")
    file.write("}\n")
    file.write("\n")
    file.write("/*!\n")
    file.write(" * Parses a cola response and converts the arguments to a service response for service Sick{}Srv\n".format(service_name))
    file.write(" * @param[in] cola_response cola ascii telegram (response from localization server)\n")
    file.write(" * @param[out] service_response converted response for service Sick{}Srv\n".format(service_name))
    file.write(" * @return true on succes or false in case of parse errors\n")
    file.write(" */\n")
    file.write("bool sick_lidar_localization::ColaEncoder::parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::Sick{}Srv::Response& service_response)\n".format(service_name))
    file.write("{\n")
    file.write("  service_response.success = false;\n")
    file.write("  if(!cola_response.parameter.empty())\n")
    file.write("  {\n")
    file.write("    // todo: convert cola_response to output parameter of service Sick{}Srv\n".format(service_name))
    file.write("    service_response.success = sick_lidar_localization::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);\n")
    file.write("  }\n")
    file.write("  return service_response.success;\n")
    file.write("}\n")
    file.write("\n")

def createColaTestcaseROS1(file,service_name):
    file.write("  call_service Sick{} \"{{}}\" \"success: True\"\n".format(service_name, service_name))

def createColaTestcaseROS2(file,service_name):
    file.write("  call_service Sick{} sick_lidar_localization/srv/Sick{}Srv \"{{}}\" \"success=True\"\n".format(service_name, service_name))

def createTestServerResponse(file,service_name):
    file.write("  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == \"{}\" && cola_request.parameter.size() == 0)\n".format(service_name))
    file.write("  {\n")
    file.write("    // todo: parse cola request for service {}\n".format(service_name))
    file.write("    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});\n")
    file.write("  }\n")
    file.write("  \n")

if not os.path.exists("srv"):
    os.makedirs("srv")

services = [
    "DevSetLidarConfig",
    "DevGetLidarConfig",
    "LocSetMap",
    "LocMap",
    "LocMapState",
    "LocInitializePose",
    "LocInitialPose",
    "LocSetPoseQualityCovWeight",
    "LocPoseQualityCovWeight",
    "LocSetPoseQualityMeanDistWeight",
    "LocPoseQualityMeanDistWeight",
    "LocSetReflectorsForSupportActive",
    "LocReflectorsForSupportActive",
    "LocSetOdometryActive",
    "LocOdometryActive",
    "LocSetOdometryPort",
    "LocOdometryPort",
    "LocSetOdometryRestrictYMotion",
    "LocOdometryRestrictYMotion",
    "LocSetAutoStartActive",
    "LocAutoStartActive",
    "LocSetAutoStartSavePoseInterval",
    "LocAutoStartSavePoseInterval",
    "LocSetRingBufferRecordingActive",
    "LocRingBufferRecordingActive",
    "DevGetLidarIdent",
    "DevGetLidarState",
    "GetSoftwareVersion",
    "LocAutoStartSavePose",
    "LocForceUpdate",
    "LocSaveRingBufferRecording",
    "LocStartDemoMapping",
    "ReportUserMessage",
    "SavePermanent",
    "LocResultPort",
    "LocResultMode",
    "LocResultEndianness",
    "LocResultState",
    "LocResultPoseInterval"
    ]

for service in services:
    createDefaultSrvFile(service)

file = open("srv/cola_services.h" , "w")
for service in services:
    createServerVariableDefinition(file, service)
file.close()

file = open("srv/cola_services.cpp" , "w")
file.write("#if defined __ROS_VERSION && __ROS_VERSION == 1\n")
for service in services:
    createServerVariableDeclarationROS1(file, service)
file.write("#elif defined __ROS_VERSION && __ROS_VERSION == 2\n")
for service in services:
    createServerVariableDeclarationROS2(file, service)
file.write("#endif // __ROS_VERSION\n")
file.close()

file = open("srv/cola_services.h" , "w")
for service in services:
    createSrvCallbackDeclaration(file, service)
file.close()

file = open("srv/cola_services.cpp" , "w")
for service in services:
    createSrvCallbackImplementation(file, service)
file.close()

file = open("srv/cola_encoder.h" , "w")
for service in services:
    createColaEncoderDeclaration(file, service)
file.close()

file = open("srv/cola_encoder.cpp" , "w")
for service in services:
    createColaEncoderImplementation(file, service)
file.close()

file = open("srv/send_cola_advanced_ros1.bash" , "w")
createColaTestcaseROS1(file, "LocIsSystemReady")
createColaTestcaseROS1(file, "LocStartLocalizing")
for service in services:
    createColaTestcaseROS1(file, service)
file.close()

file = open("srv/send_cola_advanced_ros2.bash" , "w")
createColaTestcaseROS2(file, "LocIsSystemReady")
createColaTestcaseROS2(file, "LocStartLocalizing")
for service in services:
    createColaTestcaseROS2(file, service)
file.close()

file = open("srv/testcase_generator.cpp" , "w")
for service in services:
    createTestServerResponse(file, service)
file.close()
