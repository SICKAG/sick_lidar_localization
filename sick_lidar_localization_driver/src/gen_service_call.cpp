/*
 * @brief gen_service_call implements a generic service caller for sick_lidar_localization
 * Service calls are converted to json, send to the localization device via REST API
 * and the json result is returned.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <string.h>

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/sick_services.h"

// https://stackoverflow.com/questions/4643512/replace-substring-with-another-substring-c
static std::string replaceSubstring(const std::string& input, const std::string& search, const std::string& replace)
{
    std::string subject = input;
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) 
    {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
    return subject;
}

static void printUsage(void)
{
    std::cout << "gen_service_call sends requests to the localization device via http REST API and returns its response." << std::endl;
    std::cout << "Usage: gen_service_call <command> <method> <jsondata> [--lls_device_ip=<ip-address>] [--verbose=<0|1>] [--output=<outputfile>] [--append=<outputfile>] [-d=<0|1|2>" << std::endl;
    std::cout << "       <command>: name of command (service request), f.e. LocIsSystemReady or LocStart" << std::endl;
    std::cout << "       <method>: GET or POST" << std::endl;
    std::cout << "       <jsondata>: parameter as json string, f.e. {}" << std::endl;
    std::cout << "       --lls_device_ip=<ip-address>: ip address of the localization device, default: 192.168.0.1" << std::endl;
    std::cout << "       --verbose=0: silent mode, default: 1 (print response)" << std::endl;
    std::cout << "       --output=<outputfile>: write response to <outputfile>, default: no outputfile" << std::endl;
    std::cout << "       --append=<outputfile>: append response to <outputfile>, default: no outputfile" << std::endl;
    std::cout << "       -d=1 or --dump=1: Print and execute curl command, default: 0" << std::endl;
    std::cout << "       -d=2 or --dump=2: Print curl command, default: 0" << std::endl;
}

int main(int argc, char** argv)
{
    // Default parameter and commandline arguments
    if (argc < 4)
        printUsage();
    if (argc < 2)
        exit(EXIT_FAILURE);
    std::string command, method, jsondata;
    command = argv[1];
    if(argc > 2 && (std::string(argv[2]) == "GET" || std::string(argv[2]) == "POST"))
        method = argv[2];
    if(argc > 2 && method.empty() && argv[2][0] != '-')
        jsondata = argv[2];
    else if(argc > 3 && argv[3][0] != '-')
        jsondata = argv[3];
    std::string lls_device_ip = "192.168.0.1";
    int lls_device_webserver_port = 80;
    int verbose = 1, dump = 0;
    std::string outputfile, outputfile_append;
    for (int n = 1; n < argc; n++)
    {
        if (strncmp(argv[n], "--lls_device_ip=", 11) == 0)
            lls_device_ip = std::string(argv[n] + 11);
        if (strncmp(argv[n], "--verbose=", 10) == 0)
            verbose = std::atoi(argv[n] + 10);
        if (strncmp(argv[n], "--append=", 9) == 0)
            outputfile_append = std::string(argv[n] + 9);
        if (strncmp(argv[n], "--output=", 9) == 0)
            outputfile = std::string(argv[n] + 9);
        if (strncmp(argv[n], "-d=", 3) == 0)
            dump = std::atoi(argv[n] + 3);
        if (strncmp(argv[n], "--dump=", 7) == 0)
            dump = std::atoi(argv[n] + 7);
        if (strncmp(argv[n], "--help", 6) == 0)
        {
            printUsage();
            exit(EXIT_SUCCESS);
        }
    }
    std::cout << "gen_service_call " << command << " " << method << " " << jsondata << " --lls_device_ip=" << lls_device_ip << " --verbose=" << verbose << std::endl;

    // Init curl
    sick_lidar_localization::CurlWrapper sick_curl;
    sick_curl.init(lls_device_ip, lls_device_webserver_port, verbose);

    // Set default json arguments (if not given by commandline arguments)
    std::string def_command, def_method, def_json, input_command = command;
    sick_lidar_localization::SickServices::getDefaultCommand(command, def_command, def_method, def_json);
    if (!def_command.empty())
        command = def_command;
    if (method.empty())
        method = def_method;
    if (method.empty())
        method = "POST";
    if (jsondata.empty())
        jsondata = def_json;
    if (jsondata.empty())
        jsondata = "{}";

    // Convert to data from plain ascii to json if required
    if (!sick_lidar_localization::JsonParser::isJson(jsondata))
    {
        std::cout << "WARNING gen_service_call: argument \"" << jsondata << "\" is not valid json." << std::endl;
        std::cout << "Note: calling gen_service_call with valid json expression is recommended." << std::endl;
        printUsage();
        std::cout << "Trying to convert expression to json..." << std::endl;
        jsondata = sick_lidar_localization::JsonParser::plainAsciiToJson(jsondata);
        std::cout << "Converted json expression: \"" << jsondata << "\"" << std::endl;
    }
    if (!sick_lidar_localization::JsonParser::isJson(jsondata))
    {
        std::cerr << "## ERROR gen_service_call: can't convert \"" << jsondata << "\" to json. Please call gen_service_call with json string:" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "gen_service_call request: command=" << command << " method=" << method << " jsondata=" << jsondata << std::endl;

    // Send http request
    std::string response = sick_curl.send(command, method, jsondata);

    // Save response
    if (!response.empty())
    {
        if(verbose)
          std::cout << "gen_service_call request: " << command << " (" << input_command << "), response: " << response << std::endl;
    }
    else
        std::cerr << "## ERROR gen_service_call " << command << " (" << input_command << ") failed (empty response)." << std::endl;
    FILE* fp_out = 0;
    if (!outputfile.empty())
        fp_out = fopen(outputfile.c_str(), "w");
    if (!outputfile_append.empty())
        fp_out = fopen(outputfile_append.c_str(), "a");
    if (fp_out)
    {
        fprintf(fp_out, "{ \"request\": \"%s\", \"response\": %s },\n", input_command.c_str(), response.empty() ? "{}" : response.c_str());
        fclose(fp_out);
    }

    // Debug and dump option
    // -d=1: Print curl command and execute
    // -d=2: Print curl command
    if (dump > 0)
    {
        // Examples:
        // curl -i http://localhost/api/LocGetSystemState
        // curl -i -H "Content-Type: application/json" -X POST -d {\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}} http://localhost/api/LocInitializeAtPose
        std::stringstream curl_command;
        curl_command << "curl -i ";
        if (method != "GET") // i.e. http POST request
        {
            std::string curl_json = replaceSubstring(jsondata, "\"", "\\\"");
            curl_command << "-H \"Content-Type: application/json\" -X POST -d \"" << curl_json << "\" ";
        }
        curl_command << "http://" << lls_device_ip << ":" << std::to_string(lls_device_webserver_port) << "/api/" << command;
        std::cout << "curl command: " << std::endl << curl_command.str() << std::endl;
        if (dump == 1)
        {
            // Execute curl command
            system(curl_command.str().c_str());
        }
    }

    /* Unittest with REST commands priority 1 and 2:
    sick_curl.init("localhost", server_path, verbose);
    std::string response;
    response = sick_curl.send("GetErrorLevel", "POST", "{}");                  // LocGetErrorLevelSrv.srv // "sMN GetErrorLevel" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "level": 0, "description": "No error" } }
    response = sick_curl.send("IsSystemReady", "POST", "{}");                  // LocIsSystemReadySrv.srv // "sMN GetErrorLevel" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "success": True } }
    response = sick_curl.send("LocAutoStartSavePose", "POST", "{}");           // LocAutoStartSavePoseSrv.srv // "sMN LocAutoStartSavePose" //  json response: { "header": { "status": 0, "message": "Ok" }, "data": { "success": True } }
    response = sick_curl.send("LocClearMapCache", "POST", "{}");               // LocClearMapCacheSrv.srv // "sMN LocClearMapCache" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "success": True } }
    response = sick_curl.send("LocGetMap", "POST", "{}");                      // LocGetMapSrv.srv // "sMN LocGetMap" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "mapPath": "test.vmap" } }
    response = sick_curl.send("LocGetSystemState", "POST", "{}");              // LocGetSystemStateSrv.srv // "sMN LocGetSystemState" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "systemState": "LOCALIZING" } }
    response = sick_curl.send("LocInitializeAtPose", "POST", "{\"data\":{\"x\":1000,\"y\":1000,\"yaw\":1000,\"searchradius\":1000}}");  // LocInitializeAtPoseSrv.srv // "sMN LocInitializeAtPose" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocLoadMapToCache", "POST", "{\"data\":{\"mappath\":\"test.vmap\"}}");                  // LocLoadMapToCacheSrv.srv // "sMN LocLoadMapToCache" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocRequestTimestamp", "POST", "{}");                                                   // LocRequestTimestampSrv.srv // "sMN LocRequestTimestamp" // response: { "header": std_header_ok, "data": { "timestamp": 620842067 } }
    response = sick_curl.send("LocResumeAtPose", "POST", "{\"data\":{\"x\":1000,\"y\":1000,\"yaw\":1000}}");        // LocResumeAtPoseSrv.srv // "sMN LocResumeAtPose" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSaveRingBufferRecording", "POST", "{\"data\":{\"reason\":\"YYYY-MM-DD_HH-MM-SS pose quality low\"}}");  // LocSaveRingBufferRecordingSrv.srv // "sMN LocSaveRingBufferRecording" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetKinematicVehicleModelActive", "POST", "{\"data\":{\"active\":true}}"); // LocSetKinematicVehicleModelActiveSrv.srv // "sMN LocSetKinematicVehicleModelActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetLinesForSupportActive", "POST", "{\"data\":{\"active\":true}}");                 // LocSetLinesForSupportActiveSrv.srv // "sMN LocSetLinesForSupportActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetMap", "POST", "{\"data\":{\"mappath\":\"test.vmap\"}}");                         // LocSetMapSrv.srv // "sMN LocSetMap" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetMappingActive", "POST", "{\"data\":{\"active\":true}}");                         // LocSetMappingActiveSrv.srv // "sMN LocSetMappingActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetOdometryActive", "POST", "{\"data\":{\"active\":true}}");                        // LocSetOdometryActiveSrv.srv // "sMN LocSetOdometryActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetRecordingActive", "POST", "{\"data\":{\"active\":true}}");                       // LocSetRecordingActiveSrv.srv // "sMN LocSetRecordingActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocSetRingBufferRecordingActive", "POST", "{\"data\":{\"active\":true}}");             // LocSetRingBufferRecordingActiveSrv.srv // "sMN LocSetRingBufferRecordingActive" // response: { "header": std_header_ok, "data": { "success": True } }
    response = sick_curl.send("LocStart", "POST", "{}");                                                              // LocStartSrv.srv // "sMN LocStart" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "success": True } }
    response = sick_curl.send("LocStop", "POST", "{}");                                                               // LocStopSrv.srv // "sMN LocStop" // json response: { "header": { "status": 0, "message": "Ok" }, "data": { "success": True } }
    response = sick_curl.send("LocSwitchMap", "POST", "{\"data\":{\"submapname\":\"test.vmap\"}}");                   // LocSwitchMapSrv.srv // "sMN LocSwitchMap" // response: { "header": std_header_ok, "data": { "success": True } }
    */

    return EXIT_SUCCESS;
}
