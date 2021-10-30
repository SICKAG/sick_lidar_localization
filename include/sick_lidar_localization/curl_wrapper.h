/*
 * @brief curl_wrapper wraps libcurl function calls for sick_lidar_localization
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
#ifndef __SICK_LIDAR_LOCALIZATION_CURL_WRAPPER
#define __SICK_LIDAR_LOCALIZATION_CURL_WRAPPER

#include <string>

namespace sick_lidar_localization
{
    /*
    ** @brief class CurlWrapper wraps libcurl function calls for sick_lidar_localization
    */
    class CurlWrapper
    {
    public:

        /*
        ** @brief default constructor
        */
        CurlWrapper();

        /*
        ** @brief default destructor
        */
        ~CurlWrapper();

        /*
        ** @brief Initializes the URL of the localization server
        **
        ** @param hostname IP address of the localization server
        ** @param path path to REST API, always "api" for localization API 2.0
        ** @param verbose if verbose>0: print request and responses, otherwise silent except for error messages
        */
        void init(const std::string& hostname = "192.168.0.1", const std::string& path = "api", int verbose = 1);

        /*
        ** @brief Sends a request to the localization server and returns the response.
        **        Example for a http get request: send("IsSystemReady", "GET", "")
        **        Example for a http post request: send("LocInitializeAtPose", "POST", "{\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}}")
        **
        ** @param service name of the service request, f.e. "IsSystemReady" or "LocInitializeAtPose"
        ** @param method "GET" or "POST"
        ** @param json_data optional json string for POST requests
        **
        ** @return json response from the localization server (empty string in case of errors)
        */
        std::string send(const std::string& service, const std::string& method = "GET", const std::string& json_data = "{}");

    protected:

        std::string m_url; // Server URL, i.e. "http://<hostname>/api
        int m_verbose;

    }; // class CurlWrapper

} // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_CURL_WRAPPER
