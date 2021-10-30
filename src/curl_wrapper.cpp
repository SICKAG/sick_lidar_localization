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
#include <iostream>
#include <curl/curl.h>
#include "sick_lidar_localization/sick_common.h"

 /*
 ** @brief curl write callback for response data from server. Since we expect json strings, we can just append the response data to the output string.
 */
static size_t curlWriteCallback(void* ptr, size_t size, size_t nmemb, std::string* data)
{
    data->append((char*)ptr, size * nmemb);
    return size * nmemb;
}

/*
** @brief Shortens multiple consecutive spaces to one space
*/
static std::string shortenSpaces(const std::string& src)
{
    std::string dst;
    dst.reserve(src.length());
    bool last_char_is_space = false;
    for (int n = 0; n < src.length(); n++)
    {
        if (!std::isspace(src[n]))
        {
            dst.push_back(src[n]);
            last_char_is_space = false;

        }
        else if (!last_char_is_space)
        {
            dst.push_back(' ');
            last_char_is_space = true;
        }
    }
    return dst;
}

/*
** @brief Removes leading and trailing spaces
*/
static std::string trim(const std::string& src)
{
    int first = 0, last = (int)src.length() - 1;
    while (first <= last && std::isspace(src[first]))
        first++;
    while (last >= first && std::isspace(src[last]))
        last--;
    if (first != 0 || last != (int)src.length() - 1)
        return src.substr(first, last + 1 - first);
    return src;
}

/*
** @brief default constructor
*/
sick_lidar_localization::CurlWrapper::CurlWrapper()
{
}

/*
** @brief default destructor
*/
sick_lidar_localization::CurlWrapper::~CurlWrapper()
{
}

/*
** @brief Initializes the URL of the localization server
**
** @param hostname IP address of the localization server
** @param path path to REST API, always "api" for localization API 2.0
** @param verbose if verbose>0: print request and responses, otherwise silent except for error messages
*/
void sick_lidar_localization::CurlWrapper::init(const std::string& hostname, const std::string& path, int verbose)
{
	m_url = std::string("http://") + hostname + "/" + path + "/";
    m_verbose = verbose;
}

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
std::string sick_lidar_localization::CurlWrapper::send(const std::string& service, const std::string& method, const std::string& json_data)
{
    bool do_post = false;
    if (method == "POST")
        do_post = true;
    else if (method == "GET")
        do_post = false;
    else
        ROS_ERROR_STREAM("## ERROR CurlWrapper::send(\"" << service << "\", \"" << method << "\", \"" << json_data << "\"): invalid method, \"GET\" or \"POST\" expected, treating http GET request...");
    std::string response_json, response_header;
    CURL* p_curl = curl_easy_init();
    if (p_curl)
    {
        // https://curl.se/libcurl/c/libcurl-tutorial.html
        std::string url = m_url + service;
        int curl_ok = curl_easy_setopt(p_curl, CURLOPT_URL, url.c_str());
        curl_ok |= curl_easy_setopt(p_curl, CURLOPT_NOPROGRESS, 1L);
        curl_ok |= curl_easy_setopt(p_curl, CURLOPT_TCP_KEEPALIVE, 1L);
        curl_ok |= curl_easy_setopt(p_curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
        curl_ok |= curl_easy_setopt(p_curl, CURLOPT_WRITEDATA, &response_json);
        curl_ok |= curl_easy_setopt(p_curl, CURLOPT_HEADERDATA, &response_header);
        if (do_post)
        {
            struct curl_slist* curl_hs = NULL;
            curl_hs = curl_slist_append(curl_hs, "Content-Type: application/json");
            curl_easy_setopt(p_curl, CURLOPT_HTTPHEADER, curl_hs);
            curl_ok |= curl_easy_setopt(p_curl, CURLOPT_POST, 1);
            curl_ok |= curl_easy_setopt(p_curl, CURLOPT_POSTFIELDS, json_data.c_str());
        }
        curl_ok |= curl_easy_perform(p_curl);
        if (curl_ok == CURLE_OK && !response_json.empty())
        {
            response_json = trim(shortenSpaces(response_json));
            if (m_verbose > 0)
            {
                ROS_INFO_STREAM("curl send http " << method << " request \"" << service << "\", response: \"" << response_json << "\"");
            }
        }
        else
        {
            ROS_ERROR_STREAM("## ERROR CurlWrapper::send(\"" << service << "\", \"" << method << "\", \"" << json_data << "\"): response_header = \"" << response_header << "\", response_json = \"" << response_json << "\"");
        }
        curl_easy_cleanup(p_curl);
        p_curl = NULL;
    }
    return response_json;
}
