/*
 * @brief json_parser wraps json parsing and conversions using jsoncpp.
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
#include <regex>
#ifdef _MSC_VER
#include <json/json.h>
#else
#include <jsoncpp/json/json.h>
#endif

#include "sick_lidar_localization/json_parser.h"
#include "sick_lidar_localization/sick_common.h"


bool sick_lidar_localization::JsonValue::toBool(void) const
{
    bool val = false;
    switch (m_type)
    {
    case sick_lidar_localization::JsonValue::BOOL:
        val = m_b_val;
        break;
    case sick_lidar_localization::JsonValue::INT:
        val = (m_i_val != 0);
        break;
    case sick_lidar_localization::JsonValue::STRING:
        val = (m_s_val.size() > 0 && (m_s_val[0] == '1' || m_s_val[0] == 'T' || m_s_val[0] == 't'));
        break;
    default:
        break;
    }
    return val;
}

int64_t sick_lidar_localization::JsonValue::toInt(void) const
{
    int64_t val = 0;
    switch (m_type)
    {
    case sick_lidar_localization::JsonValue::BOOL:
        val = (m_b_val ? 1 : 0);
        break;
    case sick_lidar_localization::JsonValue::INT:
        val = m_i_val;
        break;
    case sick_lidar_localization::JsonValue::DOUBLE:
        val = (int64_t)m_d_val;
        break;
    case sick_lidar_localization::JsonValue::STRING:
        val = std::strtoll(m_s_val.c_str(), 0, 0);
        break;
    default:
        break;
    }
    return val;
}

double sick_lidar_localization::JsonValue::toDouble(void) const
{
    double val = 0;
    switch (m_type)
    {
    case sick_lidar_localization::JsonValue::BOOL:
        val = (m_b_val ? 1 : 0);
        break;
    case sick_lidar_localization::JsonValue::INT:
        val = (double)m_i_val;
        break;
    case sick_lidar_localization::JsonValue::DOUBLE:
        val = m_d_val;
        break;
    case sick_lidar_localization::JsonValue::STRING:
        val = (double)std::strtold(m_s_val.c_str(), 0);
        break;
    default:
        break;
    }
    return val;
}

std::string sick_lidar_localization::JsonValue::toString(void) const
{
    std::string val("");
    switch (m_type)
    {
    case sick_lidar_localization::JsonValue::BOOL:
        val = (m_b_val ? "true" : "false");
        break;
    case sick_lidar_localization::JsonValue::INT:
        val = std::to_string(m_i_val);
        break;
    case sick_lidar_localization::JsonValue::DOUBLE:
        val = std::to_string(m_d_val);
        break;
    case sick_lidar_localization::JsonValue::STRING:
        val = m_s_val;
        break;
    default:
        break;
    }
    return val;
}

sick_lidar_localization::JsonValue::Type sick_lidar_localization::JsonValue::type(void) const
{
    return m_type;
}

std::string sick_lidar_localization::JsonValue::typeString(void) const
{
    std::string val("invalid");
    switch (m_type)
    {
    case sick_lidar_localization::JsonValue::BOOL:
        val = "bool";
        break;
    case sick_lidar_localization::JsonValue::INT:
        val = "int";
        break;
    case sick_lidar_localization::JsonValue::DOUBLE:
        val = "double";
        break;
    case sick_lidar_localization::JsonValue::STRING:
        val = "string";
        break;
    default:
        break;
    }
    return val;
}

/*
** @brief unittest for class JsonParser. Checks some predefined json messages against their expected values.
** @return true on success, false if test failed.
*/
bool sick_lidar_localization::JsonParser::unittest(int verbose)
{
    std::vector<bool> test_passed;
    std::map<std::string, sick_lidar_localization::JsonValue> response_data;

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"success\": true }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/success"].toBool() == true);

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"description\": \"No error\", \"level\": 0 }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/description"].toString() == "No error" && response_data["/data/level"].toInt() == 0);

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"locstate\": 2 }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/locstate"].toInt() == 2);

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"mappath\": \"test.vmap\" }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/mappath"].toString() == "test.vmap");

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"systemstate\": \"LOCALIZING\" }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/systemstate"].toString() == "LOCALIZING");

    response_data = sick_lidar_localization::JsonParser::parseRestResponseData("{ \"data\": { \"timestamp\": 620842067 }, \"header\": { \"message\": \"Ok\", \"status\": 0 } } }", verbose);
    test_passed.push_back(response_data["/data/timestamp"].toInt() == 620842067);

    bool okay = true;
    for (int n = 0; n < test_passed.size(); n++)
    {
        if (!test_passed[n])
        {
            okay = false;
            ROS_ERROR_STREAM("## ERROR sick_lidar_localization::JsonParser::unittest failed, unexpected result in " << (n+1) << ". testcase (" << __FILE__ << ":" << __LINE__ << ")");
        }
    }
    if(!okay)
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::JsonParser::unittest failed.");
    else if (verbose)
        ROS_INFO_STREAM("sick_lidar_localization::JsonParser::unittest passed.");
    return okay;
}

/*
** @brief Recursive parsing of all json values.
*/
static void parseJsonRecursive(const std::string& key, Json::Value& json_value, std::map<std::string, sick_lidar_localization::JsonValue>& key_value_pairs)
{
    if (json_value.type() == Json::ValueType::intValue)
    {
        key_value_pairs[key] = sick_lidar_localization::JsonValue((int64_t)json_value.asInt64());
    }
    else if (json_value.type() == Json::ValueType::uintValue)
    {
        key_value_pairs[key] = sick_lidar_localization::JsonValue((int64_t)json_value.asUInt64());
    }
    else if (json_value.type() == Json::ValueType::realValue)
    {
        key_value_pairs[key] = sick_lidar_localization::JsonValue(json_value.asDouble());
    }
    else if (json_value.type() == Json::ValueType::stringValue)
    {
        key_value_pairs[key] = sick_lidar_localization::JsonValue(json_value.asString());
    }
    else if (json_value.type() == Json::ValueType::booleanValue)
    {
        key_value_pairs[key] = sick_lidar_localization::JsonValue(json_value.asBool());
    }
    else if (json_value.type() == Json::ValueType::arrayValue)
    {
        for (Json::Value::ArrayIndex json_idx = 0; json_idx != json_value.size(); json_idx++)
        {
            parseJsonRecursive(key + "/" + std::to_string(json_idx), json_value[json_idx], key_value_pairs);
        }
    }
    else if (json_value.type() == Json::ValueType::objectValue)
    {
        Json::Value::Members members = json_value.getMemberNames();
        for (Json::Value::ArrayIndex json_idx = 0; json_idx != members.size(); json_idx++)
        {
            std::string member_name = members[json_idx];
            parseJsonRecursive(key + "/" + member_name, json_value[member_name], key_value_pairs);
        }
    }
}

/*
** @brief Parses the response data of a http GET or POST request and returns a map of key-value pairs.
** Example: json_values = parseRestResponseData("{'header': {'status': 0, 'message': 'Ok'}, 'data': {'success': True}}")
** returns a map json_values["success"] := "True" resp. toBool(json_values["success"]) == true.
**
** @param json_msg json response from SIM localization device
** @param verbose if verbose>0: print key-value pairs, otherwise silent except for error messages
**
** @return map of key-value pairs
*/
std::map<std::string, sick_lidar_localization::JsonValue> sick_lidar_localization::JsonParser::parseRestResponseData(const std::string& json_msg, int verbose)
{
    std::map<std::string, JsonValue> key_value_pairs;
    Json::Reader json_reader;
    std::istringstream json_istream(json_msg);
    Json::Value json_root;

    json_reader.parse(json_istream, json_root);
    parseJsonRecursive("", json_root, key_value_pairs);

    for (std::map<std::string, JsonValue>::const_iterator iter = key_value_pairs.cbegin(); verbose && iter != key_value_pairs.cend(); iter++)
        ROS_INFO_STREAM("json_map[\"" << iter->first << "\"] : " << iter->second.toString() << " (type " << iter->second.typeString() << ")");

    return key_value_pairs;
}

/*
** @brief Returns true, if jsondata is a valid json expression, otherwise false.
*/
bool sick_lidar_localization::JsonParser::isJson(const std::string& jsondata)
{
    Json::Reader json_reader;
    std::istringstream json_istream(jsondata);
    Json::Value json_root;
    return json_reader.parse(json_istream, json_root);
}

/*
** @brief Converts a plain ascii string to json. Example:
** plainAsciiToJson("{active:1}") returns "{\"active\":1}"
*/
std::string sick_lidar_localization::JsonParser::plainAsciiToJson(const std::string& jsondata)
{
    std::regex word_re("\\w+:"); // regex matching an entire word followed by ':'
    std::regex colon_re(":"); // regex matching a colon ':'
    std::string jsonstr1 = std::regex_replace(jsondata, word_re, "\"$&"); // replace word by "word:
    std::string jsonstr2 = std::regex_replace(jsonstr1, colon_re, "\"$&"); // replace : by ":
    std::stringstream jsonstr3;
    jsonstr3 << "{\"data\": " << jsonstr2 << "}";
    ROS_INFO_STREAM("plainAsciiToJson(" << jsondata << ")=" << jsonstr3.str());
    return jsonstr3.str();
}
