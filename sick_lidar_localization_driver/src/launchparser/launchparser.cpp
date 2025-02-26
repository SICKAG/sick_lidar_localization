/*
 * @brief LaunchParser implements launch-file support on targets not running ROS 1.
 *        LaunchParser parses a launch file using TinyXML and sets the given parameter.
 *        Using LaunchParser it's possible to use configuration by launchfiles on
 *        both Windows and Linux, both native, ROS-1 and ROS-2.
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
#include <iomanip>

#include "launchparser/launchparser.h"

#include "tinystr.h"
#include "tinyxml.h"

class paramEntryAscii
{
public:
  paramEntryAscii(std::string _nameVal, std::string _typeVal, std::string _valueVal)
  {
    nameVal = _nameVal;
    typeVal = _typeVal;
    valueVal = _valueVal;
    setCheckStatus(999,"untested");
    minMaxGiven = false;
  };

  void setPointerToXmlNode(TiXmlElement *paramEntryPtr)
  {
    this->nodePtr = paramEntryPtr;
  }

  TiXmlElement * getPointerToXmlNode(void)
  {
    return( this->nodePtr);
  }
  void setValues(std::string _nameVal, std::string _typeVal, std::string _valueVal)
  {
    nameVal = _nameVal;
    typeVal = _typeVal;
    valueVal = _valueVal;
  };


  bool isMinMaxGiven()
  {
    return(minMaxGiven);
  }

  void setMinMaxValues(std::string _valueMinVal, std::string _valueMaxVal)
  {

    valueMinVal = _valueMinVal;
    valueMaxVal = _valueMaxVal;
    minMaxGiven = true;

  };

  std::string getName()
  {
    return(nameVal);
  }

  std::string getType()
  {
    return(typeVal);
  }

  std::string getValue()
  {
    return(valueVal);
  }

  std::string getMinValue()
  {
    return(valueMinVal);
  }

  std::string getMaxValue()
  {
    return(valueMaxVal);
  }

  void setCheckStatus(int errCode, std::string errMsg)
  {
    errorCode = errCode;
    errorMsg = errMsg;
  };

  int getErrorCode()
  {
    return(errorCode);
  }

  std::string getErrorMsg()
  {
    return(errorMsg);
  }

private:
  std::string nameVal;
  std::string typeVal;
  std::string valueVal;
  std::string valueMinVal;
  std::string valueMaxVal;
  bool minMaxGiven;
  int errorCode;
  std::string errorMsg;
  TiXmlElement *nodePtr;
};

std::vector<paramEntryAscii> getParamList(TiXmlNode *paramList)
{
  std::vector<paramEntryAscii> tmpList;


  TiXmlElement *paramEntry = (TiXmlElement *)paramList->FirstChild("param"); // first child
  while (paramEntry)
  {
    std::string nameVal = "";
    std::string typeVal = "";
    std::string valueVal = "";
    std::string minValueVal = "";
    std::string maxValueVal = "";

    bool minValFnd = false;
    bool maxValFnd = false;
    // is this a param-node?
    // if this is valid than process attributes
    const char *paramVal = paramEntry->Value();
    bool searchAttributes = true;
    if (strcmp(paramVal,"param") == 0)
    {
      // expected value
    }
    else
    {
      searchAttributes = false;
    }
    if (paramEntry->Type() == TiXmlNode::TINYXML_ELEMENT)
    {
      // expected value
    }
    else
    {
      searchAttributes = false;
    }
    if (searchAttributes)
    {
      for (TiXmlAttribute* node = paramEntry->FirstAttribute(); ; node = node->Next())
      {
        const char *tag = node->Name();
        const char *val = node->Value();

        if (strcmp(tag, "name") == 0)
        {
          nameVal = val;
        }
        if (strcmp(tag, "type") == 0)
        {
          typeVal = val;
        }
        if (strcmp(tag, "value") == 0)
        {
          valueVal = val;
        }
        if (strcmp(tag, "valueMin") == 0)
        {
          minValFnd = true;
          minValueVal = val;

        }
        if (strcmp(tag, "valueMax") == 0)
        {
          maxValFnd = true;
          maxValueVal = val;
        }
        if (node == paramEntry->LastAttribute())
        {

          break;
        }
      }

      paramEntryAscii tmpEntry(nameVal, typeVal, valueVal);
      if (maxValFnd && minValFnd)
      {
        tmpEntry.setMinMaxValues(minValueVal, maxValueVal);
      }

      tmpEntry.setPointerToXmlNode(paramEntry);
      tmpList.push_back(tmpEntry);
    }
    paramEntry = (TiXmlElement *)paramEntry->NextSibling();  // go to next sibling
  }

  return(tmpList);
}

bool LaunchParser::parseFile(std::string launchFileFullName, std::vector<std::string>& nameVec,
    std::vector<std::string>& typeVec, std::vector<std::string>& valVec)
{
  bool ret = false;
  ROS_INFO_STREAM("Try loading launchfile : " << launchFileFullName);
  TiXmlDocument doc;
  doc.LoadFile(launchFileFullName.c_str());

  if (doc.Error() == true)
  {
    ROS_ERROR_STREAM("## ERROR parsing launch file " << doc.ErrorDesc() << "\nRow: " << doc.ErrorRow() << "\nCol: " << doc.ErrorCol() << "");
    return(ret);
  }
  TiXmlNode *node = doc.FirstChild("launch");
  if (node != NULL)
  {
    std::map<std::string, std::string> default_args;
    TiXmlElement *arg_node = (TiXmlElement *)node->FirstChild("arg");
    while(arg_node)
    {
      if(strcmp(arg_node->Value(), "arg") == 0 && arg_node->Type() == TiXmlNode::TINYXML_ELEMENT)
      {
        // parse default arguments, f.e. <arg name="lls_device_ip" default="192.168.0.1"/>
        const char* p_attr_name = arg_node->Attribute("name");
        const char* p_attr_default = arg_node->Attribute("default");
        if(p_attr_name && p_attr_default)
        {
          std::string attr_name(p_attr_name), attr_default(p_attr_default);
          default_args[attr_name] = attr_default;
          ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): default_args[\"" << attr_name << "\"]=\"" << default_args[attr_name] << "\"");
        }
      }
      arg_node = (TiXmlElement *)arg_node->NextSibling();  // go to next sibling
    }
    // parse all node specific parameters
    node = node->FirstChild("node");
    std::vector<paramEntryAscii> paramOrgList = getParamList(node);

    for (size_t j = 0; j < paramOrgList.size(); j++)
    {
      nameVec.push_back(paramOrgList[j].getName());
      typeVec.push_back(paramOrgList[j].getType());
      valVec.push_back(paramOrgList[j].getValue());
      if(valVec.back().substr(0, 6) == "$(arg ") // overwrite with default argument, f.e. name="lls_device_ip", type="string", value="$(arg lls_device_ip)"
      {
        std::string default_arg_name = valVec.back().substr(6, valVec.back().length() - 1 - 6);
        if (default_args.find(default_arg_name) != default_args.end())
        {
          std::string default_arg_val = default_args[default_arg_name];
          ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): name=\"" << nameVec.back() << "\", type=\""  << typeVec.back() << "\", value=\""  << valVec.back()
            << "\" overwritten by default value \""  << default_arg_val << "\"");
          valVec.back() = default_arg_val;
        }
      }
      ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): name=\"" << nameVec.back() << "\", type=\""  << typeVec.back() << "\", value=\""  << valVec.back() << "\"");
    }

    ret = true;

  }

  return(ret);
};

static std::string formatParameter(const std::vector<std::string> & args, const std::vector<int> & fill_width)
{
    std::stringstream info;
    for (size_t n = 0; n < args.size(); n++)
    {
        int w = (n < fill_width.size()) ? fill_width[n] : 20;
        info << std::setfill(' ') << std::setw(w) << std::right << args[n];
    }
    return(info.str());
}

/*!
\brief Returns true if <value> ends with <ending>, otherwise false
*/
inline bool ends_with(std::string const& value, std::string const& ending)
{
    if (ending.size() > value.size())
    {
        return false;
    }
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

/*!
\brief splitting expressions like <tag>:=<value> into <tag> and <value>
\param [In] tagVal: string expression like <tag>:=<value>
\param [Out] tag: Tag after Parsing
\param [Ozt] val: Value after Parsing
\return Result of matching process (true: matching expression found, false: no match found)
*/
static bool getTagVal(std::string tagVal, std::string& tag, std::string& val)
{
    bool ret = false;
    std::size_t pos;
    pos = tagVal.find(":=");
    tag = "";
    val = "";
    if (pos == std::string::npos)
    {
        ret = false;
    }
    else
    {
        tag = tagVal.substr(0, pos);
        val = tagVal.substr(pos + 2);
        ret = true;
    }
    return (ret);
}

// 
/*!
\brief Converts a parameter from string to <type> and sets the parameter.
\param nhPriv node handle (pointer to ROS node or 0 for native OS)
\param tag parameter name
\param type parameter type ("bool", "int", "float", "double" or "string"
\param val parameter value
*/
static void convTypeSetParam(rosNodePtr nhPriv, const std::string& tag, const std::string& type, const std::string& val)
{
    if (type == "bool" && !val.empty())
    {
        bool value = (bool)(val[0] == '1' || val[0] == 't' || val[0] == 'T');
        rosDeclareParam(nhPriv, tag, value);
        rosSetParam(nhPriv, tag, value);
    }
    else if (type == "int" && !val.empty())
    {
        int value = (int)std::stoi(val);
        rosDeclareParam(nhPriv, tag, value);
        rosSetParam(nhPriv, tag, value);
    }
    else if (type == "float" && !val.empty())
    {
        float value = (float)std::stof(val);
        rosDeclareParam(nhPriv, tag, value);
        rosSetParam(nhPriv, tag, value);
    }
    else if (type == "double" && !val.empty())
    {
        double value = (double)std::stod(val);
        rosDeclareParam(nhPriv, tag, value);
        rosSetParam(nhPriv, tag, value);
    }
    else // parameter type "string"
    {
        rosDeclareParam(nhPriv, tag, val);
        rosSetParam(nhPriv, tag, val);
    }
}

/*!
\brief Parses an optional launchfile and sets all parameters.
       This function is used at startup to enable system independant parameter handling
       for native Linux/Windows, ROS-1 and ROS-2. Parameter are overwritten by optional
       commandline arguments
\param nhPriv node handle (pointer to ROS node or 0 for native OS)
\param argc: Number of commandline arguments
\param argv: commandline arguments
\return true on success, false in case of parse errors
\sa main
*/
bool LaunchParser::parseLaunchfileSetParameter(rosNodePtr nhPriv, int argc, char** argv)
{
    // Parse launch file
    int launchArgcFileIdx = -1;
    std::vector<std::string> tagList, typeList, valList;
    for (int i = 1; i < argc; i++)
    {
        std::string extKey = ".launch";
        std::string s = argv[i];
        if (ends_with(s, extKey))
        {
            launchArgcFileIdx = i;
            LaunchParser launchParser;
            bool ret = launchParser.parseFile(s, tagList, typeList, valList);
            if (ret == false)
            {
                ROS_INFO_STREAM("Cannot parse launch file (check existence and content): >>>" << s << "<<<\n");
                exit(-1);
            }
            for (size_t j = 0; j < tagList.size(); j++)
            {
                // printf("%-30s %-10s %-20s\n", tagList[j].c_str(), typeList[j].c_str(), valList[j].c_str());
                convTypeSetParam(nhPriv, tagList[j], typeList[j], valList[j]); // convert to type <typeList[j]> and set parameter
            }
        }
    }

    // Overwrite parameter values by commandline options
    for (int i = 1; i < argc; i++)
    {
        std::string tag, val, s = argv[i];
        if (s == "--ros-args") // ignore ROS2 system tags
        {
        }
        else if (getTagVal(s, tag, val))
        {
            bool tag_found = false;
            for (size_t j = 0; j < tagList.size() && !tag_found; j++)
            {
                if (tagList[j] == tag)
                {
                    valList[j] = val;
                    convTypeSetParam(nhPriv, tagList[j], typeList[j], valList[j]); // convert to type <typeList[j]> and set parameter
                    tag_found = true;
                }
            }
            if (!tag_found) // assume string parameter
            {
                tagList.push_back(tag);
                typeList.push_back("string");
                valList.push_back(val);
                convTypeSetParam(nhPriv, tag, "string", val);
            }
        }
        else
        {
            if (launchArgcFileIdx != i)
            {
                ROS_ERROR_STREAM("## ERROR parseLaunchfileSetParameter(): Tag-Value setting not valid. Use pattern: <tag>:=<value>  (e.g. lls_device_ip:=192.168.0.4) (parameter \"" << s << "\" ignored, check commandline options)\n");
            }
        }
    }

    // Print all parameter
    assert(tagList.size() == typeList.size() && tagList.size() == valList.size());
    ROS_INFO_STREAM(formatParameter({ "LaunchParser:", "parameter name", "type", "value" }, { 13, 30, 10, 20 }));
    for (size_t i = 0; i < typeList.size() && i < tagList.size() && i < valList.size(); i++)
    {
        ROS_INFO_STREAM(formatParameter({ "LaunchParser:", tagList[i], typeList[i], valList[i] }, { 13, 30, 10, 20 }));
    }
    return true;
}

