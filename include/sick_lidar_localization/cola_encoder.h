/*
 * @brief cola_encoder encodes service requests to cola telegrams, parses cola responses and
 * converts them to service responses.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_COLA_ENCODER_H_INCLUDED
#define __SIM_LOC_COLA_ENCODER_H_INCLUDED

#include <string>
#include <vector>
#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/cola_parser.h"

namespace sick_lidar_localization
{
  /*!
   * @brief class ColaEncoder encodes service requests to cola telegrams, parses cola responses and
   * converts them to service responses.
   */
  class ColaEncoder
  {
  public:

    /*!
     * Converts the service request for service SickDevSetLidarConfigSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevSetLidarConfigSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevSetLidarConfigSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevSetLidarConfigSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevSetLidarConfigSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevSetLidarConfigSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickDevGetLidarConfigSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevGetLidarConfigSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevGetLidarConfigSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarConfigSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevGetLidarConfigSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevGetLidarConfigSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetMapSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetMapSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetMapSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetMapSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetMapSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetMapSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocMapSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocMapSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocMapSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocMapSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocMapSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocMapSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocMapStateSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocMapStateSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocMapStateSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocMapStateSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocMapStateSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocMapStateSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocInitializePoseSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocInitializePoseSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocInitializePoseSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocInitializePoseSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocInitializePoseSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocInitializePoseSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocInitialPoseSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocInitialPoseSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocInitialPoseSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocInitialPoseSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocInitialPoseSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocInitialPoseSrv::Response& service_response);
    
    
    /*!
     * Converts the service request for service SickLocSetReflectorsForSupportActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetReflectorsForSupportActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetReflectorsForSupportActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetReflectorsForSupportActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocReflectorsForSupportActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocReflectorsForSupportActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocReflectorsForSupportActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocReflectorsForSupportActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetOdometryActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetOdometryActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetOdometryActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetOdometryActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetOdometryActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocOdometryActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocOdometryActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocOdometryActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocOdometryActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocOdometryActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocOdometryActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetOdometryPortSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetOdometryPortSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetOdometryPortSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryPortSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetOdometryPortSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetOdometryPortSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocOdometryPortSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocOdometryPortSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocOdometryPortSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocOdometryPortSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocOdometryPortSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocOdometryPortSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetOdometryRestrictYMotionSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetOdometryRestrictYMotionSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryRestrictYMotionSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetOdometryRestrictYMotionSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocOdometryRestrictYMotionSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocOdometryRestrictYMotionSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocOdometryRestrictYMotionSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocOdometryRestrictYMotionSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetAutoStartActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetAutoStartActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetAutoStartActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetAutoStartActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetAutoStartActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetAutoStartActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocAutoStartActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocAutoStartActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocAutoStartActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocAutoStartActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocAutoStartActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetAutoStartSavePoseIntervalSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetAutoStartSavePoseIntervalSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetAutoStartSavePoseIntervalSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetAutoStartSavePoseIntervalSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocAutoStartSavePoseIntervalSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocAutoStartSavePoseIntervalSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartSavePoseIntervalSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocAutoStartSavePoseIntervalSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSetRingBufferRecordingActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSetRingBufferRecordingActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSetRingBufferRecordingActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSetRingBufferRecordingActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocRingBufferRecordingActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocRingBufferRecordingActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocRingBufferRecordingActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocRingBufferRecordingActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickDevGetLidarIdentSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevGetLidarIdentSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevGetLidarIdentSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarIdentSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevGetLidarIdentSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevGetLidarIdentSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickDevGetLidarStateSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevGetLidarStateSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevGetLidarStateSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarStateSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevGetLidarStateSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevGetLidarStateSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickGetSoftwareVersionSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickGetSoftwareVersionSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickGetSoftwareVersionSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickGetSoftwareVersionSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickGetSoftwareVersionSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickGetSoftwareVersionSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocAutoStartSavePoseSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocAutoStartSavePoseSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocAutoStartSavePoseSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartSavePoseSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocAutoStartSavePoseSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocAutoStartSavePoseSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocForceUpdateSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocForceUpdateSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocForceUpdateSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocForceUpdateSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocForceUpdateSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocForceUpdateSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocSaveRingBufferRecordingSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocSaveRingBufferRecordingSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocSaveRingBufferRecordingSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocSaveRingBufferRecordingSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocStartDemoMappingSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocStartDemoMappingSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocStartDemoMappingSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocStartDemoMappingSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocStartDemoMappingSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocStartDemoMappingSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickReportUserMessageSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickReportUserMessageSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickReportUserMessageSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickReportUserMessageSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickReportUserMessageSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickReportUserMessageSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickSavePermanentSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickSavePermanentSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickSavePermanentSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickSavePermanentSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickSavePermanentSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickSavePermanentSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocResultPortSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocResultPortSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocResultPortSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocResultPortSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocResultPortSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocResultPortSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocResultModeSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocResultModeSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocResultModeSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocResultModeSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocResultModeSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocResultModeSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocResultEndiannessSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocResultEndiannessSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocResultEndiannessSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocResultEndiannessSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocResultEndiannessSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocResultEndiannessSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocResultStateSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocResultStateSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocResultStateSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocResultStateSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocResultStateSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocResultStateSrv::Response& service_response);
    
    /*!
     * Converts the service request for service SickLocResultPoseIntervalSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickLocResultPoseIntervalSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickLocResultPoseIntervalSrv::Request& service_request);
    
    /*!
     * Parses a cola response and converts the arguments to a service response for service SickLocResultPoseIntervalSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickLocResultPoseIntervalSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickLocResultPoseIntervalSrv::Response& service_response);

    /*!
     * Converts the service request for service SickDevSetIMUActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevSetIMUActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevSetIMUActiveSrv::Request& service_request);

    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevSetIMUActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevSetIMUActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevSetIMUActiveSrv::Response& service_response);

    /*!
     * Converts the service request for service SickDevIMUActiveSrv into a cola ascii telegram
     * @param[in] service_request ros request for service SickDevIMUActiveSrv
     * @return cola ascii telegram
     */
    static std::string encodeServiceRequest(const sick_lidar_localization::SickDevIMUActiveSrv::Request& service_request);

    /*!
     * Parses a cola response and converts the arguments to a service response for service SickDevIMUActiveSrv
     * @param[in] cola_response cola ascii telegram (response from localization server)
     * @param[out] service_response converted response for service SickDevIMUActiveSrv
     * @return true on succes or false in case of parse errors
     */
    static bool parseServiceResponse(const sick_lidar_localization::SickLocColaTelegramMsg& cola_response, sick_lidar_localization::SickDevIMUActiveSrv::Response& service_response);

  }; // class ColaEncoder

} // namespace sick_lidar_localization
#endif // __SIM_LOC_COLA_ENCODER_H_INCLUDED
