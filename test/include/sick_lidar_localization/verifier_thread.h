/*
 * @brief sim_loc_verifier_thread implements a thread to verify sim_loc_driver messages
 * against testcases published by sim_loc_test_server.
 *
 * To verify sim_loc_driver, sim_loc_driver runs against sim_loc_test_server.
 * sim_loc_test_server generates and publishes testcases with deterministic
 * and randomly generated result port telegrams. sim_loc_driver receives
 * result port telegrams from the server via TCP, decodes the telegrams
 * and publishes SickLocResultPortTelegramMsg messages.
 *
 * sim_loc_verifier_thread subscribes to both sim_loc_driver messages and
 * sim_loc_test_server messages and compares their content. A warning will
 * be logged in case of failures or mismatches.
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
#ifndef __SIM_LOC_VERIFIER_THREAD_H_INCLUDED
#define __SIM_LOC_VERIFIER_THREAD_H_INCLUDED

#include <list>
#include <string>

#include "sick_lidar_localization/fifo_buffer.h"
#include "sick_lidar_localization/testcase_generator.h"

namespace sick_lidar_localization
{
  /*!
   * class VerifierThread implements a thread to verify sim_loc_driver messages
   * against testcases published by sim_loc_test_server. It subscribes to both
   * sim_loc_driver messages and sim_loc_test_server messages and compares their
   * content. A warning will be logged in case of failures or mismatches.
   */
  class VerifierThread
  {
  public:
    
    /*!
     * Constructor
     * @param[in] nh ros node handle
     */
    VerifierThread(ROS::NodePtr nh = 0);
    
    /*!
     * Destructor
     */
    virtual ~VerifierThread();
    
    /*!
     * Starts the verification thread, matches sim_loc_driver and sim_loc_test_server messages.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
    
    /*!
     * Stops the verification thread.
     * @return true on success, false on failure.
     */
    virtual bool stop(void);
  
    /*!
     * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     */
    virtual void messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg);
    /*! ROS2 version of function messageCbResultPortTelegrams */
    virtual void messageCbResultPortTelegramsROS2(const std::shared_ptr<sick_lidar_localization::SickLocResultPortTelegramMsg> msg) { messageCbResultPortTelegrams(*msg); }
  
    /*!
     * Callback for testcase messages (SickLocResultPortTestcaseMsg) from sim_loc_test_server.
     * @param[in] msg testcase message (SickLocResultPortTestcaseMsg)
     */
    virtual void messageCbResultPortTestcases(const sick_lidar_localization::SickLocResultPortTestcaseMsg & msg);
    /*! ROS2 version of function messageCbResultPortTelegrams */
    virtual void messageCbResultPortTestcasesROS2(const std::shared_ptr<sick_lidar_localization::SickLocResultPortTestcaseMsg> msg) { messageCbResultPortTestcases(*msg); }
    
  protected:
  
    /*!
     * Thread callback, verifies sim_loc_driver messages in m_result_port_telegram_fifo
     * against sim_loc_test_server messages in m_result_port_testcase_fifo.
     */
    virtual void runVerificationThreadCb(void);

    /*
     * member data
     */

    sick_lidar_localization::FifoBuffer<sick_lidar_localization::SickLocResultPortTelegramMsg, boost::mutex> m_result_port_telegram_fifo; ///< fifo buffer for result port telegrams from sim_loc_driver
    sick_lidar_localization::FifoBuffer<sick_lidar_localization::SickLocResultPortTestcaseMsg, boost::mutex> m_result_port_testcase_fifo; ///< fifo buffer for testcase messages from sim_loc_test_server
    bool m_verification_thread_running;                    ///< true: m_verification_thread is running, otherwise false
    boost::thread* m_verification_thread;                  ///< thread to verify sim_loc_driver
    double m_result_telegram_rate;                         ///< frequency or result port telegrams
    
  };
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_VERIFIER_THREAD_H_INCLUDED
