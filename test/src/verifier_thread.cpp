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
#include "sick_lidar_localization/ros_wrapper.h"

#include "sick_lidar_localization/utils.h"
#include "sick_lidar_localization/verifier_thread.h"

/*
 * Constructor
 */
sick_lidar_localization::VerifierThread::VerifierThread(ROS::NodePtr nh)
: m_verification_thread_running(false), m_verification_thread(0), m_result_telegram_rate(10)
{
  ROS::param<double>(nh, "/sick_lidar_localization/test_server/result_telegrams_rate", m_result_telegram_rate, m_result_telegram_rate);
}

/*
 * Destructor
 */
sick_lidar_localization::VerifierThread::~VerifierThread()
{
  stop();
}

/*
 * Starts the verification thread, matches sim_loc_driver and sim_loc_test_server messages.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::VerifierThread::start(void)
{
  m_verification_thread_running = true;
  m_verification_thread = new boost::thread(&sick_lidar_localization::VerifierThread::runVerificationThreadCb, this);
  return true;
}

/*
 * Stops the verification thread.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::VerifierThread::stop(void)
{
  m_verification_thread_running = false;
  if(m_verification_thread)
  {
    m_verification_thread->join();
    delete(m_verification_thread);
    m_verification_thread = 0;
  }
  return true;
}

/*
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_lidar_localization::VerifierThread::messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg)
{
  m_result_port_telegram_fifo.push(msg);
}

/*
 * Callback for testcase messages (SickLocResultPortTestcaseMsg) from sim_loc_test_server.
 * @param[in] msg testcase message (SickLocResultPortTestcaseMsg)
 */
void sick_lidar_localization::VerifierThread::messageCbResultPortTestcases(const sick_lidar_localization::SickLocResultPortTestcaseMsg & msg)
{
  m_result_port_testcase_fifo.push(msg);
}

/*!
 * Implementation of interface UnaryConditionIf to search for a message by TelegramCounter
 */
class TelegramCounterCondition : public sick_lidar_localization::FifoBuffer<sick_lidar_localization::SickLocResultPortTelegramMsg>::UnaryConditionIf
{
public:
  /*! Constructor. Parameter telegram_counter is the TelegramCounter to search for, i.e. condition() returns true, if element has a TelegramCounter == telegram_counter */
  TelegramCounterCondition(uint32_t telegram_counter = 0) : m_telegram_counter(telegram_counter) {}
  /*! Search callback: return true, if the search condition for an element is true, or false otherwise */
  virtual bool condition(const sick_lidar_localization::SickLocResultPortTelegramMsg & element) { return element.telegram_header.telegramcounter == m_telegram_counter; }
protected:
  /** member data */
  uint32_t m_telegram_counter; //!< TelegramCounter to search for, i.e. condition() returns true, if element has a TelegramCounter == m_telegram_counter
};

/*
 * Thread callback, verifies sim_loc_driver messages in m_result_port_telegram_fifo
 * against sim_loc_test_server messages in m_result_port_testcase_fifo.
 */
void sick_lidar_localization::VerifierThread::runVerificationThreadCb(void)
{
  ROS_INFO_STREAM("VerifierThread: verification thread for sim_loc_driver messages started");
  size_t total_verification_cnt = 0, total_verification_failed_cnt = 0;
  while(ROS::ok() && m_verification_thread_running)
  {
    while (ROS::ok() && m_verification_thread_running && m_result_port_testcase_fifo.size() < 4) // delay verification by 4 messages (messages may cross each other)
    {
      ROS::sleep(1.0 / m_result_telegram_rate);
    }
    // Match testcase from server against result port telegrams from driver, search by TelegramCounter (unique telegram id)
    sick_lidar_localization::SickLocResultPortTestcaseMsg server_testcase = m_result_port_testcase_fifo.pop();
    uint32_t server_telegram_counter = server_testcase.telegram_msg.telegram_header.telegramcounter;
    TelegramCounterCondition condition(server_telegram_counter);
    sick_lidar_localization::SickLocResultPortTelegramMsg driver_telegram = m_result_port_telegram_fifo.findFirstIf(condition, true);
    // Compare testcase and driver message
    if (server_telegram_counter == driver_telegram.telegram_header.telegramcounter) // Testcase from the server corresponds to the result telegram from the driver
    {
      if (!sick_lidar_localization::Utils::identicalByStream(driver_telegram, server_testcase.telegram_msg))
      {
        ROS_WARN_STREAM("## ERROR VerifierThread: driver message differs from server testcase (TelegramCounter " << server_telegram_counter << ")");
        ROS_WARN_STREAM("## driver message (received):  " << sick_lidar_localization::Utils::flattenToString(driver_telegram));
        ROS_WARN_STREAM("## server testcase (expected): " << sick_lidar_localization::Utils::flattenToString(server_testcase.telegram_msg));
        total_verification_failed_cnt++;
      }
      else
      {
        ROS_INFO_STREAM("VerifierThread: testcase verified and okay (TelegramCounter " << server_telegram_counter << ")");
        ROS_DEBUG_STREAM("VerifierThread: driver message (received):  " << sick_lidar_localization::Utils::flattenToString(driver_telegram));
        ROS_DEBUG_STREAM("VerifierThread: server testcase (expected): " << sick_lidar_localization::Utils::flattenToString(server_testcase.telegram_msg));
      }
      total_verification_cnt++;
    }
  }
  ROS_INFO_STREAM("VerifierThread: verification thread for sim_loc_driver messages finished");
  std::stringstream info_msg;
  info_msg << "VerifierThread: verification thread summary: " << total_verification_cnt << " messages checked, " << total_verification_failed_cnt << " failures.";
  if(ROS::ok())
    ROS_INFO_STREAM(info_msg.str());
  else
    std::cout << info_msg.str() << std::endl;
}
