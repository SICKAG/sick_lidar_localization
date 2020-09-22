/*
 * @brief sim_loc_test_server_thread implements a simple tcp server thread,
 * simulating a localization controller for unittests. It runs a thread to listen
 * and accept tcp connections from clients and generates telegrams to test
 * the ros driver for sim localization.
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

#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/cola_transmitter.h"
#include "sick_lidar_localization/random_generator.h"
#include "sick_lidar_localization/test_server_thread.h"
#include "sick_lidar_localization/testcase_generator.h"
#include "sick_lidar_localization/utils.h"

/*!
 * Constructor. The server thread does not start automatically, call start() and stop() to start and stop the server.
 * @param[in] nh ros node handle
 * @param[in] ip_port_results ip port for result telegrams, default: 2201
 * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
 */
sick_lidar_localization::TestServerThread::TestServerThread(ROS::NodePtr nh, int ip_port_results, int ip_port_cola)
: m_ip_port_results(ip_port_results), m_ip_port_cola(ip_port_cola), m_ioservice(),
  m_tcp_connection_thread_results(0), m_tcp_connection_thread_cola(0),
  m_tcp_connection_thread_running(false), m_worker_thread_running(false),
  m_tcp_acceptor_results(m_ioservice, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), m_ip_port_results)),
  m_tcp_acceptor_cola(m_ioservice, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), m_ip_port_cola)),
  m_result_telegram_rate(10), m_demo_move_in_circles(false), m_error_simulation_enabled(false), m_error_simulation_flag(NO_ERROR),
  m_error_simulation_thread(0), m_error_simulation_thread_running(false)
{
  m_tcp_acceptor_results.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  m_tcp_acceptor_cola.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  if(nh)
  {
    std::string result_testcases_topic = "/sick_lidar_localization/test_server/result_testcases"; // default topic to publish testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    ROS::param<double>(nh, "/sick_lidar_localization/test_server/result_telegrams_rate", m_result_telegram_rate, m_result_telegram_rate);
    ROS::param<std::string>(nh, "/sick_lidar_localization/test_server/result_testcases_topic", result_testcases_topic, result_testcases_topic);
    ROS::param<std::string>(nh, "/sick_lidar_localization/test_server/result_testcases_frame_id", m_result_testcases_frame_id, "result_testcases");
    ROS::param<bool>(nh, "/sim_loc_test_server/demo_circles", m_demo_move_in_circles, m_demo_move_in_circles);
    ROS::param<bool>(nh, "/sim_loc_test_server/error_simulation", m_error_simulation_enabled, m_error_simulation_enabled);
    std::string sim_loc_test_server_demo_circles, sim_loc_test_server_error_simulation;
    ROS::param<std::string>(nh, "/system/test_server/demo_circles", sim_loc_test_server_demo_circles, sim_loc_test_server_demo_circles);
    ROS::param<std::string>(nh, "/system/test_server/error_simulation", sim_loc_test_server_error_simulation, sim_loc_test_server_error_simulation);
    if(!sim_loc_test_server_demo_circles.empty())
      m_demo_move_in_circles = std::atoi(sim_loc_test_server_demo_circles.c_str()) > 0;
    if(!sim_loc_test_server_error_simulation.empty())
      m_error_simulation_enabled = std::atoi(sim_loc_test_server_error_simulation.c_str()) > 0;
    // ros publisher for testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    m_result_testcases_publisher = ROS_CREATE_PUBLISHER(nh, sick_lidar_localization::SickLocResultPortTestcaseMsg, result_testcases_topic);
  }
}

/*!
 * Destructor. Stops the server thread and closes all tcp connections.
 */
sick_lidar_localization::TestServerThread::~TestServerThread()
{
  stop();
}

/*!
 * Starts the server thread, starts to listen and accept tcp connections from clients.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::TestServerThread::start(void)
{
  if(m_error_simulation_enabled)
  {
    ROS_WARN_STREAM("TestServerThread: running in error simulation mode.");
    ROS_WARN_STREAM("Test server will intentionally react incorrect and will send false data and produce communication errors.");
    ROS_WARN_STREAM("Running test server in error simulation mode is for test purposes only. Do not expect typical results.");
    m_error_simulation_thread_running = true;
    m_error_simulation_thread =  new boost::thread(&sick_lidar_localization::TestServerThread::runErrorSimulationThreadCb, this);
  }
  else if(m_demo_move_in_circles)
  {
    ROS_INFO_STREAM("TestServerThread: running in demo mode (moving in circles, no error simulation).");
  }
  else
  {
    ROS_INFO_STREAM("TestServerThread: running in normal mode (no error simulation).");
  }
  // Start and run 3 threads to create sockets for new tcp clients on ip ports 2201 (results), 2111 (requests) and 2112 (responses)
  m_tcp_connection_thread_running = true;
  m_tcp_connection_thread_results = new boost::thread(&sick_lidar_localization::TestServerThread::runConnectionThreadResultCb, this);
  m_tcp_connection_thread_cola = new boost::thread(&sick_lidar_localization::TestServerThread::runConnectionThreadColaCb, this);
  return true;
}

/*!
 * Stops the server thread and closes all tcp connections.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::TestServerThread::stop(void)
{
  m_tcp_connection_thread_running = false;
  m_error_simulation_thread_running = false;
  m_worker_thread_running = false;
  if(m_error_simulation_thread)
  {
    m_error_simulation_thread->join();
    delete(m_error_simulation_thread);
    m_error_simulation_thread = 0;
  }
  m_ioservice.stop();
  // close both tcp acceptors for ip ports 2201 (results), 2111 (cola command requests)
  std::vector<boost::asio::ip::tcp::acceptor*> tcp_acceptors = { &m_tcp_acceptor_results, & m_tcp_acceptor_cola };
  for(std::vector<boost::asio::ip::tcp::acceptor*>::iterator iter_acceptor = tcp_acceptors.begin(); iter_acceptor != tcp_acceptors.end(); iter_acceptor++)
  {
    if((*iter_acceptor)->is_open())
    {
      (*iter_acceptor)->cancel(); // cancel possibly blocking calls of m_tcp_acceptor_results.listen() in the connection thread
      (*iter_acceptor)->close();  // close tcp acceptor
    }
  }
  // close 2 threads creating sockets for new tcp clients on ip ports 2201 (results), 2111 (cola command requests)
  std::vector<thread_ptr*> tcp_connection_threads = { &m_tcp_connection_thread_results, &m_tcp_connection_thread_cola };
  for(std::vector<thread_ptr*>::iterator iter_connection_thread = tcp_connection_threads.begin(); iter_connection_thread != tcp_connection_threads.end(); iter_connection_thread++)
  {
    thread_ptr & tcp_connection_thread = **iter_connection_thread;
    if(tcp_connection_thread)
    {
      tcp_connection_thread->join();
      delete(tcp_connection_thread);
      tcp_connection_thread = 0;
    }
  }
  // close sockets
  closeTcpConnections();
  // close worker/client threads
  closeWorkerThreads();
  return true;
}

/*!
 * Closes all tcp connections
 * @param[in] force_shutdown if true, sockets are immediately forced to shutdown
 */
void sick_lidar_localization::TestServerThread::closeTcpConnections(bool force_shutdown)
{
  for(std::list<boost::asio::ip::tcp::socket*>::iterator socket_iter = m_tcp_sockets.begin(); socket_iter != m_tcp_sockets.end(); socket_iter++)
  {
    boost::asio::ip::tcp::socket* p_socket = *socket_iter;
    try
    {
      if(p_socket && (force_shutdown || p_socket->is_open()))
      {
        p_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        p_socket->close();
      }
    }
    catch(std::exception & exc)
    {
      ROS_WARN_STREAM("TestServerThread::closeTcpConnections(): exception " << exc.what() << " on closing socket.");
    }
    *socket_iter = 0;
  }
  m_tcp_sockets.clear();
}

/*!
 * Closes a socket.
 * @param[in,out] p_socket socket to be closed
 */
void sick_lidar_localization::TestServerThread::closeSocket(socket_ptr & p_socket)
{
  boost::lock_guard<boost::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
  try
  {
    if(p_socket && p_socket->is_open())
    {
      p_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
      p_socket->close();
    }
    if(p_socket)
    {
      for(std::list<boost::asio::ip::tcp::socket*>::iterator socket_iter = m_tcp_sockets.begin(); socket_iter != m_tcp_sockets.end(); )
      {
        if(p_socket == *socket_iter)
          socket_iter = m_tcp_sockets.erase(socket_iter);
        else
          socket_iter++;
      }
      p_socket = 0;
    }
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("TestServerThread::closeSocket(): exception " << exc.what() << " on closing socket.");
  }
}

/*!
 * Stops all worker threads
 */
void sick_lidar_localization::TestServerThread::closeWorkerThreads(void)
{
  m_worker_thread_running = false;
  boost::lock_guard<boost::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
  for(std::list<thread_ptr>::iterator thread_iter = m_tcp_worker_threads.begin(); thread_iter != m_tcp_worker_threads.end(); thread_iter++)
  {
    boost::thread *p_thread = *thread_iter;
    p_thread->join();
    delete(p_thread);
  }
  m_tcp_worker_threads.clear();
}

/*!
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * Buffers the last telegram to monitor sim_loc_driver messages in error simulation mode.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_lidar_localization::TestServerThread::messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg)
{
  m_last_telegram_received.set(msg);
}

/*!
 * Thread callback, listens and accept tcp connections from clients for result telegrams.
 * Starts a new worker thread to generate result port telegrams for each tcp client.
 */
void sick_lidar_localization::TestServerThread::runConnectionThreadResultCb(void)
{
  runConnectionThreadGenericCb(m_tcp_acceptor_results, m_ip_port_results, &sick_lidar_localization::TestServerThread::runWorkerThreadResultCb);
}

/*!
 * Thread callback, listens and accept tcp connections from clients for cola telegrams.
 * Starts a new worker thread to receive command requests for each tcp client.
 */
void sick_lidar_localization::TestServerThread::runConnectionThreadColaCb(void)
{
  runConnectionThreadGenericCb(m_tcp_acceptor_cola, m_ip_port_cola, &sick_lidar_localization::TestServerThread::runWorkerThreadColaCb);
}

/*!
 * Thread callback, listens and accept tcp connections from clients.
 * Starts a worker thread for each tcp client.
 */
template<typename Callable>void sick_lidar_localization::TestServerThread::runConnectionThreadGenericCb(boost::asio::ip::tcp::acceptor & tcp_acceptor_results, int ip_port_results, Callable thread_function_cb)
{
  ROS_INFO_STREAM("TestServerThread: connection thread started");
  while(ROS::ok() && m_tcp_connection_thread_running && tcp_acceptor_results.is_open())
  {
    if(m_error_simulation_flag.get() == DONT_LISTEN) // error simulation: testserver does not open listening port
    {
      ROS::sleep(0.1);
      continue;
    }
    // normal mode: listen to tcp port, accept and connect to new tcp clients
    boost::asio::ip::tcp::socket* tcp_client_socket = new boost::asio::ip::tcp::socket(m_ioservice);
    boost::system::error_code errorcode;
    if(m_error_simulation_flag.get() != DONT_LISTEN && m_error_simulation_flag.get() != DONT_ACCECPT)
      ROS_INFO_STREAM("TestServerThread: listening to tcp connections on port " << ip_port_results);
    tcp_acceptor_results.listen();
    if(m_error_simulation_flag.get() == DONT_LISTEN || m_error_simulation_flag.get() == DONT_ACCECPT) // error simulation: testserver does not does not accecpt tcp clients
    {
      ROS::sleep(0.1);
      continue;
    }
    tcp_acceptor_results.accept(*tcp_client_socket, errorcode); // normal mode: accept new tcp client
    if(m_error_simulation_flag.get() == DONT_LISTEN || m_error_simulation_flag.get() == DONT_ACCECPT) // error simulation: testserver does not does not accecpt tcp clients
    {
      if(tcp_client_socket->is_open())
      {
        tcp_client_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        tcp_client_socket->close();
  
      }
      continue;
    }
    if (!errorcode && tcp_client_socket->is_open())
    {
      // tcp client connected, start worker thread
      ROS_INFO_STREAM("TestServerThread: established new tcp client connection");
      boost::lock_guard<boost::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
      m_tcp_sockets.push_back(tcp_client_socket);
      m_worker_thread_running = true;
      m_tcp_worker_threads.push_back(new boost::thread(thread_function_cb, this, tcp_client_socket));
    }
  }
  closeTcpConnections();
  m_tcp_connection_thread_running = false;
  ROS_INFO_STREAM("TestServerThread: connection thread finished");
}

/*!
 * Worker thread callback, generates and sends result telegrams to a tcp client.
 * There's one result worker thread for each tcp client.
 * @param[in] p_socket socket to send result telegrams to the tcp client
 */
void sick_lidar_localization::TestServerThread::runWorkerThreadResultCb(boost::asio::ip::tcp::socket* p_socket)
{
  ROS_INFO_STREAM("TestServerThread: worker thread for result telegrams started");
  sick_lidar_localization::UniformRandomInteger random_generator(0,255);
  sick_lidar_localization::UniformRandomInteger random_length(1, 512);
  sick_lidar_localization::UniformRandomInteger random_integer(0, INT_MAX);
  double circle_yaw = 0;
  while(ROS::ok() && m_worker_thread_running && p_socket && p_socket->is_open())
  {
    ROS::sleep((double)sick_lidar_localization::TestcaseGenerator::ResultPoseInterval() / m_result_telegram_rate);
    boost::system::error_code error_code;
    if (m_error_simulation_flag.get() == DONT_SEND) // error simulation: testserver does not send any telegrams
    {
      ROS_DEBUG_STREAM("TestServerThread for cresult telegrams: error simulation, server not sending any telegrams");
      continue;
    }
    if (m_error_simulation_flag.get() == SEND_RANDOM_TCP) // error simulation: testserver sends invalid random tcp packets
    {
      std::vector<uint8_t> random_data = random_generator.generate(random_length.generate()); // binary random data of random size
      boost::asio::write(*p_socket, boost::asio::buffer(random_data.data(), random_data.size()), boost::asio::transfer_exactly(random_data.size()), error_code);
      ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random data " << sick_lidar_localization::Utils::toHexString(random_data));
      continue;
    }
    // create testcase is a result port telegram with random based sythetical data
    sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = sick_lidar_localization::TestcaseGenerator::createRandomResultPortTestcase();
    if(m_demo_move_in_circles) // simulate a sensor moving in circles
    {
      testcase = sick_lidar_localization::TestcaseGenerator::createResultPortCircles(2.0, circle_yaw);
      circle_yaw = sick_lidar_localization::Utils::normalizeAngle(circle_yaw + 1.0 * M_PI / 180);
    }
    if (m_error_simulation_flag.get() == SEND_INVALID_TELEGRAMS) // error simulation: testserver sends invalid telegrams (invalid data, false checksums, etc.)
    {
      int number_random_bytes = ((random_integer.generate()) % (testcase.binary_data.size()));
      for(int cnt_random_bytes = 0; cnt_random_bytes < number_random_bytes; cnt_random_bytes++)
      {
        int byte_cnt = ((random_integer.generate()) % (testcase.binary_data.size()));
        testcase.binary_data[byte_cnt] = (uint8_t)(random_generator.generate() & 0xFF);
      }
      ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random binary telegram " << sick_lidar_localization::Utils::toHexString(testcase.binary_data));
    }
    // send binary result port telegram to tcp client (if localization is "on")
    if(sick_lidar_localization::TestcaseGenerator::LocalizationEnabled() && sick_lidar_localization::TestcaseGenerator::ResultTelegramsEnabled())
    {
      size_t bytes_written = boost::asio::write(*p_socket, boost::asio::buffer(testcase.binary_data.data(), testcase.binary_data.size()), boost::asio::transfer_exactly(testcase.binary_data.size()), error_code);
      if (error_code || bytes_written != testcase.binary_data.size())
      {
        std::stringstream error_info;
        error_info << "## ERROR TestServerThread for result telegrams: failed to send binary result port telegram, " << bytes_written << " of " << testcase.binary_data.size() << " bytes send, error code: " << error_code.message();
        if (m_error_simulation_flag.get() == NO_ERROR)
        {
          ROS_WARN_STREAM(error_info.str() << ", close socket and leave worker thread for result telegrams");
          break;
        }
        ROS_DEBUG_STREAM(error_info.str());
      }
      else
      {
        ROS_DEBUG_STREAM("TestServerThread for result telegrams: send binary result port telegram " << sick_lidar_localization::Utils::toHexString(testcase.binary_data));
      }
      // publish testcases (SickLocResultPortTestcaseMsg, i.e. binary telegram and SickLocResultPortTelegramMsg messages) for test and verification of sim_loc_driver
      testcase.header.stamp = ROS::now();
      testcase.header.frame_id = m_result_testcases_frame_id;
      ROS_PUBLISH(m_result_testcases_publisher, testcase);
    }
  }
  closeSocket(p_socket);
  ROS_INFO_STREAM("TestServerThread: worker thread for result telegrams finished");
}

/*!
 * Worker thread callback, receives command requests from a tcp client
 * and sends a synthetical command response.
 * There's one request worker thread for each tcp client.
 * @param[in] p_socket socket to receive command requests from the tcp client
 */
void sick_lidar_localization::TestServerThread::runWorkerThreadColaCb(boost::asio::ip::tcp::socket* p_socket)
{
  ROS_INFO_STREAM("TestServerThread: worker thread for command requests started");
  sick_lidar_localization::UniformRandomInteger random_generator(0,255);
  sick_lidar_localization::UniformRandomInteger random_length(1, 128);
  sick_lidar_localization::UniformRandomAsciiString random_ascii;
  while(ROS::ok() && m_worker_thread_running && p_socket && p_socket->is_open())
  {
    // Read command request from tcp client
    ServerColaRequest request;
    ROS::Time receive_timestamp;
    if(sick_lidar_localization::ColaTransmitter::receive(*p_socket, request.telegram_data, 1, receive_timestamp))
    {
      if (m_error_simulation_flag.get() == DONT_SEND) // error simulation: testserver does not send any telegrams
      {
        ROS_DEBUG_STREAM("TestServerThread for command requests: error simulation, server not sending any telegrams");
        continue;
      }
      if (m_error_simulation_flag.get() == SEND_RANDOM_TCP) // error simulation: testserver sends invalid random tcp packets
      {
        boost::system::error_code error_code;
        std::vector<uint8_t> random_data = random_generator.generate(random_length.generate()); // binary random data of random size
        boost::asio::write(*p_socket, boost::asio::buffer(random_data.data(), random_data.size()), boost::asio::transfer_exactly(random_data.size()), error_code);
        ROS_DEBUG_STREAM("TestServerThread for command requests: send random data " << sick_lidar_localization::Utils::toHexString(random_data));
        continue;
      }
      // command requests received, generate and send a synthetical response
      bool cola_binary = sick_lidar_localization::ColaAsciiBinaryConverter::IsColaBinary(request.telegram_data);
      if(cola_binary)
        request.telegram_data = sick_lidar_localization::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(request.telegram_data);
      std::string ascii_telegram = sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(request.telegram_data);
      ROS_INFO_STREAM("TestServerThread: received cola request " << ascii_telegram);
      sick_lidar_localization::SickLocColaTelegramMsg telegram_msg = sick_lidar_localization::ColaParser::decodeColaTelegram(ascii_telegram);
      // Generate a synthetical response depending on the request
      sick_lidar_localization::SickLocColaTelegramMsg telegram_answer = sick_lidar_localization::TestcaseGenerator::createColaResponse(telegram_msg);
      if (m_error_simulation_flag.get() == SEND_INVALID_TELEGRAMS) // error simulation: testserver sends invalid telegrams (invalid data, false checksums, etc.)
      {
        telegram_answer.command_name = random_ascii.generate(random_length.generate()); // random ascii string
        telegram_answer.parameter.clear();
        for(int n = random_length.generate(); n > 0; n--)
          telegram_answer.parameter.push_back(random_ascii.generate(random_length.generate())); // random ascii string
        ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random cola response " << sick_lidar_localization::Utils::flattenToString(telegram_answer));
      }
      // Send command response to tcp client
      std::vector<uint8_t> binary_response = sick_lidar_localization::ColaParser::encodeColaTelegram(telegram_answer);
      std::string ascii_response = sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(binary_response);
      if(cola_binary)
        binary_response = sick_lidar_localization::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(binary_response);
      ROS_INFO_STREAM("TestServerThread: sending cola response " << ascii_response << (cola_binary ? " (Cola-Binary)" : " (Cola-ASCII)"));
      ROS::Time send_timestamp;
      if (!sick_lidar_localization::ColaTransmitter::send(*p_socket, binary_response, send_timestamp))
      {
        ROS_WARN_STREAM("TestServerThread: failed to send cola response, ColaTransmitter::send() returned false, data hexdump: " << sick_lidar_localization::Utils::toHexString(binary_response));
      }
    }
    ROS::sleep(0.0001);
  }
  closeSocket(p_socket);
  ROS_INFO_STREAM("TestServerThread: worker thread for command requests finished");
}

/*!
 * Waits for a given time in seconds, as long as ROS::ok() and m_error_simulation_thread_running == true.
 * @param[in] seconds delay in seconds
 */
void sick_lidar_localization::TestServerThread::errorSimulationWait(double seconds)
{
  ROS::Time starttime = ROS::now();
  while(ROS::ok() && m_error_simulation_thread_running && ROS::seconds(ROS::now() - starttime) < seconds)
  {
    ROS::sleep(0.001);
  }
}

/*!
 * Waits for and returns the next telegram message from sick_lidar_localization driver.
 * @param[in] timeout_seconds wait timeout in seconds
 * @param[out] telegram_msg last telegram message received
 * @return true, if a new telegram message received, false otherwise (timeout or shutdown)
 */
bool sick_lidar_localization::TestServerThread::errorSimulationWaitForTelegramReceived(double timeout_seconds, sick_lidar_localization::SickLocResultPortTelegramMsg & telegram_msg)
{
  ROS::Time starttime = ROS::now();
  while(ROS::ok() && m_error_simulation_thread_running)
  {
    telegram_msg = m_last_telegram_received.get();
    if(ROS::timeFromHeader(&telegram_msg.header) > starttime)
      return true; // new telegram received
    if(ROS::seconds(ROS::now() - starttime) >= timeout_seconds)
      return false; // timeout
    ROS::sleep(0.001);
  }
  return false;
}

/*!
 * Thread callback, runs an error simulation and switches m_error_simulation_flag through the error test cases.
 */
void sick_lidar_localization::TestServerThread::runErrorSimulationThreadCb(void)
{
  sick_lidar_localization::SickLocResultPortTelegramMsg telegram_msg;
  size_t number_testcases = 0, number_testcases_failed = 0;
  ROS_INFO_STREAM("TestServerThread: error simulation thread started");
  
  // Error simulation: start normal execution
  number_testcases++;
  m_error_simulation_flag.set(NO_ERROR);
  ROS_INFO_STREAM("TestServerThread: 1. error simulation testcase: normal execution, expecting telegram messages from driver");
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 1. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 1. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not open listening port for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_LISTEN);
  ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: server not responding, not listening, no tcp connections accepted.");
  m_worker_thread_running = false;
  ROS::sleep(1.0 / m_result_telegram_rate);
  closeTcpConnections(true);
  errorSimulationWait(10);
  m_error_simulation_flag.set(NO_ERROR);
  ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: switched to normal execution, expecting telegram messages from driver");
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 2. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not accecpt tcp clients for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_ACCECPT);
  ROS_INFO_STREAM("TestServerThread: 3. error simulation testcase: server not responding, listening on port " << m_ip_port_results << ", but accepting no tcp clients");
  m_worker_thread_running = false;
  ROS::sleep(1.0 / m_result_telegram_rate);
  closeTcpConnections(true);
  errorSimulationWait(10);
  m_error_simulation_flag.set(NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 3. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 3. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not send telegrams for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_SEND);
  ROS_INFO_STREAM("TestServerThread: 4. error simulation testcase: server not sending telegrams");
  errorSimulationWait(10);
  m_error_simulation_flag.set(NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 4. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 4. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver sends random tcp data for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(SEND_RANDOM_TCP);
  ROS_INFO_STREAM("TestServerThread: 5. error simulation testcase: server sending random tcp data");
  errorSimulationWait(10);
  m_error_simulation_flag.set(NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 5. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 5. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver sends invalid telegrams (invalid data, false checksums, etc.) for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(SEND_INVALID_TELEGRAMS);
  ROS_INFO_STREAM("TestServerThread: 6. error simulation testcase: server sending invalid telegrams");
  errorSimulationWait(10);
  m_error_simulation_flag.set(NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 6. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 6. error simulation testcase: received telegram message \"" << sick_lidar_localization::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // End of error simulation, print summary and exit
  m_error_simulation_thread_running = false;
  ROS_INFO_STREAM("TestServerThread: error simulation thread finished");
  ROS_INFO_STREAM("TestServerThread: error simulation summary: " << (number_testcases - number_testcases_failed) << " of " << number_testcases<< " testcases passed, " << number_testcases_failed << " failures.");
  ROS_INFO_STREAM("TestServerThread: exit with ros::shutdown()");
  ROS::shutdown();
}
