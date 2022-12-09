// Copyright 2021 Oxford Technical Solutions Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file driver.hpp
 * Defines node to take NCom data and publish it in ROS messages.
 */

#ifndef OXTS_DRIVER__DRIVER_HPP_
#define OXTS_DRIVER__DRIVER_HPP_

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <boost/function.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>

// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include <oxts_driver/NComRxC.h>
#include <oxts_driver/nav_const.hpp>
#include <oxts_driver/udp_server_client.h>

using namespace std::chrono_literals;

namespace lib_oxts {

/**
 * Enumeration of timestamp modes for published topics
 */
enum PUB_TIMESTAMP_MODE {
  /** Use ROS time. */
  ROS = 0,
  /** Use NCom time. */
  NCOM = 1
};
/**
 * This class creates a subclass of Node designed to take NCom data from the
 * NCom decoder and publish it to pre-configured ROS topics.
 *
 * @todo Add config struct to hold data which will hold config parsed from the
 *       .yaml file.
 */
class OxtsDriver {
private:
  /*! Rate at which to sample NCom. Expected that this will typically match
    the rate of NCom itself, though can be set lower to save computation. */
  uint32_t ncom_rate;
  /*! The topic to publish the raw NCOM messages to */
  std::string ncom_topic;
  /*! The string to prefix the ncom topic with */
  std::string topic_prefix;
  /*! IP address of the INS to connect to */
  std::string unit_ip;
  /*! Endpoint Port of the INS to be connected to. Default 3000 for NCom. */
  int unit_port;
  /*! File path to NCom file to be used as input. Not required if running
    in real time. */
  std::string ncom_path;
  /*! Function pointer to the necesary NCom file/socket callback */
  void (lib_oxts::OxtsDriver::*timer_ncom_callback)();
  /*! Function pointer to the necesary NCom updater */
  void (lib_oxts::OxtsDriver::*update_ncom)();
  /*! Whether ot not to wait for NCom initialisation before publishing messages.
   */
  bool wait_for_init;


  std::chrono::duration<uint64_t, std::milli> ncomInterval;
  double prevRegularWeekSecond;
  
  boost::function<
    void(NComRxCInternal * ncom, std::string frame_id)> m_external_callback;

  rclcpp::TimerBase::SharedPtr timer_ncom_;

  /**
   * Callback function for NCom sampling. Receives data from chosen source
   * (UDP or file) and parses a packet to nrx.
   *
   * @todo Refactor into input class
   */
  void timerNcomSocketCallback();
  void timerNcomFileCallback();
  void getFilePacket();
  void getSocketPacket();
  void publishPacket();
  void timer_start(boost::function<void()> pub_callback, int interval);
  /**
   * Publisher for std_msgs/msg/string. Only used for debugging, currently
   * outputs lat, long, alt in string form.
   */
  //rclcpp::Publisher<safeai_interfaces::msg::Ncom>::SharedPtr pubNCom_;

public:
  /**
   * Default constructor for the OxtsDriver. Parses options from the
   * .yaml params/config file, sets up UDP connection to unit.
   */
  OxtsDriver(
  	uint32_t & ncom_rate,
  	std::string & ip,
  	uint16_t port,
    std::string & topic_prefix,
    boost::function<void(NComRxCInternal *, std::string)> pub_callback,
    std::shared_ptr<rclcpp::Node> private_nh
  );

  /** NCom decoder instance */
  NComRxC *nrx;
  /** Buffer for UDP data */
  unsigned char buff[1024];
  /** UDP Client to receive data from the device */
  networking_udp::client udpClient;
  /** Endpoint for the udpClient to receive data from */
  boost::asio::ip::udp::endpoint unitEndpointNCom;

  std::fstream inFileNCom;

  bool checkRate(double prevPktSec, double currPktSec);
  rclcpp::Time getTimestamp();
  /**
   * Convert NCom time to a ROS friendly time format. Does not convert to ROS
   * time, only the format.
   *
   * @param nrx Pointer to the decoded NCom data
   */
  rclcpp::Time getNcomTime(const NComRxC *nrx);
  /**
   * Get the IP address of the OxTS unit, as set in the .yaml params file
   *
   * @returns IP address as a string
   */
  std::string getUnitIp();
  /**
   * Get the endpoint port of the OxTS unit, as set in the .yaml params file
   *
   * @returns Port as a short
   */
  short getUnitPort();
};

} // namespace lib_oxts

#endif // OXTS_DRIVER__DRIVER_HPP_
