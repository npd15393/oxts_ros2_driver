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

#include <oxts_driver/driver.hpp>

namespace lib_oxts {


OxtsDriver::OxtsDriver(
  uint32_t & ncom_rate,
  std::string & ip,
  uint16_t port,
  std::string & topic_prefix,
  boost::function<void(NComRxCInternal *, std::string)> pub_callback,
  std::shared_ptr<rclcpp::Node> private_nh
) 
{
  // Get parameters (from config, command line, or from default)
  // Initialise configurable parameters (all params should have defaults)
  ncom_rate = ncom_rate;
  ncom_topic = "ncom";
  topic_prefix = topic_prefix;
  unit_ip = ip;
  unit_port = port;
  ncom_path = std::string("");
  wait_for_init = true;

  m_external_callback = pub_callback;

  ncomInterval = std::chrono::milliseconds(int(1000.0 / ncom_rate));

  prevRegularWeekSecond = -1;

  nrx = NComCreateNComRxC();

  if (!ncom_path.empty()) {
    ncom_path = std::filesystem::canonical(ncom_path);
    inFileNCom.open(ncom_path);
    if (!inFileNCom.is_open()) {
      fprintf(stderr,"SDK: Unable to open NCOM: %s\n",
                    ncom_path.c_str());
      return;
    } else {
      fprintf(stderr,"SDK: Opened NCOM: %s\n", ncom_path.c_str());
    }
  } else {
    unitEndpointNCom = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(this->unit_ip),
        this->unit_port);

    this->udpClient.set_local_port(this->unit_port);
    fprintf(stderr,"SDK: Connecting: %s:%d\n",
                this->unit_ip.c_str(), this->unit_port);
    
  }

  // Assign callback functions to timers (callbacks are called at a rate
  // dictated by the associated timer)
  if (!ncom_path.empty()) {
    timer_ncom_callback = &OxtsDriver::timerNcomFileCallback;
    update_ncom = &OxtsDriver::getFilePacket;
  } else {
    timer_ncom_callback = &OxtsDriver::timerNcomSocketCallback;
    update_ncom = &OxtsDriver::getSocketPacket;
  }

  // Wait for config to be populated in NCOM packets
  fprintf(stderr,"SDK: Waiting for INS config information...\n");
  while (nrx->mSerialNumber == 0 || nrx->mIsImu2VehHeadingValid == 0) {
    (*this.*update_ncom)();
  }
  fprintf(stderr,"SDK: INS config information received\n");

  // Wait for INS initialisation if option enabled
  if (wait_for_init) {
    fprintf(stderr,"SDK: Waiting for initialisation...\n");
    // Only block things that are required for 100% of OxTS navigation
    while (nrx->mInsNavMode != NAV_CONST::NAV_MODE::REAL_TIME &&
            nrx->mIsLatValid == 0 && nrx->mIsLonValid == 0 &&
            nrx->mIsAltValid == 0 && nrx->mIsHeadingValid == 0 &&
            nrx->mIsPitchValid == 0 && nrx->mIsRollValid == 0) {
      (*this.*update_ncom)();
    }
    fprintf(stderr,"INS initialised");
  } else {
    fprintf(stderr,"Waiting for approximate position...");
    // Wait for a approximate position (while uninitialised, orientation is
    // never approx/valid)
    while (nrx->mIsLatApprox == 0 && nrx->mIsLonApprox == 0 &&
            nrx->mIsAltApprox == 0 && nrx->mIsLatValid == 0 &&
            nrx->mIsLonValid == 0 && nrx->mIsAltValid == 0) {
      (*this.*update_ncom)();
    }
    fprintf(stderr,
                "SDK: Publishing before/without INS initialisation\n");
  }

  fprintf(stderr,"SDK: Publishing NCom packets at: %uHz\n",
              ncom_rate);

  timer_start(std::bind(timer_ncom_callback,this), ncomInterval.count());
}

void OxtsDriver::timer_start(boost::function<void()> pub_callback, int interval)
{
  std::thread([this, pub_callback, interval]()
  { 
    while (true)
    { 
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
      pub_callback();
      std::this_thread::sleep_until(x);
    }
  }).detach();
}

void OxtsDriver::timerNcomSocketCallback() {
  OxtsDriver::getSocketPacket();
  OxtsDriver::publishPacket();
}

void OxtsDriver::timerNcomFileCallback() {
  OxtsDriver::getFilePacket();
  OxtsDriver::publishPacket();
}

void OxtsDriver::getFilePacket() {
  char c;

  do {
      if (!this->inFileNCom.get(c)) {
          fprintf(stderr,"End of NCom file reached.");
          rclcpp::shutdown();
          return;
      }
  } while (NComNewChar(this->nrx, (unsigned char)c) != COM_NEW_UPDATE);
}

void OxtsDriver::getSocketPacket() {
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(
      this->buff, NCOM_PACKET_LENGTH, this->unitEndpointNCom);
  // fprintf(stderr,"======== ins packet size %zu",size);
  // Add data to decoder
  while (NComNewChars(this->nrx, this->buff, size) != COM_NEW_UPDATE) {
  }
  // fprintf(stderr,"======== decoder ready");
} 

void OxtsDriver::publishPacket() {
  // publish the NCOM packet
  switch (this->nrx->mOutputPacketType) {
  case OUTPUT_PACKET_REGULAR: {
    if (this->checkRate(this->prevRegularWeekSecond,
                        this->nrx->mTimeWeekSecond))
      return;
    std::string frame_id = "oxts_sn" + std::to_string(this->nrx->mSerialNumber);

    m_external_callback(this->nrx->mInternal, frame_id);
    this->prevRegularWeekSecond = this->nrx->mTimeWeekSecond;
    break;
  }
  case OUTPUT_PACKET_STATUS: {
    break;
  }
  default:
    break;
  }
}

bool OxtsDriver::checkRate(double prevPktSec, double currPktSec) {
  bool skip_packet = false;
  // perform error checking on nrx timestamps
  if (prevPktSec <= 0)
    ;
  else if (currPktSec - prevPktSec > (1.5 / this->ncom_rate)) {
    fprintf(stderr,"Packet drop detected.");
  } else if (currPktSec < prevPktSec) {
    fprintf(stderr,
      "Current packet is older than previous packet, skipping packet.");
    skip_packet = true;
  } else if (currPktSec == prevPktSec) {
    fprintf(stderr,
      "Duplicate NCOM packet detected, skipping packet.");
    skip_packet = true;
  } else if (currPktSec - prevPktSec < (0.5 / this->ncom_rate)) {
    // fprintf(stderr,
    //   "Early packet detected, ncom_rate may be misconfigured.");
  }
  return skip_packet;
}

rclcpp::Time OxtsDriver::getNcomTime(const NComRxC *nrx) {
  auto time =
      rclcpp::Time(static_cast<int32_t>(nrx->mTimeWeekSecond) +
                       (nrx->mTimeWeekCount * NAV_CONST::WEEK_SECS) +
                       nrx->mTimeUtcOffset + NAV_CONST::GPS2UNIX_EPOCH,
                   static_cast<uint32_t>((nrx->mTimeWeekSecond -
                                          std::floor(nrx->mTimeWeekSecond)) *
                                         NAV_CONST::SECS2NANOSECS));

  return time;
}

std::string OxtsDriver::getUnitIp() { return this->unit_ip; }

short OxtsDriver::getUnitPort() { return this->unit_port; }

} // namespace lib_oxts
