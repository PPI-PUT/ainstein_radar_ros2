// Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
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

#include "ainstein_radar_ros2/ainstein_radar_T-79.hpp"

#include <iostream>
#include <utility>

namespace ainstein_radar
{
AinsteinRadarDriverT79::AinsteinRadarDriverT79(
  bool sendRaw, bool sendTracked, int canId,
  std::string frameId, double RANGE_MIN,
  double RANGE_MAX, double AZIMUTH_MIN,
  double AZIMUTH_MAX)
{
  this->m_sendRaw = sendRaw;
  this->m_sendTracked = sendTracked;
  this->m_canId = canId;
  this->m_frameId = std::move(frameId);
  this->m_RANGE_MIN = RANGE_MIN;
  this->m_RANGE_MAX = RANGE_MAX;
  this->m_AZIMUTH_MIN = AZIMUTH_MIN;
  this->m_AZIMUTH_MAX = AZIMUTH_MAX;
}


void AinsteinRadarDriverT79::startRadar(
  const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & publisher,
  const rclcpp::Time & time, const rclcpp::Logger & logger)
{
  m_can_frame_msg.header.stamp = time;
  m_can_frame_msg.dlc = 8;
  m_can_frame_msg.id = RADAR_COMMAND;
  m_can_frame_msg.data[0] = RADAR_START;
  m_can_frame_msg.data[1] = RESERVED;
  if (m_sendRaw && m_sendTracked) {
    m_can_frame_msg.data[2] = RADAR_SEND_TRACKED_AND_RAW;
  } else if (m_sendRaw) {m_can_frame_msg.data[2] = RADAR_SEND_RAW;} else if (m_sendTracked) {
    m_can_frame_msg.data[2] = RADAR_SEND_TRACKED;
  } else {RCLCPP_ERROR(logger, "DATA TYPE PARAMETERS ARE BOTH SET TO FALSE"); return;}
  m_can_frame_msg.data[3] = RESERVED;
  m_can_frame_msg.data[4] = RESERVED;
  m_can_frame_msg.data[5] = RESERVED;
  m_can_frame_msg.data[6] = RESERVED;
  m_can_frame_msg.data[7] = RESERVED;

  publisher->publish(m_can_frame_msg);
}

void AinsteinRadarDriverT79::stopRadar(
  const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & publisher,
  const rclcpp::Time & time)
{
  m_can_frame_msg.header.stamp = time;
  m_can_frame_msg.dlc = 8;
  m_can_frame_msg.id = RADAR_COMMAND;
  m_can_frame_msg.data[0] = RADAR_STOP;
  m_can_frame_msg.data[1] = RESERVED;
  m_can_frame_msg.data[2] = RESERVED;
  m_can_frame_msg.data[3] = RESERVED;
  m_can_frame_msg.data[4] = RESERVED;
  m_can_frame_msg.data[5] = RESERVED;
  m_can_frame_msg.data[6] = RESERVED;
  m_can_frame_msg.data[7] = RESERVED;

  publisher->publish(m_can_frame_msg);

}

void AinsteinRadarDriverT79::updateRadarFOV(
  const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & publisher,
  const rclcpp::Time & time)
{
  m_can_frame_msg.header.stamp = time;
  m_can_frame_msg.dlc = 8;
  m_can_frame_msg.id = RADAR_COMMAND;
  m_can_frame_msg.data[0] = RADAR_UPDATE_ZOI;
  m_can_frame_msg.data[1] = static_cast<uint8_t>(m_canId);
  m_can_frame_msg.data[2] = static_cast<uint8_t>( m_RANGE_MIN * 3.0 );
  m_can_frame_msg.data[3] = static_cast<uint8_t>( m_RANGE_MAX * 3.0 );
  m_can_frame_msg.data[4] = static_cast<uint8_t>( m_AZIMUTH_MIN );
  m_can_frame_msg.data[5] = static_cast<uint8_t>( m_AZIMUTH_MAX );
  m_can_frame_msg.data[6] = RESERVED;
  m_can_frame_msg.data[7] = RESERVED;

  publisher->publish(m_can_frame_msg);

}

void AinsteinRadarDriverT79::msgCallback(
  const can_msgs::msg::Frame::SharedPtr & msg,
  const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr & publisher_raw,
  const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr & publisher_tracked,
  const rclcpp::Time & time,
  const rclcpp::Logger & logger)
{

  if (msg->id == RADAR_RETURN_COMMAND) {
    processRadarReturn(msg->data[0], logger);
  } else if (msg->id == static_cast<uint16_t>(RADAR_START_FRAME + m_canId)) {
    processRadarStart(time, logger);
  } else if (msg->id == static_cast<uint16_t>(RADAR_STOP_FRAME + m_canId)) {
    processRadarStop(publisher_raw, publisher_tracked, logger);
  } else if (msg->id == static_cast<uint16_t>(RADAR_RAW_TARGET + m_canId)) {
    m_radar_raw_data.returns.push_back(*processRadarOutput(msg));
  } else if (msg->id == static_cast<uint16_t>(RADAR_TRACKED_TARGET + m_canId)) {
    m_radar_tracked_data.returns.push_back(*processRadarOutput(msg));
  } else {
    RCLCPP_ERROR(logger, "received message with unknown id: %02x", msg->id);

  }

}

void AinsteinRadarDriverT79::processRadarReturn(
  const uint8_t & command,
  const rclcpp::Logger & logger) const
{
  switch (command) {
    case RADAR_START:
      RCLCPP_INFO(logger, "Received radar start message from radar with CAN ID %d", m_canId);
      break;
    case RADAR_STOP:
      RCLCPP_INFO(logger, "Received radar stop message from radar with CAN ID %d", m_canId);
      break;
    case RADAR_UPDATE_ZOI:
      RCLCPP_INFO(logger, "Received radar update zoi message from radar with CAN ID %d", m_canId);
      break;
    default:
      RCLCPP_ERROR(logger, "Received unknown radar message from radar with CAN ID %d", m_canId);
      break;
  }
}

void AinsteinRadarDriverT79::processRadarStart(
  const rclcpp::Time & time,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received start frame from radar with CAN ID %d", m_canId);
  if (m_sendRaw && m_sendTracked) {
    m_radar_raw_data.header.stamp = time;
    m_radar_tracked_data.header.stamp = time;
    m_radar_raw_data.header.frame_id = m_frameId;
    m_radar_tracked_data.header.frame_id = m_frameId;
    m_radar_raw_data.returns.clear();
    m_radar_tracked_data.returns.clear();
  } else if (m_sendRaw) {
    m_radar_raw_data.header.stamp = time;
    m_radar_raw_data.header.frame_id = m_frameId;
    m_radar_raw_data.returns.clear();
  } else if (m_sendTracked) {
    m_radar_tracked_data.header.stamp = time;
    m_radar_tracked_data.header.frame_id = m_frameId;
    m_radar_tracked_data.returns.clear();
  }
}

void AinsteinRadarDriverT79::processRadarStop(
  const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr & publisher_raw,
  const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr & publisher_tracked,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received start frame from radar with CAN ID %d", m_canId);
  if (m_sendRaw && m_sendTracked) {
    publisher_raw->publish(m_radar_raw_data);
    publisher_tracked->publish(m_radar_tracked_data);
  } else if (m_sendRaw) {publisher_raw->publish(m_radar_raw_data);} else if (m_sendTracked) {
    publisher_tracked->publish(m_radar_tracked_data);
  }
}

radar_msgs::msg::RadarReturn::UniquePtr AinsteinRadarDriverT79::processRadarOutput(
  const can_msgs::msg::Frame::SharedPtr & msg)
{

  radar_msgs::msg::RadarReturn::UniquePtr radar_tracked_temp =
    std::make_unique<radar_msgs::msg::RadarReturn>();

  radar_tracked_temp->amplitude = msg->data[1];
  radar_tracked_temp->range =
    static_cast<float>(static_cast<int16_t>(((msg->data[2] << 8 ) + msg->data[3])) / 100.0);
  radar_tracked_temp->doppler_velocity =
    static_cast<float>(static_cast<int16_t>((msg->data[4] << 8 ) + msg->data[5]) / 100.0);
  radar_tracked_temp->azimuth =
    static_cast<float>(static_cast<int16_t>(((msg->data[6] << 8 ) + msg->data[7])) / 100.0);
  radar_tracked_temp->elevation = 0.0;

  return radar_tracked_temp;
}
}
