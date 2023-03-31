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

#include "ainstein_radar_ros2/ainstein_radar_node_K-77.hpp"
#include <string>
#include <sstream>
#include<unistd.h>


namespace ainstein_radar
{
    AinsteinRadarNodeK77::AinsteinRadarNodeK77(const rclcpp::NodeOptions & options)
      : Node("ainstein_radar_ros2_driver_K77", options)
    {
      declare_parameter("can_send_topic", "");
      declare_parameter("can_receive_topic", "");
      declare_parameter("radar_send_raw_topic", "");
      declare_parameter("radar_send_tracked_topic", "");
      declare_parameter("send_raw", true);
      declare_parameter("send_tracked", true);
      declare_parameter("antenna_type", false);
      declare_parameter("frame_id", "");

      get_parameter("can_send_topic", can_send_topic);
      get_parameter("can_receive_topic", can_receive_topic);
      get_parameter("radar_send_raw_topic", radar_send_raw_topic);
      get_parameter("radar_send_tracked_topic", radar_send_tracked_topic);
      get_parameter("send_raw", send_raw);
      get_parameter("send_tracked", send_tracked);
      get_parameter("antenna_type", antenna_type);
      get_parameter("frame_id", frame_id);


      m_ainstein_radar = std::make_unique<ainstein_radar::AinsteinRadarDriverK77>(send_raw,
                                                                                  send_tracked,
                                                                                  antenna_type,
                                                                                  frame_id);

      usleep(1000000);
      this->create_publishers(can_send_topic, can_receive_topic, radar_send_raw_topic, radar_send_tracked_topic);

      m_ainstein_radar->startRadar(m_can_publisher, this->now(), this->get_logger());
      m_ainstein_radar->updateRadarFOV(m_can_publisher, this->now());

      RCLCPP_INFO(this->get_logger(), "Start message was sent");
    }

    AinsteinRadarNodeK77::~AinsteinRadarNodeK77(){
      m_ainstein_radar->stopRadar(m_can_publisher, this->now());
      RCLCPP_INFO(this->get_logger(), "Radar destructor complited operation");
    }

    void AinsteinRadarNodeK77::create_publishers(const std::string& can_send_topic,
                                                 const std::string& can_recive_topic,
                                                 const std::string& radar_send_raw_topic,
                                                 const std::string& radar_send_tracked_topic) {
      m_can_publisher = this->create_publisher<can_msgs::msg::Frame>(can_send_topic, 1);
      m_radar_raw_publisher = this->create_publisher<radar_msgs::msg::RadarScan>(radar_send_raw_topic, 1);
      m_radar_tracked_publisher = this->create_publisher<radar_msgs::msg::RadarScan>(radar_send_tracked_topic, 1);

      m_can_subscriber =
              this->create_subscription<can_msgs::msg::Frame>(
                can_recive_topic, 10,
                [this, can_recive_topic](
                  const can_msgs::msg::Frame::SharedPtr msg) {
                    this->m_ainstein_radar->msgCallback(msg,
                                                        m_radar_raw_publisher,
                                                        m_radar_tracked_publisher,
                                                        this->now(),
                                                        this->get_logger());
                });
    }

}  // namespace ainstein_radar

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(ainstein_radar::AinsteinRadarNodeK77)
