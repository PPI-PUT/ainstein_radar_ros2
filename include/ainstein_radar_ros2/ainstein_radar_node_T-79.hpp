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

#ifndef AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_NODE_HPP_
#define AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_NODE_HPP_

#include "ainstein_radar_T-79.hpp"

namespace ainstein_radar
{

using AinsteinRadarDriverT79Ptr = std::unique_ptr<ainstein_radar::AinsteinRadarDriverT79>;

/// \class AinsteinRadarRos2Node
/// \brief ROS 2 Node.
class AINSTEIN_RADAR_ROS2_PUBLIC AinsteinRadarNodeT79 : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit AinsteinRadarNodeT79(const rclcpp::NodeOptions & options);

  ~AinsteinRadarNodeT79();

  AinsteinRadarDriverT79Ptr m_ainstein_radar{nullptr};

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_can_publisher;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_can_subscriber;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr m_radar_raw_publisher;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr m_radar_tracked_publisher;

  void topic_callback(const can_msgs::msg::Frame::SharedPtr & msg) const;
  void create_publishers(
    const std::string & can_send_topic, const std::string & can_recive_topic,
    const std::string & radar_send_raw_topic, const std::string & radar_send_tracked_topic);

  std::string can_send_topic;
  std::string can_receive_topic;
  std::string radar_send_raw_topic;
  std::string radar_send_tracked_topic;
  bool send_raw;
  bool send_tracked;
  int radar_id;
  std::string frame_id;
  double range_min;
  double range_max;
  double azimuth_min;
  double azimuth_max;
};
}  // namespace ainstein_radar

#endif  // AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_NODE_HPP_
