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

#include <ainstein_radar_ros2/ainstein_radar_driver.hpp>

#include <rclcpp/rclcpp.hpp>


namespace ainstein_radar
{

using AinsteinRadarDriverPtr = std::unique_ptr<ainstein_radar::AinsteinRadarDriver>;

class AINSTEIN_RADAR_ROS2_PUBLIC AinsteinRadarDriverNode : public rclcpp::Node
{
public:
  explicit AinsteinRadarDriverNode(const rclcpp::NodeOptions & options);
  AinsteinRadarDriverPtr m_ainstein_radar{nullptr};
private:
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr m_radar_publisher;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_subscribers;
  rclcpp::TimerBase::SharedPtr m_timer;
  void onTimer();

};
}  // namespace ainstein_radar

#endif  // AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_NODE_HPP_
