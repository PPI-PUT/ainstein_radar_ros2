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

#ifndef AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_HPP_
#define AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_HPP_

#include "ainstein_radar_ros2/visibility_control.hpp"
#include "can_msgs/msg/frame.hpp"
#include "radar_msgs/msg/radar_scan.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <chrono>

namespace ainstein_radar
{
    class AINSTEIN_RADAR_ROS2_PUBLIC AinsteinRadarDriver
    {
    public:

        virtual ~AinsteinRadarDriver( )= default;

        // Radar requests
        virtual void startRadar(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                        const rclcpp::Time& time, const rclcpp::Logger& logger) = 0;
        virtual void stopRadar(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                       const rclcpp::Time& time) = 0;
        virtual void updateRadarFOV(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                       const rclcpp::Time& time) = 0;

        // Radar callback
        virtual void msgCallback(const can_msgs::msg::Frame::SharedPtr &msg,
                         const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_raw,
                         const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_tracked,
                         const rclcpp::Time &time,
                         const rclcpp::Logger& logger) = 0;

        //  Process functions
        virtual void processRadarReturn(const uint8_t& command, const rclcpp::Logger& logger) const = 0;
        virtual void processRadarStart(const rclcpp::Time& time, const rclcpp::Logger& logger) = 0;
        virtual void processRadarStop(const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_raw,
                              const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_tracked,
                              const rclcpp::Logger& logger) = 0;
        virtual radar_msgs::msg::RadarReturn::UniquePtr processRadarOutput(const can_msgs::msg::Frame::SharedPtr &msg) = 0;


    protected:

        can_msgs::msg::Frame m_can_frame_msg;
        radar_msgs::msg::RadarScan m_radar_raw_data;
        radar_msgs::msg::RadarScan m_radar_tracked_data;
        bool m_sendRaw = false;
        bool m_sendTracked = false;
        std::string m_frameId;

    };



}  // namespace ainstein_radar

#endif  // AINSTEIN_RADAR_ROS2__AINSTEIN_RADAR_ROS2_HPP_
