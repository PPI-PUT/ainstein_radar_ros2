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

#include "ainstein_radar_template.hpp"

namespace ainstein_radar
{
    class AinsteinRadarDriverK77 : public AinsteinRadarDriver
    {
    public:
        AinsteinRadarDriverK77(bool sendRaw, bool sendTracked, bool antennaType, std::string frameId);


        // Radar requests
        void startRadar(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                        const rclcpp::Time& time, const rclcpp::Logger& logger) override;
        void stopRadar(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                       const rclcpp::Time& time) override;
        void updateRadarFOV(const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr& publisher,
                            const rclcpp::Time& time) override;

        // Radar callback
        void msgCallback(const can_msgs::msg::Frame::SharedPtr &msg,
                         const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_raw,
                         const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_tracked,
                         const rclcpp::Time &time,
                         const rclcpp::Logger& logger) override;

        //  Process functions
        void processRadarReturn(const uint8_t& command, const rclcpp::Logger& logger) const override;
        void processRadarStart(const rclcpp::Time& time, const rclcpp::Logger& logger) override;
        void processRadarStop(const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_raw,
                              const rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr& publisher_tracked,
                              const rclcpp::Logger& logger) override;
        radar_msgs::msg::RadarReturn::UniquePtr processRadarOutput(const can_msgs::msg::Frame::SharedPtr &msg) override;


        // Radar command message
        static const uint16_t RADAR_COMMAND = 0x100;
        static const uint16_t RADAR_SELECT_ANTENNA = 0x04;
        static const uint16_t RADAR_SELECT_ANTENNA_ZERO = 0x00;
        static const uint16_t RADAR_SELECT_ANTENNA_FOUR = 0x00;
        static const uint16_t RADAR_SELECT_ANTENNA_SHORT = 0x3F;
        static const uint16_t RADAR_SELECT_ANTENNA_LONG = 0x5F;
        static const uint16_t RADAR_RUN_INFINITE= 0x00;
//        static const uint16_t RADAR_RUN_ONCE = 0x01;
        static const uint16_t RADAR_START = 0x01;
        static const uint16_t RADAR_STOP = 0x02;
        static const uint16_t RADAR_SEND_TRACKED = 0x01;
        static const uint16_t RADAR_SEND_RAW = 0x02;

        // Radar output message
        static const uint16_t RADAR_RETURN_COMMAND = 0x101;
        static const uint16_t RADAR_START_FRAME = 0x420;
        static const uint16_t RADAR_STOP_FRAME = 0x480;
        static const uint16_t RADAR_RAW_TARGET = 0x4A0;
        static const uint16_t RADAR_TRACKED_TARGET = 0x490;

        static const uint16_t RESERVED = 0xff;

    private:
        // True if long, False if short
        bool m_antenaType = false;
        static const int m_CAN_ID = 0;

    };



}  // namespace ainstein_radar
