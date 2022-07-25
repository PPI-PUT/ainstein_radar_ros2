Ainstein radar driver for ROS2
===========

This is the design document for the `ainstein_radar_ros2` package.


# Purpose / Use cases
This package is a minimal driver responsible for receiving data from Ainstein K-77
and T-79 ZOI radars via the CAN Bus.


# Overview
The package includes drivers for communicating with Ainstein's T-79 and K-77 radars via the CAN bus.
Both radars can be activated using a dedicated launch file. The package uses the ros2_socketcan node
to communicate with the CAN bus, which can be downloaded using the `rosdep` command. To use the ros2_socketcan node, the user should
provide a way to communicate with the CAN Bus using a network interface.

## Inputs / Outputs / API

### Input

### Output

| Name                     | Type                               | Description                                                          |
|--------------------------|------------------------------------|----------------------------------------------------------------------|
| `~/radar/raw_points`     | `radar_msgs::msg::RadarReturn`     | raw points received from the radar                                   |
| `~/radar/tracked_points` | ` radar_msgs::msg::RadarReturn`    | clustered groups of points into a single objects received from radar |


# Param file
## K-77
| Parameter                       | Type   | Description                                                                                      |
|---------------------------------|--------|--------------------------------------------------------------------------------------------------|
| `can_send_topic`                | String | Topic created by ros2_socketcan node to send CAN messages.                                       |
| `can_receive_topic`             | String | Topic created by ros2_socketcan node to receive CAN messages.                                    |
| `radar_send_raw_topic `         | String | Topic created by the driver to publish raw radar messages.                                       |
| `radar_send_tracked_topic`      | String | Topic created by the driver to publish tracked radar messages.                                   |
| `send_raw`                      | bool   | Publish radar raw messages. K-77 radar can publish one message "raw" or "tracked" at a time.     |
| `send_tracked`                  | bool   | Publish radar tracked messages. K-77 radar can publish one message "raw" or "tracked" at a time. |
| `antenna_type`                  | bool   | Antenna type selection. Long antenna - true, short antenna - false.                              |
| `frame_id`                      | String | TF frame of radar data, default `radar`                                                 |


## T-79
| Parameter                  | Type   | Description                                                                   |
|----------------------------|--------|-------------------------------------------------------------------------------|
| `can_send_topic`           | String | Topic created by ros2_socketcan node to send CAN messages.                    |
| `can_receive_topic`        | String | Topic created by ros2_socketcan node to receive CAN messages.                 |
| `radar_send_raw_topic `    | String | Topic created by the driver to publish raw radar messages.                    |
| `radar_send_tracked_topic` | String | Topic created by the driver to publish tracked radar messages.                |
| `send_raw`                 | bool   | Publish radar raw messages. T-79 radar can publish both messages at a time.   |
| `send_tracked`             | bool   | Publish tracked raw messages. T-79 radar can publish both messages at a time. |
| `radar_id`                 | int    | Id of the radar in the CAN Bus, default 0.                                    |
| `frame_id`                 | String | TF frame of radar data, default `radar`                                       |
| `range_min`                | double | Minimum range value. <0.5, 80.0>                                              |
| `range_max`                | double | Maximum range value. <0.5, 80.0>                                              |
| `azimuth_min`              | double | Minimum(left side) azimuth value. <0, 60>                                     |
| `azimuth_max`              | double | Maximum(right side) azimuth value. <0, 60>                                    |



## Existing Solution
Currently, there are no available ROS2 drivers for T-79 and K-77 radars. Aeinstein Radar
provides only ROS1 drivers, which can be found [here](https://github.com/AinsteinAI/ainstein_radar).

