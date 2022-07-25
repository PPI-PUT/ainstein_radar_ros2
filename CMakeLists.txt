# Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(ainstein_radar_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/ainstein_radar_K-77.cpp
    src/ainstein_radar_T-79.cpp
)

# generate component node library
ament_auto_add_library(ainstein_radar_ros2_T-79_node SHARED
    src/ainstein_radar_node_T-79.cpp
)

target_link_libraries(ainstein_radar_ros2_T-79_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(ainstein_radar_ros2_T-79_node
  PLUGIN "ainstein_radar::AinsteinRadarNodeT79"
  EXECUTABLE ainstein_radar_ros2_T-79_node_exe
)

# generate component node library
ament_auto_add_library(ainstein_radar_ros2_K-77_node SHARED
    src/ainstein_radar_node_K-77.cpp
    )

target_link_libraries(ainstein_radar_ros2_K-77_node
    ${PROJECT_NAME}
    )

rclcpp_components_register_node(ainstein_radar_ros2_K-77_node
    PLUGIN "ainstein_radar::AinsteinRadarNodeK77"
    EXECUTABLE ainstein_radar_ros2_K-77_node_exe
    )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
#  set(TEST_SOURCES test/test_ainstein_radar_driver.cpp)
#  set(TEST_AINSTEIN_RADAR_ROS2_EXE test_ainstein_radar_driver)
#  ament_add_gtest(${TEST_AINSTEIN_RADAR_ROS2_EXE} ${TEST_SOURCES})
#  target_link_libraries(${TEST_AINSTEIN_RADAR_ROS2_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
