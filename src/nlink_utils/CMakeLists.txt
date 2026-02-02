cmake_minimum_required(VERSION 3.8)
project(nlink_utils LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(nlink_utils STATIC)

target_sources(nlink_utils PRIVATE
  # C++
  src/nlink_protocol.cpp
  src/nutils.cpp
  src/init_serial.cpp
  src/protocol_extracter/nprotocol_base.cpp
  src/protocol_extracter/nprotocol_extracter.cpp
  src/protocol_extracter/linktrack_protocols.cpp

  # C
  src/nlink_unpack/nlink_linktrack_anchorframe0.c
  src/nlink_unpack/nlink_iot_frame0.c
  src/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
  src/nlink_unpack/nlink_linktrack_nodeframe0.c
  src/nlink_unpack/nlink_linktrack_nodeframe1.c
  src/nlink_unpack/nlink_linktrack_nodeframe2.c
  src/nlink_unpack/nlink_linktrack_nodeframe3.c
  src/nlink_unpack/nlink_linktrack_nodeframe4.c
  src/nlink_unpack/nlink_linktrack_nodeframe5.c
  src/nlink_unpack/nlink_linktrack_nodeframe6.c
  src/nlink_unpack/nlink_linktrack_nodeframe7.c
  src/nlink_unpack/nlink_linktrack_tagframe0.c
  src/nlink_unpack/nlink_tofsense_frame0.c
  src/nlink_unpack/nlink_tofsensem_frame0.c
  src/nlink_unpack/nlink_utils.c
)

target_compile_features(nlink_utils PUBLIC
  c_std_11
  cxx_std_17
)

target_compile_options(nlink_utils PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_include_directories(nlink_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  # 修正serial_port.hpp的路径 - 使用绝对路径
  ${CMAKE_SOURCE_DIR}/extern/serial
  # 添加ROS2头文件路径
  ${rclcpp_INCLUDE_DIRS}
  # 添加ROS IDL生成的头文件路径
  ${CMAKE_CURRENT_BINARY_DIR}/../../rosidl_generator_cpp
)

ament_target_dependencies(nlink_utils
  rclcpp
  std_msgs
  rosidl_default_runtime
)

# target_link_libraries(nlink_utils PUBLIC
#   rclcpp
#   std_msgs
#   rosidl_default_runtime
# )

add_dependencies(nlink_utils
  nlink_parser__rosidl_generator_cpp
  nlink_parser__rosidl_typesupport_cpp
)
