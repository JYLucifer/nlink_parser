cmake_minimum_required(VERSION 3.14)
project(serial LANGUAGES CXX)

# ========== asio ==========
include(FetchContent)

FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-34-2
)

FetchContent_MakeAvailable(asio)

# asio 是 header-only
add_library(asio INTERFACE)


target_include_directories(asio INTERFACE
  ${asio_SOURCE_DIR}/asio/include
)

# ========== serial 库 ==========
add_library(serial
  serial_port.cpp
)

target_include_directories(serial
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(serial
  PUBLIC asio
)

target_compile_features(serial PUBLIC cxx_std_17)
