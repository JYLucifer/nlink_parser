// ==================【修正】使用指针传递 ==================
#include "nlink_utils/init_serial.h"   
#include <rclcpp/rclcpp.hpp>           
#include <string>                     
#include "serial/serial_port.hpp"

// 全局io_context，用于所有串口实例
asio::io_context g_ioc;

void initSerial(
  SerialPort *serial,  // SerialPort指针
  const rclcpp::Node::SharedPtr &node
) {
  try {
    std::string port_name;   
    int baud_rate;           

    node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    node->declare_parameter<int>("baud_rate", 921600);

    node->get_parameter("port_name", port_name);
    node->get_parameter("baud_rate", baud_rate);

    RCLCPP_INFO(
      node->get_logger(),
      "try to open serial port with %s,%d",
      port_name.c_str(),
      baud_rate
    );

    // ==================【关键修改】使用placement new在已有内存上构造对象 ==================
    // 而不是尝试赋值
    new(serial) SerialPort(g_ioc, port_name, baud_rate,
        [node](const std::string& data) {
            // 数据接收回调函数
            RCLCPP_DEBUG(node->get_logger(), "Received %zu bytes", data.size());
        }
    );

    RCLCPP_INFO(
      node->get_logger(),
      "Serial port initialized successfully with Asio"
    );

  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Serial port initialization failed: %s",
      e.what()
    );
    std::exit(EXIT_FAILURE);
  }
}

void runSerialIO() {
    g_ioc.run();
}

void stopSerialIO() {
    g_ioc.stop();
}
