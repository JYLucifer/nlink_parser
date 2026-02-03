#include "nlink_utils/init_serial.h"   
#include <rclcpp/rclcpp.hpp>           
#include <string>                     


#include "serial/serial_port.hpp"

// 全局io_context，用于所有串口实例
asio::io_context g_ioc;

void initSerial(
  SerialPort *serial,  // 改为SerialPort指针
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

    // 使用SerialPort的构造函数，它会自动完成端口配置
    // 注意：SerialPort在构造时就会尝试打开串口
    *serial = SerialPort(g_ioc, port_name, baud_rate,
        [node](const std::string& data) {
            // 数据接收回调函数
            // 这里可以处理接收到的串口数据
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

// 添加io_context运行函数（需要在主线程中调用）
void runSerialIO() {
    // 运行io_context来处理异步操作
    g_ioc.run();
}

// 停止串口IO（在程序退出时调用）
void stopSerialIO() {
    g_ioc.stop();
}