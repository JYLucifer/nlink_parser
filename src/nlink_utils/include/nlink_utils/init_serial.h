#ifndef INIT_SERIAL_H
#define INIT_SERIAL_H

#include "serial/serial_port.hpp"           // 使用主管提供的SerialPort类
#include <rclcpp/rclcpp.hpp>        


// 声明全局io_context
extern asio::io_context g_ioc;

/**
 * @brief 初始化串口（基于Asio的SerialPort类）
 * @param serial SerialPort对象指针
 * @param node ROS 2节点共享指针
 */
void initSerial(
  SerialPort *serial,                // 改为SerialPort指针
  const rclcpp::Node::SharedPtr &node
);

/**
 * @brief 运行串口IO服务
 */
void runSerialIO();

/**
 * @brief 停止串口IO服务
 */
void stopSerialIO();

#endif // INIT_SERIAL_Hz