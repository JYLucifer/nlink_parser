#ifndef INIT_SERIAL_H
#define INIT_SERIAL_H

#include "serial/serial_port.hpp"  // 使用自定义的SerialPort类
#include <rclcpp/rclcpp.hpp>
#include <memory>

// 前向声明
namespace asio {
    class io_context;
}
extern asio::io_context g_ioc;  // 全局io_context声明

/**
 * @brief 初始化串口（基于Asio的SerialPort）
 * @param serial SerialPort对象的智能指针
 * @param node ROS2节点共享指针
 * @param data_callback 数据到达回调函数
 */
std::unique_ptr<SerialPort> initSerial(
    const rclcpp::Node::SharedPtr &node,
    SerialPort::data_ready_cb_f data_callback = nullptr
);

/**
 * @brief 运行串口IO服务
 */
void runSerialIO();

/**
 * @brief 停止串口IO服务
 */
void stopSerialIO();

#endif // INIT_SERIAL_H