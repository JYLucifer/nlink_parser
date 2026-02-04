#include "nlink_utils/init_serial.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

// 全局io_context定义
asio::io_context g_ioc;

std::unique_ptr<SerialPort> initSerial(
    const rclcpp::Node::SharedPtr &node,
    SerialPort::data_ready_cb_f data_callback)
{
    try {
        std::string port_name;
        int baud_rate;

        // 声明和获取参数
        node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        node->declare_parameter<int>("baud_rate", 921600);
        node->get_parameter("port_name", port_name);
        node->get_parameter("baud_rate", baud_rate);

        RCLCPP_INFO(node->get_logger(), "尝试以 %s,%d 打开串口", 
                   port_name.c_str(), baud_rate);

        // 创建并返回SerialPort智能指针
        auto serial = std::make_unique<SerialPort>(
            g_ioc, 
            port_name, 
            baud_rate,
            data_callback ? data_callback : [node](const std::string& data) {
                // 默认回调：只是记录数据
                RCLCPP_DEBUG(node->get_logger(), "收到 %zu 字节数据", data.size());
            }
        );

        RCLCPP_INFO(node->get_logger(), "串口初始化成功（基于Asio）");
        return serial;

    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "串口初始化失败: %s", e.what());
        throw;
    }
}

void runSerialIO() {
    g_ioc.run();
}

void stopSerialIO() {
    g_ioc.stop();
}