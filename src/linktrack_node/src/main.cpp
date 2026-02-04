// ==================【修正】linktrack_node/src/main.cpp ==================
#include <rclcpp/rclcpp.hpp>
#include "linktrack_node/linktrackinit.h"
#include "nlink_utils/init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

// 声明全局io_context
extern asio::io_context g_ioc;

void printHexData(const std::string &data) {
  if (!data.empty()) {
    std::cout << "data received: ";
    for (size_t i = 0; i < data.size(); ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << int(uint8_t(data.at(i))) << " ";
    }
    std::cout << std::endl;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("linktrack_parser");
  
  // 创建协议解析器
  NProtocolExtracter protocol_extraction;
  
  // 【核心修改】使用新的异步initSerial函数
  auto serial = initSerial(node, [&protocol_extraction](const std::string& data) {
      // 异步回调：数据到达时自动处理
      protocol_extraction.AddNewData(data);
      // 可选：打印接收到的数据（十六进制格式）
      printHexData(data);
  });
  
  // 创建Init对象，传入新的SerialPort指针
  linktrack::Init init(node, &protocol_extraction, serial.get());
  
  // 在独立线程中运行IO上下文
  std::thread io_thread([]() {
      g_ioc.run();
  });

  RCLCPP_INFO(node->get_logger(), "LinkTrack节点已启动，串口运行在异步模式");

  rclcpp::Rate loop_rate(1000);
  while (rclcpp::ok()) {
    // 【重要】主循环不再需要手动轮询串口
    rclcpp::spin_some(node);  
    loop_rate.sleep();
  }

  // 清理资源
  RCLCPP_INFO(node->get_logger(), "正在关闭LinkTrack节点...");
  rclcpp::shutdown();
  g_ioc.stop();
  io_thread.join();

  RCLCPP_INFO(node->get_logger(), "LinkTrack节点已安全关闭");
  return EXIT_SUCCESS;
}