// ==================【修正】linktrack_aoa_node/src/main.cpp ==================
#include "rclcpp/rclcpp.hpp"
#include "nlink_utils/init_serial.h"
#include "linktrack_aoa_node/linktrackaoainit.h"
#include <memory>
#include <thread>

// 声明全局io_context
extern asio::io_context g_ioc;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("linktrack_aoa");

  // 创建协议解析器
  NProtocolExtracter protocol_extraction;
  
  // 【核心修改】使用新的异步initSerial函数
  auto serial = initSerial(node, [&protocol_extraction](const std::string& data) {
      // 异步回调：数据到达时自动处理
      protocol_extraction.AddNewData(data);
  });
  
  // 创建Init对象，传入新的SerialPort指针
  linktrack_aoa::Init aoaInit(node, &protocol_extraction, serial.get());

  // 在独立线程中运行IO上下文
  std::thread io_thread([]() {
      g_ioc.run();
  });

  rclcpp::WallRate loop_rate(1000);
  while (rclcpp::ok()) {
    // 【重要】主循环不再需要手动轮询串口
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // 清理资源
  rclcpp::shutdown();
  g_ioc.stop();
  io_thread.join();
  return 0;
}