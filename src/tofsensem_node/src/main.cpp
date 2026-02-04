#include <rclcpp/rclcpp.hpp>  
#include "tofsensem_node/tofsenseminit.h"  
#include "nlink_utils/init_serial.h"  
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <chrono>
#include <thread>
#include <memory>  

// 声明全局io_context（在init_serial.cpp中定义）
extern asio::io_context g_ioc;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tofsensem_parser");
  
  // ==================【关键修改1】移除旧的serial::Serial声明 ==================
  // 删除这行：serial::Serial serial;
  
  // 创建协议解析器
  auto extracter = std::make_shared<NProtocolExtracter>();
  
  // ==================【关键修改2】使用新的异步initSerial函数 ==================
  auto serial = initSerial(node, [extracter](const std::string& data) {
      // 异步回调：数据到达时自动处理
      extracter->AddNewData(data);
  });
  
  // 创建Init对象
  tofsensem::Init init(extracter, node);
  
  // ==================【关键修改3】在独立线程中运行IO上下文 ==================
  std::thread io_thread([]() {
      g_ioc.run();  // 启动Asio事件循环，处理所有异步操作
  });

  RCLCPP_INFO(node->get_logger(), "TofSenseM节点已启动，串口运行在异步模式");

  rclcpp::Rate loop_rate(1000);
  while (rclcpp::ok()) {
    // ==================【关键修改4】不再需要手动轮询串口！ ==================
    // 删除所有 serial.available() 和 serial.read() 代码
    rclcpp::spin_some(node);  // 只处理ROS事件
    loop_rate.sleep();
  }

  // ==================【关键修改5】正确的资源清理 ==================
  RCLCPP_INFO(node->get_logger(), "正在关闭节点...");
  rclcpp::shutdown();
  g_ioc.stop();        // 停止IO服务
  io_thread.join();    // 等待IO线程结束

  RCLCPP_INFO(node->get_logger(), "节点已安全关闭");
  return EXIT_SUCCESS;
}
