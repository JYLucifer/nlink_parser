#include <rclcpp/rclcpp.hpp>  
#include "tofsensem_node/tofsenseminit.h"  
#include "nlink_utils/init_serial.h"  
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <chrono>
#include <thread>
#include <memory>  

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("tofsensem_parser");
  
  serial::Serial serial;
  initSerial(&serial,node);
  
  auto extracter = std::make_shared<NProtocolExtracter>();
  tofsensem::Init init(extracter,node);
  
  rclcpp::Rate loop_rate(1000);
  
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      extracter->AddNewData(str_received);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}