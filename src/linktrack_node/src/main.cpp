#include <rclcpp/rclcpp.hpp>
#include "linktrack_node/linktrackinit.h"
#include "nlink_utils/init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

#include <iomanip>
#include <iostream>

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
  
  serial::Serial serial;
  initSerial(&serial,node);
  
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(node, &protocol_extraction, &serial);

  rclcpp::Rate loop_rate(1000);
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    rclcpp::spin_some(node);  
    loop_rate.sleep();
  }

  rclcpp::shutdown(); 
  return EXIT_SUCCESS;
}
