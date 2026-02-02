#include <rclcpp/rclcpp.hpp>

#include "tofsense_node/tofsenseinit.h"
#include "nlink_utils/init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tofsense_parser");
  serial::Serial serial;
  initSerial(&serial,node);

  NProtocolExtracter extracter;
  tofsense::Init init(node, &extracter, &serial);
  rclcpp::Rate rate(1000);
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    if (available_bytes) {
      std::string str_received;
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
