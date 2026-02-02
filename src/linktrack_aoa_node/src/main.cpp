#include "rclcpp/rclcpp.hpp"
#include "nlink_utils/init_serial.h"
#include "linktrack_aoa_node/linktrackaoainit.h"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("linktrack_aoa");

  serial::Serial serial;
  initSerial(&serial,node);

  NProtocolExtracter protocol_extraction;
  linktrack_aoa::Init aoaInit(node, &protocol_extraction, &serial);

  rclcpp::WallRate loop_rate(1000);

  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    if (available_bytes) {
      std::string str_received;
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
