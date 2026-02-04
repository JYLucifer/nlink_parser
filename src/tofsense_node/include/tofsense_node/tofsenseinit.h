#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <unordered_map>
#include "protocol_extracter/nprotocol_extracter.h"
#include "nlink_parser/msg/tofsense_frame0.hpp"
#include "nlink_parser/msg/tofsense_cascade.hpp"

// 前向声明新的 SerialPort 类（替代 serial::Serial）
class SerialPort;

namespace tofsense
{
    class Init
    {
    public:
        // ==================【关键修改】使用 SerialPort* 替代 serial::Serial* ==================
        explicit Init(
            rclcpp::Node::SharedPtr node,
            NProtocolExtracter *protocol_extraction,
            SerialPort *serial);  // 改为 SerialPort*

    private:
        void InitFrame0(NProtocolExtracter *protocol_extraction);

        rclcpp::Node::SharedPtr node_;
        SerialPort *serial_;  // 改为 SerialPort*

        std::unordered_map<
            NProtocolBase *,
            rclcpp::Publisher<nlink_parser::msg::TofsenseFrame0>::SharedPtr>
            publishers_frame0_;

        rclcpp::Publisher<nlink_parser::msg::TofsenseCascade>::SharedPtr publisher_cascade_;
        std::map<int, nlink_parser::msg::TofsenseFrame0> frame0_map_;

        const int frequency_ = 10;
        bool is_inquire_mode_ = true;

        rclcpp::TimerBase::SharedPtr timer_scan_;
        rclcpp::TimerBase::SharedPtr timer_read_;

        uint8_t node_index_ = 0;
    };

} // namespace tofsense

#endif // TOFSENSEINIT_H