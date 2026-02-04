// ==================【修正】linktrack_aoa_node/src/linktrackaoainit.cpp ==================
#include "linktrack_aoa_node/linktrackaoainit.h"
#include <std_msgs/msg/string.hpp>
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nlink_utils/nutils.h"
#include "protocol_extracter/linktrack_protocols.h"
// 【新增】包含新的SerialPort头文件
#include "serial/serial_port.hpp"

namespace linktrack_aoa
{
std::map<NLinkProtocol*, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
nlink_parser::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser::msg::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;
// 【修改】将静态全局变量类型改为 SerialPort*
static SerialPort* g_serial = nullptr; // 修改类型

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength
{
    // ... 保持不变 ...
};

// 【修改】更新构造函数初始化列表和实现，使用SerialPort*
Init::Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter* protocol_extraction,
    SerialPort* serial) // 参数类型已改
    : node_(node)
{
    g_serial = serial; // 赋值给已修改类型的全局变量
    initDataTransmission();
    initNodeFrame0(protocol_extraction);
    initAoaNodeFrame0(protocol_extraction);
}

// 【修改】数据透传回调函数，适配SerialPort的write接口
static void DTCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (g_serial)
    {
        // SerialPort的write方法直接接受std::string
        g_serial->write(msg->data);
    }
}

void Init::initDataTransmission()
{
    // ... 保持不变，但dt_sub_的回调现在使用上面的DTCallback ...
}

void Init::initNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    // ... 保持不变，注意lambda捕获和数据处理逻辑 ...
}

void Init::initAoaNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    // ... 保持不变，注意lambda捕获和数据处理逻辑 ...
}

} // namespace linktrack_aoa