#pragma once  // 预处理指令，确保头文件只被编译一次，防止重复包含

#include <asio.hpp>        // 包含Asio库，提供异步I/O功能
#include <deque>           // 包含双端队列容器，用于写入队列管理
#include <functional>      // 包含函数对象库，用于回调函数
#include <string>          // 包含字符串库

class SerialPort {
public:
  // 定义数据到达回调函数类型，使用std::function包装，参数为接收到的数据字符串
  using data_ready_cb_f = std::function<void(const std::string &data)>;
  
  // 构造函数：初始化串口对象
  // ioc - Asio的IO上下文，用于管理所有异步操作
  // port_name - 串口设备名称（如"/dev/ttyUSB0"或"COM1"）
  // baud_rate - 波特率（如9600、115200）
  // cb - 数据到达回调函数
  // rx_buf_size - 接收缓冲区大小，默认为4096字节
  SerialPort(asio::io_context &ioc, const std::string &port_name, int baud_rate,
             data_ready_cb_f cb, int rx_buf_size = 4096);

  // 析构函数：对象销毁时自动关闭串口
  ~SerialPort() { close(); }

  // 向串口写入数据（线程安全）
  // data - 要发送的数据字符串
  void write(const std::string &data);

private:
  // 关闭串口连接
  void close();
  
  // 开始异步读取操作
  void start_async_read();
  
  // 开始异步写入操作
  void start_async_write();

  // 成员变量
  asio::serial_port port_;      // Asio串口对象，封装底层串口操作
  asio::io_context &ioc_;       // IO上下文引用，用于调度异步操作
  std::string port_name_;       // 串口设备名称
  int baud_rate_;               // 波特率
  data_ready_cb_f data_ready_cb_; // 数据到达回调函数
  std::string read_buf_;        // 读取数据缓冲区
  std::deque<std::string> write_queue_; // 写入队列，支持多线程安全写入
};