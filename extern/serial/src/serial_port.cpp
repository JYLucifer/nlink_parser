#include "serial/serial_port.hpp"  // 包含自定义串口头文件

// 构造函数实现
SerialPort::SerialPort(asio::io_context &ioc, const std::string &port_name,
                       int baud_rate, data_ready_cb_f cb, int rx_buf_size)
    : port_(ioc),           // 初始化串口对象，关联IO上下文
      ioc_(ioc),            // 初始化IO上下文引用
      port_name_(port_name), // 初始化串口设备名
      baud_rate_(baud_rate), // 初始化波特率
      data_ready_cb_(cb) {  // 初始化回调函数
      
  port_.open(port_name_);   // 打开指定串口设备
  
  using namespace asio;     // 使用asio命名空间，简化代码
  
  // 设置串口参数
  port_.set_option(serial_port::baud_rate(baud_rate_));           // 设置波特率
  port_.set_option(serial_port::flow_control(serial_port::flow_control::none)); // 无流控制
  port_.set_option(serial_port::parity(serial_port::parity::none)); // 无奇偶校验
  port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one)); // 1位停止位
  port_.set_option(serial_port::character_size(8));              // 8位数据位
  
  read_buf_.resize(rx_buf_size);  // 调整读取缓冲区大小
  start_async_read();             // 开始异步读取循环
}

// 关闭串口实现
void SerialPort::close() {
  asio::error_code ec;      // 错误代码对象，用于捕获异常但不抛出
  
  ec = port_.cancel(ec);    // 取消所有未完成的异步操作
  ec = port_.close(ec);     // 关闭串口连接
  
  if (ec) {                 // 如果发生错误
    throw std::runtime_error(ec.message()); // 抛出运行时错误异常
  }
}

// 写入数据实现（线程安全）
void SerialPort::write(const std::string &data) {
  // 使用asio::post确保线程安全，将写入操作提交到IO上下文的任务队列
  asio::post(ioc_, [this, data]() {  // Lambda表达式捕获this指针和数据
    write_queue_.push_back(data);    // 将数据添加到写入队列尾部
    
    // 如果队列中只有当前数据，开始异步写入
    if (write_queue_.size() == 1) {
      start_async_write();           // 启动写入流程
    }
  });
}

// 开始异步读取实现
void SerialPort::start_async_read() {
  // 异步读取一些数据（不一定会填满整个缓冲区）
  port_.async_read_some(
      asio::buffer(read_buf_),  // 使用缓冲区包装读取缓冲区
      
      // Lambda回调函数，在读取完成时调用
      [this](asio::error_code ec, std::size_t bytes_transferred) {
        if (!ec) {  // 如果没有错误发生
          // 将接收到的数据转换为字符串
          std::string data(read_buf_.data(), bytes_transferred);
          
          // 如果回调函数有效，调用它处理数据
          if (data_ready_cb_) {
            data_ready_cb_(data);  // 执行用户定义的回调函数
          }
        } else {    // 如果发生错误
          close();  // 关闭串口
          throw std::runtime_error(ec.message()); // 抛出异常
        }
        start_async_read();  // 递归调用，继续下一次异步读取
      });
}

// 开始异步写入实现
void SerialPort::start_async_write() {
  if (write_queue_.empty())  // 如果写入队列为空，直接返回
    return;

  // 异步写入队列前部的数据
  asio::async_write(port_, asio::buffer(write_queue_.front()),
                    
                    // 写入完成回调函数
                    [this](asio::error_code ec, std::size_t bytes_transferred) {
                      (void)bytes_transferred;  // 显式忽略未使用参数，避免警告
                      
                      if (!ec) {  // 如果写入成功
                        write_queue_.pop_front();  // 从队列中移除已发送的数据
                      } else {    // 如果写入失败
                        close();  // 关闭串口
                        throw std::runtime_error(ec.message()); // 抛出异常
                      }
                      start_async_write();  // 继续处理队列中的下一个数据
                    });
}