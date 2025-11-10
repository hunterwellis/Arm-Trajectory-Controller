#pragma once

#include <string>
#include <termios.h>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"

class SerialPort {
  public:
    SerialPort(const std::string &port, unsigned int baudrate = 9600);
    ~SerialPort();

    bool open_port();
    void close_port();
    bool is_open() const { return open_; };

    bool write_data(const std::string &data);
    std::string read_data();

  private:
    std::string port_;
    unsigned int baudrate_;
    int fd_;
    bool open_;

    bool configure_port();

    rclcpp::Logger logger_ = rclcpp::get_logger("serial_port");
};
