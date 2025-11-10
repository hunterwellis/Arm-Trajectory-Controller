#include "serial_port.hpp"

SerialPort::SerialPort(const std::string &port, unsigned int baudrate)
  : port_(port), baudrate_(baudrate), fd_(-1), open_(false) {}

SerialPort::~SerialPort() {
  close_port();
}

bool SerialPort::open_port() {
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s",
        port_.c_str());
    return false;
  }
  
  // config port
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Error getting termios attributes %s",
        port_.c_str());
    close(fd_);
    return false;
  }

  cfsetospeed(&tty, baudrate_);
  cfsetispeed(&tty, baudrate_);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0; // no signaling chars, no echo
  tty.c_oflag = 0; // no remapping, no delays
  tty.c_cc[VMIN]  = 0; // read doesn't block
  tty.c_cc[VTIME] = 10; // 1 second read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls
  tty.c_cflag &= ~(PARENB | PARODD); // no parity
  tty.c_cflag &= ~CSTOPB; // one stop bit
  tty.c_cflag &= ~CRTSCTS; // no hardware flow control

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Error setting termios attributes %s",
        port_.c_str());
    close(fd_);
    return false;
  }

  open_ = true;
  return true;
}

void SerialPort::close_port() {
  if (open_) {
    close(fd_);
    open_ = false;
  }
  RCLCPP_INFO(logger_, "Closed Serial Port %s", port_.c_str());
};

bool SerialPort::write_data(const std::string &data) {
  if (!open_) {
    RCLCPP_ERROR(logger_, "Port %s is not open", port_.c_str());
    return false;
  }
  ssize_t bytes_written = write(fd_, data.c_str(), data.size());
  RCLCPP_INFO(logger_, "Wrote %lu of %lu bytes to %s:\n\t %s",
      bytes_written, data.size(), port_.c_str(), data.c_str());
  return (data.size() == (unsigned long) bytes_written);
}

std::string SerialPort::read_data() {
  if (!open_) {
    RCLCPP_ERROR(logger_, "Port %s is not open", port_.c_str());
    return "";
  }
  char buffer[256];
  int n = read(fd_, buffer, sizeof(buffer));
  if (n > 0) {
    std::string msg(buffer, n);
    RCLCPP_INFO(logger_, "Recieved %d bytes via %s:\n\t %s",
        n, port_.c_str(), msg.c_str());
    return msg;
  }
  return "";
}
