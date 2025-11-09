#include "serial_port/serial_port.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>

class SerialPortNode : public rclcpp::Node {
  public:
    SerialPortNode()
      : Node("serial_port") {

        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baudrate", 115200);

        std::string port = get_parameter("port").as_string();
        int baudrate = get_parameter("baudrate").as_int();

        serial_ = std::make_unique<SerialPort>(port, baudrate);
        if (!serial_->open_port()) {
          RCLCPP_ERROR(get_logger(),
              "Failed to open serial port: %s",
              port.c_str());
          return;
        }

        publisher_ = create_publisher<std_msgs::msg::String>("serial_rx", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
            [this]() { serial_rx(); }
        );

        subscriber_ = create_subscription<std_msgs::msg::String>(
            "serial_tx", 10,
            [this] (const std_msgs::msg::String::SharedPtr msg) {
              serial_->write_data(msg->data);
            }
        );
      }

  private:
    std::unique_ptr<SerialPort> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    void serial_rx() {
      auto data = serial_->read_data();
      if (!data.empty()) {
        std_msgs::msg::String msg;
        msg.data = data;
        publisher_->publish(msg);
      }
    }
};

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialPortNode>());
  rclcpp::shutdown();
  
  return 0;
}
