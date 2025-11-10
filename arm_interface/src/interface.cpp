#include "serial_port/serial_port.hpp"

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmInterface : public rclcpp::Node {
  public:
    ArmInterface() : Node("arm_interface") {
      declare_parameter<std::string>("port", "/dev/ttyACM0");
      declare_parameter<int>("baudrate", 115200);

      std::string port = get_parameter("port").as_string();
      int baudrate = get_parameter("baudrate").as_int();

      serial_ = std::make_unique<SerialPort>(port, baudrate);

      subscriber_ = create_subscription<std_msgs::msg::String>(
        "serial_tx", 10,
        [this] (const std_msgs::msg::String::SharedPtr msg) {
          serial_->write_data(msg->data);
        }
      );
    }

    // bool send_joint_state(sensor_msgs::msg::JointState msg) {
    //   return false;
    // }
    //
    // bool send_trajectory();

  private:
    std::unique_ptr<SerialPort> serial_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    // std::string js_to_string(sensor_msgs::msg::JointState msg);
};

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmInterface>());
  rclcpp::shutdown();

  return 0;
}
