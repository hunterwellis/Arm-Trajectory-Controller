#include "serial_port/serial_port.hpp"

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmInterface : public rclcpp::Node {
  public:
    ArmInterface() : Node("arm_interface") {
      this->declare_parameter("port", "/dev/ttyACM0");
      this->declare_parameter("baudrate", 115200);

      std::string port = get_parameter("port").as_string();
      int baudrate = get_parameter("baudrate").as_int();

      serial_ = std::make_unique<SerialPort>(port, baudrate);

      // serial tx channel
      serial_publisher_ = create_publisher<std_msgs::msg::String>("serial_tx", 10);

      // subscribe to rx messages
      serial_subscriber_ = create_subscription<std_msgs::msg::String>(
        "serial_rx", 10,
        [this] (const std_msgs::msg::String::SharedPtr msg) {
          serial_->write_data(msg->data);
        }
      );

      // subscribe to joint_state messages
      joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
          "robot_state_publisher", 10,
          [this] (const sensor_msgs::msg::JointState msg) {
            std_msgs::msg::String str = serial_format(msg);
            serial_publisher_->publish(str);
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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

    void serial_rx() {
      auto data = serial_->read_data();
      if (!data.empty()) {
        std_msgs::msg::String msg;
        msg.data = data;
        serial_publisher_->publish(msg);
      }
    }

    std_msgs::msg::String serial_format(sensor_msgs::msg::JointState msg) const {
      std_msgs::msg::String str_msg;
      return str_msg;
    }
    
    // rclcpp::Logger logger_ = rclcpp::get_logger("arm_interface");
};

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmInterface>());
  rclcpp::shutdown();

  return 0;
}
