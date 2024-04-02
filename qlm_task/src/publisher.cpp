#include <chrono>
#include <memory>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64.hpp" 

using namespace std::chrono_literals;


class Publisher_test : public rclcpp::Node
{
public:
  Publisher_test()
  : Node("publisher_node"), count_(0)
  {
    /*Creating publisher and topic named "input_numbers"*/
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("input_numbers", 10);
    timer_ = this->create_wall_timer(
      1500ms, std::bind(&Publisher_test::timer_callback, this));
  }

private:
  void timer_callback()
  {
    /*Filling data elements*/
    auto message = std_msgs::msg::Float64();
    /*Fault injection for testing*/
    if(count_%10==3 || count_%10==4)
    {
      RCLCPP_INFO(this->get_logger(), "Fault injection sending ZERO");
      message.data = 0;
    }
    else
    {
      /*Generating random number between 0 and 10*/
      message.data = ((double)rand() / RAND_MAX) * 10;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    }
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher_test>());
  rclcpp::shutdown();
  return 0;
}
