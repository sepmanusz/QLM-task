#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

class MyNode : public rclcpp::Node
{
public:
  /*Creating a SR node*/
  MyNode()
  : Node("sender_receiver"),received_count_(0)
  { 
    /*Creating subscriber and subscribe to input_numbers topic*/
    subscription_ = this->create_subscription<std_msgs::msg::Float64>
    (
      "input_numbers",
      10,
      std::bind(&MyNode::topic_callback, this, _1)
    );
    /*Creating publisher and topic named division_result*/
    result_publisher_ = this->create_publisher<std_msgs::msg::Float64>("division_result", 10);
  }

private:
  /* Subscriber callback function*/
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    /*Count the incoming datas*/
    received_count_++;
    /*Filling the buffer with the incoming datas*/
    buffer_.push_back(msg->data);
    /*If the counter is even we need to call the divider function*/
    if (received_count_ % 2 == 0)
    {
        divide_and_publish();
    }
    /*Error handling -> Reset the buffer to prevent overflow*/
    if (buffer_.size() >= MAX_BUFFER_SIZE)
    {
        clear_buffer();
    }
  }
  /*Divider and publisher fuction*/
  void divide_and_publish()
  {/*Read the last 2 elemnts of the buffer*/
    double num1 = buffer_[buffer_.size() - 2];
    double num2 = buffer_[buffer_.size() - 1];
    /*Error handling-> if the divider is zero we skip the division and log the warning messeage*/
    if (num2 == 0)
    {
      RCLCPP_WARN(get_logger(), "Division by zero occurred! Skipping division...");
      return;
    }
    /*Divide the numbers and log the result*/
    double result = num1 / num2;
    RCLCPP_INFO(get_logger(), "Num1: %f Num2: %f Result of division: %f", num1,num2,result);
    /*Publish the calculated value to the topic*/    
    auto result_msg = std_msgs::msg::Float64();
    result_msg.data = result;
    result_publisher_->publish(result_msg);
    
  }
  /*Buffer reset function*/
  void clear_buffer()
  {
      buffer_.clear();
      received_count_ = 0;
      RCLCPP_INFO(get_logger(), "Buffer cleared");
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr result_publisher_;
  /*Added variables, you can change the buffer size to a specific number*/
  std::vector<double> buffer_;
  size_t received_count_;
  const size_t MAX_BUFFER_SIZE = 20; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = (std::make_shared<MyNode>());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
