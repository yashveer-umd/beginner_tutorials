// Copyright [2022] <Yashveer jain (yashveer@umd.edu)>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief Subscriber class Node for topic "topic"
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("subscriber") {
    try{
      subscription_ = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Subscriber");  
     }
    catch(...){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error encountered at time of initialization!!");
      RCLCPP_FATAL_STREAM(this->get_logger(), "Subscriber may not work!!");
    }
  }

 private:
  /**
   * @brief callback to handle the message transmit from the publisher on the topic
   * 
   * @param msg 
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);
  return 0;
}

