// Copyright [2022] <Yashveer jain (yashveer@umd.edu)>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HNADLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief Publisher class which has publisher Node and 
 *         server node which handle the change in the string
 *
 */
class Publisher : public rclcpp::Node {
 public:
  Publisher(char *transformations[]) : Node("publisher") {
    try {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
          "This parameter is updated by given argument in launch file and is "
          "used by publisher and subscriber node!";
      this->declare_parameter("frequency", 2, param_desc);
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(), "Param value : " << frequency);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(int((1000 / frequency))),
          std::bind(&Publisher::timer_callback, this));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher");
      server = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&Publisher::changeString, this, std::placeholders::_1,
                    std::placeholders::_2));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Server");

      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Publish static transforms once at startup
      this->make_transforms(transformations);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the transform");
    } catch (...) {
      RCLCPP_ERROR_STREAM(this->get_logger(), 
      "Error encountered at time of initialization!!"); 
      RCLCPP_FATAL_STREAM(this->get_logger(), 
      "Publisher may not work!!");
    }
  } 

  /**
   * @brief Callback function for processing server request and generate
   * response
   *
   * @param request : string
   * @param response :string
   */
  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    response->op = request->ip + " Edited by service";

    server_resp_message = response->op;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ninput: '%s'",
                request->ip.c_str());  //+
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'",
                response->op.c_str());
  }

 private:
  /**
   * @brief timer callback publish the message on topic after certain interval
   * of time
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_resp_message;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Able to insert message data ");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  void make_transforms(char * transformation[])
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr server;
  std::string server_resp_message = "Hi Terpians!! This is Yashveer";
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  
  // Obtain parameters from command line arguments
  if (argc < 8) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"), "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw %d",argc);
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("rclcpp"), "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw"<<argc<<" "<<argv);
    return 1;
  }
  
  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Your static turtle name cannot be world");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Publisher>(argv);
  rclcpp::spin(node);
  rclcpp::shutdown();

  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);
  return 0;
}

