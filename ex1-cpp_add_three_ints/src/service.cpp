#include "service.hpp"
#include <random>
#include <string>

AddThreeIntsServiceNode::AddThreeIntsServiceNode() : Node("add_three_ints")
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(0, 1000);

  host_name_ = std::string("host_") + std::to_string(distrib(gen));

  RCLCPP_INFO(this->get_logger(), "I am %s. Starting service /add_three_ints.", host_name_.c_str());

  service_ = this->create_service<AddThreeInts>(
      "add_three_ints",
      std::bind(&AddThreeIntsServiceNode::add_three_ints_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void AddThreeIntsServiceNode::add_three_ints_callback(const std::shared_ptr<AddThreeInts::Request> request,
                                                      std::shared_ptr<AddThreeInts::Response> response)
{
  response->sum = request->a + request->b + request->c;
  response->server_name = host_name_;

  RCLCPP_INFO(this->get_logger(), "Incoming request: a: %d, b: %d, c: %d. Sending: %d",
              request->a, request->b, request->c, response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddThreeIntsServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
