#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "bench_msgs/srv/add_three_ints.hpp"

#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

using namespace std::chrono_literals;
using AddThreeInts = bench_msgs::srv::AddThreeInts;

class AddThreeIntsClientNode : public rclcpp::Node
{
public:
  AddThreeIntsClientNode() : Node("add_three_ints_client_async")
  {
    this->client_ = this->create_client<AddThreeInts>("add_three_ints");
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    this->request_ = std::make_shared<AddThreeInts::Request>();
  }

  void send_request(int a, int b, int c)
  {
    request_->a = a;
    request_->b = b;
    request_->c = c;

    auto result_future = client_->async_send_request(request_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
    else
    {
      auto response = result_future.get();
      RCLCPP_INFO(this->get_logger(), "Result of add_three_ints: %d + %d + %d = %ld", a, b, c, response->sum);
    }
  }

private:
  rclcpp::Client<AddThreeInts>::SharedPtr client_;
  std::shared_ptr<AddThreeInts::Request> request_;
};

std::string get_hostname()
{
  char buffer[256];
  gethostname(buffer, sizeof(buffer));
  return std::string(buffer);
}

std::string get_host_ip()
{
  struct sockaddr_in sa;
  char ip[INET_ADDRSTRLEN];
  inet_pton(AF_INET, "127.0.0.1", &(sa.sin_addr));
  inet_ntop(AF_INET, &(sa.sin_addr), ip, INET_ADDRSTRLEN);
  return std::string(ip);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<AddThreeIntsClientNode>();
  std::string host_name = get_hostname();
  std::string host_ip = get_host_ip();

  int a = 0;
  int b = 1;
  int c = 2;

  while (rclcpp::ok())
  {
    RCLCPP_INFO(client_node->get_logger(), "I am %s on %s. Sending request %d, %d, %d",
                host_name.c_str(), host_ip.c_str(), a, b, c);
    client_node->send_request(a, b, c);
    a++;
    b++;
    c++;

    // sleep to not overwhelm the service
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    rclcpp::spin_some(client_node);
  }

  rclcpp::shutdown();
  return 0;
}
