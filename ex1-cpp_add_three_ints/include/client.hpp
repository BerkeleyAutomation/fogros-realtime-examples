#ifndef ADD_THREE_INTS_ASYNC_CLIENT_NODE_HPP
#define ADD_THREE_INTS_ASYNC_CLIENT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "shared_srvs/srv/add_three_ints.hpp"

using AddThreeInts = shared_srvs::srv::AddThreeInts;

class AddThreeIntsClientNode : public rclcpp::Node {
public:
    AddThreeIntsClientNode();
    void send_request(int a, int b, int c);

private:
    rclcpp::Client<AddThreeInts>::SharedPtr client_;
    std::shared_ptr<AddThreeInts::Request> request_;
};

#endif // ADD_THREE_INTS_ASYNC_CLIENT_NODE_HPP
