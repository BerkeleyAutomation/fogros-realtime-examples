#ifndef ADD_THREE_INTS_SERVICE_NODE_HPP
#define ADD_THREE_INTS_SERVICE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "shared_srvs/srv/add_three_ints.hpp"

using AddThreeInts = shared_srvs::srv::AddThreeInts;

class AddThreeIntsServiceNode : public rclcpp::Node {
public:
    AddThreeIntsServiceNode();

private:
    void add_three_ints_callback(const std::shared_ptr<AddThreeInts::Request> request,
                                 std::shared_ptr<AddThreeInts::Response> response);
    rclcpp::Service<AddThreeInts>::SharedPtr service_;
    std::string host_name_;
};

#endif // ADD_THREE_INTS_SERVICE_NODE_HPP
