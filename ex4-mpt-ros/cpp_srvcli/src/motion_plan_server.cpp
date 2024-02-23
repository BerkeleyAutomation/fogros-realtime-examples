#include "rclcpp/rclcpp.hpp"
#include "mpt_interfaces/srv/mpt.hpp"                                        // CHANGE
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpt_interfaces/msg/motion_plan_request.hpp"

#include <memory>

void motion_plan(const std::shared_ptr<mpt_interfaces::srv::MPT::Request> request,     // CHANGE
          std::shared_ptr<mpt_interfaces::srv::MPT::Response>       response)  // CHANGE
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nmax_planning_time: %ld" " min_planning_time: %ld" " min_solution_cost: %ld",  // CHANGE
                request->motion_plan_request.max_planning_time, request->motion_plan_request.min_planning_time, request->motion_plan_request.min_solution_cost);                                         // CHANGE
  response->motion_plan.layout.data_offset = 0;
  response->motion_plan.layout.dim.emplace_back();
  response->motion_plan.layout.dim[0].label = "failed (but in a good way)";
  response->motion_plan.layout.dim[0].size = 0;
  response->motion_plan.layout.dim[0].stride = 7;
  std::cout << "Plan output: " << response->motion_plan.layout.dim[0].label << std::endl;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motion_plan_server");   // CHANGE

  rclcpp::Service<mpt_interfaces::srv::MPT>::SharedPtr service =               // CHANGE
    node->create_service<mpt_interfaces::srv::MPT>("mpt",  &motion_plan);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to motion_plan.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}