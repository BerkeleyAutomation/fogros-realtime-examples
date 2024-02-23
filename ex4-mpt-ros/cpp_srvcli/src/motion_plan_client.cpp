#include "rclcpp/rclcpp.hpp"
#include "mpt_interfaces/srv/mpt.hpp"                                        // CHANGE
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpt_interfaces/msg/motion_plan_request.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>


using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motion_plan_client");  // CHANGE
  rclcpp::Client<mpt_interfaces::srv::MPT>::SharedPtr client =                // CHANGE
    node->create_client<mpt_interfaces::srv::MPT>("mpt");          // CHANGE

  auto request = std::make_shared<mpt_interfaces::srv::MPT::Request>();       // CHANGE
  request->motion_plan_request.max_planning_time = 15.0;
  request->motion_plan_request.min_planning_time = 0.0;
  request->motion_plan_request.min_solution_cost = 1000000.0;         
  request->motion_plan_request.start_names = std::vector<std::string>(
            {{ "tx","ty","tz", "angle", "axis.x", "axis.y", "axis.z" }});
  request->motion_plan_request.goal_names = request->motion_plan_request.start_names;
  request->motion_plan_request.bounds_names = std::vector<std::string>(
            {{ "tx", "ty", "tz" }});    
  request->motion_plan_request.start_config = std::vector<double>(
            {{270.0,160.0,-200.0,0.0,1.0,0.0,0.0 }});        
  request->motion_plan_request.goal_config = std::vector<double>(
            {{270.0,160.0,-400.0,0.0,1.0,0.0,0.0 }}); 
  request->motion_plan_request.bounds_min = std::vector<double>(
            {{53.46,-21.25,-476.86}}); 
  request->motion_plan_request.bounds_max = std::vector<double>(
            {{402.96,269.25,-91.0}});                                                    // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout << "Plan output: " << result.get()->motion_plan.layout.dim[0].label << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service motion_plan_client");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}