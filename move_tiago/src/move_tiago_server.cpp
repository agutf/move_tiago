// #include "move_tiago_msgs/srv/move_tiago.hpp"
// #include "rclcpp/rclcpp.hpp"

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <geometry_msgs/msg/pose_stamped.hpp>

// #include <memory>

// class MoveTiagoServerNode : public rclcpp::Node {
// public:
//   MoveTiagoServerNode(rclcpp::NodeOptions options) : Node("move_tiago_server", options) {
//     srv_ = create_service<move_tiago_msgs::srv::MoveTiago>(
//         "move_tiago", std::bind(&MoveTiagoServerNode::move, this,
//                                 std::placeholders::_1, std::placeholders::_2));
//   }

//   ~MoveTiagoServerNode() { delete move_group; }

//   void setMoveGroup(moveit::planning_interface::MoveGroupInterface *m) {
//     this->move_group = m;
//   }

// private:
//   rclcpp::Service<move_tiago_msgs::srv::MoveTiago>::SharedPtr srv_;
//   moveit::planning_interface::MoveGroupInterface *move_group;

//   void move(const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Request> request,
//             const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Response> response) {

//     RCLCPP_INFO(this->get_logger(), "Requested move_tiago Service");

//     geometry_msgs::msg::PoseStamped randomPose = move_group->getRandomPose();

//     RCLCPP_INFO(this->get_logger(), "Ramdom pose obtained");

//     move_group->setPoseTarget(randomPose);

//     move_group->move();

//     response->success = true;
//   }
// };

// int main(int argc, char **argv) {

//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);

//   auto move_tiago_server_node = std::make_shared<MoveTiagoServerNode>(node_options);

//   const std::string PLANNING_GROUP = "arm";
  
//   auto move_group = new moveit::planning_interface::MoveGroupInterface(
//       move_tiago_server_node, PLANNING_GROUP);

//   move_group->setMaxVelocityScalingFactor(1.0);
//   move_group->setMaxAccelerationScalingFactor(1.0);

//   move_tiago_server_node->setMoveGroup(move_group);

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(move_tiago_server_node);
//   std::thread spinner = std::thread([&executor]() { executor.spin(); });

//   spinner.join();

//   rclcpp::shutdown();

//   return 0;
// }

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "arm");

auto random_pose = move_group_interface.getRandomPose();

RCLCPP_INFO(logger, "Valor de X en la POSE RANDOM: %f", random_pose.pose.position.x);
RCLCPP_INFO(logger, "Valor de Y en la POSE RANDOM: %f", random_pose.pose.position.y);
RCLCPP_INFO(logger, "Valor de Z en la POSE RANDOM: %f", random_pose.pose.position.z);

move_group_interface.setPoseTarget(random_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}