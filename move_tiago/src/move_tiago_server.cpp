#include <rclcpp/rclcpp.hpp>
#include "move_tiago_msgs/srv/move_tiago.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

// Combined MoveTiagoServer class, which includes service and pose printing functionality
class MoveTiagoServer : public rclcpp::Node {
public:
  // Constructor to initialize the node, MoveGroupInterface, and executor
  MoveTiagoServer(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control", options), // Initialize the node with the name "robot_control"
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "arm"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
  {
    // Create the service for printing the current pose
    service_ = this->create_service<move_tiago_msgs::srv::MoveTiago>(
              "move_tiago", std::bind(&MoveTiagoServer::move, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // Add the node to the executor and start the executor thread
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
      executor_->spin(); // Run the executor to process callbacks
    });
  }

private:
  void move(const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Request> request,
    const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Requested move_tiago Service");

  geometry_msgs::msg::PoseStamped randomPose = move_group_interface_.getRandomPose();

  RCLCPP_INFO(this->get_logger(), "Ramdom pose obtained");

  move_group_interface_.setPoseTarget(randomPose);

  move_group_interface_.move();

  response->success = true;
  }

  // Member variables
  rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
  moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
  rclcpp::Service<move_tiago_msgs::srv::MoveTiago>::SharedPtr service_;  // Service pointer for pose requests
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
  std::thread executor_thread_;  // Thread to run the executor
};

// Main function - Entry point of the program
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); // Initialize ROS 2

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true); // Allow automatic parameter declaration
  node_options.use_intra_process_comms(false); // Disable intra-process communication

  auto node = std::make_shared<MoveTiagoServer>(node_options); // Create the MoveTiagoServer object and start the node

  rclcpp::spin(node); // Spin the main thread to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 system
  return 0; // Exit the program
}
