#include "move_tiago_msgs/srv/move_tiago.hpp"
#include "rclcpp/rclcpp.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

class MoveTiagoServerNode : public rclcpp::Node {
public:
  MoveTiagoServerNode(rclcpp::NodeOptions options) : Node("move_tiago_server", options) {
    srv_ = create_service<move_tiago_msgs::srv::MoveTiago>(
        "move_tiago", std::bind(&MoveTiagoServerNode::move, this,
                                std::placeholders::_1, std::placeholders::_2));
  }

  ~MoveTiagoServerNode() { delete move_group; }

  void setMoveGroup(moveit::planning_interface::MoveGroupInterface *m) {
    this->move_group = m;
  }

private:
  rclcpp::Service<move_tiago_msgs::srv::MoveTiago>::SharedPtr srv_;
  moveit::planning_interface::MoveGroupInterface *move_group;

  void move(const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Request> request,
            const std::shared_ptr<move_tiago_msgs::srv::MoveTiago::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Requested move_tiago Service");

    geometry_msgs::msg::PoseStamped randomPose = move_group->getRandomPose();

    RCLCPP_INFO(this->get_logger(), "Ramdom pose obtained");

    move_group->setPoseTarget(randomPose);

    move_group->move();

    response->success = true;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_tiago_server_node = std::make_shared<MoveTiagoServerNode>(node_options);

  const std::string PLANNING_GROUP = "arm";
  
  auto move_group = new moveit::planning_interface::MoveGroupInterface(
      move_tiago_server_node, PLANNING_GROUP);

  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->setMaxAccelerationScalingFactor(1.0);

  move_tiago_server_node->setMoveGroup(move_group);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_tiago_server_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  spinner.join();

  rclcpp::shutdown();

  return 0;
}
