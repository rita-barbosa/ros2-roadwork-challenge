#include "rclcpp/rclcpp.hpp"
// TODO: Subscribe to your plan; check validity, control the vehicle and publish /vehicle/command.
// TODO: Define your own internal ROS messages and update build dependencies.
// This deliberately provides no subscriptions, publishers, timers or algorithms.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rclcpp::Node>("act"));
  rclcpp::shutdown();
}
