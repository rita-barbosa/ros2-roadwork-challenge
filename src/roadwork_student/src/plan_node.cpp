#include "rclcpp/rclcpp.hpp"
// TODO: Subscribe to your scene; implement permission, traffic, path and speed decisions; publish your plan.
// TODO: Define your own internal ROS messages and update build dependencies.
// This deliberately provides no subscriptions, publishers, timers or algorithms.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rclcpp::Node>("plan"));
  rclcpp::shutdown();
}
