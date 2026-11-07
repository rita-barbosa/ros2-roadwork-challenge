// Small ROS-side helpers shared by the three nodes (time conversion, sim time).
#pragma once
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rws_ros {

inline double to_sec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

// Negative values ("never received") are encoded as time 0; validity flags carry the meaning.
inline builtin_interfaces::msg::Time to_msg(double s) {
  builtin_interfaces::msg::Time t;
  if (!std::isfinite(s) || s < 0) s = 0;
  const int64_t ns = static_cast<int64_t>(std::llround(s * 1e9));
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

// All solution nodes must run on simulation time (/clock).
inline void ensure_sim_time(rclcpp::Node& node) {
  bool sim = false;
  node.get_parameter("use_sim_time", sim);
  if (!sim) {
    RCLCPP_WARN(node.get_logger(), "use_sim_time was false; forcing use_sim_time=true (simulation clock required)");
    node.set_parameter(rclcpp::Parameter("use_sim_time", true));
  }
}

inline double now_sec(rclcpp::Node& node) { return node.get_clock()->now().seconds(); }

inline std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else o += c;
  }
  return o;
}

}  // namespace rws_ros
