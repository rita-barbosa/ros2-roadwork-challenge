// Shared types and constants for the student sense-plan-act pipeline.
// This header (and the other roadwork_student headers) is ROS-free on purpose:
// the algorithms are unit-testable without a ROS graph and the three nodes are
// thin adapters between ROS messages and these structures.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace rws {

constexpr double kPi = 3.14159265358979323846;

// Published vehicle / road constants (ASSIGNMENT.md, INTERFACES.md). Allowed inputs.
constexpr double kWheelbase = 2.4;       // m
constexpr double kEgoRadius = 0.90;      // m, collision disc around rear-axle reference
constexpr double kRoadHalfWidth = 3.5;   // m, road y = -3.5..+3.5
constexpr double kOwnLaneY = -1.75;      // m
constexpr double kMaxSteer = 0.45;       // rad
constexpr double kSteerSlew = 0.6;       // rad/s
constexpr double kMaxAccel = 1.5;        // m/s^2
constexpr double kMaxBrake = 3.0;        // m/s^2
constexpr double kSpeedLimit = 4.0;      // m/s (violation above 4.05)
constexpr double kGoalX = 80.2;          // m, nominal aiming point x
constexpr double kGoalMinX = 80.0;       // m, completion requires x >= 80

// Tolerance for "future" stamps. A node's now() is the latest /clock sample it
// has processed; /clock is 20 Hz and, in rclcpp, handled on a separate clock
// thread, so a sensor message stamped at tick t may be processed while the node
// clock still reads t - 0.05 s. Anything further ahead than one clock period
// (plus slack) is rejected as a future stamp.
constexpr double kFutureTolerance = 0.06;  // s

inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline bool finite_all(std::initializer_list<double> v) {
  for (double x : v) if (!std::isfinite(x)) return false;
  return true;
}

// stamp < 0 means "never received".
inline bool is_future(double now, double stamp) { return stamp > now + kFutureTolerance; }
inline bool is_fresh(double now, double stamp, double ttl) {
  return stamp >= 0.0 && !is_future(now, stamp) && (now - stamp) <= ttl;
}

struct EgoState {
  double stamp = -1.0;
  double x = 0.0, y = 0.0, yaw = 0.0, v = 0.0;
  bool valid = false;
};

struct Point2 {
  double x = 0.0, y = 0.0;
};

// S1 scope: the panel is either confirmed GO or not (UNKNOWN).
enum class Panel : uint8_t { Unknown = 0, Go = 1 };

inline const char* to_string(Panel p) { return p == Panel::Go ? "GO" : "UNKNOWN"; }

// Work-zone geometry as published by the beacon (map frame, metres).
struct Geometry {
  double entry_x = 0.0, work_start = 0.0, work_end = 0.0, exit_x = 0.0;
  double speed_limit = 0.0;
  int pass_side = 0;
  bool same_as(const Geometry& o, double tol = 1e-3) const {
    return std::abs(entry_x - o.entry_x) < tol && std::abs(work_start - o.work_start) < tol &&
           std::abs(work_end - o.work_end) < tol && std::abs(exit_x - o.exit_x) < tol &&
           std::abs(speed_limit - o.speed_limit) < tol && pass_side == o.pass_side;
  }
};

// Internal scene (mirror of roadwork_student_interfaces/msg/Scene).
struct SceneData {
  double stamp = -1.0;
  EgoState ego;
  // panel
  Panel panel_raw = Panel::Unknown, panel_state = Panel::Unknown;
  double camera_stamp = -1.0, green_fraction = 0.0;
  int panel_go_count = 0, camera_rejected = 0;
  // beacon (geometry only)
  bool geometry_known = false;
  double beacon_stamp = -1.0;
  Geometry geometry;
  int beacon_rejected = 0;
};

}  // namespace rws
