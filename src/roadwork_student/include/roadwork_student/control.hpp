// Act-stage command guard (ROS-free): validates the plan, saturates and rate-limits.
#pragma once
#include <string>

#include "roadwork_student/common.hpp"

namespace rws {

struct GuardConfig {
  double max_speed = 3.8;        // hard cap below the 4 m/s limit (violation > 4.05)
  double max_steer = kMaxSteer;  // rad
  double steer_slew = kSteerSlew;  // rad/s, same as the vehicle actuator
  double max_plan_age = 0.30;    // s, plan stamp must be younger than this
};

struct PlanInput {
  bool have = false;
  double stamp = -1, valid_until = -1, speed = 0, steering = 0, speed_limit = 0;
  bool hold = true;
};

struct GuardOutput {
  double speed = 0, steering = 0;
  bool plan_ok = false;
  std::string reason;
};

class CommandGuard {
 public:
  explicit CommandGuard(const GuardConfig& c = GuardConfig{}) : cfg_(c) {}

  GuardOutput step(const PlanInput& p, double now) {
    GuardOutput o;
    if (!p.have) o.reason = "no plan received";
    else if (!finite_all({p.stamp, p.valid_until, p.speed, p.steering, p.speed_limit})) o.reason = "non-finite plan";
    else if (is_future(now, p.stamp)) o.reason = "plan stamp in the future";
    else if (now > p.valid_until + 1e-9) o.reason = "plan expired";
    else if (now - p.stamp > cfg_.max_plan_age) o.reason = "plan too old";
    else o.plan_ok = true;

    double target_steer = last_steer_;  // on any failure: brake, hold steering
    if (o.plan_ok) {
      const double cap = p.speed_limit > 0 ? std::min(cfg_.max_speed, p.speed_limit) : cfg_.max_speed;
      o.speed = p.hold ? 0.0 : clampd(p.speed, 0.0, cap);
      target_steer = clampd(p.steering, -cfg_.max_steer, cfg_.max_steer);
      o.reason = p.hold ? "plan requests hold" : "tracking plan";
    } else {
      o.speed = 0.0;
    }
    const double dt = last_time_ < 0 ? 0.05 : clampd(now - last_time_, 0.0, 0.10);
    const double max_step = cfg_.steer_slew * dt;
    o.steering = last_steer_ + clampd(target_steer - last_steer_, -max_step, max_step);
    last_steer_ = o.steering;
    last_time_ = now;
    return o;
  }

 private:
  GuardConfig cfg_;
  double last_steer_ = 0.0, last_time_ = -1.0;
};

}  // namespace rws
