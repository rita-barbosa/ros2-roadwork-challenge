// act_node: subscribes to /solution/plan, validates expiry, saturates and
// rate-limits the targets and is the ONLY publisher of /vehicle/command.
// A missing/expired/malformed plan yields speed 0 with steering held, published
// by us with a fresh stamp (we do not rely on the simulator watchdog).
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roadwork_interfaces/msg/vehicle_command.hpp"
#include "roadwork_student/control.hpp"
#include "roadwork_student/ros_helpers.hpp"
#include "roadwork_student_interfaces/msg/plan.hpp"

using namespace std::chrono_literals;
using rws_ros::to_msg;
using rws_ros::to_sec;
using PlanMsg = roadwork_student_interfaces::msg::Plan;
using CommandMsg = roadwork_interfaces::msg::VehicleCommand;

class ActNode : public rclcpp::Node {
 public:
  ActNode() : Node("act") {
    rws_ros::ensure_sim_time(*this);
    rws::GuardConfig c;
    c.max_speed = declare_parameter("max_speed", c.max_speed);
    c.max_plan_age = declare_parameter("max_plan_age", c.max_plan_age);
    guard_ = std::make_unique<rws::CommandGuard>(c);

    plan_sub_ = create_subscription<PlanMsg>("/solution/plan", rclcpp::QoS(1).reliable(),
                                             [this](PlanMsg::ConstSharedPtr m) { on_plan(*m); });
    cmd_pub_ = create_publisher<CommandMsg>("/vehicle/command", rclcpp::QoS(10).reliable());
    // Keep commanding (braking) even when plans stop arriving.
    timer_ = create_timer(50ms, [this]() {
      const double now = rws_ros::now_sec(*this);
      if (last_pub_ < 0 || now - last_pub_ >= 0.099) send(now);
    });
    RCLCPP_INFO(get_logger(), "act_node ready: speed cap %.2f m/s, steering +/-%.2f rad, slew %.2f rad/s", c.max_speed,
                c.max_steer, c.steer_slew);
  }

 private:
  void on_plan(const PlanMsg& m) {
    plan_.have = true;
    plan_.stamp = to_sec(m.header.stamp);
    plan_.valid_until = to_sec(m.valid_until);
    plan_.speed = m.target_speed;
    plan_.steering = m.target_steering;
    plan_.speed_limit = m.speed_limit;
    plan_.hold = m.hold;
    send(rws_ros::now_sec(*this));
  }

  void send(double now) {
    const rws::GuardOutput o = guard_->step(plan_, now);
    CommandMsg c;
    c.header.stamp = to_msg(now);
    c.header.frame_id = "base_link";
    c.speed = o.speed;
    c.steering_angle = o.steering;
    cmd_pub_->publish(c);
    last_pub_ = now;
    if (o.plan_ok != last_ok_) {
      if (o.plan_ok) RCLCPP_INFO(get_logger(), "plan valid at t=%.2f: %s", now, o.reason.c_str());
      else RCLCPP_WARN(get_logger(), "plan invalid at t=%.2f: %s -> commanding speed 0", now, o.reason.c_str());
      last_ok_ = o.plan_ok;
    }
  }

  std::unique_ptr<rws::CommandGuard> guard_;
  rws::PlanInput plan_;
  double last_pub_ = -1.0;
  bool last_ok_ = false;
  rclcpp::Subscription<PlanMsg>::SharedPtr plan_sub_;
  rclcpp::Publisher<CommandMsg>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActNode>());
  rclcpp::shutdown();
  return 0;
}
