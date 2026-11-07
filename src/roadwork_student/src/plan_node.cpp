// plan_node: subscribes to /solution/scene, waits for a confirmed GO, plans the
// left bypass and speed, computes pure-pursuit steering and publishes
// /solution/plan (internal), /solution/state (JSON String) and /solution/path.
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roadwork_student/planning.hpp"
#include "roadwork_student/ros_helpers.hpp"
#include "roadwork_student_interfaces/msg/plan.hpp"
#include "roadwork_student_interfaces/msg/scene.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using rws_ros::to_msg;
using rws_ros::to_sec;
using SceneMsg = roadwork_student_interfaces::msg::Scene;
using PlanMsg = roadwork_student_interfaces::msg::Plan;

static rws::SceneData from_msg(const SceneMsg& m) {
  rws::SceneData s;
  s.stamp = to_sec(m.header.stamp);
  s.ego.valid = m.ego_valid;
  s.ego.stamp = m.ego_valid ? to_sec(m.odom_stamp) : -1.0;
  s.ego.x = m.ego_x; s.ego.y = m.ego_y; s.ego.yaw = m.ego_yaw; s.ego.v = m.ego_speed;
  auto panel = [](uint8_t value) {
    if (value == SceneMsg::PANEL_GO) return rws::Panel::Go;
    if (value == SceneMsg::PANEL_STOP) return rws::Panel::Stop;
    return rws::Panel::Unknown;
  };
  s.panel_raw = panel(m.panel_raw);
  s.panel_state = panel(m.panel_state);
  s.camera_stamp = to_sec(m.camera_stamp);
  s.green_fraction = m.panel_green_fraction;
  s.panel_go_count = static_cast<int>(m.panel_go_count);
  s.geometry_known = m.geometry_known;
  s.beacon_stamp = m.geometry_known ? to_sec(m.beacon_stamp) : -1.0;
  s.geometry.entry_x = m.entry_x; s.geometry.work_start = m.work_start; s.geometry.work_end = m.work_end;
  s.geometry.exit_x = m.exit_x; s.geometry.speed_limit = m.speed_limit; s.geometry.pass_side = m.pass_side;
  return s;
}

class PlanNode : public rclcpp::Node {
 public:
  PlanNode() : Node("plan") {
    rws_ros::ensure_sim_time(*this);
    rws::PlannerConfig c;
    c.path.bypass_y = declare_parameter("bypass_y", c.path.bypass_y);
    c.speed.cruise = declare_parameter("cruise_speed", c.speed.cruise);
    c.speed.lat_accel = declare_parameter("max_lateral_accel", c.speed.lat_accel);
    c.scene_timeout = declare_parameter("scene_timeout", c.scene_timeout);
    c.plan_validity = declare_parameter("plan_validity", c.plan_validity);
    planner_ = std::make_unique<rws::Planner>(c);
    scene_timeout_ = c.scene_timeout;

    scene_sub_ = create_subscription<SceneMsg>("/solution/scene", rclcpp::QoS(1).reliable(),
                                               [this](SceneMsg::ConstSharedPtr m) { on_scene(*m); });
    plan_pub_ = create_publisher<PlanMsg>("/solution/plan", rclcpp::QoS(1).reliable());
    state_pub_ = create_publisher<std_msgs::msg::String>("/solution/state", rclcpp::QoS(10).reliable());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/solution/path", rclcpp::QoS(1).reliable().transient_local());

    // Input deadline: no scene => publish a hold plan so Act brakes under our control.
    watchdog_ = create_timer(50ms, [this]() {
      const double now = rws_ros::now_sec(*this);
      if (last_scene_rx_ < 0 || now - last_scene_rx_ > scene_timeout_)
        publish(planner_->timeout_output(now, last_scene_rx_ < 0 ? "no scene received yet" : "scene deadline missed"),
                nullptr, now);
    });
    path_timer_ = create_timer(1s, [this]() { publish_path(); });
    RCLCPP_INFO(get_logger(), "plan_node ready: bypass_y=%.2f cruise=%.2f m/s", c.path.bypass_y, c.speed.cruise);
  }

 private:
  void on_scene(const SceneMsg& m) {
    const double now = rws_ros::now_sec(*this);
    last_scene_rx_ = now;
    const rws::SceneData s = from_msg(m);
    const rws::PlanOutput out = planner_->step(s, now);
    publish(out, &s, now);
    if (planner_->path_version() != published_path_version_) publish_path();
  }

  void publish(const rws::PlanOutput& o, const rws::SceneData* s, double now) {
    PlanMsg p;
    p.header.stamp = to_msg(o.stamp);
    p.header.frame_id = "base_link";
    p.scene_stamp = to_msg(o.scene_stamp);
    p.odom_stamp = to_msg(o.odom_stamp);
    p.valid_until = to_msg(o.valid_until);
    p.target_speed = o.speed;
    p.target_steering = o.steering;
    p.speed_limit = o.speed_limit;
    p.hold = o.hold;
    p.state = static_cast<uint8_t>(o.state);
    p.state_name = rws::to_string(o.state);
    p.reason = o.reason;
    plan_pub_->publish(p);

    if (o.state != last_state_) {
      RCLCPP_INFO(get_logger(), "state %s -> %s at t=%.2f x=%.2f: %s", rws::to_string(last_state_),
                  rws::to_string(o.state), now, s ? s->ego.x : 0.0, o.reason.c_str());
      last_state_ = o.state;
    }
    if (o.go && !logged_go_) {
      RCLCPP_INFO(get_logger(), "GO latched at t=%.2f x=%.2f", now, s ? s->ego.x : 0.0);
      logged_go_ = true;
    }

    // Decision state for observability and failure analysis (JSON in std_msgs/String).
    auto age = [now](double st) { return st >= 0 ? now - st : -1.0; };
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "{\"t\":%.3f,\"state\":\"%s\",\"reason\":\"%s\",\"go\":%s,\"panel\":\"%s\",\"go_count\":%d,"
        "\"x\":%.3f,\"y\":%.3f,\"yaw\":%.4f,\"v\":%.3f,\"target_speed\":%.3f,\"steering\":%.4f,\"cte\":%.3f,"
        "\"path_s\":%.2f,\"v_path\":%.3f,\"v_goal\":%.3f,\"scene_age\":%.3f,\"odom_age\":%.3f}",
        now, rws::to_string(o.state), rws_ros::json_escape(o.reason).c_str(), o.go ? "true" : "false",
        s ? rws::to_string(s->panel_state) : "UNKNOWN", s ? s->panel_go_count : 0, s ? s->ego.x : 0.0,
        s ? s->ego.y : 0.0, s ? s->ego.yaw : 0.0, s ? s->ego.v : 0.0, o.speed, o.steering, o.cte, o.path_s, o.v_path,
        o.v_goal, s ? age(s->stamp) : -1.0, s ? age(s->ego.stamp) : -1.0);
    std_msgs::msg::String st;
    st.data = buf;
    state_pub_->publish(st);
  }

  void publish_path() {
    const auto& path = planner_->path();
    if (!path.valid()) return;
    nav_msgs::msg::Path msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "map";
    const auto& pts = path.points();
    for (size_t i = 0; i < pts.size(); i += 2) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = pts[i].x;
      ps.pose.position.y = pts[i].y;
      ps.pose.orientation.z = std::sin(pts[i].yaw / 2.0);
      ps.pose.orientation.w = std::cos(pts[i].yaw / 2.0);
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
    if (published_path_version_ != planner_->path_version()) {
      RCLCPP_INFO(get_logger(), "reference path v%d: lane change x=[%.2f, %.2f], return x=[%.2f, %.2f], max steer %.3f rad",
                  planner_->path_version(), path.x0(), path.x1(), path.x2(), path.x3(), path.max_steer());
      published_path_version_ = planner_->path_version();
    }
  }

  std::unique_ptr<rws::Planner> planner_;
  double scene_timeout_ = 0.25, last_scene_rx_ = -1.0;
  int published_path_version_ = 0;
  rws::State last_state_ = rws::State::Init;
  bool logged_go_ = false;
  rclcpp::Subscription<SceneMsg>::SharedPtr scene_sub_;
  rclcpp::Publisher<PlanMsg>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_, path_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanNode>());
  rclcpp::shutdown();
  return 0;
}
