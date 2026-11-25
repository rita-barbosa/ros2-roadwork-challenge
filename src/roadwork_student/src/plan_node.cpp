// plan_node: subscribes to /solution/scene, stops before the entry line until
// entry is permitted (confirmed GO, fresh beacon, a safe gap in opposing
// traffic and a clear LiDAR corridor), plans the left bypass and speed, brakes
// for obstacles in the corridor, computes pure-pursuit steering and publishes
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
  s.beacon_valid_for = std::isfinite(m.beacon_valid_for) ? m.beacon_valid_for : 0.0;
  s.geometry.entry_x = m.entry_x; s.geometry.work_start = m.work_start; s.geometry.work_end = m.work_end;
  s.geometry.exit_x = m.exit_x; s.geometry.speed_limit = m.speed_limit; s.geometry.pass_side = m.pass_side;
  s.radar_stamp = to_sec(m.radar_stamp) > 0 ? to_sec(m.radar_stamp) : -1.0;
  s.radar_pose_dt = m.radar_pose_dt;
  s.radar_detections = static_cast<int>(m.radar_detections);
  s.radar_moving = static_cast<int>(m.radar_moving);
  s.radar_rejected = static_cast<int>(m.radar_rejected);
  for (const auto& tm : m.tracks) {
    rws::Track t;
    t.id = static_cast<int>(tm.id);
    t.stamp = to_sec(tm.stamp);
    t.x = tm.x; t.y = tm.y; t.vx = tm.vx;
    t.hits = static_cast<int>(tm.hits);
    t.confirmed = tm.confirmed;
    t.coasting = tm.coasting;
    // Malformed tracks are dropped here (internal interface validation).
    if (!rws::finite_all({t.stamp, t.x, t.y, t.vx})) continue;
    s.tracks.push_back(t);
  }
  s.lidar_stamp = to_sec(m.lidar_stamp) > 0 ? to_sec(m.lidar_stamp) : -1.0;
  s.lidar_pose_dt = m.lidar_pose_dt;
  s.lidar_returns = static_cast<int>(m.lidar_returns);
  s.lidar_rejected = static_cast<int>(m.lidar_rejected);
  if (m.obstacle_x.size() != m.obstacle_y.size()) {
    s.lidar_stamp = -1.0;  // malformed evidence is treated as missing evidence
  } else {
    for (size_t k = 0; k < m.obstacle_x.size(); ++k)
      if (std::isfinite(m.obstacle_x[k]) && std::isfinite(m.obstacle_y[k]))
        s.obstacle_points.push_back({m.obstacle_x[k], m.obstacle_y[k]});
  }
  for (const auto& om : m.obstacles) s.clusters.push_back({om.x, om.y, om.radius, om.range_min, static_cast<int>(om.n_points)});
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
    c.stop_distance = std::max(0.0, declare_parameter("stop_distance", c.stop_distance));
    c.beacon_max_age = declare_parameter("beacon_max_age", c.beacon_max_age);
    c.camera_timeout = declare_parameter("camera_timeout", c.camera_timeout);
    c.use_traffic_gap = declare_parameter("use_traffic_gap", c.use_traffic_gap);
    c.radar_timeout = declare_parameter("radar_timeout", c.radar_timeout);
    c.gap_time = declare_parameter("gap_time", c.gap_time);
    c.gap_v_nominal = declare_parameter("gap_v_nominal", c.gap_v_nominal);
    c.gap_a_nominal = declare_parameter("gap_a_nominal", c.gap_a_nominal);
    c.traffic_radius = declare_parameter("traffic_radius", c.traffic_radius);
    c.gap_margin = declare_parameter("gap_margin", c.gap_margin);
    c.use_obstacle_check = declare_parameter("use_obstacle_check", c.use_obstacle_check);
    c.lidar_timeout = declare_parameter("lidar_timeout", c.lidar_timeout);
    c.corridor_half_width = declare_parameter("corridor_half_width", c.corridor_half_width);
    c.obstacle_horizon = declare_parameter("obstacle_horizon", c.obstacle_horizon);
    c.obstacle_standoff = declare_parameter("obstacle_standoff", c.obstacle_standoff);
    c.obstacle_decel = declare_parameter("obstacle_decel", c.obstacle_decel);
    c.obstacle_clear_time = declare_parameter("obstacle_clear_time", c.obstacle_clear_time);
    cfg_ = c;
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
      check_episode(now);
      if (last_scene_rx_ < 0 || now - last_scene_rx_ > scene_timeout_)
        publish(planner_->timeout_output(now, last_scene_rx_ < 0 ? "no scene received yet" : "scene deadline missed"),
                nullptr, now);
    });
    path_timer_ = create_timer(1s, [this]() { publish_path(); });
    RCLCPP_INFO(get_logger(),
                "plan_node ready: bypass_y=%.2f cruise=%.2f m/s stop_distance=%.2f m traffic_gap=%s obstacle_check=%s",
                c.path.bypass_y, c.speed.cruise, c.stop_distance, c.use_traffic_gap ? "on" : "OFF (ablation)",
                c.use_obstacle_check ? "on" : "OFF (ablation)");
  }

 private:
  void check_episode(double now) {
    if (!episode_.restarted(now)) return;
    RCLCPP_WARN(get_logger(), "/clock jumped back to t=%.2f: new episode, resetting decision state", now);
    planner_ = std::make_unique<rws::Planner>(cfg_);
    last_scene_rx_ = -1.0;
    published_path_version_ = 0;
    last_state_ = rws::State::Init;
    logged_go_ = false;
    last_gap_ok_ = true;
    last_beacon_fresh_ = false;
    last_blocked_ = false;
  }

  void on_scene(const SceneMsg& m) {
    const double now = rws_ros::now_sec(*this);
    check_episode(now);
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
    if (s && s->geometry_known && o.beacon_fresh != last_beacon_fresh_) {
      if (o.beacon_fresh) RCLCPP_INFO(get_logger(), "beacon FRESH at t=%.2f (age %.2f s)", now, o.beacon_age);
      else RCLCPP_WARN(get_logger(), "beacon STALE at t=%.2f (age %.2f s)%s", now, o.beacon_age,
                       o.go ? ": committed, continuing on cached geometry" : ": no entry until a fresh beacon");
      last_beacon_fresh_ = o.beacon_fresh;
    }
    if (s && o.blocked != last_blocked_) {
      if (o.blocked)
        RCLCPP_INFO(get_logger(), "corridor BLOCKED at t=%.2f x=%.2f v=%.2f: return at x=%.2f y=%.2f, %.2f m ahead", now,
                    s->ego.x, s->ego.v, o.block.x, o.block.y, o.block_dist);
      else RCLCPP_INFO(get_logger(), "corridor CLEAR at t=%.2f x=%.2f", now, s->ego.x);
      last_blocked_ = o.blocked;
    }
    if (s && o.gap_ok != last_gap_ok_) {
      RCLCPP_INFO(get_logger(), "traffic gap %s at t=%.2f x=%.2f: %s", o.gap_ok ? "SAFE" : "UNSAFE", now, s->ego.x,
                  o.gap_reason.c_str());
      last_gap_ok_ = o.gap_ok;
    }

    // Decision state for observability and failure analysis (JSON in std_msgs/String).
    auto age = [now](double st) { return st >= 0 ? now - st : -1.0; };
    auto cap = [](double v) { return std::isfinite(v) ? v : -1.0; };  // -1 = no cap
    char buf[2048];
    std::snprintf(
        buf, sizeof(buf),
        "{\"t\":%.3f,\"state\":\"%s\",\"reason\":\"%s\",\"go\":%s,\"panel\":\"%s\",\"go_count\":%d,"
        "\"x\":%.3f,\"y\":%.3f,\"yaw\":%.4f,\"v\":%.3f,\"target_speed\":%.3f,\"steering\":%.4f,\"cte\":%.3f,"
        "\"path_s\":%.2f,\"v_path\":%.3f,\"v_goal\":%.3f,\"v_stop\":%.3f,\"v_yield\":%.3f,\"d_line\":%.3f,"
        "\"gap_ok\":%s,\"radar_fresh\":%s,\"n_tracks\":%d,\"conflict_id\":%d,\"conflict_x\":%.2f,"
        "\"conflict_vx\":%.2f,\"t_clear\":%.2f,\"radar_age\":%.3f,"
        "\"beacon_fresh\":%s,\"beacon_age\":%.3f,\"camera_age\":%.3f,"
        "\"lidar_fresh\":%s,\"lidar_age\":%.3f,\"n_obstacle_points\":%d,\"blocked\":%s,\"block_x\":%.2f,"
        "\"block_y\":%.2f,\"block_dist\":%.2f,\"v_obs\":%.3f,"
        "\"scene_age\":%.3f,\"odom_age\":%.3f}",
        now, rws::to_string(o.state), rws_ros::json_escape(o.reason).c_str(), o.go ? "true" : "false",
        s ? rws::to_string(s->panel_state) : "UNKNOWN", s ? s->panel_go_count : 0, s ? s->ego.x : 0.0,
        s ? s->ego.y : 0.0, s ? s->ego.yaw : 0.0, s ? s->ego.v : 0.0, o.speed, o.steering, o.cte, o.path_s, o.v_path,
        o.v_goal, cap(o.v_stop), cap(o.v_yield), o.d_line, o.gap_ok ? "true" : "false",
        o.radar_fresh ? "true" : "false", o.n_tracks, o.conflict_id, o.conflict_x, o.conflict_vx, o.t_clear,
        s ? age(s->radar_stamp) : -1.0, o.beacon_fresh ? "true" : "false", o.beacon_age, o.camera_age,
        o.lidar_fresh ? "true" : "false", o.lidar_age, o.n_obstacle_points, o.blocked ? "true" : "false", o.block.x,
        o.block.y, o.block_dist, cap(o.v_obs),
        s ? age(s->stamp) : -1.0, s ? age(s->ego.stamp) : -1.0);
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

  rws::PlannerConfig cfg_;
  std::unique_ptr<rws::Planner> planner_;
  rws_ros::EpisodeWatch episode_;
  double scene_timeout_ = 0.25, last_scene_rx_ = -1.0;
  int published_path_version_ = 0;
  rws::State last_state_ = rws::State::Init;
  bool logged_go_ = false, last_gap_ok_ = true, last_beacon_fresh_ = false, last_blocked_ = false;
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
