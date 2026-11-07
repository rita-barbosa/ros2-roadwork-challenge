// sense_node: subscribes to odometry, the panel camera and the roadwork beacon,
// validates and interprets them, and publishes the internal scene on
// /solution/scene (20 Hz, on odometry).
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roadwork_interfaces/msg/roadwork_beacon.hpp"
#include "roadwork_student/ros_helpers.hpp"
#include "roadwork_student/sense_core.hpp"
#include "roadwork_student_interfaces/msg/scene.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using rws_ros::to_msg;
using rws_ros::to_sec;
using SceneMsg = roadwork_student_interfaces::msg::Scene;

class SenseNode : public rclcpp::Node {
 public:
  SenseNode() : Node("sense") {
    rws_ros::ensure_sim_time(*this);
    rws::SenseConfig c;
    c.odom_timeout = declare_parameter("odom_timeout", c.odom_timeout);
    c.camera_timeout = declare_parameter("camera_timeout", c.camera_timeout);
    c.go_confirm = declare_parameter("go_confirm_frames", c.go_confirm);
    core_ = std::make_unique<rws::SenseCore>(c);

    const auto sensor_qos = rclcpp::SensorDataQoS();  // simulator sensors are best-effort
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vehicle/odometry", sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m); });
    beacon_sub_ = create_subscription<roadwork_interfaces::msg::RoadworkBeacon>(
        "/infra/roadwork/beacon", rclcpp::QoS(10).reliable(),
        [this](roadwork_interfaces::msg::RoadworkBeacon::ConstSharedPtr m) { on_beacon(*m); });

    scene_pub_ = create_publisher<SceneMsg>("/solution/scene", rclcpp::QoS(1).reliable());
    // If odometry stops, keep publishing scenes (ego_valid=false) so Plan sees the cause.
    watchdog_ = create_timer(50ms, [this]() {
      const double now = rws_ros::now_sec(*this);
      if (last_scene_ < 0 || now - last_scene_ > 0.075) publish_scene(now);
    });
    RCLCPP_INFO(get_logger(), "sense_node ready: GO needs %d consecutive frames", c.go_confirm);
  }

 private:
  // Throttled per input (a single RCLCPP_WARN_THROTTLE site would let one input hide the others).
  void warn(const std::string& what, const std::string& why) {
    const double now = rws_ros::now_sec(*this);
    auto it = last_warn_.find(what);
    if (it != last_warn_.end() && now - it->second < 2.0 && now >= it->second) return;
    last_warn_[what] = now;
    RCLCPP_WARN(get_logger(), "rejected %s: %s", what.c_str(), why.c_str());
  }
  std::map<std::string, double> last_warn_;

  void on_odom(const nav_msgs::msg::Odometry& m) {
    const double now = rws_ros::now_sec(*this);
    if (m.header.frame_id != "map") return warn("odometry", "frame_id is not map");
    const auto& q = m.pose.pose.orientation;
    rws::EgoState e;
    e.stamp = to_sec(m.header.stamp);
    e.x = m.pose.pose.position.x;
    e.y = m.pose.pose.position.y;
    e.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    e.v = m.twist.twist.linear.x;
    std::string why;
    if (!core_->on_odom(e, now, &why)) return warn("odometry", why);
    publish_scene(now);
  }

  void on_image(const sensor_msgs::msg::Image& m) {
    const double now = rws_ros::now_sec(*this);
    std::string why;
    if (!core_->on_image(to_sec(m.header.stamp), m.data, static_cast<int>(m.width), static_cast<int>(m.height),
                         static_cast<int>(m.step), m.encoding, now, &why))
      return warn("image", why);
    const rws::SceneData s = core_->scene(now);
    if (s.panel_state != last_panel_) {
      RCLCPP_INFO(get_logger(), "panel %s -> %s at t=%.2f (green=%.2f go_count=%d)", rws::to_string(last_panel_),
                  rws::to_string(s.panel_state), now, s.green_fraction, s.panel_go_count);
      last_panel_ = s.panel_state;
    }
  }

  void on_beacon(const roadwork_interfaces::msg::RoadworkBeacon& m) {
    const double now = rws_ros::now_sec(*this);
    rws::BeaconSample b;
    b.stamp = to_sec(m.header.stamp);
    b.valid_for = m.valid_for;
    b.g.entry_x = m.entry_x;
    b.g.work_start = m.work_start;
    b.g.work_end = m.work_end;
    b.g.exit_x = m.exit_x;
    b.g.speed_limit = m.speed_limit;
    b.g.pass_side = m.pass_side;
    const bool had = core_->scene(now).geometry_known;
    std::string why;
    if (!core_->on_beacon(b, now, &why)) return warn("beacon", why);
    if (!had)
      RCLCPP_INFO(get_logger(), "beacon geometry cached: entry=%.2f work=[%.2f, %.2f] exit=%.2f limit=%.2f",
                  b.g.entry_x, b.g.work_start, b.g.work_end, b.g.exit_x, b.g.speed_limit);
  }

  void publish_scene(double now) {
    const rws::SceneData s = core_->scene(now);
    SceneMsg m;
    m.header.stamp = to_msg(now);
    m.header.frame_id = "map";
    m.ego_valid = s.ego.valid;
    m.odom_stamp = to_msg(s.ego.stamp);
    m.ego_x = s.ego.x;
    m.ego_y = s.ego.y;
    m.ego_yaw = s.ego.yaw;
    m.ego_speed = s.ego.v;

    m.panel_raw = static_cast<uint8_t>(s.panel_raw);
    m.panel_state = static_cast<uint8_t>(s.panel_state);
    m.camera_stamp = to_msg(s.camera_stamp);
    m.panel_green_fraction = static_cast<float>(s.green_fraction);
    m.panel_go_count = static_cast<uint32_t>(s.panel_go_count);
    m.camera_rejected = static_cast<uint32_t>(s.camera_rejected);

    m.geometry_known = s.geometry_known;
    m.beacon_stamp = to_msg(s.beacon_stamp);
    m.entry_x = s.geometry.entry_x;
    m.work_start = s.geometry.work_start;
    m.work_end = s.geometry.work_end;
    m.exit_x = s.geometry.exit_x;
    m.speed_limit = s.geometry.speed_limit;
    m.pass_side = static_cast<int8_t>(s.geometry.pass_side);
    m.beacon_rejected = static_cast<uint32_t>(s.beacon_rejected);
    scene_pub_->publish(m);
    last_scene_ = now;
  }

  std::unique_ptr<rws::SenseCore> core_;
  double last_scene_ = -1.0;
  rws::Panel last_panel_ = rws::Panel::Unknown;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<roadwork_interfaces::msg::RoadworkBeacon>::SharedPtr beacon_sub_;
  rclcpp::Publisher<SceneMsg>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SenseNode>());
  rclcpp::shutdown();
  return 0;
}
