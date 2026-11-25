// sense_node: subscribes to odometry, the panel camera, the roadwork beacon, the
// radar and the LiDAR, validates and interprets them (panel state, cached
// geometry, moving-object tracks, map-frame obstacle returns and clusters) and
// publishes the internal scene on /solution/scene (20 Hz, on odometry) plus
// RViz markers of its perception on /solution/perception (10 Hz).
#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roadwork_interfaces/msg/radar_detections.hpp"
#include "roadwork_interfaces/msg/roadwork_beacon.hpp"
#include "roadwork_student/ros_helpers.hpp"
#include "roadwork_student/sense_core.hpp"
#include "roadwork_student_interfaces/msg/scene.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    c.max_assoc_dt = declare_parameter("max_assoc_dt", c.max_assoc_dt);
    c.tracker.moving_threshold = declare_parameter("radar_moving_threshold", c.tracker.moving_threshold);
    c.tracker.gate = declare_parameter("radar_gate", c.tracker.gate);
    c.tracker.confirm_hits = declare_parameter("radar_confirm_hits", c.tracker.confirm_hits);
    c.tracker.miss_limit = declare_parameter("radar_miss_limit", c.tracker.miss_limit);
    c.tracker.max_coast = declare_parameter("radar_max_coast", c.tracker.max_coast);
    cfg_ = c;
    core_ = std::make_unique<rws::SenseCore>(c);

    const auto sensor_qos = rclcpp::SensorDataQoS();  // simulator sensors are best-effort
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vehicle/odometry", sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m); });
    beacon_sub_ = create_subscription<roadwork_interfaces::msg::RoadworkBeacon>(
        "/infra/roadwork/beacon", rclcpp::QoS(10).reliable(),
        [this](roadwork_interfaces::msg::RoadworkBeacon::ConstSharedPtr m) { on_beacon(*m); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan", sensor_qos, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { on_scan(*m); });
    radar_sub_ = create_subscription<roadwork_interfaces::msg::RadarDetections>(
        "/radar/detections", sensor_qos,
        [this](roadwork_interfaces::msg::RadarDetections::ConstSharedPtr m) { on_radar(*m); });

    scene_pub_ = create_publisher<SceneMsg>("/solution/scene", rclcpp::QoS(1).reliable());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/solution/perception", rclcpp::QoS(5));
    // If odometry stops, keep publishing scenes (ego_valid=false) so Plan sees the cause.
    watchdog_ = create_timer(50ms, [this]() {
      const double now = rws_ros::now_sec(*this);
      check_episode(now);
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

  void check_episode(double now) {
    if (!episode_.restarted(now)) return;
    RCLCPP_WARN(get_logger(), "/clock jumped back to t=%.2f: new episode, resetting perception state", now);
    core_ = std::make_unique<rws::SenseCore>(cfg_);
    last_scene_ = -1.0;
    last_panel_ = rws::Panel::Unknown;
    known_tracks_.clear();
    last_warn_.clear();
  }

  void on_odom(const nav_msgs::msg::Odometry& m) {
    const double now = rws_ros::now_sec(*this);
    check_episode(now);
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

  void on_scan(const sensor_msgs::msg::LaserScan& m) {
    const double now = rws_ros::now_sec(*this);
    if (m.header.frame_id != "base_link") return warn("scan", "frame_id is not base_link");
    rws::LidarScanIn s;
    s.stamp = to_sec(m.header.stamp);
    s.angle_min = m.angle_min;
    s.angle_increment = m.angle_increment;
    s.range_min = m.range_min;
    s.range_max = m.range_max;
    s.ranges = m.ranges;
    std::string why;
    if (!core_->on_scan(s, now, &why)) return warn("scan", why);
    publish_markers(now);
  }

  // Debug visualisation only (never an algorithm input): clusters and tracks.
  void publish_markers(double now) {
    using visualization_msgs::msg::Marker;
    const rws::SceneData s = core_->scene(now);
    visualization_msgs::msg::MarkerArray out;
    Marker clear;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);
    int id = 0;
    auto base = [&](const char* ns, int type) {
      Marker mk;
      mk.header.stamp = to_msg(now);
      mk.header.frame_id = "map";
      mk.ns = ns;
      mk.id = id++;
      mk.type = type;
      mk.action = Marker::ADD;
      mk.pose.orientation.w = 1.0;
      return mk;
    };
    for (const auto& c : s.clusters) {
      Marker mk = base("lidar_clusters", Marker::CYLINDER);
      mk.pose.position.x = c.x;
      mk.pose.position.y = c.y;
      mk.pose.position.z = 0.3;
      mk.scale.x = mk.scale.y = 2.0 * std::max(0.15, c.radius);
      mk.scale.z = 0.6;
      mk.color.r = 1.0; mk.color.g = 0.6; mk.color.b = 0.1; mk.color.a = 0.5;
      out.markers.push_back(mk);
    }
    for (const auto& t : s.tracks) {
      Marker mk = base("radar_tracks", Marker::SPHERE);
      mk.pose.position.x = t.x_at(now);
      mk.pose.position.y = t.y;
      mk.pose.position.z = 0.8;
      mk.scale.x = mk.scale.y = mk.scale.z = t.confirmed ? 1.0 : 0.5;
      mk.color.r = 0.95; mk.color.g = 0.1; mk.color.b = t.coasting ? 0.9 : 0.1; mk.color.a = 0.9;
      out.markers.push_back(mk);
      Marker tx = base("radar_tracks", Marker::TEXT_VIEW_FACING);
      tx.pose.position.x = t.x_at(now);
      tx.pose.position.y = t.y + 1.2;
      tx.pose.position.z = 1.0;
      tx.scale.z = 0.7;
      tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "T%d vx=%.1f%s", t.id, t.vx, t.coasting ? " (coast)" : "");
      tx.text = buf;
      out.markers.push_back(tx);
    }
    markers_pub_->publish(out);
  }

  void on_radar(const roadwork_interfaces::msg::RadarDetections& m) {
    const double now = rws_ros::now_sec(*this);
    if (m.header.frame_id != "base_link") return warn("radar", "frame_id is not base_link");
    std::vector<rws::RadarDet> dets;
    dets.reserve(m.detections.size());
    for (const auto& d : m.detections) dets.push_back({d.range, d.azimuth, d.radial_velocity});
    std::string why;
    if (!core_->on_radar(to_sec(m.header.stamp), dets, now, &why)) return warn("radar", why);
    // Log track births/deaths (confirmed tracks only) for failure analysis.
    const auto& tracks = core_->scene(now).tracks;
    std::map<int, bool> alive;
    for (const auto& t : tracks) {
      if (!t.confirmed) continue;
      alive[t.id] = true;
      if (!known_tracks_.count(t.id))
        RCLCPP_INFO(get_logger(), "track %d confirmed at t=%.2f: x=%.1f y=%.1f vx=%.2f", t.id, now, t.x, t.y, t.vx);
    }
    for (const auto& [id, _] : known_tracks_)
      if (!alive.count(id)) RCLCPP_INFO(get_logger(), "track %d dropped at t=%.2f", id, now);
    known_tracks_ = alive;
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
    m.beacon_valid_for = s.beacon_valid_for;
    m.entry_x = s.geometry.entry_x;
    m.work_start = s.geometry.work_start;
    m.work_end = s.geometry.work_end;
    m.exit_x = s.geometry.exit_x;
    m.speed_limit = s.geometry.speed_limit;
    m.pass_side = static_cast<int8_t>(s.geometry.pass_side);
    m.beacon_rejected = static_cast<uint32_t>(s.beacon_rejected);

    m.radar_stamp = to_msg(s.radar_stamp);
    m.radar_pose_dt = static_cast<float>(s.radar_pose_dt);
    m.radar_detections = static_cast<uint32_t>(s.radar_detections);
    m.radar_moving = static_cast<uint32_t>(s.radar_moving);
    m.radar_rejected = static_cast<uint32_t>(s.radar_rejected);
    m.tracks.reserve(s.tracks.size());
    for (const auto& t : s.tracks) {
      roadwork_student_interfaces::msg::Track tm;
      tm.id = static_cast<uint32_t>(t.id);
      tm.stamp = to_msg(t.stamp);
      tm.x = t.x;
      tm.y = t.y;
      tm.vx = t.vx;
      tm.hits = static_cast<uint32_t>(t.hits);
      tm.confirmed = t.confirmed;
      tm.coasting = t.coasting;
      m.tracks.push_back(tm);
    }

    m.lidar_stamp = to_msg(s.lidar_stamp);
    m.lidar_pose_dt = static_cast<float>(s.lidar_pose_dt);
    m.lidar_returns = static_cast<uint32_t>(s.lidar_returns);
    m.lidar_rejected = static_cast<uint32_t>(s.lidar_rejected);
    m.obstacle_x.reserve(s.obstacle_points.size());
    m.obstacle_y.reserve(s.obstacle_points.size());
    for (const auto& q : s.obstacle_points) {
      m.obstacle_x.push_back(static_cast<float>(q.x));
      m.obstacle_y.push_back(static_cast<float>(q.y));
    }
    for (const auto& c : s.clusters) {
      roadwork_student_interfaces::msg::Obstacle om;
      om.x = c.x;
      om.y = c.y;
      om.radius = c.radius;
      om.range_min = c.range_min;
      om.n_points = static_cast<uint32_t>(c.n);
      m.obstacles.push_back(om);
    }
    scene_pub_->publish(m);
    last_scene_ = now;
  }

  rws::SenseConfig cfg_;
  std::unique_ptr<rws::SenseCore> core_;
  rws_ros::EpisodeWatch episode_;
  double last_scene_ = -1.0;
  rws::Panel last_panel_ = rws::Panel::Unknown;
  std::map<int, bool> known_tracks_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<roadwork_interfaces::msg::RoadworkBeacon>::SharedPtr beacon_sub_;
  rclcpp::Subscription<roadwork_interfaces::msg::RadarDetections>::SharedPtr radar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<SceneMsg>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SenseNode>());
  rclcpp::shutdown();
  return 0;
}
