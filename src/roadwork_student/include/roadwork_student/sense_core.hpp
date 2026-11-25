// SenseCore: all Sense-stage state and logic, independent of ROS.
// sense_node converts ROS messages into these calls and SceneData into a Scene msg.
#pragma once
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "roadwork_student/common.hpp"
#include "roadwork_student/perception.hpp"

namespace rws {

struct SenseConfig {
  double odom_timeout = 0.20;  // s (odometry is 20 Hz)
  double camera_timeout = 0.30;  // s (camera is 10 Hz)
  int go_confirm = 3;          // consecutive GO frames required
  double radar_max_range = 80.0;  // m, published radar range
  // Measurement association: a sensor sample is transformed with the odometry
  // sample nearest to its acquisition stamp, extrapolated over the remaining
  // gap at constant speed/heading. Samples further than this from every
  // odometry sample are rejected.
  double max_assoc_dt = 0.15;  // s
  double odom_history = 2.0;   // s of odometry kept for association
  PanelClassifierConfig panel;
  TrackerConfig tracker;
};

class SenseCore {
 public:
  explicit SenseCore(const SenseConfig& c = SenseConfig{})
      : cfg_(c), panel_filter_(c.go_confirm, 0.15), tracker_(c.tracker) {}

  // Odometry: exact ego state in map frame.
  bool on_odom(const EgoState& in, double now, std::string* why = nullptr) {
    if (!finite_all({in.stamp, in.x, in.y, in.yaw, in.v})) return reject(&odom_rejected_, why, "non-finite odometry");
    if (in.stamp < 0 || is_future(now, in.stamp)) return reject(&odom_rejected_, why, "future/negative odometry stamp");
    if (ego_.valid && in.stamp <= ego_.stamp) return reject(&odom_rejected_, why, "repeated/out-of-order odometry");
    ego_ = in;
    ego_.valid = true;
    hist_.push_back(ego_);
    while (!hist_.empty() && hist_.front().stamp < ego_.stamp - cfg_.odom_history) hist_.pop_front();
    return true;
  }

  // Ego pose at time t: nearest odometry sample, extrapolated at constant
  // speed and heading over the residual gap |t - sample stamp| (reported in err).
  bool pose_at(double t, EgoState* out, double* err = nullptr) const {
    const EgoState* best = nullptr;
    double bd = std::numeric_limits<double>::infinity();
    for (const auto& h : hist_) {
      const double d = std::abs(h.stamp - t);
      if (d < bd) { bd = d; best = &h; }
    }
    if (!best || bd > cfg_.max_assoc_dt) return false;
    EgoState p = *best;
    const double dt = t - best->stamp;
    p.x += p.v * std::cos(p.yaw) * dt;
    p.y += p.v * std::sin(p.yaw) * dt;
    p.stamp = t;
    *out = p;
    if (err) *err = bd;
    return true;
  }

  // Radar scan (base_link polar detections). Malformed detections are dropped
  // individually; the scan itself is rejected if its stamp is invalid or no
  // odometry can be associated with it.
  bool on_radar(double stamp, const std::vector<RadarDet>& dets, double now, std::string* why = nullptr) {
    if (!std::isfinite(stamp) || stamp < 0 || is_future(now, stamp))
      return reject(&radar_rejected_, why, "invalid/future radar stamp");
    if (radar_stamp_ >= 0 && stamp <= radar_stamp_) return reject(&radar_rejected_, why, "repeated/out-of-order radar scan");
    EgoState pose;
    double err = 0;
    if (!pose_at(stamp, &pose, &err)) return reject(&radar_rejected_, why, "no odometry near the radar stamp");
    std::vector<RadarDet> valid;
    valid.reserve(dets.size());
    for (const auto& d : dets) {
      if (!finite_all({d.range, d.azimuth, d.radial_velocity}) || d.range < 0.1 ||
          d.range > cfg_.radar_max_range + 0.5 || std::abs(d.azimuth) > cfg_.tracker.fov + 0.01) {
        ++radar_rejected_;
        continue;
      }
      valid.push_back(d);
    }
    radar_moving_ = tracker_.update(stamp, pose, valid);
    radar_detections_ = static_cast<int>(valid.size());
    radar_stamp_ = stamp;
    radar_pose_dt_ = err;
    return true;
  }

  bool on_image(double stamp, const std::vector<uint8_t>& data, int width, int height, int step,
                const std::string& encoding, double now, std::string* why = nullptr) {
    PanelResult pr = classify_panel(data, width, height, step, encoding, cfg_.panel);
    if (!pr.valid) {
      panel_filter_.count_rejected();
      if (why) *why = pr.reason;
      return false;
    }
    if (!panel_filter_.update(stamp, now, pr.cls, why)) return false;
    panel_ = pr;
    return true;
  }

  bool on_beacon(const BeaconSample& b, double now, std::string* why = nullptr) {
    return beacon_.update(b, now, why);
  }

  SceneData scene(double now) const {
    SceneData s;
    s.stamp = now;
    s.ego = ego_;
    s.ego.valid = ego_.valid && is_fresh(now, ego_.stamp, cfg_.odom_timeout);

    s.panel_raw = panel_filter_.last_raw();
    s.camera_stamp = panel_filter_.last_stamp();
    s.panel_state = is_fresh(now, s.camera_stamp, cfg_.camera_timeout) ? panel_filter_.state() : Panel::Unknown;
    s.green_fraction = panel_.green_fraction;
    s.panel_go_count = panel_filter_.go_count();
    s.camera_rejected = panel_filter_.rejected();

    s.geometry_known = beacon_.geometry_known();
    s.beacon_stamp = beacon_.stamp();
    s.beacon_valid_for = beacon_.valid_for();
    s.geometry = beacon_.geometry();
    s.beacon_rejected = beacon_.rejected();

    s.radar_stamp = radar_stamp_;
    s.radar_pose_dt = radar_pose_dt_;
    s.radar_detections = radar_detections_;
    s.radar_moving = radar_moving_;
    s.radar_rejected = radar_rejected_;
    s.tracks = tracker_.tracks();
    return s;
  }

  int odom_rejected() const { return odom_rejected_; }

 private:
  bool reject(int* counter, std::string* why, const std::string& r) {
    ++*counter;
    if (why) *why = r;
    return false;
  }

  SenseConfig cfg_;
  EgoState ego_;
  PanelFilter panel_filter_;
  PanelResult panel_;
  BeaconMonitor beacon_;
  int odom_rejected_ = 0;
  std::deque<EgoState> hist_;
  TrafficTracker tracker_;
  double radar_stamp_ = -1.0, radar_pose_dt_ = 0.0;
  int radar_detections_ = 0, radar_moving_ = 0, radar_rejected_ = 0;
};

}  // namespace rws
