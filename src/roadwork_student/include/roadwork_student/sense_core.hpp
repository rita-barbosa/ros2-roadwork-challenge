// SenseCore: all Sense-stage state and logic, independent of ROS.
// sense_node converts ROS messages into these calls and SceneData into a Scene msg.
#pragma once
#include <string>
#include <vector>

#include "roadwork_student/common.hpp"
#include "roadwork_student/perception.hpp"

namespace rws {

struct SenseConfig {
  double odom_timeout = 0.20;  // s (odometry is 20 Hz)
  double camera_timeout = 0.30;  // s (camera is 10 Hz)
  int go_confirm = 3;          // consecutive GO frames required
  PanelClassifierConfig panel;
};

class SenseCore {
 public:
  explicit SenseCore(const SenseConfig& c = SenseConfig{}) : cfg_(c), panel_filter_(c.go_confirm, 0.15) {}

  // Odometry: exact ego state in map frame.
  bool on_odom(const EgoState& in, double now, std::string* why = nullptr) {
    if (!finite_all({in.stamp, in.x, in.y, in.yaw, in.v})) return reject(&odom_rejected_, why, "non-finite odometry");
    if (in.stamp < 0 || is_future(now, in.stamp)) return reject(&odom_rejected_, why, "future/negative odometry stamp");
    if (ego_.valid && in.stamp <= ego_.stamp) return reject(&odom_rejected_, why, "repeated/out-of-order odometry");
    ego_ = in;
    ego_.valid = true;
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
    s.geometry = beacon_.geometry();
    s.beacon_rejected = beacon_.rejected();
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
};

}  // namespace rws
