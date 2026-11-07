// Perception building blocks used by sense_node (ROS-free). S1 scope: the panel
// GO detector and the beacon geometry cache.
#pragma once
#include <string>
#include <vector>

#include "roadwork_student/common.hpp"

namespace rws {

// ---------------------------------------------------------------------------
// Panel classification (single frame)
// ---------------------------------------------------------------------------
// The panel crop is a 96x64 rgb8 image with a centred disc (radius ~18 px);
// green = GO; per-pixel intensity noise +/-18. We count "strongly green" pixels
// inside an inner disc (radius 0.22*min(w,h), safely inside the panel disc) and
// check that an outer ring (background) is not coloured, which rejects frames
// that are not a centred panel. Anything that is not clearly GO is UNKNOWN.
struct PanelResult {
  Panel cls = Panel::Unknown;
  bool valid = false;  // image well-formed
  double green_fraction = 0.0, ring_colored_fraction = 0.0;
  int inner_pixels = 0;
  std::string reason;
};

struct PanelClassifierConfig {
  int min_channel = 110;     // green channel must exceed this (nominal 200)
  int min_contrast = 70;     // green - other channels (nominal 175)
  double go_fraction = 0.80;
  double max_ring_colored = 0.10;
};

inline PanelResult classify_panel(const std::vector<uint8_t>& data, int width, int height, int step,
                                  const std::string& encoding,
                                  const PanelClassifierConfig& c = PanelClassifierConfig{}) {
  PanelResult r;
  if (encoding != "rgb8") { r.reason = "encoding is not rgb8"; return r; }
  if (width < 16 || height < 16) { r.reason = "image too small"; return r; }
  if (step < width * 3) { r.reason = "step smaller than width*3"; return r; }
  if (data.size() < static_cast<size_t>(step) * static_cast<size_t>(height)) {
    r.reason = "data shorter than step*height"; return r;
  }
  r.valid = true;
  const double cx = width / 2.0, cy = height / 2.0;
  const double m = std::min(width, height);
  const double r_in = 0.22 * m, r_ring_lo = 0.36 * m, r_ring_hi = 0.46 * m;
  int green = 0, inner = 0, ring = 0, ring_col = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double d = std::hypot(x - cx, y - cy);
      const bool in_inner = d < r_in, in_ring = d >= r_ring_lo && d < r_ring_hi;
      if (!in_inner && !in_ring) continue;
      const size_t i = static_cast<size_t>(y) * step + static_cast<size_t>(x) * 3;
      const int R = data[i], G = data[i + 1], B = data[i + 2];
      const bool is_green = G >= c.min_channel && G - R >= c.min_contrast && G - B >= c.min_contrast;
      if (in_inner) { ++inner; green += is_green; }
      else { ++ring; ring_col += is_green; }
    }
  }
  r.inner_pixels = inner;
  r.green_fraction = inner ? double(green) / inner : 0.0;
  r.ring_colored_fraction = ring ? double(ring_col) / ring : 0.0;
  if (r.ring_colored_fraction > c.max_ring_colored) { r.reason = "background coloured"; return r; }
  if (r.green_fraction >= c.go_fraction) { r.cls = Panel::Go; r.reason = "green disc"; }
  else r.reason = "not a green disc";
  return r;
}

// ---------------------------------------------------------------------------
// Temporal GO confirmation
// ---------------------------------------------------------------------------
// GO is confirmed only after `go_confirm` consecutive GO frames whose stamps are
// at most `max_gap` apart (camera is 10 Hz). Otherwise the state is UNKNOWN.
class PanelFilter {
 public:
  explicit PanelFilter(int go_confirm = 3, double max_gap = 0.15) : go_confirm_(go_confirm), max_gap_(max_gap) {}

  // Returns false (and fills why) when the frame is rejected.
  bool update(double stamp, double now, Panel cls, std::string* why = nullptr) {
    if (!std::isfinite(stamp) || stamp < 0) { ++rejected_; if (why) *why = "invalid stamp"; return false; }
    if (is_future(now, stamp)) { ++rejected_; if (why) *why = "future stamp"; return false; }
    if (last_stamp_ >= 0 && stamp <= last_stamp_) { ++rejected_; if (why) *why = "repeated/out-of-order frame"; return false; }
    if (cls == Panel::Go) {
      const bool contiguous = last_raw_ == Panel::Go && last_stamp_ >= 0 && stamp - last_stamp_ <= max_gap_;
      go_count_ = contiguous ? go_count_ + 1 : 1;
    } else {
      go_count_ = 0;
    }
    last_raw_ = cls;
    last_stamp_ = stamp;
    return true;
  }

  Panel state() const { return go_count_ >= go_confirm_ ? Panel::Go : Panel::Unknown; }
  Panel last_raw() const { return last_raw_; }
  double last_stamp() const { return last_stamp_; }
  int go_count() const { return go_count_; }
  int rejected() const { return rejected_; }
  void count_rejected() { ++rejected_; }

 private:
  int go_confirm_;
  double max_gap_;
  Panel last_raw_ = Panel::Unknown;
  double last_stamp_ = -1.0;
  int go_count_ = 0, rejected_ = 0;
};

// ---------------------------------------------------------------------------
// Beacon geometry cache
// ---------------------------------------------------------------------------
// Validates beacons and caches the newest consistent work-zone geometry.
struct BeaconSample {
  double stamp = -1.0, valid_for = 0.0;
  Geometry g;
};

class BeaconMonitor {
 public:
  bool update(const BeaconSample& b, double now, std::string* why = nullptr) {
    auto reject = [&](const char* r) { ++rejected_; if (why) *why = r; return false; };
    const Geometry& g = b.g;
    if (!finite_all({b.stamp, b.valid_for, g.entry_x, g.work_start, g.work_end, g.exit_x, g.speed_limit}))
      return reject("non-finite field");
    if (b.stamp < 0) return reject("negative stamp");
    if (is_future(now, b.stamp)) return reject("future stamp");
    if (have_ && b.stamp <= last_.stamp) return reject("repeated/out-of-order beacon");
    if (g.pass_side != 1) return reject("unsupported pass_side (only +1/left)");
    if (!(g.entry_x > 0.0 && g.entry_x < g.work_start && g.work_start < g.work_end &&
          g.work_end < g.exit_x && g.exit_x < kGoalMinX))
      return reject("inconsistent geometry ordering");
    if (g.speed_limit <= 0.5 || g.speed_limit > kSpeedLimit + 0.05) return reject("implausible speed limit");
    last_ = b;
    have_ = true;
    return true;
  }
  bool geometry_known() const { return have_; }
  const Geometry& geometry() const { return last_.g; }
  double stamp() const { return have_ ? last_.stamp : -1.0; }
  int rejected() const { return rejected_; }

 private:
  BeaconSample last_;
  bool have_ = false;
  int rejected_ = 0;
};

}  // namespace rws
