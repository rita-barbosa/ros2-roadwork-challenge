// Perception building blocks used by sense_node (ROS-free): panel state and
// beacon geometry cache.
#pragma once
#include <limits>
#include <string>
#include <vector>

#include "roadwork_student/common.hpp"

namespace rws {

// ---------------------------------------------------------------------------
// Panel classification (single frame)
// ---------------------------------------------------------------------------
// The panel crop is a 96x64 rgb8 image with a centred disc (radius ~18 px);
// Green means GO and red means STOP. Count strongly coloured pixels inside an
// inner disc and verify the outer background ring is not coloured. Ambiguous
// frames are UNKNOWN.
struct PanelResult {
  Panel cls = Panel::Unknown;
  bool valid = false;  // image well-formed
  double green_fraction = 0.0, red_fraction = 0.0, ring_colored_fraction = 0.0;
  int inner_pixels = 0;
  std::string reason;
};

struct PanelClassifierConfig {
  int min_channel = 110;     // dominant colour channel must exceed this (nominal 200)
  int min_contrast = 70;     // dominant channel - other channels (nominal 175)
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
  int green = 0, red = 0, inner = 0, ring = 0, ring_col = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double d = std::hypot(x - cx, y - cy);
      const bool in_inner = d < r_in, in_ring = d >= r_ring_lo && d < r_ring_hi;
      if (!in_inner && !in_ring) continue;
      const size_t i = static_cast<size_t>(y) * step + static_cast<size_t>(x) * 3;
      const int R = data[i], G = data[i + 1], B = data[i + 2];
      const bool is_green = G >= c.min_channel && G - R >= c.min_contrast && G - B >= c.min_contrast;
      const bool is_red = R >= c.min_channel && R - G >= c.min_contrast && R - B >= c.min_contrast;
      if (in_inner) { ++inner; green += is_green; red += is_red; }
      else { ++ring; ring_col += is_green || is_red; }
    }
  }
  r.inner_pixels = inner;
  r.green_fraction = inner ? double(green) / inner : 0.0;
  r.red_fraction = inner ? double(red) / inner : 0.0;
  r.ring_colored_fraction = ring ? double(ring_col) / ring : 0.0;
  if (r.ring_colored_fraction > c.max_ring_colored) { r.reason = "background coloured"; return r; }
  if (r.green_fraction >= c.go_fraction) { r.cls = Panel::Go; r.reason = "green disc"; }
  else if (r.red_fraction >= c.go_fraction) { r.cls = Panel::Stop; r.reason = "red disc"; }
  else r.reason = "ambiguous or blank panel";
  return r;
}

// ---------------------------------------------------------------------------
// Temporal GO confirmation
// ---------------------------------------------------------------------------
// STOP is reported immediately; GO requires `go_confirm` consecutive frames
// whose stamps are at most `max_gap` apart (camera is 10 Hz).
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

  Panel state() const {
    if (last_raw_ == Panel::Stop) return Panel::Stop;
    return go_count_ >= go_confirm_ ? Panel::Go : Panel::Unknown;
  }
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
    if (b.valid_for <= 0.0 || b.valid_for > 10.0) return reject("implausible validity interval");
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
  double valid_for() const { return have_ ? last_.valid_for : 0.0; }
  int rejected() const { return rejected_; }

 private:
  BeaconSample last_;
  bool have_ = false;
  int rejected_ = 0;
};

// ---------------------------------------------------------------------------
// LiDAR: projection into the map frame and clustering
// ---------------------------------------------------------------------------
// Each finite return r_i at beam angle a_i (base_link) is projected with the
// ego pose associated with the scan stamp: p = (x + r cos(yaw + a_i),
// y + r sin(yaw + a_i)). Returns at the published road edges (|y| >= 3.5 -
// edge_margin) are free-space boundaries, not obstacles, and are removed.
// The remaining returns are grouped in beam order: a new cluster starts when
// two consecutive object returns are more than cluster_gap apart or separated
// by more than max_ray_gap beams (wrap-around at +/-pi is merged).
struct LidarConfig {
  double edge_margin = 0.25;  // m
  double cluster_gap = 0.8;   // m
  int max_ray_gap = 2;        // beams
};

struct LidarScanIn {
  double stamp = -1.0, angle_min = 0.0, angle_increment = 0.0, range_min = 0.0, range_max = 0.0;
  std::vector<float> ranges;
};

struct LidarResult {
  std::vector<Point2> points;  // object returns (map frame)
  std::vector<Cluster> clusters;
  int returns = 0, malformed = 0;
};

// Validates the scan layout (false + why on failure) and projects/clusters it.
inline bool process_scan(const LidarScanIn& s, const EgoState& pose, LidarResult* out, std::string* why = nullptr,
                         const LidarConfig& c = LidarConfig{}) {
  auto fail = [&](const char* r) { if (why) *why = r; return false; };
  const size_t n = s.ranges.size();
  if (n < 2) return fail("empty scan");
  if (!finite_all({s.angle_min, s.angle_increment, s.range_min, s.range_max}) || s.angle_increment <= 0.0 ||
      s.range_min < 0.0 || s.range_max <= s.range_min)
    return fail("malformed scan parameters");
  if (s.angle_min < -kPi - 1e-3 || s.angle_min + s.angle_increment * static_cast<double>(n - 1) > kPi + 1e-3)
    return fail("scan covers more than one revolution");
  *out = LidarResult{};
  struct Hit { Point2 p; double r; size_t i; };
  std::vector<Hit> hits;
  for (size_t i = 0; i < n; ++i) {
    const double r = s.ranges[i];
    if (std::isnan(r) || (std::isinf(r) && r < 0)) { ++out->malformed; continue; }
    if (!std::isfinite(r) || r < s.range_min || r > s.range_max) continue;  // no return
    ++out->returns;
    const double th = pose.yaw + s.angle_min + s.angle_increment * static_cast<double>(i);
    const Point2 p{pose.x + r * std::cos(th), pose.y + r * std::sin(th)};
    if (std::abs(p.y) >= kRoadHalfWidth - c.edge_margin) continue;  // road edge
    hits.push_back({p, r, i});
    out->points.push_back(p);
  }
  // Cluster in beam order.
  std::vector<std::vector<size_t>> groups;
  for (size_t k = 0; k < hits.size(); ++k) {
    const bool join = k > 0 && hits[k].i - hits[k - 1].i <= static_cast<size_t>(c.max_ray_gap) + 1 &&
                      std::hypot(hits[k].p.x - hits[k - 1].p.x, hits[k].p.y - hits[k - 1].p.y) <= c.cluster_gap;
    if (!join) groups.emplace_back();
    groups.back().push_back(k);
  }
  if (groups.size() > 1) {  // wrap-around between the last and first beams
    const Hit& a = hits[groups.back().back()];
    const Hit& b = hits[groups.front().front()];
    if (b.i + n - a.i <= static_cast<size_t>(c.max_ray_gap) + 1 &&
        std::hypot(a.p.x - b.p.x, a.p.y - b.p.y) <= c.cluster_gap) {
      groups.front().insert(groups.front().begin(), groups.back().begin(), groups.back().end());
      groups.pop_back();
    }
  }
  for (const auto& g : groups) {
    Cluster cl;
    cl.n = static_cast<int>(g.size());
    cl.range_min = std::numeric_limits<double>::infinity();
    for (size_t k : g) { cl.x += hits[k].p.x; cl.y += hits[k].p.y; cl.range_min = std::min(cl.range_min, hits[k].r); }
    cl.x /= cl.n;
    cl.y /= cl.n;
    for (size_t k : g) cl.radius = std::max(cl.radius, std::hypot(hits[k].p.x - cl.x, hits[k].p.y - cl.y));
    out->clusters.push_back(cl);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Radar: ego-motion compensation and moving-object tracking
// ---------------------------------------------------------------------------
// The radar reports range, azimuth (base_link) and the radial velocity, which
// includes the ego motion: a static object at azimuth az has
// v_r = -v_ego * cos(az). The compensated radial velocity
//     v_comp = v_r + v_ego * cos(az)
// is the projection of the object's own map-frame velocity on the line of
// sight. Detections with |v_comp| >= moving_threshold are moving objects (the
// static works and parked obstacles are left to the LiDAR). Traffic is assumed
// to move along the road, so vx = v_comp / cos(map bearing) when the bearing is
// not too oblique; otherwise vx is corrected from the position residual.
//
// Tracks are associated greedily to the nearest moving detection inside a gate
// around their constant-velocity prediction. A track that is NOT detected
// while it is predicted inside the radar FOV accumulates misses and is deleted;
// a track predicted OUTSIDE the FOV/range (e.g. passing beside the ego) is
// coasted by prediction, because a missing detection is not an empty road.
struct TrackerConfig {
  double moving_threshold = 0.6;  // m/s; noise on v_r is +/-noise (<= 0.05 m/s)
  double gate = 3.0;              // m around the predicted position
  double alpha = 0.5;             // position smoothing gain
  double vel_gain = 0.5;          // smoothing of the radial-velocity projection
  double beta = 0.4;              // velocity correction from the position residual
  double min_cos = 0.5;           // use v_comp / cos(bearing) only when |cos| >= this
  double fov = kPi / 3.0;         // radar half field of view (+/-60 deg)
  double fov_margin = 0.08;       // rad: a prediction this far inside the FOV must be detected
  double max_range = 80.0;        // m
  int confirm_hits = 2;           // radar updates before Plan uses a track
  int miss_limit = 3;             // consecutive misses inside the FOV before deletion
  double max_coast = 20.0;        // s a coasting track survives without updates
  double drop_behind = 30.0;      // m: tracks this far behind the ego are dropped
};

class TrafficTracker {
 public:
  explicit TrafficTracker(const TrackerConfig& c = TrackerConfig{}) : cfg_(c) {}

  // `pose` is the ego pose associated with the scan stamp (see SenseCore::pose_at).
  // Returns the number of moving detections in the scan.
  int update(double stamp, const EgoState& pose, const std::vector<RadarDet>& dets) {
    struct Meas {
      double x, y, vx;
      bool has_vx;
    };
    std::vector<Meas> moving;
    for (const auto& d : dets) {
      const double comp = d.radial_velocity + pose.v * std::cos(d.azimuth);
      if (std::abs(comp) < cfg_.moving_threshold) continue;
      const double th = pose.yaw + d.azimuth, c = std::cos(th);
      Meas m{pose.x + d.range * c, pose.y + d.range * std::sin(th), 0.0, false};
      if (std::abs(c) >= cfg_.min_cos) { m.vx = comp / c; m.has_vx = true; }
      moving.push_back(m);
    }

    std::vector<bool> used(moving.size(), false);
    for (auto& t : tracks_) {
      const double dt = stamp - t.stamp;
      const double px = t.x + t.vx * dt, py = t.y;
      int best = -1;
      double bd = cfg_.gate;
      for (size_t i = 0; i < moving.size(); ++i) {
        if (used[i]) continue;
        const double d = std::hypot(moving[i].x - px, moving[i].y - py);
        if (d < bd) { bd = d; best = static_cast<int>(i); }
      }
      if (best >= 0) {
        used[best] = true;
        const Meas& m = moving[best];
        double vx = t.vx;
        if (m.has_vx) vx += cfg_.vel_gain * (m.vx - vx);
        else if (dt > 1e-3) vx += cfg_.beta * (m.x - px) / dt;
        t.x = px + cfg_.alpha * (m.x - px);
        t.y = py + cfg_.alpha * (m.y - py);
        t.vx = vx;
        t.stamp = stamp;
        ++t.hits;
        t.misses = 0;
        t.coasting = false;
        t.confirmed = t.hits >= cfg_.confirm_hits;
      } else {
        const double ox = px - pose.x, oy = py - pose.y;
        const double az = wrap_angle(std::atan2(oy, ox) - pose.yaw);
        const bool observable =
            std::hypot(ox, oy) <= cfg_.max_range - 2.0 && std::abs(az) <= cfg_.fov - cfg_.fov_margin;
        if (observable || !t.confirmed) { ++t.misses; t.coasting = false; }
        else t.coasting = true;
      }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& t) {
                                   if (t.misses >= (t.confirmed ? cfg_.miss_limit : 1)) return true;
                                   if (t.coasting && stamp - t.stamp > cfg_.max_coast) return true;
                                   return t.x_at(stamp) < pose.x - cfg_.drop_behind;
                                 }),
                  tracks_.end());

    for (size_t i = 0; i < moving.size(); ++i) {
      if (used[i]) continue;
      Track t;
      t.id = next_id_++;
      t.stamp = stamp;
      t.x = moving[i].x;
      t.y = moving[i].y;
      t.vx = moving[i].has_vx ? moving[i].vx : 0.0;
      t.hits = 1;
      t.confirmed = cfg_.confirm_hits <= 1;
      tracks_.push_back(t);
    }
    return static_cast<int>(moving.size());
  }

  const std::vector<Track>& tracks() const { return tracks_; }

 private:
  TrackerConfig cfg_;
  std::vector<Track> tracks_;
  int next_id_ = 1;
};

}  // namespace rws
