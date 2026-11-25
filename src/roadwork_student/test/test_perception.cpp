// Unit tests for perception building blocks. The panel test also prints a
// confusion matrix on independently generated samples (own RNG and seeds, not
// the simulator's), used as held-out recognition evidence in the report.
#include <limits>
#include <random>

#include "check.hpp"
#include "roadwork_student/perception.hpp"
#include "roadwork_student/sense_core.hpp"

using namespace rws;

// Independent generator following the documented panel contract:
// 96x64 rgb8, disc radius 18 centred, dominant channel ~200, others ~25,
// background ~35, uniform per-pixel noise of +/-amp shared by the channels.
// GO and STOP draw their corresponding coloured discs; UNKNOWN draws a blank.
static std::vector<uint8_t> make_panel(Panel panel, std::mt19937& rng, int amp = 18, int w = 96, int h = 64,
                                       double cx_off = 0.0, double cy_off = 0.0) {
  std::uniform_int_distribution<int> n(-amp, amp);
  std::vector<uint8_t> img(static_cast<size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const double dx = x - (w / 2 + cx_off), dy = y - (h / 2 + cy_off);
      const bool disk = panel != Panel::Unknown && dx * dx + dy * dy < 18 * 18;
      const int noise = n(rng);
      for (int k = 0; k < 3; ++k) {
        const int v = disk ? (k == (panel == Panel::Go ? 1 : 0) ? 200 : 25) : 35;
        img[(static_cast<size_t>(y) * w + x) * 3 + k] = static_cast<uint8_t>(std::clamp(v + noise, 0, 255));
      }
    }
  return img;
}

static void test_panel_confusion() {
  std::mt19937 rng(20260501u);  // held-out seed, unrelated to simulator seeds
  int cm[3][3] = {{0}};
  const int N = 400;
  for (int t = 0; t < 3; ++t)
    for (int i = 0; i < N; ++i) {
      // Vary noise amplitude up to 1.5x the documented bound and jitter the disc by +/-2 px.
      std::uniform_int_distribution<int> amp(0, 27), off(-2, 2);
      const Panel truth = t == 0 ? Panel::Unknown : (t == 1 ? Panel::Stop : Panel::Go);
      const auto img = make_panel(truth, rng, amp(rng), 96, 64, off(rng), off(rng));
      const PanelResult r = classify_panel(img, 96, 64, 96 * 3, "rgb8");
      CHECK(r.valid);
      const int prediction = r.cls == Panel::Unknown ? 0 : (r.cls == Panel::Stop ? 1 : 2);
      ++cm[t][prediction];
    }
  std::printf("panel confusion matrix (rows/cols = UNKNOWN, STOP, GO), N=%d per class\n", N);
  const char* names[3] = {"UNKNOWN", "STOP", "GO"};
  for (int t = 0; t < 3; ++t)
    std::printf("  %-8s %4d %4d %4d\n", names[t], cm[t][0], cm[t][1], cm[t][2]);
  CHECK(cm[0][1] == 0 && cm[0][2] == 0);  // blank panels never authorize either state
  CHECK(cm[1][1] >= N * 99 / 100);
  CHECK(cm[2][2] >= N * 99 / 100);
}

static void test_panel_malformed() {
  std::mt19937 rng(1);
  auto img = make_panel(Panel::Go, rng);
  CHECK(!classify_panel(img, 96, 64, 96 * 3, "bgr8").valid);
  CHECK(!classify_panel(img, 96, 64, 96 * 2, "rgb8").valid);
  std::vector<uint8_t> shorter(img.begin(), img.begin() + 1000);
  CHECK(!classify_panel(shorter, 96, 64, 96 * 3, "rgb8").valid);
  // Fully coloured frame (not a centred panel) must not be GO.
  std::vector<uint8_t> green(96 * 64 * 3);
  for (size_t i = 0; i < green.size(); i += 3) { green[i] = 20; green[i + 1] = 210; green[i + 2] = 20; }
  CHECK(classify_panel(green, 96, 64, 96 * 3, "rgb8").cls != Panel::Go);
}

static void test_panel_filter() {
  PanelFilter f(3, 0.15);
  CHECK(f.update(0.0, 0.0, Panel::Go));
  CHECK(f.state() == Panel::Unknown);
  CHECK(f.update(0.1, 0.1, Panel::Go));
  CHECK(f.state() == Panel::Unknown);
  CHECK(f.update(0.2, 0.2, Panel::Go));
  CHECK(f.state() == Panel::Go);
  CHECK(!f.update(0.2, 0.3, Panel::Go));    // repeated frame
  CHECK(!f.update(1.0, 0.3, Panel::Go));    // future frame
  CHECK(f.update(0.3, 0.3, Panel::Unknown));  // non-GO frame resets the count
  CHECK(f.state() == Panel::Unknown);
  CHECK(f.update(0.35, 0.35, Panel::Stop));  // STOP is explicit and immediate
  CHECK(f.state() == Panel::Stop);
  CHECK(f.update(0.4, 0.4, Panel::Go));
  CHECK(f.go_count() == 1);
  CHECK(f.update(0.5, 0.5, Panel::Go));
  CHECK(f.update(0.9, 0.9, Panel::Go));     // gap > max_gap resets the count
  CHECK(f.go_count() == 1);
  CHECK(f.rejected() == 2);
}

static BeaconSample beacon(double t) {
  BeaconSample b;
  b.stamp = t;
  b.valid_for = 1.2;
  b.g = Geometry{25, 42, 49, 68, 4, 1};
  return b;
}

static void test_beacon() {
  BeaconMonitor m;
  CHECK(!m.geometry_known());
  CHECK(m.update(beacon(0.0), 0.0));
  CHECK(m.geometry_known());
  CHECK_NEAR(m.geometry().entry_x, 25.0, 1e-12);
  CHECK(!m.update(beacon(0.0), 0.5));  // repeated
  auto bad = beacon(1.0);
  bad.g.pass_side = -1;
  CHECK(!m.update(bad, 1.0));
  bad = beacon(1.0);
  bad.g.work_end = 40;  // inconsistent ordering
  CHECK(!m.update(bad, 1.0));
  bad = beacon(1.0);
  bad.valid_for = std::nan("");
  CHECK(!m.update(bad, 1.0));
  CHECK(!m.update(beacon(5.0), 1.0));  // future
  CHECK(m.rejected() == 5);
  bad = beacon(1.0);
  bad.valid_for = 0.0;  // a beacon without a positive validity interval can never be fresh
  CHECK(!m.update(bad, 1.0));
  CHECK(m.update(beacon(1.0), 1.0));
  CHECK_NEAR(m.valid_for(), 1.2, 1e-12);
}

static void test_sense_core() {
  SenseCore core;
  EgoState e;
  e.stamp = 0.0; e.x = 5; e.y = -1.75;  // valid left false: SenseCore sets it
  CHECK(core.on_odom(e, 0.0));
  CHECK(!core.on_odom(e, 0.0));  // repeated
  SceneData s = core.scene(0.0);
  CHECK(s.ego.valid);
  CHECK(!s.geometry_known);
  CHECK(s.panel_state == Panel::Unknown);
  CHECK(core.scene(0.5).ego.valid == false);  // odometry stale

  SenseCore panel_core;
  std::mt19937 rng(1);
  CHECK(panel_core.on_image(0.0, make_panel(Panel::Stop, rng), 96, 64, 96 * 3, "rgb8", 0.0));
  CHECK(panel_core.scene(0.1).panel_state == Panel::Stop);
  CHECK(panel_core.scene(0.31).panel_state == Panel::Unknown);  // stale evidence cannot authorize motion
}

// Radar detection of a map-frame object seen from ego pose e (exact, no noise).
static RadarDet radar_of(const EgoState& e, double ox, double oy, double ovx) {
  const double dx = ox - e.x, dy = oy - e.y, r = std::hypot(dx, dy);
  RadarDet d;
  d.range = r;
  d.azimuth = wrap_angle(std::atan2(dy, dx) - e.yaw);
  d.radial_velocity = ((ovx - e.v * std::cos(e.yaw)) * dx - e.v * std::sin(e.yaw) * dy) / r;
  return d;
}

static void test_tracker() {
  {  // Static works seen from a moving ego: compensated, no track.
    TrafficTracker tr;
    EgoState e;
    e.x = 20; e.y = -1.75; e.v = 3.5;
    for (int k = 0; k < 5; ++k) {
      e.x += 0.35;
      CHECK(tr.update(0.1 * k, e, {radar_of(e, 43, -1.75, 0.0), radar_of(e, 47, -1.75, 0.0)}) == 0);
    }
    CHECK(tr.tracks().empty());
  }
  {  // Oncoming vehicle in the opposing lane: confirmed track with vx ~ -3 m/s.
    TrafficTracker tr;
    EgoState e;
    e.x = 23.1; e.y = -1.75; e.v = 0.0;
    double cx = 60.0;
    for (int k = 0; k < 5; ++k, cx -= 0.3) tr.update(0.1 * k, e, {radar_of(e, cx, 1.75, -3.0)});
    CHECK(tr.tracks().size() == 1);
    const Track& t = tr.tracks().front();
    CHECK(t.confirmed && !t.coasting);
    CHECK_NEAR(t.vx, -3.0, 0.05);
    CHECK_NEAR(t.x, cx + 0.3, 0.05);
    CHECK_NEAR(t.y, 1.75, 0.05);
    // Drive the car past the ego with the real +/-60 deg FOV: beside and behind
    // the ego there are no detections and the track coasts on its prediction.
    for (int k = 5; k < 130; ++k, cx -= 0.3) {
      const RadarDet d = radar_of(e, cx, 1.75, -3.0);
      tr.update(0.1 * k, e, std::abs(d.azimuth) <= kPi / 3 ? std::vector<RadarDet>{d} : std::vector<RadarDet>{});
    }
    CHECK(tr.tracks().size() == 1);
    CHECK(tr.tracks().front().coasting);
    CHECK(tr.tracks().front().hits >= 100);
    CHECK_NEAR(tr.tracks().front().x_at(12.9), 60.0 - 3.0 * 12.9, 0.3);
  }
  {  // A confirmed track predicted INSIDE the FOV but not detected is deleted after miss_limit scans.
    TrafficTracker tr;
    EgoState e;
    e.x = 5; e.y = -1.75;
    for (int k = 0; k < 3; ++k) tr.update(0.1 * k, e, {radar_of(e, 70 - 0.3 * k, 1.75, -3.0)});
    CHECK(tr.tracks().size() == 1);
    tr.update(0.3, e, {});
    tr.update(0.4, e, {});
    CHECK(tr.tracks().size() == 1);
    tr.update(0.5, e, {});
    CHECK(tr.tracks().empty());
  }
}

static void test_sense_radar() {
  SenseCore core;
  EgoState e;
  e.stamp = 1.0; e.x = 23.1; e.y = -1.75;
  // No odometry yet: the scan cannot be associated with a pose.
  CHECK(!core.on_radar(1.0, {radar_of(e, 60, 1.75, -3)}, 1.0));
  CHECK(core.on_odom(e, 1.0));
  RadarDet bad;
  bad.range = std::nan("");
  CHECK(core.on_radar(1.0, {radar_of(e, 60, 1.75, -3), bad}, 1.0));  // malformed detection dropped
  CHECK(!core.on_radar(1.0, {}, 1.05));                               // repeated scan
  CHECK(!core.on_radar(2.0, {}, 1.05));                               // future scan
  const SceneData s = core.scene(1.05);
  CHECK(s.radar_detections == 1 && s.radar_moving == 1);
  CHECK(s.radar_rejected == 4);  // no pose, malformed detection, repeated, future
  CHECK_NEAR(s.radar_stamp, 1.0, 1e-12);
  CHECK(s.tracks.size() == 1 && !s.tracks.front().confirmed);
  // A scan 0.05 s newer than the newest odometry is associated by extrapolation.
  e.stamp = 1.05; e.v = 2.0;
  CHECK(core.on_odom(e, 1.05));
  CHECK(core.on_radar(1.10, {}, 1.10));
  CHECK_NEAR(core.scene(1.1).radar_pose_dt, 0.05, 1e-9);
  CHECK(!core.on_radar(1.40, {}, 1.40));  // 0.35 s from any odometry sample: rejected
}

// Independent 2D ray caster following the LiDAR contract (360 beams from -pi,
// road edges at y = +/-3.5, circles), used to build synthetic scans.
struct Circ { double x, y, r; };
static LidarScanIn make_scan(double stamp, const EgoState& e, const std::vector<Circ>& objs) {
  LidarScanIn s;
  s.stamp = stamp; s.angle_min = -kPi; s.angle_increment = 2 * kPi / 360; s.range_min = 0.1; s.range_max = 40;
  s.ranges.assign(360, std::numeric_limits<float>::infinity());
  for (int i = 0; i < 360; ++i) {
    const double a = e.yaw - kPi + 2 * kPi * i / 360.0, dx = std::cos(a), dy = std::sin(a);
    double best = 40.0;
    bool hit = false;
    if (std::abs(dy) > 1e-9) {
      const double t = ((dy > 0 ? 3.5 : -3.5) - e.y) / dy;
      if (t > 0 && t < best) { best = t; hit = true; }
    }
    for (const auto& o : objs) {
      const double ox = o.x - e.x, oy = o.y - e.y, p = ox * dx + oy * dy, disc = o.r * o.r - (ox * ox + oy * oy - p * p);
      if (disc >= 0) {
        const double t = p - std::sqrt(disc);
        if (t > 0 && t < best) { best = t; hit = true; }
      }
    }
    if (hit) s.ranges[i] = static_cast<float>(best);
  }
  return s;
}

static void test_lidar() {
  EgoState e;
  e.stamp = 2.0; e.x = 45; e.y = 1.25; e.v = 3;
  {  // Road edges are removed; the works and a temporary obstacle form separate clusters.
    LidarResult r;
    const auto scan = make_scan(2.0, e, {{43, -1.75, 1.0}, {47, -1.75, 1.0}, {57, 1.75, 0.65}});
    CHECK(process_scan(scan, e, &r));
    CHECK(r.returns > 300);  // most beams hit an edge
    for (const auto& p : r.points) CHECK(std::abs(p.y) < 3.25);
    CHECK(r.clusters.size() == 3);
    int near_obstacle = 0;
    for (const auto& c : r.clusters) {
      if (std::hypot(c.x - 57, c.y - 1.75) < 0.65 + 0.2) {
        ++near_obstacle;
        CHECK(c.n >= 4);
        CHECK_NEAR(c.range_min, std::hypot(57 - 45, 0.5) - 0.65, 0.05);
      }
    }
    CHECK(near_obstacle == 1);
    // Every obstacle return lies on one of the circles (projection uses the pose).
    for (const auto& p : r.points) {
      const double d = std::min({std::abs(std::hypot(p.x - 43, p.y + 1.75) - 1.0),
                                 std::abs(std::hypot(p.x - 47, p.y + 1.75) - 1.0),
                                 std::abs(std::hypot(p.x - 57, p.y - 1.75) - 0.65)});
      CHECK(d < 1e-3);
    }
  }
  {  // Malformed scans and rays.
    LidarResult r;
    auto scan = make_scan(2.0, e, {});
    scan.ranges[10] = std::nanf("");
    scan.ranges[11] = -std::numeric_limits<float>::infinity();
    CHECK(process_scan(scan, e, &r));
    CHECK(r.malformed == 2);
    auto bad = scan;
    bad.angle_increment = 0;
    CHECK(!process_scan(bad, e, &r));
    bad = scan;
    bad.angle_increment = 2 * kPi / 300;  // 360 beams would exceed one revolution
    CHECK(!process_scan(bad, e, &r));
    bad = scan;
    bad.ranges.clear();
    CHECK(!process_scan(bad, e, &r));
  }
  {  // SenseCore: the scan is projected with the pose of ITS stamp, not the newest one.
    SenseCore core;
    EgoState e0 = e, e1 = e;
    e1.stamp = 2.05; e1.x = 45.15;
    const auto scan = make_scan(2.0, e0, {{57, 1.75, 0.65}});
    CHECK(!core.on_scan(scan, 2.05));  // no odometry yet
    CHECK(core.on_odom(e0, 2.0));
    CHECK(core.on_odom(e1, 2.05));
    CHECK(core.on_scan(scan, 2.05));
    const SceneData s = core.scene(2.05);
    CHECK_NEAR(s.lidar_stamp, 2.0, 1e-12);
    CHECK_NEAR(s.lidar_pose_dt, 0.0, 1e-12);
    CHECK(!s.obstacle_points.empty());
    for (const auto& p : s.obstacle_points) CHECK(std::abs(std::hypot(p.x - 57, p.y - 1.75) - 0.65) < 1e-3);
    CHECK(!core.on_scan(scan, 2.1));  // repeated scan
    CHECK(core.scene(2.1).lidar_rejected == 2);
  }
}

int main() {
  test_panel_confusion();
  test_panel_malformed();
  test_panel_filter();
  test_beacon();
  test_sense_core();
  test_tracker();
  test_sense_radar();
  test_lidar();
  return TEST_SUMMARY("test_perception");
}
