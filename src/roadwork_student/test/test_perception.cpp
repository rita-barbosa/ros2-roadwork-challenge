// Unit tests for perception building blocks. The panel test also prints a
// confusion matrix on independently generated samples (own RNG and seeds, not
// the simulator's), used as held-out recognition evidence in the report.
#include <random>

#include "check.hpp"
#include "roadwork_student/perception.hpp"
#include "roadwork_student/sense_core.hpp"

using namespace rws;

// Independent generator following the documented panel contract:
// 96x64 rgb8, disc radius 18 centred, dominant channel ~200, others ~25,
// background ~35, uniform per-pixel noise of +/-amp shared by the channels.
// go=true draws a green disc, go=false a blank panel.
static std::vector<uint8_t> make_panel(bool go, std::mt19937& rng, int amp = 18, int w = 96, int h = 64,
                                       double cx_off = 0.0, double cy_off = 0.0) {
  std::uniform_int_distribution<int> n(-amp, amp);
  std::vector<uint8_t> img(static_cast<size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const double dx = x - (w / 2 + cx_off), dy = y - (h / 2 + cy_off);
      const bool disk = go && dx * dx + dy * dy < 18 * 18;
      const int noise = n(rng);
      for (int k = 0; k < 3; ++k) {
        const int v = disk ? (k == 1 ? 200 : 25) : 35;
        img[(static_cast<size_t>(y) * w + x) * 3 + k] = static_cast<uint8_t>(std::clamp(v + noise, 0, 255));
      }
    }
  return img;
}

static void test_panel_confusion() {
  std::mt19937 rng(20260501u);  // held-out seed, unrelated to simulator seeds
  int cm[2][2] = {{0}};
  const int N = 400;
  for (int t = 0; t < 2; ++t)
    for (int i = 0; i < N; ++i) {
      // Vary noise amplitude up to 1.5x the documented bound and jitter the disc by +/-2 px.
      std::uniform_int_distribution<int> amp(0, 27), off(-2, 2);
      const auto img = make_panel(t == 1, rng, amp(rng), 96, 64, off(rng), off(rng));
      const PanelResult r = classify_panel(img, 96, 64, 96 * 3, "rgb8");
      CHECK(r.valid);
      ++cm[t][r.cls == Panel::Go ? 1 : 0];
    }
  std::printf("panel confusion matrix (rows = truth, cols = predicted UNKNOWN/GO), N=%d per class\n", N);
  const char* names[2] = {"BLANK", "GO"};
  for (int t = 0; t < 2; ++t) std::printf("  %-8s %4d %4d\n", names[t], cm[t][0], cm[t][1]);
  CHECK(cm[0][1] == 0);  // no false GO from blank panels
  CHECK(cm[1][1] >= N * 99 / 100);
}

static void test_panel_malformed() {
  std::mt19937 rng(1);
  auto img = make_panel(true, rng);
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
}

int main() {
  test_panel_confusion();
  test_panel_malformed();
  test_panel_filter();
  test_beacon();
  test_sense_core();
  return TEST_SUMMARY("test_perception");
}
