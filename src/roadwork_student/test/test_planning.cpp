// Unit tests for path geometry, decision logic and the Act command guard.
#include "check.hpp"
#include "roadwork_student/control.hpp"
#include "roadwork_student/planning.hpp"

using namespace rws;

static Geometry geom(double shift) { return Geometry{25, 42 + shift, 49 + shift, 68 + shift, 4, 1}; }

static void test_path_geometry() {
  for (double shift : {-2.0, 0.0, 2.0}) {
    const Geometry g = geom(shift);
    ReferencePath p;
    std::string why;
    CHECK(p.build(g, -5, 95, PathConfig{}, SpeedConfig{}, &why));
    CHECK(p.max_steer() < 0.35);
    const double works[2] = {43 + shift, 47 + shift};
    for (const auto& q : p.points()) {
      // Lane rule: no opposing-lane occupancy before entry or after exit (disc edge, with 0.3 m margin).
      if (q.x + kEgoRadius < g.entry_x) CHECK(q.y + kEgoRadius <= -0.3);
      if (q.x - kEgoRadius > g.exit_x) CHECK(q.y + kEgoRadius <= -0.3);
      // Road edges, with margin.
      CHECK(std::abs(q.y) + kEgoRadius <= kRoadHalfWidth - 0.5);
      // Disc-to-disc clearance to both work circles (radius 1.0) of at least 0.9 m on the path itself.
      for (double wx : works) CHECK(std::hypot(q.x - wx, q.y - kOwnLaneY) - 1.0 - kEgoRadius >= 0.9);
      CHECK(q.vmax > 1.0 && q.vmax <= 3.5 + 1e-9);
    }
    CHECK_NEAR(p.y_at(10), kOwnLaneY, 1e-12);
    CHECK_NEAR(p.y_at(45 + shift), 1.25, 1e-12);
    CHECK_NEAR(p.y_at(80.2), kOwnLaneY, 1e-12);
  }
  ReferencePath p;
  Geometry tight = geom(0);
  tight.work_start = 28;  // lane change would be too short
  CHECK(!p.build(tight, -5, 95, PathConfig{}, SpeedConfig{}));
}

static SceneData scene(double now, double x, double v, Panel panel) {
  SceneData s;
  s.stamp = now;
  s.ego.stamp = now; s.ego.x = x; s.ego.y = kOwnLaneY; s.ego.v = v; s.ego.valid = true;
  s.panel_state = panel; s.panel_raw = panel; s.camera_stamp = now;
  s.geometry_known = true; s.beacon_stamp = now;
  s.geometry = geom(0);
  return s;
}

static void test_decisions() {
  {  // No beacon geometry: stay in INIT and hold.
    Planner pl;
    SceneData s = scene(0, 5, 0, Panel::Go);
    s.geometry_known = false;
    const PlanOutput o = pl.step(s, 0);
    CHECK(o.state == State::Init);
    CHECK(o.speed == 0.0 && o.hold);
  }
  {  // GO not yet confirmed: hold in WAIT_GO; once GO is confirmed it is latched.
    Planner pl;
    const PlanOutput w = pl.step(scene(0.2, 5, 0, Panel::Unknown), 0.2);
    CHECK(w.state == State::WaitGo);
    CHECK(w.speed == 0.0 && w.hold);
    CHECK(!pl.go_latched());
    const PlanOutput go = pl.step(scene(0.4, 5, 0, Panel::Go), 0.4);
    CHECK(pl.go_latched());
    CHECK(go.state == State::Approach);
    CHECK(go.speed > 0.0 && !go.hold);
    const PlanOutput later = pl.step(scene(0.5, 6, 1.0, Panel::Unknown), 0.5);  // latched
    CHECK(later.speed > 0.0);
  }
  {  // States follow the position along the route.
    Planner pl;
    pl.step(scene(1, 5, 0, Panel::Go), 1);
    CHECK(pl.step(scene(2, 30, 3, Panel::Go), 2).state == State::Bypass);
    CHECK(pl.step(scene(3, 55, 3, Panel::Go), 3).state == State::Return);
    CHECK(pl.step(scene(4, 70, 3, Panel::Go), 4).state == State::Final);
    const PlanOutput g = pl.step(scene(5, 80.15, 0.0, Panel::Go), 5);
    CHECK(g.state == State::Goal);
    CHECK(g.speed == 0.0);
    CHECK(pl.step(scene(6, 80.15, 0.0, Panel::Go), 6).state == State::Goal);  // latched
  }
  {  // Odometry or scene too old => SAFE_STOP.
    Planner pl;
    SceneData s = scene(1, 10, 2, Panel::Go);
    CHECK(pl.step(s, 1.5).state == State::SafeStop);
    CHECK(pl.timeout_output(2, "no scene").speed == 0.0);
  }
}

static void test_guard() {
  CommandGuard g;
  PlanInput p;
  GuardOutput o = g.step(p, 1.0);
  CHECK(!o.plan_ok && o.speed == 0.0);
  p.have = true; p.stamp = 1.0; p.valid_until = 1.25; p.speed = 10; p.steering = 1.0; p.speed_limit = 3.7; p.hold = false;
  o = g.step(p, 1.05);
  CHECK(o.plan_ok);
  CHECK_NEAR(o.speed, 3.7, 1e-12);
  CHECK_NEAR(o.steering, 0.6 * 0.05, 1e-9);  // slew-limited
  o = g.step(p, 1.30);  // expired
  CHECK(!o.plan_ok && o.speed == 0.0);
  const double held = o.steering;
  CHECK(std::abs(held) > 0.0);
  p.stamp = 2.0; p.valid_until = 2.25;
  o = g.step(p, 1.35);  // future stamp beyond tolerance
  CHECK(!o.plan_ok && o.speed == 0.0);
  p.stamp = 1.4; p.valid_until = 1.65; p.speed = std::nan("");
  CHECK(!g.step(p, 1.45).plan_ok);
  p.speed = 2.0; p.hold = true;
  o = g.step(p, 1.5);
  CHECK(o.plan_ok && o.speed == 0.0);
}

int main() {
  test_path_geometry();
  test_decisions();
  test_guard();
  return TEST_SUMMARY("test_planning");
}
