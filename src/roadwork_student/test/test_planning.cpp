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
  s.geometry_known = true; s.beacon_stamp = now; s.beacon_valid_for = 1.2;
  s.geometry = geom(0);
  s.radar_stamp = now;  // fresh radar, no moving traffic
  return s;
}

static Track track(int id, double stamp, double x, double vx) {
  Track t;
  t.id = id; t.stamp = stamp; t.x = x; t.y = 1.75; t.vx = vx; t.hits = 5; t.confirmed = true;
  return t;
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
  {  // No confirmed GO: approach in WAIT_GO and hold at the stop point before the line.
    Planner pl;
    const double stop_x = 25 - kEgoRadius - pl.config().stop_distance;  // reference x at the stop point
    const PlanOutput far = pl.step(scene(0.1, 5, 0, Panel::Stop), 0.1);
    CHECK(far.state == State::WaitGo);
    CHECK(far.speed > 0.0 && !far.hold);  // far from the line: drive towards it
    CHECK(!pl.go_latched());
    const PlanOutput near = pl.step(scene(0.2, stop_x - 2.0, 3.0, Panel::Unknown), 0.2);
    CHECK(near.state == State::WaitGo);
    CHECK(near.speed > 0.0 && near.speed < 3.0);  // braking towards the stop point
    const PlanOutput at = pl.step(scene(0.3, stop_x, 0, Panel::Stop), 0.3);
    CHECK(at.state == State::WaitGo);
    CHECK(at.speed == 0.0 && at.hold);
    CHECK_NEAR(at.d_line, pl.config().stop_distance, 1e-9);
    CHECK(pl.step(scene(0.4, stop_x + 0.05, 0, Panel::Stop), 0.4).speed == 0.0);  // slight overshoot: hold
    CHECK(!pl.go_latched());
    // GO at the stop line: drive on; GO is latched once stopping before the line is impossible.
    const PlanOutput go = pl.step(scene(0.5, stop_x, 0, Panel::Go), 0.5);
    CHECK(go.state == State::Approach);
    CHECK(go.speed > 0.0 && !go.hold);
    CHECK(!pl.go_latched());  // at rest it could still stop before the line
    pl.step(scene(0.6, stop_x + 0.5, 2.0, Panel::Go), 0.6);
    CHECK(pl.go_latched());
    const PlanOutput later = pl.step(scene(0.7, stop_x + 0.7, 2.0, Panel::Unknown), 0.7);  // latched
    CHECK(later.speed > 0.0 && later.state == State::Approach);
  }
  {  // GO far from the line is not yet a commitment: losing it brings the vehicle back to WAIT_GO.
    Planner pl;
    CHECK(pl.step(scene(1, 5, 0, Panel::Go), 1).state == State::Approach);
    CHECK(!pl.go_latched());
    const PlanOutput lost = pl.step(scene(2, 12, 3.4, Panel::Stop), 2);
    CHECK(lost.state == State::WaitGo);
    CHECK(lost.speed > 0.0);  // still far enough to stop comfortably: no early braking
    const PlanOutput brake = pl.step(scene(3, 20, 3.4, Panel::Stop), 3);
    CHECK(brake.state == State::WaitGo);
    CHECK(brake.speed < 3.0);
    CHECK(!pl.go_latched());
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

static void test_traffic_gap() {
  Planner pl;
  const double stop_x = 25 - kEgoRadius - pl.config().stop_distance;
  {  // Oncoming car in the opposing lane ahead: GO, but YIELD at the stop line.
    SceneData s = scene(1, stop_x, 0, Panel::Go);
    s.tracks = {track(1, 1, 60, -3)};
    const PlanOutput o = pl.step(s, 1);
    CHECK(o.state == State::Yield);
    CHECK(!o.gap_ok && o.conflict_id == 1);
    CHECK(o.speed == 0.0 && o.hold);
    CHECK(!pl.go_latched());
  }
  {  // Unconfirmed tracks and tracks in the own lane do not block.
    SceneData s = scene(2, stop_x, 0, Panel::Go);
    Track t = track(2, 2, 60, -3);
    t.confirmed = false;
    Track own = track(3, 2, 60, -3);
    own.y = -1.75;
    s.tracks = {t, own};
    CHECK(pl.step(s, 2).state == State::Approach);
  }
  {  // Car already past the interval and moving away: safe.
    SceneData s = scene(3, stop_x, 0, Panel::Go);
    s.tracks = {track(4, 3, 20, -3)};
    const PlanOutput o = pl.step(s, 3);
    CHECK(o.gap_ok && o.state == State::Approach);
    CHECK(o.speed > 0.0);
  }
  {  // Car far away: arrives only after the ego has cleared the interval (+gap_time).
    SceneData s = scene(4, stop_x, 0, Panel::Go);
    s.tracks = {track(5, 4, 200, -3)};
    CHECK(pl.step(s, 4).gap_ok);
    s.tracks = {track(5, 4, 110, -3)};  // ~45 m before the interval end at 3 m/s: too close
    CHECK(!pl.step(s, 4).gap_ok);
  }
  {  // The prediction uses the plan time: an old track state is advanced to now.
    SceneData s = scene(5, stop_x, 0, Panel::Go);
    s.tracks = {track(6, 0.0, 33, -3)};  // at x=33 five seconds ago -> now at x=18
    CHECK(pl.step(s, 5).gap_ok);
  }
  {  // Stale radar: the opposing lane cannot be certified (information missing, not traffic).
    SceneData s = scene(6, stop_x, 0, Panel::Go);
    s.radar_stamp = 5.5;
    const PlanOutput o = pl.step(s, 6);
    CHECK(o.state == State::WaitInfo && !o.gap_ok && !o.radar_fresh);
    CHECK(o.speed == 0.0 && o.hold);
  }
  {  // Committed, a conflict appears before the opposing lane: stop before it if still possible.
    Planner p2;
    p2.step(scene(1, 5, 0, Panel::Go), 1);
    p2.step(scene(2, 22.5, 3.0, Panel::Go), 2);  // inside the braking distance of the line: committed
    CHECK(p2.go_latched());
    const double yield_x = p2.path().opp_lo() - p2.config().yield_before;
    SceneData s = scene(3, yield_x - 1.4, 2.0, Panel::Go);
    s.tracks = {track(7, 3, 50, -3)};
    const PlanOutput o = p2.step(s, 3);
    CHECK(o.state == State::Yield);
    CHECK(o.speed < 2.0 && o.v_yield < 2.0);
    // Too close to stop before the opposing lane: continue (committed).
    SceneData s2 = scene(4, yield_x - 0.3, 3.0, Panel::Go);
    s2.tracks = {track(7, 4, 47, -3)};
    CHECK(p2.step(s2, 4).state == State::Bypass);
  }
  {  // Ablation switch: the gap check is skipped.
    PlannerConfig c;
    c.use_traffic_gap = false;
    Planner p3(c);
    SceneData s = scene(1, stop_x, 0, Panel::Go);
    s.tracks = {track(8, 1, 60, -3)};
    CHECK(p3.step(s, 1).state == State::Approach);
  }
}

static void test_information_loss() {
  Planner pl;
  const double stop_x = 25 - kEgoRadius - pl.config().stop_distance;
  {  // GO, but the newest beacon is older than its validity: WAIT_INFO at the stop line.
    SceneData s = scene(10, stop_x, 0, Panel::Go);
    s.beacon_stamp = 8.5;
    const PlanOutput o = pl.step(s, 10);
    CHECK(o.state == State::WaitInfo && !o.beacon_fresh);
    CHECK(o.speed == 0.0 && o.hold);
    CHECK(!pl.go_latched());
    // Far from the line the vehicle still approaches (and later stops before it).
    SceneData far = scene(10.05, 8, 2, Panel::Go);
    far.beacon_stamp = 8.5;
    const PlanOutput f = pl.step(far, 10.05);
    CHECK(f.state == State::WaitInfo && f.speed > 0.0);
  }
  {  // A beacon announcing a shorter validity is honoured.
    SceneData s = scene(10, stop_x, 0, Panel::Go);
    s.beacon_stamp = 9.5; s.beacon_valid_for = 0.3;
    CHECK(pl.step(s, 10).state == State::WaitInfo);
  }
  {  // No beacon ever: INIT (no geometry), never entry.
    Planner p2;
    SceneData s = scene(1, stop_x, 0, Panel::Go);
    s.geometry_known = false; s.beacon_stamp = -1;
    CHECK(p2.step(s, 1).state == State::Init);
  }
  {  // Blank camera (UNKNOWN) together with a stale beacon: WAIT_GO explains the panel first.
    SceneData s = scene(10, stop_x, 0, Panel::Unknown);
    s.beacon_stamp = 8.5;
    const PlanOutput o = pl.step(s, 10);
    CHECK(o.state == State::WaitGo);
    CHECK(o.reason.find("blank") != std::string::npos);
  }
  {  // Recovery: fresh beacon and GO again -> entry permitted.
    const PlanOutput o = pl.step(scene(11, stop_x, 0, Panel::Go), 11);
    CHECK(o.state == State::Approach && o.beacon_fresh && o.speed > 0.0);
  }
  {  // After commitment the cached geometry carries the manoeuvre: beacon and camera loss do not stop it.
    Planner p3;
    p3.step(scene(1, 5, 0, Panel::Go), 1);
    p3.step(scene(2, 22.5, 3.0, Panel::Go), 2);
    CHECK(p3.go_latched());
    SceneData s = scene(5, 35, 3.0, Panel::Unknown);
    s.beacon_stamp = 2.0;
    s.camera_stamp = 2.0;
    const PlanOutput o = p3.step(s, 5);
    CHECK(o.state == State::Bypass && o.speed > 1.0 && !o.hold);
    CHECK(o.reason.find("cached geometry") != std::string::npos);
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
  test_traffic_gap();
  test_information_loss();
  test_guard();
  return TEST_SUMMARY("test_planning");
}
