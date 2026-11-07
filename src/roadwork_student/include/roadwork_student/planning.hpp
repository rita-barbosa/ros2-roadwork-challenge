// Planning: reference path, speed profile, pure pursuit and the S1 decision
// state machine used by plan_node (ROS-free).
#pragma once
#include <limits>
#include <string>
#include <vector>

#include "roadwork_student/common.hpp"

namespace rws {

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct PathConfig {
  double own_y = kOwnLaneY;      // own-lane centre
  double bypass_y = 1.25;        // lateral offset while passing the works
  double start_before_entry = 0.5;   // lane change starts at entry_x - this
  double settle_before_work = 2.0;   // fully on the bypass at work_start - this
  double leave_after_work = 3.0;     // start returning at work_end + this
  double settle_before_exit = 2.5;   // back in own lane at exit_x - this
  double ds = 0.2;                   // sampling step along x
  double extend_after_goal = 15.0;   // path continues past the goal (look-ahead)
  double min_transition = 8.0;       // reject geometry giving shorter lane changes
};

struct SpeedConfig {
  double cruise = 3.5;        // m/s, below the 4 m/s limit
  double lat_accel = 1.5;     // m/s^2, v <= sqrt(a_lat / |kappa|)
  double steer_rate = 0.5;    // rad/s, below the 0.6 rad/s slew: v <= rate / |d delta / ds|
  double decel = 1.2;         // m/s^2 used to slow down before curvature limits
  double horizon = 15.0;      // m of path examined for the speed profile
};

struct PlannerConfig {
  PathConfig path;
  SpeedConfig speed;
  // Input deadlines (simulation time)
  double scene_timeout = 0.25, odom_timeout = 0.20;
  double plan_validity = 0.25;   // Plan.valid_until = stamp + this
  double control_delay = 0.15;   // s, subtracted from distances in the speed profiles
  // Pure pursuit
  double lookahead_min = 3.0, lookahead_gain = 0.5, lookahead_max = 6.0;
  // Goal
  double goal_x = 80.35, goal_done_x = 80.10, goal_decel = 1.0, creep_speed = 0.35;
  double speed_margin = 0.3;       // stay this far below the beacon speed limit
};

// ---------------------------------------------------------------------------
// Reference path y(x): own lane -> smooth left bypass -> smooth return -> own lane
// ---------------------------------------------------------------------------
// Lane changes use the quintic "smootherstep" p(s) = 10s^3 - 15s^4 + 6s^5,
// whose first and second derivatives vanish at both ends, so curvature is
// continuous (zero at the joins) and the steering never has to jump.
struct PathPoint {
  double x = 0, y = 0, yaw = 0, kappa = 0, dkappa = 0, s = 0, vmax = 0;
};

class ReferencePath {
 public:
  bool build(const Geometry& g, double x_begin, double x_end, const PathConfig& c, const SpeedConfig& sc,
             std::string* why = nullptr) {
    pts_.clear();
    max_steer_ = 0.0;
    cfg_ = c;
    x0_ = g.entry_x - c.start_before_entry;
    x1_ = g.work_start - c.settle_before_work;
    x2_ = g.work_end + c.leave_after_work;
    x3_ = g.exit_x - c.settle_before_exit;
    if (x1_ - x0_ < c.min_transition || x3_ - x2_ < c.min_transition || x2_ <= x1_) {
      if (why) *why = "work-zone geometry leaves too little room for the lane changes";
      return false;
    }
    x_begin_ = x_begin;
    double s = 0.0, px = 0, py = 0;
    for (double x = x_begin; x <= x_end + 1e-9; x += c.ds) {
      double y, y1, y2, y3;
      eval(x, &y, &y1, &y2, &y3);
      PathPoint p;
      p.x = x;
      p.y = y;
      p.yaw = std::atan(y1);
      const double q = 1.0 + y1 * y1;
      p.kappa = y2 / std::pow(q, 1.5);
      const double dk_dx = y3 / std::pow(q, 1.5) - 3.0 * y1 * y2 * y2 / std::pow(q, 2.5);
      p.dkappa = dk_dx / std::sqrt(q);
      if (!pts_.empty()) s += std::hypot(x - px, y - py);
      p.s = s;
      // Speed limits from lateral acceleration and steering-rate feasibility.
      double v = sc.cruise;
      if (std::abs(p.kappa) > 1e-6) v = std::min(v, std::sqrt(sc.lat_accel / std::abs(p.kappa)));
      const double Lk = kWheelbase * p.kappa;
      const double ddelta_ds = kWheelbase / (1.0 + Lk * Lk) * std::abs(p.dkappa);
      if (ddelta_ds > 1e-6) v = std::min(v, sc.steer_rate / ddelta_ds);
      p.vmax = v;
      max_steer_ = std::max(max_steer_, std::abs(std::atan(Lk)));
      pts_.push_back(p);
      px = x;
      py = y;
    }
    if (max_steer_ > kMaxSteer - 0.05) {
      if (why) *why = "path curvature exceeds steering capability";
      pts_.clear();
      return false;
    }
    return true;
  }

  bool valid() const { return !pts_.empty(); }
  const std::vector<PathPoint>& points() const { return pts_; }
  double x0() const { return x0_; }
  double x1() const { return x1_; }
  double x2() const { return x2_; }
  double x3() const { return x3_; }
  double max_steer() const { return max_steer_; }

  double y_at(double x) const {
    double y, a, b, c;
    eval(x, &y, &a, &b, &c);
    return y;
  }

  size_t index_at_x(double x) const {
    if (pts_.empty()) return 0;
    const double f = (x - x_begin_) / cfg_.ds;
    if (f <= 0) return 0;
    return std::min(pts_.size() - 1, static_cast<size_t>(f));
  }

  size_t nearest(double x, double y) const {
    const size_t c = index_at_x(x);
    const size_t lo = c > 30 ? c - 30 : 0, hi = std::min(pts_.size() - 1, c + 30);
    size_t best = c;
    double bd = std::numeric_limits<double>::infinity();
    for (size_t i = lo; i <= hi; ++i) {
      const double d = std::hypot(pts_[i].x - x, pts_[i].y - y);
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

 private:
  static void smootherstep(double s, double* p, double* p1, double* p2, double* p3) {
    *p = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
    *p1 = 30.0 * s * s * (1.0 - s) * (1.0 - s);
    *p2 = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s);
    *p3 = 60.0 * (1.0 - 6.0 * s + 6.0 * s * s);
  }
  void eval(double x, double* y, double* y1, double* y2, double* y3) const {
    const double A = cfg_.bypass_y - cfg_.own_y;
    *y1 = *y2 = *y3 = 0.0;
    if (x <= x0_) { *y = cfg_.own_y; return; }
    if (x >= x3_) { *y = cfg_.own_y; return; }
    if (x >= x1_ && x <= x2_) { *y = cfg_.bypass_y; return; }
    double p, p1, p2, p3;
    if (x < x1_) {
      const double L = x1_ - x0_;
      smootherstep((x - x0_) / L, &p, &p1, &p2, &p3);
      *y = cfg_.own_y + A * p;
      *y1 = A * p1 / L; *y2 = A * p2 / (L * L); *y3 = A * p3 / (L * L * L);
    } else {
      const double L = x3_ - x2_;
      smootherstep((x - x2_) / L, &p, &p1, &p2, &p3);
      *y = cfg_.bypass_y - A * p;
      *y1 = -A * p1 / L; *y2 = -A * p2 / (L * L); *y3 = -A * p3 / (L * L * L);
    }
  }

  std::vector<PathPoint> pts_;
  PathConfig cfg_;
  double x_begin_ = 0, x0_ = 0, x1_ = 0, x2_ = 0, x3_ = 0, max_steer_ = 0;
};

// Pure pursuit on the rear-axle reference. Returns the front-wheel angle.
inline double pure_pursuit(const ReferencePath& path, size_t nearest, const EgoState& e, double lookahead,
                           Point2* target = nullptr) {
  const auto& p = path.points();
  size_t k = nearest;
  while (k + 1 < p.size() && p[k].s - p[nearest].s < lookahead) ++k;
  const double dx = p[k].x - e.x, dy = p[k].y - e.y;
  const double lx = std::cos(e.yaw) * dx + std::sin(e.yaw) * dy;
  const double ly = -std::sin(e.yaw) * dx + std::cos(e.yaw) * dy;
  const double ld2 = std::max(1e-6, lx * lx + ly * ly);
  if (target) *target = Point2{p[k].x, p[k].y};
  return clampd(std::atan(kWheelbase * 2.0 * ly / ld2), -kMaxSteer, kMaxSteer);
}

// ---------------------------------------------------------------------------
// Decision state machine (S1: GO throughout, static works)
// ---------------------------------------------------------------------------
// INIT (no odometry/geometry) -> WAIT_GO (hold until the panel is confirmed GO)
// -> APPROACH -> BYPASS -> RETURN -> FINAL -> GOAL. GO is latched once confirmed.
// SAFE_STOP: odometry or scene missing/stale (braking).
enum class State : uint8_t {
  Init = 0, WaitGo = 1, Approach = 2, Bypass = 3, Return = 4, Final = 5, Goal = 6, SafeStop = 7
};

inline const char* to_string(State s) {
  switch (s) {
    case State::Init: return "INIT";
    case State::WaitGo: return "WAIT_GO";
    case State::Approach: return "APPROACH";
    case State::Bypass: return "BYPASS";
    case State::Return: return "RETURN";
    case State::Final: return "FINAL";
    case State::Goal: return "GOAL";
    default: return "SAFE_STOP";
  }
}

struct PlanOutput {
  double stamp = 0, valid_until = 0, scene_stamp = -1, odom_stamp = -1;
  double speed = 0, steering = 0, speed_limit = 0;
  bool hold = true;
  State state = State::Init;
  std::string reason;
  // diagnostics (published on /solution/state)
  bool go = false;  // GO latched
  double cte = 0, path_s = 0, v_path = 0, v_goal = 0, lookahead = 0;
  Point2 target;
};

class Planner {
 public:
  explicit Planner(const PlannerConfig& c = PlannerConfig{}) : cfg_(c) {}

  const ReferencePath& path() const { return path_; }
  bool go_latched() const { return go_; }
  const PlannerConfig& config() const { return cfg_; }

  // Called when no scene arrives at all (input deadline missed).
  PlanOutput timeout_output(double now, const std::string& why) {
    PlanOutput o;
    base(&o, now);
    return stop(o, goal_reached_ ? State::Goal : State::SafeStop, why);
  }

  PlanOutput step(const SceneData& sc, double now) {
    PlanOutput o;
    base(&o, now);
    o.scene_stamp = sc.stamp;
    o.odom_stamp = sc.ego.stamp;
    if (goal_reached_) return stop(o, State::Goal, "goal reached");
    if (!sc.ego.valid || !is_fresh(now, sc.ego.stamp, cfg_.odom_timeout)) return stop(o, State::SafeStop, "odometry missing or stale");
    if (!is_fresh(now, sc.stamp, cfg_.scene_timeout)) return stop(o, State::SafeStop, "scene stale");
    const EgoState& e = sc.ego;
    const double v = std::max(0.0, e.v);

    // Geometry from the beacon; frozen once the vehicle starts driving.
    if (sc.geometry_known && !go_ && (!path_.valid() || !sc.geometry.same_as(geom_))) {
      std::string why;
      ReferencePath p;
      if (p.build(sc.geometry, -5.0, cfg_.goal_x + cfg_.path.extend_after_goal, cfg_.path, cfg_.speed, &why)) {
        path_ = p;
        geom_ = sc.geometry;
        ++path_version_;
      } else {
        return stop(o, State::Init, "cannot plan with beacon geometry: " + why);
      }
    }
    if (!path_.valid()) return stop(o, State::Init, "waiting for beacon geometry");
    const Geometry& g = geom_;

    // GO check: wait for a confirmed GO, then latch it.
    if (!go_) {
      if (sc.panel_state != Panel::Go) return stop(o, State::WaitGo, "waiting for confirmed GO");
      go_ = true;
    }
    o.go = go_;

    // Projection on the reference path.
    const auto& pts = path_.points();
    const size_t i = path_.nearest(e.x, e.y);
    const PathPoint& pn = pts[i];
    o.path_s = pn.s;
    o.cte = -std::sin(pn.yaw) * (e.x - pn.x) + std::cos(pn.yaw) * (e.y - pn.y);

    // Speed: minimum of path profile, goal profile and limit.
    o.v_path = profile_speed(i, v);
    double target = o.v_path;
    if (e.x >= cfg_.goal_done_x) {
      o.v_goal = 0.0;
    } else {
      const double d = cfg_.goal_x - e.x - v * cfg_.control_delay;
      o.v_goal = std::max(cfg_.creep_speed, std::sqrt(2.0 * cfg_.goal_decel * std::max(0.0, d)));
    }
    target = std::min(target, o.v_goal);
    o.speed_limit = std::min(kSpeedLimit, g.speed_limit) - cfg_.speed_margin;
    target = clampd(target, 0.0, o.speed_limit);

    // Steering: pure pursuit on the reference path.
    o.lookahead = clampd(cfg_.lookahead_min + cfg_.lookahead_gain * v, cfg_.lookahead_min, cfg_.lookahead_max);
    o.steering = pure_pursuit(path_, i, e, o.lookahead, &o.target);
    last_steer_ = o.steering;
    o.speed = target;
    o.hold = target < 1e-3;

    // State from the position along the route.
    if (e.x + kEgoRadius < g.entry_x) { o.state = State::Approach; o.reason = "GO confirmed; approaching work zone"; }
    else if (e.x < path_.x2()) { o.state = State::Bypass; o.reason = "passing the works on the left"; }
    else if (e.x < g.exit_x) { o.state = State::Return; o.reason = "returning to own lane"; }
    else { o.state = State::Final; o.reason = "own lane, approaching goal"; }
    if (e.x >= cfg_.goal_done_x) {
      o.state = State::Final;
      o.reason = "at goal line, braking";
      if (v < 0.15) { goal_reached_ = true; o.state = State::Goal; o.reason = "goal reached"; }
    }
    return o;
  }

  int path_version() const { return path_version_; }

 private:
  void base(PlanOutput* o, double now) const {
    o->stamp = now;
    o->valid_until = now + cfg_.plan_validity;
    o->speed_limit = kSpeedLimit - cfg_.speed_margin;
    o->go = go_;
  }
  PlanOutput stop(PlanOutput o, State s, const std::string& why) const {
    o.speed = 0.0;
    o.hold = true;
    o.steering = last_steer_;
    o.state = s;
    o.reason = why;
    return o;
  }

  double profile_speed(size_t i, double v) const {
    const auto& p = path_.points();
    double best = cfg_.speed.cruise;
    for (size_t k = i; k < p.size() && p[k].s <= p[i].s + cfg_.speed.horizon; ++k) {
      const double d = std::max(0.0, p[k].s - p[i].s - v * cfg_.control_delay);
      best = std::min(best, std::sqrt(p[k].vmax * p[k].vmax + 2.0 * cfg_.speed.decel * d));
    }
    return best;
  }

  PlannerConfig cfg_;
  ReferencePath path_;
  Geometry geom_;
  int path_version_ = 0;
  bool go_ = false, goal_reached_ = false;
  double last_steer_ = 0.0;
};

}  // namespace rws
