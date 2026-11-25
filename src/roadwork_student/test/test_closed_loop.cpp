// Offline closed-loop test: SenseCore -> Planner -> CommandGuard against the
// supplied roadwork_core simulator, WITHOUT ROS. It emulates simulator_node's
// publication pattern (a message is "received" when its stamp changes) and feeds
// only the Observation fields a solution may use (odometry, image, beacon,
// radar, LiDAR) to the algorithms - never objects(), scenario id, schedules or metrics.
// The scenario id and event times are used here only to choose the episode and
// to score it.
//
// This is algorithm-level evidence only; it does not replace ROS acceptance
// (DDS, QoS, executor timing). Usage:
//   test_closed_loop                     run the S1-S5 regression suite
//   test_closed_loop S SEED [SHIFT] [NOISE] [DELAY_TICKS] [TRAFFIC] [--no-gap] [--no-obstacle] [--trace=FILE.csv]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "roadwork_core/simulator.hpp"
#include "roadwork_student/control.hpp"
#include "roadwork_student/planning.hpp"
#include "roadwork_student/sense_core.hpp"

struct Options {
  int scenario = 1;
  unsigned seed = 1;
  double shift = 0.0, noise = 0.02, traffic = 3.0;
  int delay_ticks = 0;
  bool no_gap = false, no_obstacle = false;
  const char* trace = nullptr;
};

struct RunResult {
  rw::Metrics m;
  double time = 0;
  std::string last_state;
  int transitions = 0;
  double max_abs_cte = 0;
  double first_motion_time = -1.0;
  double entry_time = -1.0;  // first time the vehicle front crossed x = 25
  // Largest front position (x + radius) while stopped without permission (WAIT_GO); -1 if never.
  double wait_stop_front = -1.0;
  double yield_time = 0.0;  // s spent in YIELD
  double blocked_time = 0.0;  // s spent in BLOCKED
};

// S2 also requires the vehicle to have waited at the stop line, i.e. at rest in
// WAIT_GO with its front between the configured stop point (minus a tolerance)
// and the entry line. The entry line itself is checked by entry_violation.
static bool stopped_at_line(const RunResult& r) {
  const double stop_front = 25.0 - rws::PlannerConfig{}.stop_distance;
  return r.wait_stop_front >= stop_front - 0.3 && r.wait_stop_front < 25.0;
}
// S4: the vehicle must not enter before both camera and beacon are back
// (t >= 8 s, used for scoring only) -- stricter than entry_violation, which
// only checks the blackout itself.
static bool ok(int s, const RunResult& r) {
  if (!r.m.pass()) return false;
  if (s == 2) return stopped_at_line(r);
  if (s == 4) return r.entry_time >= 8.0;
  return true;
}

static RunResult run(const Options& opt) {
  rw::Config cfg;
  cfg.scenario = opt.scenario;
  cfg.seed = opt.seed;
  cfg.work_shift = opt.shift;
  cfg.noise = opt.noise;
  cfg.traffic_speed = opt.traffic;
  rw::Simulator sim(cfg);
  rws::SenseCore sense;
  rws::PlannerConfig pc;
  pc.use_traffic_gap = !opt.no_gap;
  pc.use_obstacle_check = !opt.no_obstacle;
  rws::Planner planner(pc);
  rws::CommandGuard guard;
  std::FILE* trace = opt.trace ? std::fopen(opt.trace, "w") : nullptr;
  if (trace)
    std::fprintf(trace, "t,x,y,yaw,v,steer,cmd_v,cmd_steer,state,panel,cte,gap_ok,n_tracks,conflict_x,reason\n");

  double last_cam = -1, last_beacon = -1, last_radar = -1, last_lidar = -1;
  std::vector<rw::Command> queue;  // transport delay emulation
  RunResult res;
  rws::State prev = rws::State::Init;
  while (!sim.done()) {
    const rw::Observation& o = sim.obs;
    const double now = sim.time;
    rws::EgoState e;
    e.stamp = o.odom_stamp; e.x = o.ego.x; e.y = o.ego.y; e.yaw = o.ego.yaw; e.v = o.ego.v; e.valid = true;
    sense.on_odom(e, now);
    if (o.camera_stamp != last_cam) {
      last_cam = o.camera_stamp;
      sense.on_image(o.camera_stamp, o.image.rgb, o.image.width, o.image.height, o.image.width * 3, "rgb8", now);
    }
    if (o.beacon.stamp != last_beacon && o.beacon.stamp >= 0) {
      last_beacon = o.beacon.stamp;
      rws::BeaconSample b;
      b.stamp = o.beacon.stamp; b.valid_for = o.beacon.valid_for;
      b.g.entry_x = o.beacon.entry_x; b.g.work_start = o.beacon.work_start; b.g.work_end = o.beacon.work_end;
      b.g.exit_x = o.beacon.exit_x; b.g.speed_limit = o.beacon.speed_limit; b.g.pass_side = o.beacon.pass_side;
      sense.on_beacon(b, now);
    }
    if (o.radar_stamp != last_radar && o.radar_stamp >= 0) {
      last_radar = o.radar_stamp;
      std::vector<rws::RadarDet> dets;
      for (const auto& h : o.radar) dets.push_back({h.range, h.azimuth, h.radial_velocity});
      sense.on_radar(o.radar_stamp, dets, now);
    }
    if (o.lidar_stamp != last_lidar && o.lidar_stamp >= 0 && !o.ranges.empty()) {
      last_lidar = o.lidar_stamp;
      rws::LidarScanIn scan;
      scan.stamp = o.lidar_stamp;
      scan.angle_min = -rw::pi;
      scan.angle_increment = 2 * rw::pi / 360;
      scan.range_min = 0.1;
      scan.range_max = 40.0;
      scan.ranges = o.ranges;
      sense.on_scan(scan, now);
    }
    const rws::SceneData scene = sense.scene(now);
    const rws::PlanOutput plan = planner.step(scene, now);
    rws::PlanInput pin;
    pin.have = true; pin.stamp = plan.stamp; pin.valid_until = plan.valid_until; pin.speed = plan.speed;
    pin.steering = plan.steering; pin.speed_limit = plan.speed_limit; pin.hold = plan.hold;
    const rws::GuardOutput cmd = guard.step(pin, now);
    if (res.first_motion_time < 0 && cmd.speed > 0.01) res.first_motion_time = now;
    if (res.entry_time < 0 && o.ego.x + rws::kEgoRadius > 25.0) res.entry_time = now;
    if (plan.state == rws::State::WaitGo && o.ego.v < 0.1 && o.ego.x > 6.0)
      res.wait_stop_front = std::max(res.wait_stop_front, o.ego.x + rws::kEgoRadius);
    if (plan.state == rws::State::Yield) res.yield_time += rw::dt;
    if (plan.state == rws::State::Blocked) res.blocked_time += rw::dt;

    if (plan.state != prev) { ++res.transitions; prev = plan.state; }
    if (scene.ego.x > 20 && scene.ego.x < 75) res.max_abs_cte = std::max(res.max_abs_cte, std::abs(plan.cte));
    if (trace)
      std::fprintf(trace, "%.2f,%.3f,%.3f,%.4f,%.3f,%.4f,%.3f,%.4f,%s,%s,%.3f,%d,%d,%.2f,\"%s\"\n", now, o.ego.x,
                   o.ego.y, o.ego.yaw, o.ego.v, o.ego.steer, cmd.speed, cmd.steering, rws::to_string(plan.state),
                   rws::to_string(scene.panel_state), plan.cte, plan.gap_ok ? 1 : 0, plan.n_tracks, plan.conflict_x,
                   plan.reason.c_str());
    res.last_state = rws::to_string(plan.state);

    queue.push_back({now, cmd.speed, cmd.steering});
    const size_t delay = static_cast<size_t>(opt.delay_ticks);
    rw::Command applied = queue.size() > delay ? queue[queue.size() - 1 - delay] : rw::Command{};
    if (queue.size() > 64) queue.erase(queue.begin());
    sim.step(applied);
  }
  if (trace) std::fclose(trace);
  res.m = sim.metrics;
  res.time = sim.time;
  return res;
}

static void print(const Options& o, const RunResult& r) {
  const auto& m = r.m;
  std::printf(
      "S%d seed=%u shift=%+.1f noise=%.3f traffic=%.1f delay=%d%s | pass=%d finished=%d coll=%d off=%d entry=%d "
      "lane=%d speed=%d timeout=%d | t=%.2f min_clear=%.3f vmax=%.3f stopped=%.2f rejected=%d max|cte|=%.3f "
      "first_motion=%.2f entry_t=%.2f wait_stop_front=%.3f yield=%.2f blocked=%.2f state=%s\n",
      o.scenario, o.seed, o.shift, o.noise, o.traffic, o.delay_ticks,
      o.no_gap ? (o.no_obstacle ? " no-gap no-obstacle" : " no-gap") : (o.no_obstacle ? " no-obstacle" : ""), m.pass(),
      m.finished, m.collision, m.offroad, m.entry_violation, m.lane_violation, m.speed_violation, m.timeout, r.time,
      m.min_clearance, m.max_speed, m.stopped_seconds, m.rejected_commands, r.max_abs_cte, r.first_motion_time,
      r.entry_time, r.wait_stop_front, r.yield_time, r.blocked_time, r.last_state.c_str());
}

int main(int argc, char** argv) {
  Options opt;
  std::vector<const char*> pos;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--no-gap")) opt.no_gap = true;
    else if (!std::strcmp(argv[i], "--no-obstacle")) opt.no_obstacle = true;
    else if (!std::strncmp(argv[i], "--trace=", 8)) opt.trace = argv[i] + 8;
    else pos.push_back(argv[i]);
  }
  if (pos.size() >= 2) {
    opt.scenario = std::atoi(pos[0]);
    opt.seed = static_cast<unsigned>(std::atoi(pos[1]));
    if (pos.size() > 2) opt.shift = std::atof(pos[2]);
    if (pos.size() > 3) opt.noise = std::atof(pos[3]);
    if (pos.size() > 4) opt.delay_ticks = std::atoi(pos[4]);
    if (pos.size() > 5) opt.traffic = std::atof(pos[5]);
    const RunResult r = run(opt);
    print(opt, r);
    return ok(opt.scenario, r) ? 0 : 1;
  }
  // Regression suite: S1-S5 x seeds 1..3 x default and boundary geometry/noise,
  // with 0 and 2 ticks (0.1 s) of extra command delay; S3 also at the traffic
  // speed bounds.
  int failures = 0, runs = 0;
  for (int s : {1, 2, 3, 4, 5})
    for (unsigned seed : {1u, 2u, 3u})
      for (double shift : {0.0, -2.0, 2.0})
        for (double noise : {0.02, 0.0, 0.05})
          for (int delay : {0, 2})
            for (double traffic : s == 3 ? std::vector<double>{3.0, 2.0, 4.0} : std::vector<double>{3.0}) {
              Options o;
              o.scenario = s; o.seed = seed; o.shift = shift; o.noise = noise; o.delay_ticks = delay;
              o.traffic = traffic;
              const RunResult r = run(o);
              ++runs;
              if (!ok(s, r)) ++failures;
              if (!ok(s, r) || (shift == 0.0 && noise == 0.02 && delay == 0 && traffic == 3.0)) print(o, r);
            }
  std::printf("closed-loop S1-S5 suite: %d/%d runs passed\n", runs - failures, runs);
  return failures == 0 ? 0 : 1;
}
