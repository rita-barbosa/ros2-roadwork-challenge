// Offline closed-loop test: SenseCore -> Planner -> CommandGuard against the
// supplied roadwork_core simulator, WITHOUT ROS. It emulates simulator_node's
// publication pattern (a message is "received" when its stamp changes) and feeds
// only the Observation fields the S1 pipeline uses (odometry, image, beacon) to the
// algorithms - never objects(), scenario id, schedules or metrics.
//
// This is algorithm-level evidence only; it does not replace ROS acceptance
// (DDS, QoS, executor timing). Usage:
//   test_closed_loop                     run the S1 regression suite (seeds x geometry/noise/delay)
//   test_closed_loop S SEED [SHIFT] [NOISE] [DELAY_TICKS] [trace.csv]
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "roadwork_core/simulator.hpp"
#include "roadwork_student/control.hpp"
#include "roadwork_student/planning.hpp"
#include "roadwork_student/sense_core.hpp"

struct RunResult {
  rw::Metrics m;
  double time = 0;
  std::string last_state;
  int transitions = 0;
  double max_abs_cte = 0;
};

static RunResult run(int scenario, unsigned seed, double shift, double noise, int delay_ticks,
                     const char* trace_path) {
  rw::Config cfg;
  cfg.scenario = scenario;
  cfg.seed = seed;
  cfg.work_shift = shift;
  cfg.noise = noise;
  rw::Simulator sim(cfg);
  rws::SenseCore sense;
  rws::Planner planner;
  rws::CommandGuard guard;
  std::FILE* trace = trace_path ? std::fopen(trace_path, "w") : nullptr;
  if (trace) std::fprintf(trace, "t,x,y,yaw,v,steer,cmd_v,cmd_steer,state,panel,cte,reason\n");

  double last_cam = -1, last_beacon = -1;
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
    const rws::SceneData scene = sense.scene(now);
    const rws::PlanOutput plan = planner.step(scene, now);
    rws::PlanInput pin;
    pin.have = true; pin.stamp = plan.stamp; pin.valid_until = plan.valid_until; pin.speed = plan.speed;
    pin.steering = plan.steering; pin.speed_limit = plan.speed_limit; pin.hold = plan.hold;
    const rws::GuardOutput cmd = guard.step(pin, now);

    if (plan.state != prev) { ++res.transitions; prev = plan.state; }
    if (scene.ego.x > 20 && scene.ego.x < 75) res.max_abs_cte = std::max(res.max_abs_cte, std::abs(plan.cte));
    if (trace)
      std::fprintf(trace, "%.2f,%.3f,%.3f,%.4f,%.3f,%.4f,%.3f,%.4f,%s,%s,%.3f,\"%s\"\n", now, o.ego.x, o.ego.y,
                   o.ego.yaw, o.ego.v, o.ego.steer, cmd.speed, cmd.steering, rws::to_string(plan.state),
                   rws::to_string(scene.panel_state), plan.cte, plan.reason.c_str());
    res.last_state = rws::to_string(plan.state);

    queue.push_back({now, cmd.speed, cmd.steering});
    rw::Command applied = queue.size() > static_cast<size_t>(delay_ticks) ? queue[queue.size() - 1 - delay_ticks]
                                                                          : rw::Command{};
    if (queue.size() > 64) queue.erase(queue.begin());
    sim.step(applied);
  }
  if (trace) std::fclose(trace);
  res.m = sim.metrics;
  res.time = sim.time;
  return res;
}

static void print(int s, unsigned seed, double shift, double noise, int delay, const RunResult& r) {
  const auto& m = r.m;
  std::printf(
      "S%d seed=%u shift=%+.1f noise=%.3f delay=%d | pass=%d finished=%d coll=%d off=%d entry=%d lane=%d speed=%d "
      "timeout=%d | t=%.2f min_clear=%.3f vmax=%.3f stopped=%.2f rejected=%d max|cte|=%.3f state=%s\n",
      s, seed, shift, noise, delay, m.pass(), m.finished, m.collision, m.offroad, m.entry_violation, m.lane_violation,
      m.speed_violation, m.timeout, r.time, m.min_clearance, m.max_speed, m.stopped_seconds, m.rejected_commands,
      r.max_abs_cte, r.last_state.c_str());
}

int main(int argc, char** argv) {
  if (argc >= 3) {
    const int s = std::atoi(argv[1]);
    const unsigned seed = static_cast<unsigned>(std::atoi(argv[2]));
    const double shift = argc > 3 ? std::atof(argv[3]) : 0.0;
    const double noise = argc > 4 ? std::atof(argv[4]) : 0.02;
    const int delay = argc > 5 ? std::atoi(argv[5]) : 0;
    const char* trace = argc > 6 ? argv[6] : nullptr;
    const RunResult r = run(s, seed, shift, noise, delay, trace);
    print(s, seed, shift, noise, delay, r);
    return r.m.pass() ? 0 : 1;
  }
  // S1 regression suite: seeds 1..3, default and boundary geometry/noise,
  // with 0 and 2 ticks (0.1 s) of extra command delay.
  int failures = 0, runs = 0;
  for (int s : {1})
    for (unsigned seed : {1u, 2u, 3u})
      for (double shift : {0.0, -2.0, 2.0})
        for (double noise : {0.02, 0.0, 0.05})
          for (int delay : {0, 2}) {
            const RunResult r = run(s, seed, shift, noise, delay, nullptr);
            ++runs;
            if (!r.m.pass()) ++failures;
            if (!r.m.pass() || (shift == 0.0 && noise == 0.02 && delay == 0)) print(s, seed, shift, noise, delay, r);
          }
  std::printf("closed-loop S1 suite: %d/%d runs passed\n", runs - failures, runs);
  return failures == 0 ? 0 : 1;
}
