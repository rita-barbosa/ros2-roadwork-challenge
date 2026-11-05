#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
namespace rw {
constexpr double pi=3.14159265358979323846;
constexpr double dt=0.05, ego_radius=0.90, wheelbase=2.4;
inline double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
inline double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
struct Pose {double x=5,y=-1.75,yaw=0,v=0,steer=0;};
struct Command {double stamp=-100,speed=0,steering=0;};
struct RadarHit {double range=0,azimuth=0,radial_velocity=0;};
struct Beacon {double stamp=-100,valid_for=1.2,entry_x=25,work_start=42,work_end=49,exit_x=68,speed_limit=4; int pass_side=1;};
struct Frame {int width=96,height=64; std::vector<uint8_t> rgb;};
// Only these measurements are available to a solution. Angles are radians; SI units.
struct Observation {
 double time=0,odom_stamp=-100,lidar_stamp=-100,radar_stamp=-100,camera_stamp=-100;
 Pose ego; Beacon beacon; std::vector<float> ranges; std::vector<RadarHit> radar; Frame image;
};
inline bool fresh(double now,double stamp,double ttl){return stamp>=0 && now>=stamp-1e-6 && now-stamp<=ttl;}
}
