#pragma once
#include "types.hpp"
#include <random>
#include <stdexcept>
#include <string>
namespace rw {
struct Circle {double x,y,r,vx; int kind;}; // kind 0 works, 1 traffic, 2 temporary obstacle
struct Config {int scenario=1; unsigned seed=1; double work_shift=0,traffic_speed=3,noise=0.02;};
struct Metrics {
 bool collision=false,offroad=false,entry_violation=false,lane_violation=false,speed_violation=false,finished=false,timeout=false;
 double min_clearance=1e9,stopped_seconds=0,max_speed=0;
 int rejected_commands=0;
 bool pass() const{return finished&&!collision&&!offroad&&!entry_violation&&!lane_violation&&!speed_violation;}
};
class Simulator {
 public:
 Config cfg; Pose ego; Metrics metrics; double time=0; int tick=0;
 double hazard_start=-1; bool entered=false; Observation obs;
 explicit Simulator(Config c={}):cfg(c),rng(c.seed){
  if(c.scenario<1||c.scenario>5||std::abs(c.work_shift)>2||c.traffic_speed<2||c.traffic_speed>4||c.noise<0||c.noise>0.05)
   throw std::invalid_argument("scenario 1..5, work_shift +/-2, traffic_speed 2..4, noise 0..0.05 required");
  sample();
 }
 bool degraded() const{return cfg.scenario==4&&time>=2&&time<8;}
 bool go() const{return cfg.scenario!=2||time>=10;}
 Beacon beacon() const {Beacon b;b.stamp=time;b.work_start+=cfg.work_shift;b.work_end+=cfg.work_shift;b.exit_x+=cfg.work_shift;return b;}
 std::vector<Circle> objects() const{
  std::vector<Circle> o={{43+cfg.work_shift,-1.75,1.0,0,0},{47+cfg.work_shift,-1.75,1.0,0,0}};
  if(cfg.scenario==3){double x=82-cfg.traffic_speed*time;if(x>-5)o.push_back({x,1.75,0.9,-cfg.traffic_speed,1});}
  if(hazard_start>=0&&time-hazard_start<8)o.push_back({57+cfg.work_shift,1.75,0.65,0,2});
  return o;
 }
 bool done() const{return metrics.finished||metrics.collision||metrics.offroad||metrics.timeout;}
 void step(Command c){
  if(done())return;
  bool valid=std::isfinite(c.speed)&&std::isfinite(c.steering)&&std::isfinite(c.stamp)&&fresh(time,c.stamp,.30);
  if(!valid){c.speed=0;c.steering=ego.steer;metrics.rejected_commands++;}
  double target=clamp(c.speed,0.,8.);
  ego.v+=clamp(target-ego.v,-3.*dt,1.5*dt);
  ego.steer+=clamp(clamp(c.steering,-.45,.45)-ego.steer,-.6*dt,.6*dt);
  // Rear-axle reference with a circular collision envelope centred on it (teaching model).
  double yaw_rate=ego.v/wheelbase*std::tan(ego.steer);
  ego.x+=ego.v*std::cos(ego.yaw+yaw_rate*dt*.5)*dt;
  ego.y+=ego.v*std::sin(ego.yaw+yaw_rate*dt*.5)*dt;
  ego.yaw=angle(ego.yaw+yaw_rate*dt);
  time=++tick*dt;
  if(cfg.scenario==5&&hazard_start<0&&ego.x>=45+cfg.work_shift)hazard_start=time;
  if(!entered&&ego.x+ego_radius>25){entered=true;if(!go()||degraded())metrics.entry_violation=true;}
  if(std::abs(ego.y)+ego_radius>3.5||ego.x<0)metrics.offroad=true;
  if(ego.y+ego_radius>0 && (ego.x+ego_radius<25 || ego.x-ego_radius>68+cfg.work_shift))metrics.lane_violation=true;
  for(const auto&o:objects()){
   double clearance=std::hypot(o.x-ego.x,o.y-ego.y)-o.r-ego_radius;
   metrics.min_clearance=std::min(metrics.min_clearance,clearance);
   if(clearance<=0)metrics.collision=true;
  }
  metrics.max_speed=std::max(metrics.max_speed,ego.v);
  if(ego.v>4.05)metrics.speed_violation=true;
  if(ego.v<.1)metrics.stopped_seconds+=dt;
  if(ego.x>=80&&std::abs(ego.y+1.75)<.6&&std::abs(ego.yaw)<.15&&ego.v<.15)metrics.finished=true;
  if(time>=180&&!metrics.finished)metrics.timeout=true;
  sample();
 }
 private:
 std::mt19937 rng;
 double uniform(double amplitude){return amplitude*(2.*static_cast<double>(rng())/std::mt19937::max()-1.);}
 void sample(){
  obs.time=time;obs.odom_stamp=time;obs.ego=ego;
  if(tick%10==0&&!degraded())obs.beacon=beacon();
  if(tick%2!=0)return;
  auto objs=objects();
  obs.lidar_stamp=time;obs.ranges.assign(360,std::numeric_limits<float>::infinity());
  for(int i=0;i<360;i++){
   double a=ego.yaw-pi+2*pi*i/360.,dx=std::cos(a),dy=std::sin(a),distance=40.;bool hit=false;
   if(std::abs(dy)>1e-9){double t=((dy>0?3.5:-3.5)-ego.y)/dy;if(t>0&&t<distance){distance=t;hit=true;}}
   for(const auto&o:objs){
    double ox=o.x-ego.x,oy=o.y-ego.y,projection=ox*dx+oy*dy;
    double discriminant=o.r*o.r-(ox*ox+oy*oy-projection*projection);
    if(discriminant>=0){double t=projection-std::sqrt(discriminant);if(t>0&&t<distance){distance=t;hit=true;}}
   }
   if(hit)obs.ranges[i]=static_cast<float>(clamp(distance+uniform(cfg.noise),.10,40.));
  }
  obs.radar_stamp=time;obs.radar.clear();
  for(const auto&o:objs){
   double dx=o.x-ego.x,dy=o.y-ego.y,r=std::hypot(dx,dy),az=angle(std::atan2(dy,dx)-ego.yaw);
   if(r>.1&&r<=80&&std::abs(az)<=pi/3){
    double vr=((o.vx-ego.v*std::cos(ego.yaw))*dx-ego.v*std::sin(ego.yaw)*dy)/r;
    obs.radar.push_back({std::max(.1,r+uniform(cfg.noise*3)),az+uniform(.002),vr+uniform(cfg.noise)});
   }
  }
  obs.camera_stamp=time;obs.image.rgb.resize(96*64*3);
  // A cropped infrastructure-panel image. Not a perspective camera or road scene.
  // Blank panels model loss of visual evidence; no class label is published.
  for(int y=0;y<64;y++)for(int x=0;x<96;x++){
   bool disk=(x-48)*(x-48)+(y-32)*(y-32)<18*18&&!degraded();
   int n=static_cast<int>(uniform(18));
   for(int k=0;k<3;k++){
    int v=disk?((k==(go()?1:0))?200:25):35;
    obs.image.rgb[(y*96+x)*3+k]=static_cast<uint8_t>(clamp(v+n,0,255));
   }
  }
 }
};
}
