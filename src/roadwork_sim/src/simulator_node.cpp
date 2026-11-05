#include "roadwork_core/simulator.hpp"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "roadwork_interfaces/msg/vehicle_command.hpp"
#include "roadwork_interfaces/msg/roadwork_beacon.hpp"
#include "roadwork_interfaces/msg/radar_detections.hpp"
namespace rw {
inline double seconds(const builtin_interfaces::msg::Time& t) { return t.sec+1e-9*t.nanosec; }
inline builtin_interfaces::msg::Time stamp(double t) {
  builtin_interfaces::msg::Time s;
  const auto ns=int64_t(std::llround(t*1e9));
  s.sec=int32_t(ns/1000000000); s.nanosec=uint32_t(ns%1000000000); return s;
}
}
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
class SimulatorNode:public rclcpp::Node{
 public:
 SimulatorNode():Node("roadwork_simulator"){
  rw::Config c;c.scenario=declare_parameter<int>("scenario",1);c.seed=static_cast<unsigned>(declare_parameter<int>("seed",1));
  c.work_shift=declare_parameter<double>("work_shift",0.);c.traffic_speed=declare_parameter<double>("traffic_speed",3.);
  c.noise=declare_parameter<double>("noise",.02);visualize=declare_parameter<bool>("visualize",true);
  result_path=declare_parameter<std::string>("result_path","roadwork_result.json");sim=std::make_unique<rw::Simulator>(c);
  auto qos=rclcpp::SensorDataQoS();
  odom=create_publisher<nav_msgs::msg::Odometry>("/vehicle/odometry",qos);
  lidar=create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan",qos);
  radar=create_publisher<roadwork_interfaces::msg::RadarDetections>("/radar/detections",qos);
  image=create_publisher<sensor_msgs::msg::Image>("/camera/image_raw",qos);
  beacon=create_publisher<roadwork_interfaces::msg::RoadworkBeacon>("/infra/roadwork/beacon",10);
  clock=create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);
  tf=create_publisher<tf2_msgs::msg::TFMessage>("/tf",10);
  markers=create_publisher<visualization_msgs::msg::MarkerArray>("/world/markers",10);
  metrics=create_publisher<std_msgs::msg::String>("/challenge/metrics",rclcpp::QoS(1).transient_local());
  cmd=create_subscription<roadwork_interfaces::msg::VehicleCommand>("/vehicle/command",10,[this](roadwork_interfaces::msg::VehicleCommand::SharedPtr m){command={rw::seconds(m->header.stamp),m->speed,m->steering_angle};});
  timer=create_wall_timer(std::chrono::milliseconds(50),[this](){
   if(!sim->done())sim->step(command);
   publish();
   if(sim->done()&&!saved){
    saved=true;auto s=result();std::ofstream out(result_path);
    if(out){out<<s<<'\n';RCLCPP_INFO(get_logger(),"Result saved to %s: %s",result_path.c_str(),s.c_str());}
    else RCLCPP_ERROR(get_logger(),"Cannot write result to %s. Metrics remain available on topic.",result_path.c_str());
   }
  });
 }
 private:
 std::unique_ptr<rw::Simulator>sim;rw::Command command;bool saved=false,visualize=true;std::string result_path;
 rclcpp::TimerBase::SharedPtr timer;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
 rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar;
 rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
 rclcpp::Publisher<roadwork_interfaces::msg::RadarDetections>::SharedPtr radar;
 rclcpp::Publisher<roadwork_interfaces::msg::RoadworkBeacon>::SharedPtr beacon;
 rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
 rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics;
 rclcpp::Subscription<roadwork_interfaces::msg::VehicleCommand>::SharedPtr cmd;
 std::string result(){
  const auto&m=sim->metrics;std::ostringstream s;s<<std::boolalpha;
  s<<"{\"scenario\":"<<sim->cfg.scenario<<",\"seed\":"<<sim->cfg.seed<<",\"work_shift\":"<<sim->cfg.work_shift
   <<",\"traffic_speed\":"<<sim->cfg.traffic_speed<<",\"noise\":"<<sim->cfg.noise<<",\"time\":"<<sim->time
   <<",\"terminal\":"<<sim->done()<<",\"pass\":"<<m.pass()<<",\"finished\":"<<m.finished
   <<",\"collision\":"<<m.collision<<",\"offroad\":"<<m.offroad<<",\"entry_violation\":"<<m.entry_violation
   <<",\"lane_violation\":"<<m.lane_violation<<",\"speed_violation\":"<<m.speed_violation<<",\"timeout\":"<<m.timeout
   <<",\"min_clearance\":"<<(m.min_clearance<1e8?m.min_clearance:-1)<<",\"stopped_seconds\":"<<m.stopped_seconds
   <<",\"max_speed\":"<<m.max_speed<<",\"rejected_commands\":"<<m.rejected_commands<<"}";return s.str();
 }
 void publish(){
  const auto&o=sim->obs;auto t=rw::stamp(sim->time);
  rosgraph_msgs::msg::Clock cl;cl.clock=t;clock->publish(cl);
  nav_msgs::msg::Odometry d;d.header.stamp=t;d.header.frame_id="map";d.child_frame_id="base_link";
  d.pose.pose.position.x=o.ego.x;d.pose.pose.position.y=o.ego.y;d.pose.pose.orientation.z=std::sin(o.ego.yaw/2);d.pose.pose.orientation.w=std::cos(o.ego.yaw/2);
  d.twist.twist.linear.x=o.ego.v;d.twist.twist.angular.z=o.ego.v/rw::wheelbase*std::tan(o.ego.steer);odom->publish(d);
  tf2_msgs::msg::TFMessage tr;geometry_msgs::msg::TransformStamped trans;
  trans.header=d.header;trans.child_frame_id="base_link";trans.transform.translation.x=o.ego.x;trans.transform.translation.y=o.ego.y;trans.transform.rotation=d.pose.pose.orientation;tr.transforms.push_back(trans);tf->publish(tr);
  if(sim->tick%2==0){
   sensor_msgs::msg::LaserScan l;l.header.stamp=rw::stamp(o.lidar_stamp);l.header.frame_id="base_link";
   l.angle_min=-rw::pi;l.angle_increment=2*rw::pi/360;l.angle_max=l.angle_min+359*l.angle_increment;l.range_min=.1;l.range_max=40;l.scan_time=.1;l.ranges=o.ranges;lidar->publish(l);
   sensor_msgs::msg::Image im;im.header.stamp=rw::stamp(o.camera_stamp);im.header.frame_id="panel_crop";im.height=o.image.height;im.width=o.image.width;im.encoding="rgb8";im.step=im.width*3;im.data=o.image.rgb;image->publish(im);
   roadwork_interfaces::msg::RadarDetections rd;rd.header.stamp=rw::stamp(o.radar_stamp);rd.header.frame_id="base_link";
   for(const auto&r:o.radar){roadwork_interfaces::msg::RadarDetection hit;hit.range=r.range;hit.azimuth=r.azimuth;hit.radial_velocity=r.radial_velocity;rd.detections.push_back(hit);}radar->publish(rd);
   if(visualize)draw(t);
  }
  if(sim->tick%10==0&&!sim->degraded()){
   const auto&b=o.beacon;roadwork_interfaces::msg::RoadworkBeacon m;m.header.stamp=rw::stamp(b.stamp);m.header.frame_id="map";m.valid_for=b.valid_for;m.entry_x=b.entry_x;m.work_start=b.work_start;m.work_end=b.work_end;m.exit_x=b.exit_x;m.speed_limit=b.speed_limit;m.pass_side=b.pass_side;beacon->publish(m);
  }
  if(sim->tick%10==0||sim->done()){std_msgs::msg::String m;m.data=result();metrics->publish(m);}
 }
 void draw(const builtin_interfaces::msg::Time&t){
  using M=visualization_msgs::msg::Marker;visualization_msgs::msg::MarkerArray out;
  M clear;clear.action=M::DELETEALL;out.markers.push_back(clear);int id=0;
  auto shape=[&](int type,double x,double y,double sx,double sy,double red,double green,double blue,double z=.0){
   M m;m.header.stamp=t;m.header.frame_id="map";m.ns="world";m.id=id++;m.type=type;m.action=M::ADD;
   m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=z;m.pose.orientation.w=1.;m.scale.x=sx;m.scale.y=sy;m.scale.z=.12;
   m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=1.;out.markers.push_back(m);
  };
  shape(M::CUBE,45,0,90,7,.16,.18,.22,-.15);
  for(int x=0;x<90;x+=5)shape(M::CUBE,x,0,2,.06,.9,.9,.9,-.05);
  shape(M::CUBE,25,-1.75,.15,3.5,.95,.95,.95,-.02);
  for(const auto&c:sim->objects())shape(M::CYLINDER,c.x,c.y,2*c.r,2*c.r,c.kind==0?.95:.8,c.kind==0?.5:.15,.1,.1);
  shape(M::CYLINDER,sim->ego.x,sim->ego.y,1.8,1.8,.1,.65,.95,.15);
  shape(M::CUBE,80.2,-1.75,.12,3.5,.2,.85,.5,-.01);
  shape(M::CYLINDER,25,-4.2,1.,1.,sim->go()?0.1:.9,sim->go()?.9:.1,.1,.1);
  M label;label.header.stamp=t;label.header.frame_id="map";label.ns="world";label.id=id++;label.type=M::TEXT_VIEW_FACING;label.action=M::ADD;label.pose.position.x=35;label.pose.position.y=6;label.pose.orientation.w=1;label.scale.z=1.;label.color.r=label.color.g=label.color.b=label.color.a=1;
  label.text="IASYS Roadwork Challenge | S"+std::to_string(sim->cfg.scenario)+" | t="+std::to_string(static_cast<int>(sim->time))+" s";
  out.markers.push_back(label);markers->publish(out);
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SimulatorNode>());}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();}
