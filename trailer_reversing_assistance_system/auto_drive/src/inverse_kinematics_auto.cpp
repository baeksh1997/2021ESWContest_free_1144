#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "hunter_msgs/HunterStatus.h"
#include "wrp_sdk/platforms/hunter/hunter_base.hpp"
#include "math.h"
#define SQR(x) ((x)*(x))
#define PI  3.1415926535

using namespace std;
using namespace westonrobot;

static std_msgs::Float32 angle;
static geometry_msgs::Pose CarPose;
static geometry_msgs::Pose TrailerPose;
static geometry_msgs::Twist CarVelo;
static geometry_msgs::Twist TrailerVelo;
static hunter_msgs::HunterStatus Hstate;
/*
  Vehicle_Yaw [0] : Car Yaw Position, [1] : Trailer Yaw Position, [2] : Car / Trailer BTW Yaw Radian Angle
*/

static double Dcar_hunter = 0.30;
static double Dtrailer_hunter = 0.745;
static double L = 0.648;

static double pre_data = 0;
static double dt = 0.01;
static double constant_car_velo = -0.15;
double LPF(double input, double tau){
  cout<<"pre_data:"<<pre_data<<endl;
  pre_data = ((tau * pre_data) + (dt * input)) / (tau + dt);
  return pre_data;
}

static double error;
static double pre_error;
static double Kp = 0.05;
static double Kd = 0.005;
static double result;
static double control(double des_w, double current_w){
    error = des_w - current_w;
    result = Kp*error + Kd*(error - pre_error)/dt;
    pre_error = error;
    return result;
}

static double CarVelMtx_hunter[2];
static double TrailerVelMtx[2];//v1 w1
static double KinematicsMtx_hunter[4];

static double hunter_angle = 0;

void EncoderCallback(const std_msgs::Int16::ConstPtr& msg){
    angle.data = -(double)msg->data/1000.0;
}
void HunterstateCallback(const hunter_msgs::HunterStatus::ConstPtr& state){
    Hstate.steering_angle = state->steering_angle;
}
void VeloKinematicsCallback(const geometry_msgs::Twist::ConstPtr& msg){
  TrailerVelo = *msg;
  TrailerVelMtx[0] = TrailerVelo.linear.x;
  TrailerVelMtx[1] = TrailerVelo.angular.z;

  KinematicsMtx_hunter[0] = cos(angle.data);
  KinematicsMtx_hunter[1] = Dcar_hunter*sin(angle.data);
  KinematicsMtx_hunter[2] = sin(angle.data)/Dtrailer_hunter;
  KinematicsMtx_hunter[3] = -cos(angle.data);


  CarVelMtx_hunter[0] = (1/(pow(cos(angle.data),2)+(Dcar_hunter/Dtrailer_hunter)*pow(sin(angle.data),2)))*
          (KinematicsMtx_hunter[0]*TrailerVelMtx[0]+KinematicsMtx_hunter[1]*TrailerVelMtx[1]);
  CarVelMtx_hunter[1] = (1/(pow(cos(angle.data),2)+(Dcar_hunter/Dtrailer_hunter)*pow(sin(angle.data),2)))*
          (KinematicsMtx_hunter[2]*TrailerVelMtx[0]+KinematicsMtx_hunter[3]*TrailerVelMtx[1]);

  hunter_angle = atan2f64(CarVelMtx_hunter[1]*L,constant_car_velo); //dw
  if(hunter_angle > 1.7) hunter_angle-=3.1415;
  else if(hunter_angle < -1.7) hunter_angle+=3.1415;

  CarVelo.linear.x = constant_car_velo;//CarVelMtx_hunter[0];
  CarVelo.angular.z = -hunter_angle / 0.596;
  cout <<"theta : "<< angle.data << endl;
  cout <<"velocity : "<< constant_car_velo << endl;
  cout <<"desired angle : "<< hunter_angle / 0.596 << endl;
  cout <<"current angle : "<< Hstate.steering_angle << endl;
  cout <<"error : "<< error << endl << endl;

  CarVelo.linear.y = 0;
  CarVelo.linear.z = 0;
  CarVelo.angular.x = 0;
  CarVelo.angular.y = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematics_auto");
  ros::NodeHandle nh;

  ros::Publisher auto_init_pub = nh.advertise<std_msgs::String>("/initialize", 100);
  ros::Publisher CarVelopub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Subscriber Kinematicsub = nh.subscribe("/cmd_vel/teleop",100,VeloKinematicsCallback);
  ros::Subscriber Encodersub = nh.subscribe("/encoder_theta",100,EncoderCallback);
  ros::Subscriber Hunterstatesub = nh.subscribe("/hunter_status", 100,HunterstateCallback);

  std_msgs::String auto_init_cmd;
  auto_init_cmd.data = "initialize";
  auto_init_pub.publish(auto_init_cmd);

  ros::Rate rate(100); //100hz
  while (ros::ok())
  {
    CarVelopub.publish(CarVelo);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
