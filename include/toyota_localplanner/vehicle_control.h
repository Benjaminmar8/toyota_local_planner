#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class vehicle_control {

public:
  vehicle_control();
  virtual ~vehicle_control();

  void vehicle_init();       // init all variables
  void vehicle_shutdown();   // clear all variables

  double compute_steer(const nav_msgs::Odometry& vehicleOdom, const nav_msgs::Path& plannedPath, int id, double vehicle_pathpoint_dist, double prev_steer_effort);
  double compute_drive(const nav_msgs::Odometry& vehicleOdom, double vehicle_canV, double vehicle_canStr, double target_velocity, double time_to_obs, double dist_to_obs, double prev_drive_effort);
  double compute_brake(const nav_msgs::Odometry& vehicleOdom);

  //std_msgs::Float32MultiArray compute_ctr(const nav_msgs::Odometry& vehicleOdom, const nav_msgs::Path& plannedPath);

  // utility functions
  double angledifference(double firstangle, double secondangle);
  int checksign(double Ax,double Ay,double Bx,double By,double Cx,double Cy);

  // units: meter, m/s, m^2/s, degree, degree/s
  // These paramters related to the physical properties of the vehicle and should be fixed.
  struct vehicle{
      int    weight;
      double wheelR;          // wheel radius
      double length;
      double width;
      double axelDist;        // axel distance
      double height;
      double maxV;            // maximum velocity
      double minV;            // minimum velocity
      double clampV;          // velocity clamp as safty limit
      double maxSteerWheelA;  // maximum steering wheel angle
      double minSteerWheelA;
      double maxSteerVehicA;  // maximum vehicle steering angle
      double minSteerVehicA;
      double turnR;             // turning radius
      double maxAccel;          // maximum acceleration
      double maxDecel;          // maximum deceleration
      double steerRatio;        // steering ratio
  };

  // units: meter, m/s, m^2/s, degree, degree/s
  // These parameters can be modified to obtain different control behavior of the vehicle
  struct control{
      double steerWheelRate;        // maximum rate steering wheel can be turned
      double steerVehicleRate;      // maximum rate vehicle can be turned
      double steer_gain;            // steering gain, unitless
      double steer_gain_theta;      // steering gain for /detla_theta, unitless
      double minctr_dist;           // minimum croos track error distance before the path is considered as invalid
      double minVelocity_steer;     // minimum velocity for steering control to be active
      double minlookahead_dist_odom;     // minimum look ahead distance
      double minLookahead_time_odom;     // minimum look ahead time
      double minLookahead_dist_path; // minimum look ahead distance on planned path
      double minLookahead_time_path; // minimum look ahead time on planned path
      bool lookahead_modeFlag;      // True: look ahead in both distance/time mode, false: look ahead in only distance mode
      double stop_dist_path;             // absolute dist to obstacle to stop (path)
      double stop_time_path;             // absolute time to obstacle to stop (path)
      double stop_dist_odom;             // absolute dist to obstacle to stop (odom)
      double stop_time_odom;             // absolute time to obstacle to stop (odom)
      bool stop_modeFlag;           // True: stop in both distance/time mode, false; stop in only distance mode
      double brake_graident;        // gradient to slow down the vehicle when detected obstacle in front
      double steer_lp_gain;               // steering control input low pass filter gain
      double drive_lp_gain;               // drive control input low pass filter gain
      double forwad_sim_dist;       // forward simulation distance
      bool   drive_mode;            // false: velocity_mode, true: stroke_mode
      bool   steer_mode;            // false: angle_mode,    true: stroke_mode
      double target_velocity;               // target velocity in m/s
  };

  vehicle vehicle_paramters;
  control control_paramters;

};


#endif

