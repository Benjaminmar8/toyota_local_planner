#ifndef TOYOTA_LOCALPLANNER_TOYOTA_LOCALPLANNER_H_
#define TOYOTA_LOCALPLANNER_TOYOTA_LOCALPLANNER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <base_local_planner/costmap_model.h>
#include <toyota_localplanner/vehicle_control.h>
#include <visualization_msgs/Marker.h>
#include <queue>

namespace toyota_localplanner {
  class ToyotaLocalPlanner : public nav_core::BaseLocalPlanner{
    public:
      ToyotaLocalPlanner();
      ~ToyotaLocalPlanner();
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      std_msgs::Float32 steer, drive;
      std_msgs::Float32 actual_str, actual_vel;
      unsigned int id;
      //std::queue<std_msgs::Float32> steer_queue, drive_queue;

  private:
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void pathCallback(const nav_msgs::Path::ConstPtr& msg);
      void actualvelCallback(const std_msgs::Float32::ConstPtr& msg);
      void actualstrCallback(const std_msgs::Float32::ConstPtr& msg);

      double compute_obs_dist_path(int);
      double compute_obs_time_path(int);
      double compute_obs_dist_odom();
      double compute_obs_time_odom();

      ros::Subscriber odom_sub_;
      ros::Subscriber path_sub_;
      ros::Subscriber actual_vel_sub_;
      ros::Subscriber actual_str_sub_;

      ros::Publisher footprint_projection_path_pub_;
      ros::Publisher footprint_projection_odom_pub_;
      ros::Publisher marker_publisher;
      ros::Publisher steer_publisher;
      ros::Publisher drive_publisher;

      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      base_local_planner::CostmapModel* costmap_model_;
      costmap_2d::Costmap2D* costmap_;

      nav_msgs::Odometry vehicle_odom_;
      nav_msgs::Path global_path_;
      geometry_msgs::PolygonStamped footprint_projection_path_;
      geometry_msgs::PolygonStamped footprint_projection_odom_;
      visualization_msgs::Marker marker;

      boost::mutex odom_lock_;

      //bool newPathFlag;
      vehicle_control* COMS_;

  protected:
      //parameter
      std::string odom_topic_;
      std::string path_topic_;
      std::string steer_topic_;
      std::string drive_topic_;
      double inscribed_radius_;
      double circumscribed_radius_;
      //double lookahead_step_;
      //double lookahead_distance_;
      //double lookahead_time_;
      double goal_distance_bias_;
      double goal_heading_bias_;
      double lethal_cost_threshold_;
      double planner_patience_;
      double cost_violation_times_;

      //double steer_gain_;


      bool invalid_goal_;
      //bool use_lookahead_distance_;

      
  };
}
#endif
