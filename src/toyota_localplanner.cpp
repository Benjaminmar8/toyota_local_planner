#include <toyota_localplanner/toyota_localplanner.h>
#include <limits>
#include <algorithm>    // std::min
#include <pluginlib/class_list_macros.h>
#include <toyota_localplanner/vehicle_control.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <costmap_2d/costmap_2d_ros.h>

#define PI 3.1416

PLUGINLIB_DECLARE_CLASS(toyota_localplanner, ToyotaLocalPlanner, toyota_localplanner::ToyotaLocalPlanner, nav_core::BaseLocalPlanner)

namespace toyota_localplanner {
  ToyotaLocalPlanner::ToyotaLocalPlanner(): tf_(NULL), costmap_ros_(NULL) {
     COMS_ = new vehicle_control();
     actual_str.data = 0.0;
     actual_vel.data = 0.0;
     id = 0;
  }

  ToyotaLocalPlanner::~ToyotaLocalPlanner(){
      delete COMS_;
  }

  void ToyotaLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

      ros::NodeHandle node_private("~/" + name);
      node_private.param("odom_topic", odom_topic_, std::string("odom"));
      node_private.param("path_topic", path_topic_, std::string("path"));
      node_private.param("drive_topic", drive_topic_, std::string("drive"));
      node_private.param("steer_topic", steer_topic_, std::string("steer"));
      node_private.param("inscribed_radius", inscribed_radius_, 1.0);
      node_private.param("circumscribed_radius", circumscribed_radius_, 1.0);
      node_private.param("goal_distance_bias", goal_distance_bias_, 0.5);
      node_private.param("goal_heading_bias", goal_heading_bias_, 0.5);
      node_private.param("lethal_cost_threshold", lethal_cost_threshold_, 255.0);
      node_private.param("planner_patience", planner_patience_, 40.0);

      // vehicle control parameter
      node_private.param("steerWheelRate",COMS_->control_paramters.steerWheelRate,20.0);         // default 20 degree/input
      node_private.param("steerVehicleRate",COMS_->control_paramters.steerVehicleRate,2.0);      // default 2 degree/input
      node_private.param("steer_gain",COMS_->control_paramters.steer_gain,0.85);               // default 0.85
      node_private.param("steer_gain_theta",COMS_->control_paramters.steer_gain_theta,1.0);               // default 1.0
      node_private.param("minctr_dist",COMS_->control_paramters.minctr_dist,2.0);              // default 2.0
      node_private.param("minVelocity_steer",COMS_->control_paramters.minVelocity_steer,0.2);  // default 0.2 m/s
      node_private.param("minlookahead_dist_odom",COMS_->control_paramters.minlookahead_dist_odom,5.0);  // default minimum look ahead distance
      node_private.param("minLookahead_time_odom",COMS_->control_paramters.minLookahead_time_odom,4.0);  // default 4.0 seconds
      node_private.param("minlookahead_dist_path",COMS_->control_paramters.minLookahead_dist_path,5.0);  // default minimum look ahead distance
      node_private.param("minLookahead_time_path",COMS_->control_paramters.minLookahead_time_path,4.0);  // default 4.0 seconds
      node_private.param("lookahead_modeFlag",COMS_->control_paramters.lookahead_modeFlag, false); //True: look ahead in both distance/time mode, false: look ahead in only distance mode
      node_private.param("stop_dist_path",COMS_->control_paramters.stop_dist_path,2.0); // absolute dist to obstacle to stop
      node_private.param("stop_time_path",COMS_->control_paramters.stop_time_path,2.0); // absolute time to obstacle to stop
      node_private.param("stop_dist_odom",COMS_->control_paramters.stop_dist_odom,1.0); // absolute dist to obstacle to stop
      node_private.param("stop_time_odom",COMS_->control_paramters.stop_time_odom,2.0); // absolute time to obstacle to stop
      node_private.param("stop_modeFlag",COMS_->control_paramters.stop_modeFlag,false);  // True: stop in both distance/time mode, false; stop in only distance mode
      node_private.param("steer_lp_gain", COMS_->control_paramters.steer_lp_gain,0.1);   // steer low pass filter gain, [0,1]
      node_private.param("drive_lp_gain", COMS_->control_paramters.drive_lp_gain,0.3);   // drive low pass filter gain, [0,1]
      node_private.param("forwad_sim_dist",COMS_->control_paramters.forwad_sim_dist,0.1);               // foward simulation distance
      node_private.param("drive_mode", COMS_->control_paramters.drive_mode, false);     // false: velocity_mode, true: stroke_mode
      node_private.param("steer_mode", COMS_->control_paramters.steer_mode, false);     // false: velocity_mode, true: stroke_mode
      node_private.param("target_velocity", COMS_->control_paramters.target_velocity, 1.2); // target velocity 1.25 m/s

      ros::NodeHandle node;
      odom_sub_ = node.subscribe<nav_msgs::Odometry>(odom_topic_, 1, boost::bind(&ToyotaLocalPlanner::odomCallback, this, _1));
      path_sub_ = node.subscribe<nav_msgs::Path>(path_topic_, 1, boost::bind(&ToyotaLocalPlanner::pathCallback, this, _1));

      actual_vel_sub_ = node.subscribe<std_msgs::Float32>("/actual_vel",1,boost::bind(&ToyotaLocalPlanner::actualvelCallback, this, _1));
      actual_str_sub_ = node.subscribe<std_msgs::Float32>("/actual_str",1,boost::bind(&ToyotaLocalPlanner::actualstrCallback, this, _1));

      footprint_projection_path_pub_ = node.advertise<geometry_msgs::PolygonStamped>("footprint_projection_path",100, false);
      footprint_projection_odom_pub_ = node.advertise<geometry_msgs::PolygonStamped>("footprint_projection_odom",100, false);
      //marker_publisher = node.advertise<visualization_msgs::Marker>("frontaxle_marker",100); // cross track error marker
      steer_publisher = node.advertise<std_msgs::Float32>(steer_topic_,100);
      drive_publisher = node.advertise<std_msgs::Float32>(drive_topic_,100);

      footprint_projection_path_.header.frame_id = "/map";
      footprint_projection_odom_.header.frame_id = "/map";
      invalid_goal_ = false;
      steer.data = 0;
      drive.data = 0;

      // ****** vehicle obj *******//
      COMS_->vehicle_paramters.wheelR = 0.24;
      COMS_->vehicle_paramters.length = 2.406;
      COMS_->vehicle_paramters.width = 0.93;
      COMS_->vehicle_paramters.axelDist = 1.530;
      COMS_->vehicle_paramters.clampV = 5;                 // at high speed,control should be tuned
      COMS_->vehicle_paramters.minSteerVehicA = -33;  //default -35
      COMS_->vehicle_paramters.maxSteerVehicA = 33;   //default 35
      COMS_->vehicle_paramters.minSteerWheelA = -660; //default -450
      COMS_->vehicle_paramters.maxSteerWheelA = 660;  //default 450
      COMS_->vehicle_paramters.steerRatio = COMS_->vehicle_paramters.maxSteerWheelA / COMS_->vehicle_paramters.maxSteerVehicA;
  }

  double ToyotaLocalPlanner::compute_obs_dist_path(int id){
      double dist_acc = 0;
      // check points starting from <id to ?m ahead> of path for collision
      while(dist_acc<=COMS_->control_paramters.minLookahead_dist_path && id<global_path_.poses.size()-1){
          double x = global_path_.poses.at(id).pose.position.x;
          double y = global_path_.poses.at(id).pose.position.y;
          double theta = tf::getYaw(global_path_.poses.at(id).pose.orientation);
          std::vector<geometry_msgs::Point> footprint_pts;
          costmap_ros_->getOrientedFootprint(x,y,theta,footprint_pts);
          double cost = costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,inscribed_radius_,circumscribed_radius_);
          // footprint projection
          if(fabs(vehicle_odom_.twist.twist.linear.x) >= 0.1 || fabs(vehicle_odom_.twist.twist.linear.y) >= 0.1){
              for(unsigned int j=0; j<footprint_pts.size(); j++){
                  geometry_msgs::Point32 fppt;
                  fppt.x = footprint_pts.at(j).x;
                  fppt.y = footprint_pts.at(j).y;
                  fppt.z = footprint_pts.at(j).z;
                  footprint_projection_path_.polygon.points.push_back(fppt);
              }
              footprint_projection_path_.header.stamp = ros::Time::now();
              footprint_projection_path_pub_.publish(footprint_projection_path_);
          }
          // check cost for collision
          if(cost<0 || cost >= lethal_cost_threshold_){
              ROS_INFO("Cost Viloation: Path Distance to obstacle %f", dist_acc);
              return dist_acc;
          }
          dist_acc += sqrt(pow(global_path_.poses.at(id).pose.position.x - global_path_.poses.at(id+1).pose.position.x,2) +
                      pow(global_path_.poses.at(id).pose.position.y - global_path_.poses.at(id+1).pose.position.y,2));
          id++;
      }
      return dist_acc + COMS_->control_paramters.stop_dist_path;  // force dist_acc >= COMS_->control_paramters.stop_dist_path;
  }

  double ToyotaLocalPlanner::compute_obs_time_path(int id){
      double time_acc = 0;

      return time_acc;
  }

  double ToyotaLocalPlanner::compute_obs_dist_odom(){
      double dist_acc = 0;    // accumulated arc length
      //boost::mutex::scoped_lock lock(odom_lock_);
      double vehicle_steerangle = actual_str.data/COMS_->vehicle_paramters.steerRatio /180 * PI;         // instantanenous vehicle steering angle
      double x = vehicle_odom_.pose.pose.position.x;
      double y = vehicle_odom_.pose.pose.position.y;
      double vehicle_heading = tf::getYaw(vehicle_odom_.pose.pose.orientation);
      double theta = 0.0;                                                                                // accumulated theta
      double delta_theta = 2 * asin(COMS_->control_paramters.forwad_sim_dist*sin(-vehicle_steerangle)/(2*COMS_->vehicle_paramters.axelDist));  // theta increment
      double r = COMS_->vehicle_paramters.axelDist / sin(fabs(vehicle_steerangle));                            // instantanenous turning radius

      while(dist_acc<=COMS_->control_paramters.minlookahead_dist_odom){
          theta = theta + delta_theta;
          if(fabs(vehicle_steerangle) > 0.001745333){
              dist_acc = fabs(theta) * r;
              vehicle_heading = vehicle_heading + delta_theta;
          }
          else{
              dist_acc = dist_acc + COMS_->control_paramters.forwad_sim_dist;
          }
          x = x + cos(vehicle_heading)*COMS_->control_paramters.forwad_sim_dist;
          y = y + sin(vehicle_heading)*COMS_->control_paramters.forwad_sim_dist;

          std::vector<geometry_msgs::Point> footprint_pts;
          costmap_ros_->getOrientedFootprint(x,y,vehicle_heading,footprint_pts);
          double cost = costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,inscribed_radius_,circumscribed_radius_);
          // footprint projection
          if(sqrt(pow(vehicle_odom_.twist.twist.linear.x,2) + pow(vehicle_odom_.twist.twist.linear.y,2)) >= 0.1){
              for(unsigned int j=0; j<footprint_pts.size(); j++){
                  geometry_msgs::Point32 fppt;
                  fppt.x = footprint_pts.at(j).x;
                  fppt.y = footprint_pts.at(j).y;
                  fppt.z = footprint_pts.at(j).z;
                  footprint_projection_odom_.polygon.points.push_back(fppt);
              }
              footprint_projection_odom_.header.stamp = ros::Time::now();
              footprint_projection_odom_pub_.publish(footprint_projection_odom_);
          }
          // check cost for collision
          if(cost<0 || cost >= lethal_cost_threshold_){
              ROS_INFO("Cost Viloation: Odom Distance to obstacle %f", dist_acc);
              return dist_acc;
          }

      }
      return dist_acc;
  }

  double ToyotaLocalPlanner::compute_obs_time_odom(){
      double time_acc = 0;

      return time_acc;
  }

  // from the current pose of the vehicle, look ahead certain distance on the global path, check for
  // 1) path collision: definite collision
  // 2) oriented footprint on path collision: definite collision
  // signal gloabl path planner for new global path
  // slow down the vehicle to wait for new path
  // if cost become negative (definite collision) or very high cost (close to obstacles), stop the vehicle until new path available
  bool ToyotaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){  // by returning false to inform global planner to replan

      if(!global_path_.poses.empty()){
          double vehicle_heading = tf::getYaw(vehicle_odom_.pose.pose.orientation);
          double front_axle_x = vehicle_odom_.pose.pose.position.x; //+ cos(vehicle_heading)*1.5; 
          double front_axle_y = vehicle_odom_.pose.pose.position.y; //+ sin(vehicle_heading)*1.5; 
          // find nearest point on path
          double dist = std::numeric_limits<double>::max();
          
          for(unsigned int i=id; i< std::min(id+20, global_path_.poses.size()); i++){  // only look for the 20 points ahead
                
              double temp = sqrt(pow(front_axle_x - global_path_.poses.at(i).pose.position.x,2) +
                                 pow(front_axle_y - global_path_.poses.at(i).pose.position.y,2));
              if(temp < dist){
                  dist = temp;
                  id = i;
              }
          }
          // check path validity, if nearest path-vehicle distance > minctr_dist, abort the path
          if(dist >= COMS_->control_paramters.minctr_dist){
               ROS_WARN("All points on path has a path-vehicle distance larger than %f, stop and replan", COMS_->control_paramters.minctr_dist);
               //std_msgs::Float32 temp;
               drive.data = 0.0;
               drive_publisher.publish(drive);
               return false;
          }

          // compute distance/time to obstacle on path
          double obs_dist_path = compute_obs_dist_path(id);
          double obs_time_path = compute_obs_time_path(id);
          // compute distance/time to obstacle using odometry projection
          double obs_dist_odom = compute_obs_dist_odom();
          double obs_time_odom = compute_obs_time_odom();


          if(!COMS_->control_paramters.lookahead_modeFlag){      // dist mode
              if(obs_dist_path <= COMS_->control_paramters.stop_dist_path || obs_dist_odom <= COMS_->control_paramters.stop_dist_odom){
                  ROS_INFO("obs_dist_path is %f and obs_dist_odom is %f", obs_dist_path, obs_dist_odom);
                  //std_msgs::Float32 temp;
                  drive.data = 0.0;
                  drive_publisher.publish(drive);
                  cost_violation_times_++;
                  if(cost_violation_times_ < planner_patience_){
                      ROS_INFO("Stop vehicle, waiting for obstacles to be cleared or replan");
                      return true;
                  }
                  else{
                      ROS_INFO("Replan initiated");
                      return false;
                  }
              }
          }
          else{                         // dist and time mode
          }
          cost_violation_times_ = 0;
          footprint_projection_path_.polygon.points.clear();
          footprint_projection_odom_.polygon.points.clear();

          //////////////////////////////////////////////////////////////////////

          if(sqrt(pow(vehicle_odom_.twist.twist.linear.x,2) + pow(vehicle_odom_.twist.twist.linear.y,2)) >= COMS_->control_paramters.minVelocity_steer){
              steer.data = COMS_->compute_steer(vehicle_odom_,global_path_,id,dist,steer.data);
              steer_publisher.publish(steer);
          }
          drive.data = COMS_->compute_drive(vehicle_odom_, actual_vel.data, actual_str.data, COMS_->control_paramters.target_velocity, obs_dist_path,obs_time_path,drive.data);
          drive_publisher.publish(drive);
      }

      return true;  
  }

  bool ToyotaLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
      return true;
    
  }

  bool ToyotaLocalPlanner::isGoalReached(){
      if(!global_path_.poses.empty()){
          geometry_msgs::Pose goal;
          double dist_diff, heading_diff;
          goal = global_path_.poses.at(std::min(id+20, global_path_.poses.size()-1)).pose;
          dist_diff = sqrt(pow(vehicle_odom_.pose.pose.position.x - goal.position.x, 2) + pow(vehicle_odom_.pose.pose.position.y - goal.position.y, 2));
          heading_diff = COMS_->angledifference(tf::getYaw(vehicle_odom_.pose.pose.orientation), tf::getYaw(goal.orientation));
          if(dist_diff<= goal_distance_bias_ && heading_diff <= goal_heading_bias_){
              ROS_INFO("Goal Reached, stop and reset steer");
              steer.data = 0;
              drive.data = 0;
              id = 0;
              steer_publisher.publish(steer);
              drive_publisher.publish(drive);
              return true;
          }
          /*else if(cost_violation_times_ >= planner_patience_)
          {
              ROS_INFO("Goal can't be reached, stop, reset steer and abort");
              /steer.data = 0;
              drive.data = 0;
              steer_publisher.publish(steer);
              drive_publisher.publish(drive);
              return true;
          }*/
      }
      /*if(invalid_goal_){
          invalid_goal_ = false;
          std_msgs::Float32 temp;
          temp.data = 0.0;
          steer_publisher.publish(temp);
          drive_publisher.publish(temp);
          return true;
      }*/
      return false;
    
  }


  void ToyotaLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
      //ROS_DEBUG("odom received");
      boost::mutex::scoped_lock lock(odom_lock_);
      vehicle_odom_.child_frame_id = msg->child_frame_id;
      vehicle_odom_.header = msg->header;
      vehicle_odom_.pose = msg->pose;
      vehicle_odom_.twist = msg->twist;
      //ROS_INFO("Linear velocity is %f km/hr", sqrt(pow(vehicle_odom_.twist.twist.linear.x,2) + pow(vehicle_odom_.twist.twist.linear.y,2)) * 3.6);
  }

  void ToyotaLocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg){
      //ROS_DEBUG("path received");
      global_path_ = *msg;
      cost_violation_times_ = 0;
      id = 0;
      //newPathFlag = true;
      for(unsigned int i=0; i<global_path_.poses.size(); i++){
           std::cout<<"path point "<< i <<": " << global_path_.poses.at(i).pose.position.x <<", "<< global_path_.poses.at(i).pose.position.y<<", "<<tf::getYaw(global_path_.poses.at(i).pose.orientation) <<std::endl;
      }
  }

  void ToyotaLocalPlanner::actualvelCallback(const std_msgs::Float32::ConstPtr &msg){
      actual_vel.data = msg->data; // in km/hr
      //ROS_INFO("Velocity difference is %f km/hr", fabs(sqrt(pow(vehicle_odom_.twist.twist.linear.x,2) + pow(vehicle_odom_.twist.twist.linear.y,2))*3.6 - actual_vel.data));
  }

  void ToyotaLocalPlanner::actualstrCallback(const std_msgs::Float32::ConstPtr &msg){
      actual_str.data = msg->data; // in degree
      //ROS_INFO("Steering angle is %f degree",actual_str.data);
  }
  
}


/*double ToyotaLocalPlanner::compute_obs_dist_odom(){
      double dist_acc = 0;
      //boost::mutex::scoped_lock lock(odom_lock_);
      while(dist_acc<=COMS_->control_paramters.minlookahead_dist_odom){
          double vehicle_heading = tf::getYaw(vehicle_odom_.pose.pose.orientation);
          double x = vehicle_odom_.pose.pose.position.x + cos(vehicle_heading)*(dist_acc);
          double y = vehicle_odom_.pose.pose.position.y + sin(vehicle_heading)*(dist_acc);
          std::vector<geometry_msgs::Point> footprint_pts;
          costmap_ros_->getOrientedFootprint(x,y,vehicle_heading,footprint_pts);
          double cost = costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,inscribed_radius_,circumscribed_radius_);
          // footprint projection
          if(fabs(vehicle_odom_.twist.twist.linear.x) >= 0.1 || fabs(vehicle_odom_.twist.twist.linear.y) >= 0.1){
              for(unsigned int j=0; j<footprint_pts.size(); j++){
                  geometry_msgs::Point32 fppt;
                  fppt.x = footprint_pts.at(j).x;
                  fppt.y = footprint_pts.at(j).y;
                  fppt.z = footprint_pts.at(j).z;
                  footprint_projection_odom_.polygon.points.push_back(fppt);
              }
              footprint_projection_odom_.header.stamp = ros::Time::now();
              footprint_projection_odom_pub_.publish(footprint_projection_odom_);
          }
          // check cost for collision
          if(cost<0 || cost >= lethal_cost_threshold_){
              ROS_INFO("Cost Viloation: Odom Distance to obstacle %f", dist_acc);
              return dist_acc;
          }
          dist_acc = dist_acc + 0.1;
      }
      return dist_acc;
  }

  double ToyotaLocalPlanner::compute_obs_time_odom(){
      double time_acc = 0;

      return time_acc;
  }*/

/*marker.header.stamp = ros::Time::now();
          marker.header.frame_id = "/odom";
          marker.pose.position.x = front_axle_x;
          marker.pose.position.y = front_axle_y;
          marker.scale.x = 0.1;
          marker.scale.y = 0.1;
          marker.scale.z = 0.1;
          marker.color.a = 1.0;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker_publisher.publish(marker);*/

/*if(!use_lookahead_distance_){
              // check points starting from <id to ?steps ahead> of path for 1) path collision 2) footprint collision
              for(unsigned int i=id; i<std::min(id+lookahead_step_, (double)global_path_.poses.size()); i++){ //global_path_.poses.size();
                  // check for oriented footprint on path
                  double x = global_path_.poses.at(i).pose.position.x;
                  double y = global_path_.poses.at(i).pose.position.y;
                  double theta = tf::getYaw(global_path_.poses.at(i).pose.orientation);

                  std::vector<geometry_msgs::Point> footprint_pts;
                  //costmap_ros_->getOrientedFootprint(footprint_pts);
                  costmap_ros_->getOrientedFootprint(x,y,theta,footprint_pts);

                  double cost = costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,inscribed_radius_,circumscribed_radius_);
                  //std::cout<<"cost: "<<costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,inscribed_radius_,circumscribed_radius_)<<std::endl;
                  if(cost<0 || cost >= lethal_cost_threshold_){
                      std::cout<<"cost violation: "<<cost<<std::endl;
                      std_msgs::Float32 temp;
                      temp.data = 0.0;
                      steer_publisher.publish(temp);
                      drive_publisher.publish(temp);
                      return false;
                  }

                  // publish projected footprint: only when id changed, or new path received, or antoher way is odom.x/y > threshold
                  // to be done
                  if(fabs(vehicle_odom_.twist.twist.linear.x) >= 0.3 || fabs(vehicle_odom_.twist.twist.linear.y) >= 0.3){
                      for(unsigned int j=0; j<footprint_pts.size(); j++){
                          geometry_msgs::Point32 fppt;
                          fppt.x = footprint_pts.at(j).x;
                          fppt.y = footprint_pts.at(j).y;
                          fppt.z = footprint_pts.at(j).z;
                          footprint_projection_.polygon.points.push_back(fppt);
                      }
                      footprint_projection_.header.stamp = ros::Time::now();
                      footprint_projection_pub_.publish(footprint_projection_);
                  }
              }
          }*/

/*double x, y, cost;
unsigned int cx, cy;
x = global_path_.poses.at(i).pose.position.x;
y = global_path_.poses.at(i).pose.position.y;
costmap_->worldToMap(x,y,cx,cy);
cost = costmap_model_->pointCost(cx,cy);

if(cost <= 0 && i<global_path_.poses.size()-1){
    std::cout<<"point cost is "<<cost<<std::endl;
    return false;
}
else{
    std::cout<<"invalid goal, abort "<<cost<<std::endl;
    invalid_goal_ = true;
    return true;
}*/


//std::cout<<tf::getYaw(vehicle_odom_.pose.pose.orientation)<<std::endl;
/*marker.header.stamp = ros::Time::now();
marker.header.frame_id = "/map";
marker.pose.position.x = vehicle_odom_.pose.pose.position.x + cos(tf::getYaw(vehicle_odom_.pose.pose.orientation))*COMS_->toyotaComs.axelDist/2;
marker.pose.position.y = vehicle_odom_.pose.pose.position.y + sin(tf::getYaw(vehicle_odom_.pose.pose.orientation))*COMS_->toyotaComs.axelDist/2;
marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0;
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker_publisher.publish(marker);*/

//std::cout<<footprint_pts.at(0)<<std::endl;
//std::cout<<footprint_pts.at(1)<<std::endl;
//std::cout<<footprint_pts.at(2)<<std::endl;
//std::cout<<footprint_pts.at(3)<<std::endl;
//std::cout<<footprint_pts.size()<<std::endl;


//if(costmap_model_->footprintCost(vehicle_odom_.pose.pose.position,footprint_pts,0.5,0.5)>0)
//    std::cout<<"ok"<<std::endl;
//else
//    std::cout<<"crash"<<std::endl;

//std::cout<<costmap_ros_->getRobotFootprint().at(0)<<std::endl;
//std::cout<<costmap_ros_->getRobotFootprint().at(1)<<std::endl;
//std::cout<<costmap_ros_->getRobotFootprint().at(2)<<std::endl;
//std::cout<<costmap_ros_->getRobotFootprint().at(3)<<std::endl;
//std::cout<<costmap_ros_->getRobotFootprint().size()<<std::endl;

//tf::Quaternion q;
//tf::quaternionMsgToTF(global_path_.poses.at(i).pose.orientation, q);
//q = q.normalize();
//tf::quaternionTFToMsg(q,global_path_.poses.at(i).pose.orientation);


/*this->toyotaComs.wheelR = 0.24;
this->toyotaComs.length = 2.406;
this->toyotaComs.width = 0.93;
this->toyotaComs.axelDist = 1.530;
this->toyotaComs.targetV = 3;
this->toyotaComs.clampV = 5;                 // at high speed,control should be tuned
this->toyotaComs.floorV   = 2.5;
this->toyotaComs.minSteerVehicA = -45;
this->toyotaComs.maxSteerVehicA = 45;
this->toyotaComs.minSteerWheelA = -450;
this->toyotaComs.maxSteerWheelA = 450;
this->toyotaComs.steerRatio = toyotaComs.maxSteerWheelA/toyotaComs.maxSteerVehicA;
this->toyotaComs.sK = 0.85;
this->vehicle_init();*/
