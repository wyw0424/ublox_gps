/*
 * pointcloud process ROS.cpp
 * Date: 2019-08-06
*/
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


extern int startGPS(const std::string& ntripCaster, const std::string& auth, const std::string& devAddr);
extern int updateGPS(double t, std::ofstream& out, double &latitude, double &longitude, double &altitude, bool &fix_valid, bool &diff_valid, int &fix_status, double &signal_score, int &num_valid_satellites);//double &v_north, double &v_east, double &v_down);

class gps_node
{

private:
  ros::NodeHandle node_;

  // Subscriber
  ros::Subscriber trigger_command_sub_;

  // Publisher
  ros::Publisher filter_pub_, odom_pub_, pub_undistorted_pc_, gps_pub_;

  // Service
  //ros::ServiceServer clear_num_service_;

  // Params
  bool first_pose_;
  bool trigger_camera_;

  int pic_count_;

  double latitude, longitude, altitude;
  double signal_score;
  
  bool fix_valid;
  bool diff_valid;
  int fix_status;
  int num_valid_satellites;

  std::string fix_frame_;
  std::string base_link_;
  std::string data_file_, gps_data_file_;

  tf::TransformListener listener;
  tf::StampedTransform frame_transform_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped last_pose_;

  std::ofstream out_data_path_;
  std::ofstream out_gps_path_;

public:

  gps_node();
  ~gps_node();
  void triggerCallback(const std_msgs::Bool::ConstPtr& msg);
  double get_covariance(double signal_score, int num_valid_satellites);
  void update();
};

gps_node::gps_node()
: first_pose_(true)
, pic_count_(0)
, trigger_camera_(false)
, latitude(0.0)
, longitude(0.0)
, altitude(0.0)
, num_valid_satellites(0)
, signal_score(0.0)
{
  // parameters
  std::string ntripIP, ntripUser, devAddr;
  ros::param::get("/gps_node/data_file", data_file_);
  ros::param::get("/gps_node/gps_data_file", gps_data_file_);
  ros::param::get("/gps_node/gps_dev_addr", devAddr);
  ros::param::get("/gps_node/gps_ntrip_userpwd", ntripUser);
  ros::param::get("/gps_node/gps_ntrip_server", ntripIP);
  if(!ros::param::get("/gps_node/fix_frame", fix_frame_))
  {
      fix_frame_ = "/map";
  }
  if(!ros::param::get("/gps_node/base_link", base_link_))
  {
      base_link_ = "/base_link";
  }
  // subscribers

  trigger_command_sub_ = node_.subscribe<std_msgs::Bool> ("/trigger", 1, &gps_node::triggerCallback, this);
  // imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("/imu/data", 1, &gps_node::imuCallback, this);

  // publishers
  gps_pub_ = node_.advertise<sensor_msgs::NavSatFix>("/gps", 1);

  // Services
  //clear_num_service_ = node_.advertiseService("/image_process/clear_num_service", &gps_node::clear_num_service, this);

  last_pose_ = current_pose_;

  
  startGPS(ntripIP, ntripUser, devAddr);
  out_data_path_.open(data_file_, std::ios_base::app);
  out_gps_path_.open(gps_data_file_, std::ios_base::app);

}

gps_node::~gps_node()
{
  out_data_path_.close();
  out_gps_path_.close();
}

void gps_node::triggerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  trigger_camera_ = true;
}

double gps_node::get_covariance(double signal_score, int num_valid_satellites)
{
  double covaraince = 99999.0;
  //std::cout << "signal_score: " << signal_score << std::endl;
  //std::cout << "num_valid_satellites: " << num_valid_satellites << std::endl;
  //std::cout << "covaraince: " <<covaraince/signal_score << std::endl;
  if(signal_score ==0)
  {
    return covaraince;
  }
  if(num_valid_satellites < 10)
  {
    return covaraince/signal_score;
  }
  else if(num_valid_satellites >=10 && num_valid_satellites < 15)
  {
    return covaraince = 0.5;
  }
  else
  {
    return covaraince = 0.1;
  }
}

void gps_node::update()
{
  double gps_time = ros::Time::now().toSec();
  //double v_north, v_east, v_down;


  if(updateGPS(gps_time, out_gps_path_, latitude, longitude, altitude, fix_valid, diff_valid, fix_status, signal_score, num_valid_satellites))//v_north, v_east, v_down))
  {
    //std::cout << "fix_valid: " << fix_valid << std::endl;
    //std::cout << "diff_valid: " << diff_valid << std::endl;
    //std::cout << "latitude: " << latitude << ", longitude: " << longitude << ", altitude: " << altitude <<std::endl;
    //std::cout << std::setprecision(14) << "v_north: " << v_north << ", v_east: " << v_east << ", v_down: " << v_down << std::endl;
    
    // gps NavSatFix
    sensor_msgs::NavSatFix gps_data;
    gps_data.header.stamp = ros::Time::now();
    gps_data.header.frame_id = "gps_frame";
    gps_data.status.status = fix_valid;
    gps_data.status.service = diff_valid;
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    double covaraince = get_covariance(signal_score, num_valid_satellites);

    //std::cout << "covaraince: " << covaraince << std::endl;

    gps_data.position_covariance[0] = covaraince;
    gps_data.position_covariance[3] = covaraince;
    gps_data.position_covariance[6] = covaraince;
    gps_pub_.publish(gps_data);
    
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_node");
  gps_node node;

  ROS_INFO("slam front end ros node started...");

  ros::Rate rate(100);

  while(ros::ok())
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }
  return 0;
}


