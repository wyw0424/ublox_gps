#include "Uart.h"
#include "UbloxGPS.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <eigen3/Eigen/Dense>


class ublox_gps_ros_node
{

private:
  ros::NodeHandle node_;

  // Subscriber
  //ros::Subscriber trigger_command_sub_;

  // Publisher
  ros::Publisher gps_pub_, gps_pose_pub_;

  // Service
  //ros::ServiceServer clear_num_service_;

  // Params
  double pic_distance_threshold_;
  bool first_pose_;
  double first_x, first_y, first_z;
public:

  
  ublox_gps_ros_node();
  ~ublox_gps_ros_node();
  double DegToRad(double deg);
  void update(double latitude, double longtitude, double altitude);
};

ublox_gps_ros_node::ublox_gps_ros_node()
{
  // parameters
  //ros::param::get("/ublox_gps_ros_node/data_file", data_file_);

  // subscribers
  // imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("/imu/data", 1, &ublox_gps_ros_node::imuCallback, this);

  // publishers
  gps_pub_ = node_.advertise<sensor_msgs::NavSatFix>("/fix", 1);
  gps_pose_pub_ = node_.advertise<nav_msgs::Odometry>("/gps_pose", 1);
  // Services
  //clear_num_service_ = node_.advertiseService("/image_process/clear_num_service", &ublox_gps_ros_node::clear_num_service, this);
  first_pose_ = true;
}

ublox_gps_ros_node::~ublox_gps_ros_node()
{

}

// Converts from degrees to radians.
double ublox_gps_ros_node::DegToRad(double deg) { return M_PI * deg / 180.; }


void ublox_gps_ros_node::update(double latitude, double longitude, double altitude)
{

    
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  double x = (N + altitude) * cos_phi * cos_lambda;
  double y = (N + altitude) * cos_phi * sin_lambda;
  double z = (b_squared / a_squared * N + altitude) * sin_phi;

  if(first_pose_)
  {
    first_x = x;
    first_y = y;
    first_z = z;
    
    first_pose_ = false;
  }
  x = x - first_x;
  y = y - first_y;
  z=  z - first_z;
  
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.),Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude), Eigen::Vector3d::UnitZ());
      
    // gps NavSatFix
    sensor_msgs::NavSatFix gps_data;
    gps_data.header.stamp = ros::Time::now();
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    gps_pub_.publish(gps_data);
    
    // gps Odometry
    nav_msgs::Odometry gps_pose;
    gps_pose.header.stamp = ros::Time::now();
    gps_pose.pose.pose.position.x = x;
    gps_pose.pose.pose.position.y = y;
    gps_pose.pose.pose.position.z = z;

    gps_pose.pose.pose.orientation.w = rotation.w();
    gps_pose.pose.pose.orientation.x = rotation.x();
    gps_pose.pose.pose.orientation.y = rotation.y();
    gps_pose.pose.pose.orientation.z = rotation.z();

    gps_pose_pub_.publish(gps_pose);
}


void convertTime(long long input, long long& ms, long long& s, long long& m, long long& h, long long& d)
{
    ms = (input % 1000); input = (input / 1000);
    s = (input % 60); input = (input / 60);
    m = ((int)input % 60); input = (input / 60);
    h = ((int)input % 24); d = (int)(input / 24);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_gps_ros_node");
    ublox_gps_ros_node node;
    ROS_INFO("ublox_gps_ros_node started...");

    ros::Rate rate(40);
    
    wincomm::Uart uart;
    int baudrate = 115200;
    if (!uart.open("/dev/ttyUSB0", baudrate)) return 1;

    std::string cmd("BSS-UBX-115200-10HZ-PVT-RHXZ-D1\r\n");
    uart.writeFromBuffer(cmd.data(), cmd.size());
    
    wincomm::UbloxGPS gps;
    gps.start(&uart);
    
    long long msec, seconds, minutes, hours, days;
    while (ros::ok())
    {
        ros::spinOnce();
        if (gps.readFromBuffer())
        {
            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_POSITION)
            {
                const wincomm::UbloxGPS::NavigationData& navData = gps.getLatestNavigation();
                convertTime((long long)navData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                printf("D%lld, T%02lld:%02lld:%02lld.%03lld: lat/lon/alt = %lf, %lf, %lf\n", days + 1,
                       hours, minutes, seconds, msec, navData.latitude, navData.longitude, navData.altitude);
                double latitude = navData.latitude;
                double longitude = navData.longitude;
                double altitude = navData.altitude;
                node.update(latitude, longitude, altitude);
            }
            
            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_VELOCITY)
            {
                const wincomm::UbloxGPS::VelocityData& velData = gps.getLatestVelocity();
                convertTime((long long)velData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                printf("D%lld, T%02lld:%02lld:%02lld.%03lld: velocity-N/E/D = %lf, %lf, %lf\n", days + 1,
                       hours, minutes, seconds, msec, velData.velocityN, velData.velocityE, velData.velocityD);
            }
            gps.clearDirtyMessageTypes();
        }
        rate.sleep();
    }
    return 0;
}
