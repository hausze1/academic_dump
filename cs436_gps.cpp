#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Hau Sze CS636 HW 4 Spring 2014
// Publisher for GPS data
ros::Publisher pub_vo;
// Publisher for IMU data
ros::Publisher pub_imu;

// Save the previous GPS odometry (to compute velocity)
nav_msgs::Odometry odometry_previous;

// Save the previous GPS data
nav_msgs::Odometry gps_old;

// This callback is executed each time an odometry of the satellite is received
// fill the velocity part of the new odometry and then publish it.
void gps_callback( const nav_msgs::Odometry& odometry_new )
{

  // time of the previous measurement in seconds
  double time_previous = (odometry_previous.header.stamp.sec + 
			  odometry_previous.header.stamp.nsec/1000000000.0);
  // time of the new measurement in seconds
  double time_new =      (odometry_new.header.stamp.sec + 
			  odometry_new.header.stamp.nsec/1000000000.0);

  // Compute the time increment between the new and previous odometry 
  // messages
  double dt = time_new - time_previous;

  // Result is the odometry that we send to the EKF
  nav_msgs::Odometry result( odometry_new );
  // create a new gps data



  // Compute the x, y, z velocity by using the current and 
  // previous GPS data
  // extract the position information from the new and previous odometry
double pose_oldx = odometry_previous.pose.pose.position.x;
double pose_newx = odometry_new.pose.pose.position.x;
double pose_oldy = odometry_previous.pose.pose.position.y;
double pose_newy = odometry_new.pose.pose.position.y;
double pose_oldz = odometry_previous.pose.pose.position.z;
double pose_newz = odometry_new.pose.pose.position.z;
  // use the finite difference method to compute the linear velocity
  result.twist.twist.linear.x = (pose_newx-pose_oldx)/dt ;
  result.twist.twist.linear.y = (pose_newy-pose_oldy)/dt  ;
  result.twist.twist.linear.z = (pose_newz-pose_oldz)/dt  ;

  // Set the covariance of the pose.
  // The member covariance is an 6x6 matrix that is represented as an 
  // 1x36 vector

// define a variable for the pose variance
double a_pose = 0.05;

  result.pose.covariance[0]   = a_pose;        // X covariance
  result.pose.covariance[0+6+1] = a_pose;      // Y covariance
  result.pose.covariance[0+6+6+2] = a_pose;    // Z covariance

  // Set the covariance of the velocity
  // The member covariance is an 6x6 matrix that is represented as an 
  // 1x36 vector

// define a variable for the twist variance
double a_twist = pow((0.68/3),2);  // eyeball estimation of the 3 standard deviation

  result.twist.covariance[0]   = a_twist;               // X covariance
  result.twist.covariance[0+6+1] = a_twist;             // Y covariance
  result.twist.covariance[0+6+6+2] = a_twist;           // Z covariance

  // publish the result to the EKF
  pub_vo.publish( result );

  // save the new odometry
  odometry_previous = odometry_new;

}

// This callback is executed each time IMU data is received
void imu_callback( const sensor_msgs::Imu& imu )
{

  sensor_msgs::Imu result( imu );

  // set the covariance of the orientation
// define a variable for the orientation variance
double a_orient = 0.01;
  result.orientation_covariance[0] =  a_orient;
  result.orientation_covariance[4] =  a_orient;
  result.orientation_covariance[8] =  a_orient;

  // set the covariance of the angular velocity
// define a variable for the angular velocity variance
double a_angular = pow((0.03/3),2);  // another eyeball estimation using 3 standard deviation
  result.angular_velocity_covariance[0] =  a_angular;
  result.angular_velocity_covariance[4] = a_angular;
  result.angular_velocity_covariance[8] = a_angular;

  // set the covariance of the linear acceleration
// define a variable for the linear acc. variance
double a_linear = pow((0.069/3),2); // last eyeball estimate
  result.linear_acceleration_covariance[0] =  a_linear;
  result.linear_acceleration_covariance[4] =  a_linear;
  result.linear_acceleration_covariance[8] =  a_linear;

  // publish the result to the EKF
  pub_imu.publish( result );

}

int main( int argc, char** argv )
{

  ros::init( argc, argv, "cs436_gps" );

  ros::NodeHandle nh;

  // We will publish the topics that are used by the EKF
  pub_vo = nh.advertise<nav_msgs::Odometry>( "/vo", 1 );
  pub_imu = nh.advertise<sensor_msgs::Imu>( "/imu_data", 1 );

  // We will intercept the GPS odometry
  ros::Subscriber sub_gps_odometry;
  sub_gps_odometry = nh.subscribe( "/gps/odom", 1, gps_callback );

  // We will intercept the IMU data
  ros::Subscriber sub_imu;
  sub_imu = nh.subscribe( "/imu", 1, imu_callback );

  ros::spin();

  return 0;

}
