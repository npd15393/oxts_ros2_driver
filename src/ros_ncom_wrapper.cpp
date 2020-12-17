#include "ros_ncom_wrapper.hpp"



sensor_msgs::msg::NavSatFix RosNComWrapper::wrap_nav_sat_fix(const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::NavSatFix();

  msg.header.stamp.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msg.header.stamp.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))*NAV_CONST::SECS2NANOSECS);
  msg.header.frame_id = "WGS84"; // @TODO Change this

  msg.status.status = nrx->mGpsPosMode; // @TODO Change to ROS code
  msg.status.service = 8; // @TODO: 1 GPS, 2 GLO, 4 Bei, 8 GAL

  msg.latitude  = nrx->mLat;
  msg.longitude = nrx->mLon;
  msg.altitude  = nrx->mAlt;

  // @TODO: This accuracy is not actually a covariance. Also should be in ENU
  msg.position_covariance[0] = nrx->mNorthAcc;
  msg.position_covariance[4] = nrx->mEastAcc;
  msg.position_covariance[8] = nrx->mAltAcc;

  msg.position_covariance_type = 2; // @TODO Change to ROS code
  


  return msg;
}


 nav_msgs::msg::Odometry RosNComWrapper::wrap_odometry (const NComRxC *nrx)
 {
  auto msg = nav_msgs::msg::Odometry();

  msg.header.stamp.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msg.header.stamp.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))*NAV_CONST::SECS2NANOSECS);
  msg.header.frame_id = "WGS84"; // @TODO Change this

  msg.child_frame_id = "";

  // Together, msgs Point and Quaternion make a geometry_msgs/Pose
  // geometry_msgs/Point
  msg.pose.pose.position.x = 0.0; // float64, make local coords
  msg.pose.pose.position.y = 0.0; // float64, make local coords
  msg.pose.pose.position.z = 0.0; // float64, make local coords

  // geometry_msgs/Quaternion
  msg.pose.pose.orientation.x = 0.0; // float64, make local coords
  msg.pose.pose.orientation.y = 0.0; // float64, make local coords
  msg.pose.pose.orientation.z = 0.0; // float64, make local coords
  msg.pose.pose.orientation.w = 0.0; // float64, make local coords
  
  msg.pose.covariance[0] = 0.0;
  // ...
  msg.pose.covariance[35] = 0.0;

  // geometry_msgs/TwistWithCovariance
  // This expresses velocity in free space broken into its linear and angular parts.
  msg.twist.twist.linear.x  = nrx->mVn; // @TODO Check coordinate frame
  msg.twist.twist.linear.y  = nrx->mVe; // @TODO Check coordinate frame
  msg.twist.twist.linear.z  = nrx->mVd; // @TODO Check coordinate frame
  msg.twist.twist.angular.x = nrx->mWx; // @TODO Check coordinate frame
  msg.twist.twist.angular.y = nrx->mWy; // @TODO Check coordinate frame
  msg.twist.twist.angular.z = nrx->mWz; // @TODO Check coordinate frame
  // @TODO Check coordinate frame
  msg.twist.covariance[0] = 0.0;
  // ...
  msg.twist.covariance[35] = 0.0;


  return msg;
 }


std_msgs::msg::String RosNComWrapper:wrap_string (const NComRxC *nrx)
{
  auto msg = std_msgs::msg::String();
  msg.data = "Lat, Long, Alt : " + std::to_string(nrx->mLat) + ", "
                                 + std::to_string(nrx->mLon) + ", "
                                 + std::to_string(nrx->mAlt);

  return msg;
}