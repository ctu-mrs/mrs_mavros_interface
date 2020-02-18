#define VERSION "0.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Transform.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

#include <std_srvs/Trigger.h>

//}

namespace mrs_mavros_interface
{

namespace mavros_interface
{

/* class MavrosInterface //{ */

class MavrosInterface : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  std::string _version_;

  ros::NodeHandle nh_;
  bool            is_initialized = false;

private:
  ros::Subscriber subscriber_odometry;
  ros::Publisher  publisher_odometry;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

private:
  ros::ServiceServer service_server_jump_emulation;
  bool               emulateJump(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  double             jump_offset = 0;

  ros::ServiceServer service_server_gps_covariance;
  bool               emulateGPSCovariance(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  double             covariance_low = 1.0;
  double             covariance_high = 11.0;
  double             cov_ = 1.0;

private:
  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;
  bool              _simulation_ = false;
  std::string       uav_name_;
  std::string       fcu_frame_id_;
  std::string       local_origin_frame_id_;
};

//}

/* onInit() //{ */

void MavrosInterface::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "MavrosInterface");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MavrosInterface]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("uav_name", uav_name_);
  local_origin_frame_id_ = uav_name_ + "/local_origin";
  fcu_frame_id_          = uav_name_ + "/fcu";
  param_loader.load_param("enable_profiler", profiler_enabled_);
  param_loader.load_param("simulation", _simulation_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry = nh_.subscribe("odometry_in", 1, &MavrosInterface::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_odometry = nh_.advertise<nav_msgs::Odometry>("odometry_out", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  if (_simulation_) {
    service_server_jump_emulation = nh_.advertiseService("emulate_jump", &MavrosInterface::emulateJump, this);
    service_server_gps_covariance = nh_.advertiseService("emulate_gps_covariance", &MavrosInterface::emulateGPSCovariance, this);
  }

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = mrs_lib::Profiler(nh_, "MavrosInterface", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[MavrosInterface]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdometry() //{ */

void MavrosInterface::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOdometry");

  nav_msgs::Odometry updated_odometry = *msg;

  // --------------------------------------------------------------
  // |        extract the vectors from the original message       |
  // --------------------------------------------------------------

  tf::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  tf::Vector3 angular(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

  // --------------------------------------------------------------
  // |           prepare the quaternion for the rotation          |
  // --------------------------------------------------------------

  tf::Quaternion rotation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // --------------------------------------------------------------
  // |                     rotate the vectors                     |
  // --------------------------------------------------------------

  tf::Vector3 rotated_velocity = tf::quatRotate(rotation, velocity);
  tf::Vector3 rotated_angular  = tf::quatRotate(rotation, angular);

  // --------------------------------------------------------------
  // |                        emulate jump                        |
  // --------------------------------------------------------------

  if (_simulation_) {
    updated_odometry.pose.pose.position.x += jump_offset;
    updated_odometry.pose.covariance.at(0) = cov_;
    updated_odometry.pose.covariance.at(7) = cov_;
  }

  // --------------------------------------------------------------
  // |        update the odometry message with the new data       |
  // --------------------------------------------------------------

  updated_odometry.twist.twist.linear.x = rotated_velocity[0];
  updated_odometry.twist.twist.linear.y = rotated_velocity[1];
  updated_odometry.twist.twist.linear.z = rotated_velocity[2];

  updated_odometry.twist.twist.angular.x = rotated_angular[0];
  updated_odometry.twist.twist.angular.y = rotated_angular[1];
  updated_odometry.twist.twist.angular.z = rotated_angular[2];

  updated_odometry.header.frame_id = local_origin_frame_id_;
  updated_odometry.child_frame_id  = fcu_frame_id_;

  // --------------------------------------------------------------
  // |                  publish the new odometry                  |
  // --------------------------------------------------------------

  try {
    publisher_odometry.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(updated_odometry)));
  }
  catch (...) {
    ROS_ERROR("[MavrosInterface]: Exception caught during publishing topic %s.", publisher_odometry.getTopic().c_str());
  }
}

//}

/* emulateJump() //{ */

bool MavrosInterface::emulateJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  /* jump_offset += 2.0; */
  jump_offset = 1000;

  /* if (jump_offset > 3 && jump_offset < 10) { */
  /*   jump_offset = 10; */
  /* } */

  /* if (jump_offset > 11) { */
  /*   jump_offset = 1000; */
  /* } */


  ROS_INFO("[MavrosInterface]: Emulated jump: %f", jump_offset);

  res.message = "yep";
  res.success = true;

  return true;
}

//}

/* emulateGPSCovariance() //{ */

bool MavrosInterface::emulateGPSCovariance([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized) {
    res.message = "nope";
    res.success = false;
    return true;
  }

  if (!_simulation_) {
    res.message = "nope";
    res.success = false;
    return true;
  }

  if (cov_ == covariance_high) {
    cov_ = covariance_low;
  } else {
    cov_ = covariance_high;
  }

  ROS_INFO("[MavrosInterface]: Emulated GPS covariance: %f", cov_);

  res.message = "yep";
  res.success = true;

  return true;
}

//}

}  // namespace mavros_interface

}  // namespace mrs_mavros_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mavros_interface::mavros_interface::MavrosInterface, nodelet::Nodelet)
