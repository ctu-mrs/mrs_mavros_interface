#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <tf/LinearMath/Transform.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/Profiler.h>
#include <std_srvs/Trigger.h>

namespace mrs_mavros_interface
{

//{ class MavrosInterface

class MavrosInterface : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
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

private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_odometry_callback;
};

//}

//{ onInit()

void MavrosInterface::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry = nh_.subscribe("odometry_in", 1, &MavrosInterface::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------
  //
  publisher_odometry = nh_.advertise<nav_msgs::Odometry>("odometry_out", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  service_server_jump_emulation = nh_.advertiseService("emulate_jump", &MavrosInterface::emulateJump, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                  = new mrs_lib::Profiler(nh_, "MavrosInterface");
  routine_odometry_callback = profiler->registerRoutine("callbackOdometry");

  // | ----------------------- finish init ---------------------- |

  is_initialized = true;

  ROS_INFO("[MavrosInterface]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackOdometry()

void MavrosInterface::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_odometry_callback->start();

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

  updated_odometry.pose.pose.position.x += jump_offset;

  // --------------------------------------------------------------
  // |        update the odometry message with the new data       |
  // --------------------------------------------------------------

  updated_odometry.twist.twist.linear.x = rotated_velocity[0];
  updated_odometry.twist.twist.linear.y = rotated_velocity[1];
  updated_odometry.twist.twist.linear.z = rotated_velocity[2];

  updated_odometry.twist.twist.angular.x = rotated_angular[0];
  updated_odometry.twist.twist.angular.y = rotated_angular[1];
  updated_odometry.twist.twist.angular.z = rotated_angular[2];

  updated_odometry.child_frame_id = "local_origin";

  // --------------------------------------------------------------
  // |                  publish the new odometry                  |
  // --------------------------------------------------------------

  try {
    publisher_odometry.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(updated_odometry)));
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_odometry.getTopic().c_str());
  }

  routine_odometry_callback->end();
}

//}

//{ emulateJump()

bool MavrosInterface::emulateJump(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  jump_offset += 2.0;

  if (jump_offset > 3) {
    jump_offset = 10;
  }

  res.message = "yep";
  res.success = true;

  return true;
}

//}
}  // namespace mrs_mavros_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mavros_interface::MavrosInterface, nodelet::Nodelet)
