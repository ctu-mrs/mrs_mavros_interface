#define VERSION "0.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Int64.h>

#include <mavros_msgs/State.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/Vec1.h>

#include <mrs_lib/ParamLoader.h>

#include <mrs_lib/Profiler.h>

#include <mutex>

//}

#define btoa(x) ((x) ? "true" : "false")

namespace mrs_mavros_interface
{

namespace mavros_diagnostics
{

/* class MavrosDiagnostics //{ */

class MavrosDiagnostics : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  std::string _version_;

  bool is_initialized = false;

private:
  ros::Subscriber subscriber_diagnostics;
  ros::Subscriber subscriber_mavros_state;
  ros::Subscriber subscriber_simulation_num_satelites;

  ros::Publisher publisher_diagnostics;

  ros::ServiceServer service_sim_satellites;

private:
  int    min_satellites_visible_ = 6;
  double min_voltage_            = 14.0;
  double max_cpu_load_           = 90.0;

private:
  void callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);
  void callbackMavrosState(const mavros_msgs::StateConstPtr &msg);
  void callbackNumSatellites(const std_msgs::Int64ConstPtr &msg);

  bool callbackSimSatellites(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res);

private:
  diagnostic_msgs::DiagnosticArray mavros_diag;
  mrs_msgs::MavrosDiagnostics      diag;
  std::mutex                       mutex_diag;

private:
  std::string mode;
  int         satellites_visible, fix_type, errors_comm;
  int         last_errors_comm = 0;
  float       eph, epv, cpu_load, voltage, current;
  bool        offboard       = false;
  bool        armed          = false;
  bool        last_offboard  = false;
  bool        last_armed     = false;
  bool        sim_satellites = false;

private:
  std::mutex mutex_satellites_visible;

private:
  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;
};

//}

/* onInit() //{ */

void MavrosDiagnostics::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         parameters                         |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "MavrosDiagnostics");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MavrosDiagnostics]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", profiler_enabled_);

  param_loader.load_param("min_satellites_visible", min_satellites_visible_);
  param_loader.load_param("min_voltage", min_voltage_);
  param_loader.load_param("max_cpu_load", max_cpu_load_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_diagnostics  = nh_.subscribe("diagnostics_in", 1, &MavrosDiagnostics::callbackDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state = nh_.subscribe("mavros_state_in", 1, &MavrosDiagnostics::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_simulation_num_satelites =
      nh_.subscribe("num_satelites_in", 1, &MavrosDiagnostics::callbackNumSatellites, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_diagnostics = nh_.advertise<mrs_msgs::MavrosDiagnostics>("diagnostics_out", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  // subscribe for simulating different number of satellites
  service_sim_satellites = nh_.advertiseService("simulate_satellites_visible_in", &MavrosDiagnostics::callbackSimSatellites, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = mrs_lib::Profiler(nh_, "MavrosInterface", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MavrosDiagnostics]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[MavrosDiagnostics]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

/* callbackDiagnostics() //{ */

void MavrosDiagnostics::callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackDiagnostics");

  std::scoped_lock lock(mutex_satellites_visible);

  for (size_t i = 0; i < msg->status.size(); i++) {

    // GPS
    if (msg->status[i].name.find("GPS") != std::string::npos) {

      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Satellites visible
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Satellites visible") == 0) {
          if (!sim_satellites) {
            try {
              satellites_visible = std::stoi(msg->status[i].values[j].value);
            }
            catch (...) {
              satellites_visible = 0;
              ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid Satellites visible value: %s. Setting to 0;", msg->status[i].values[j].value.c_str());
            }
          }
        }

        // Fix type
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Fix type") == 0) {
          try {
            fix_type = std::stoi(msg->status[i].values[j].value);
          }
          catch (...) {
            fix_type = 0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid Fix type value: %s. Setting to 0;", msg->status[i].values[j].value.c_str());
          }
        }

        // EPH
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPH (m)") == 0) {
          try {
            eph = std::stof(msg->status[i].values[j].value);
          }
          catch (...) {
            eph = 0.0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid EPH value: %s. Setting to 0.0;", msg->status[i].values[j].value.c_str());
          }
        }

        // EPV
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPV (m)") == 0) {
          try {
            epv = std::stof(msg->status[i].values[j].value);
          }
          catch (...) {
            epv = 0.0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid EPV value: %s. Setting to 0.0;", msg->status[i].values[j].value.c_str());
          }
        }
      }
    }

    // System
    if (msg->status[i].name.find("System") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // CPU Load
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "CPU Load (%)") == 0) {
          try {
            cpu_load = std::stof(msg->status[i].values[j].value);
          }
          catch (...) {
            cpu_load = 0.0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid CPU Load value: %s. Setting to 0.0;", msg->status[i].values[j].value.c_str());
          }
        }

        // Errors comm
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Errors comm") == 0) {
          try {
            errors_comm = std::stoi(msg->status[i].values[j].value);
          }
          catch (...) {
            errors_comm = 0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid Errors comm value: %s. Setting to 0;", msg->status[i].values[j].value.c_str());
          }
        }
      }
    }

    // Heartbeat
    if (msg->status[i].name.find("Heartbeat") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Mode
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Mode") == 0) {
          mode = msg->status[i].values[j].value;
          if (std::strcmp(mode.c_str(), "OFFBOARD") == 0) {
            offboard = true;
          } else {
            offboard = false;
          }
        }
      }
    }

    // Battery
    if (msg->status[i].name.find("Battery") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Voltage
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Voltage") == 0) {
          try {
            voltage = std::stof(msg->status[i].values[j].value);
          }
          catch (...) {
            voltage = 0.0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid Voltage value: %s. Setting to 0.0;", msg->status[i].values[j].value.c_str());
          }
        }

        // Current
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Current") == 0) {
          try {
            current = std::stof(msg->status[i].values[j].value);
          }
          catch (...) {
            current = 0.0;
            ROS_ERROR_ONCE("[MavrosDiagnostics]: Invalid Current value: %s. Setting to 0.0;", msg->status[i].values[j].value.c_str());
          }
        }
      }
    }
  }

  if (satellites_visible > 0 && satellites_visible < min_satellites_visible_) {
    ROS_WARN("[MavrosDiagnostics]: Low number of satellites visible: %d", satellites_visible);
  }

  if (cpu_load > max_cpu_load_) {
    ROS_WARN("[MavrosDiagnostics]: High PixHawk CPU load: %f %%", cpu_load);
  }

  if (errors_comm > last_errors_comm) {
    last_errors_comm = errors_comm;
    ROS_WARN("[MavrosDiagnostics]: Communication errors detected: %d", errors_comm);
  }

  if (voltage < min_voltage_) {
    ROS_WARN("[MavrosDiagnostics]: Low battery voltage: %f V", voltage);
  }

  if (offboard != last_offboard) {
    last_offboard = offboard;
    ROS_WARN("[MavrosDiagnostics]: Offboard: %s", btoa(offboard));
  }

  if (armed != last_armed) {
    last_armed = armed;
    ROS_WARN("[MavrosDiagnostics]: Armed: %s", btoa(armed));
  }

  {
    std::scoped_lock lock(mutex_diag);

    diag.header.stamp           = ros::Time::now();
    diag.gps.satellites_visible = satellites_visible;
    diag.gps.fix_type           = fix_type;
    diag.gps.eph                = eph;
    diag.gps.epv                = epv;
    diag.system.cpu_load        = cpu_load;
    diag.system.errors_comm     = errors_comm;
    diag.heartbeat.mode         = mode;
    diag.battery.voltage        = voltage;
    diag.battery.current        = current;
    diag.state.offboard         = offboard;

    try {
      publisher_diagnostics.publish(diag);
    }
    catch (...) {
      ROS_ERROR("[MavrosDiagnostics]: Exception caught during publishing topic %s.", publisher_diagnostics.getTopic().c_str());
    }
  }
}

//}

/* callbackMavrosState() //{ */

void MavrosDiagnostics::callbackMavrosState(const mavros_msgs::StateConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackMavrosState");

  armed = msg->armed;

  {
    std::scoped_lock lock(mutex_diag);

    diag.state.armed = armed;
  }
}

//}

/* callbackNumSatellites() //{ */

void MavrosDiagnostics::callbackNumSatellites(const std_msgs::Int64ConstPtr &msg) {

  ROS_WARN_STREAM_THROTTLE(1, "[MavrosDiagnostics] num_satelites " << msg->data);

  {
    std::scoped_lock lock(mutex_satellites_visible);

    sim_satellites     = true;
    satellites_visible = msg->data;
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackSimSatellites() //{ */

bool MavrosDiagnostics::callbackSimSatellites(mrs_msgs::Vec1::Request &req, mrs_msgs::Vec1::Response &res) {

  if (req.goal < 0) {
    sim_satellites = false;
  } else {

    {
      std::scoped_lock lock(mutex_satellites_visible);

      sim_satellites     = true;
      satellites_visible = int(req.goal);
    }
  }

  res.success = true;
  res.message = std::string("Successfully set visible satellites to: ") + std::to_string((int)req.goal);

  return true;
}

//}

}  // namespace mavros_diagnostics

}  // namespace mrs_mavros_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mavros_interface::mavros_diagnostics::MavrosDiagnostics, nodelet::Nodelet)
