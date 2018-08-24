#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <mavros_msgs/State.h>

#include <mrs_msgs/MavrosDiagnostics.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Profiler.h>

#include <mutex>

#define btoa(x) ((x) ? "true" : "false")

namespace mrs_mavros_interface
{

//{ class MavrosDiagnostics

class MavrosDiagnostics : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

private:
  ros::Subscriber subscriber_diagnostics;
  ros::Subscriber subscriber_mavros_state;
  ros::Publisher  publisher_diagnostics;

private:
  int    min_satellites_visible_ = 6;
  double min_voltage_            = 14.0;
  double max_cpu_load_           = 90.0;

private:
  void callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);
  void callbackMavrosState(const mavros_msgs::StateConstPtr &msg);

private:
  diagnostic_msgs::DiagnosticArray mavros_diag;
  mrs_msgs::MavrosDiagnostics      diag;
  std::mutex                       mutex_diag;

private:
  std::string mode;
  int         satellites_visible, fix_type, errors_comm;
  int         last_errors_comm = 0;
  float       eph, epv, cpu_load, voltage, current;
  bool        offboard, armed;
  bool        last_offboard = false;
  bool        last_armed    = false;

private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_diagnostics_callback;
  mrs_lib::Routine * routine_mavros_state_callback;
};

//}

//{ onInit()

void MavrosDiagnostics::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         parameters                         |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "MavrosDiagnostics");
  param_loader.load_param("min_satellites_visible", min_satellites_visible_);
  param_loader.load_param("min_voltage", min_voltage_);
  param_loader.load_param("max_cpu_load", max_cpu_load_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_diagnostics  = nh_.subscribe("diagnostics_in", 1, &MavrosDiagnostics::callbackDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_mavros_state = nh_.subscribe("mavros_state_in", 1, &MavrosDiagnostics::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_diagnostics = nh_.advertise<mrs_msgs::MavrosDiagnostics>("diagnostics_out", 1);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                      = new mrs_lib::Profiler(nh_, "MavrosInterface");
  routine_diagnostics_callback  = profiler->registerRoutine("callbackDiagnostics");
  routine_mavros_state_callback = profiler->registerRoutine("callbackMavrosState");

  // | ----------------------- finish init ---------------------- |

  is_initialized = true;

  ROS_INFO("[MavrosDiagnostics]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackDiagnostics()
void MavrosDiagnostics::callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_diagnostics_callback->start();


  for (size_t i = 0; i < msg->status.size(); i++) {

    // GPS
    if (msg->status[i].name.find("GPS") != std::string::npos) {


      for (size_t j = 0; j < msg->status[i].values.size(); j++) {


        // Satellites visible
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Satellites visible") == 0) {
          satellites_visible = std::stoi(msg->status[i].values[j].value);
        }

        // Fix type
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Fix type") == 0) {
          fix_type = std::stoi(msg->status[i].values[j].value);
        }

        // EPH
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPH (m)") == 0) {
          eph = std::stof(msg->status[i].values[j].value);
        }

        // EPV
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPV (m)") == 0) {
          epv = std::stof(msg->status[i].values[j].value);
        }
      }
    }

    // System
    if (msg->status[i].name.find("System") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // CPU Load
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "CPU Load (%)") == 0) {
          cpu_load = std::stof(msg->status[i].values[j].value);
        }

        // Errors comm
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Errors comm") == 0) {
          errors_comm = std::stoi(msg->status[i].values[j].value);
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
          voltage = std::stof(msg->status[i].values[j].value);
        }

        // Current
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Current") == 0) {
          current = std::stof(msg->status[i].values[j].value);
        }
      }
    }
  }

  if (satellites_visible < min_satellites_visible_) {
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

  mutex_diag.lock();
  {
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
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_diagnostics.getTopic().c_str());
    }
  }
  mutex_diag.unlock();

  routine_diagnostics_callback->end();
}
//}

//{ callbackMavrosState()
void MavrosDiagnostics::callbackMavrosState(const mavros_msgs::StateConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_mavros_state_callback->start();

  armed = msg->armed;

  mutex_diag.lock();
  { diag.state.armed = armed; }
  mutex_diag.unlock();

  routine_mavros_state_callback->end();
}
//}

}  // namespace mrs_mavros_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mavros_interface::MavrosDiagnostics, nodelet::Nodelet)
