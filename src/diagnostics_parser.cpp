#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/Profiler.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <mrs_msgs/MavrosDiagnostics.h>

namespace mrs_mavros_interface
{

//{ class DiagnosticsParser

class DiagnosticsParser : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

private:
  ros::Subscriber subscriber_diagnostics;
  ros::Publisher  publisher_diagnostics;

private:
  void callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);

private:
  diagnostic_msgs::DiagnosticArray mavros_diag;

private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_diagnostics_callback;
};

//}

//{ onInit()

void DiagnosticsParser::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_diagnostics = nh_.subscribe("diagnostics_in", 1, &DiagnosticsParser::callbackDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_diagnostics = nh_.advertise<mrs_msgs::MavrosDiagnostics>("diagnostics_out", 1);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                     = new mrs_lib::Profiler(nh_, "MavrosInterface");
  routine_diagnostics_callback = profiler->registerRoutine("callbackDiagnostics");

  // | ----------------------- finish init ---------------------- |

  is_initialized = true;

  ROS_INFO("[DiagnosticsParser]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackDiagnostics()
void DiagnosticsParser::callbackDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {

  if (!is_initialized)
    return;

  routine_diagnostics_callback->start();

  mrs_msgs::MavrosDiagnostics diag;
  diag.header.stamp = ros::Time::now();

  for (size_t i = 0; i < msg->status.size(); i++) {

    // GPS
    if (msg->status[i].name.find("GPS") != std::string::npos) {


      for (size_t j = 0; j < msg->status[i].values.size(); j++) {


        // Satellites visible
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Satellites visible") == 0) {
          diag.gps.satellites_visible = std::stoi(msg->status[i].values[j].value);
        }

        // Fix type
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Fix type") == 0) {
          diag.gps.fix_type = std::stoi(msg->status[i].values[j].value);
        }

        // EPH
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPH (m)") == 0) {
          diag.gps.eph = std::stof(msg->status[i].values[j].value);
        }

        // EPV
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "EPV (m)") == 0) {
          diag.gps.epv = std::stof(msg->status[i].values[j].value);
        }
      }
    }

    // System
    if (msg->status[i].name.find("System") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // CPU Load
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "CPU Load (%)") == 0) {
          diag.system.cpu_load = std::stof(msg->status[i].values[j].value);
        }

        // Errors comm
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Errors comm") == 0) {
          diag.system.errors_comm = std::stoi(msg->status[i].values[j].value);
        }
      }
    }

    // Heartbeat
    if (msg->status[i].name.find("Heartbeat") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Mode
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Mode") == 0) {
          diag.heartbeat.mode = msg->status[i].values[j].value;
        }
      }
    }

    // Battery
    if (msg->status[i].name.find("Battery") != std::string::npos) {
      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Voltage
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Voltage") == 0) {
          diag.battery.voltage = std::stof(msg->status[i].values[j].value);
        }

        // Current
        if (std::strcmp((msg->status[i].values[j].key).c_str(), "Current") == 0) {
          diag.battery.current = std::stof(msg->status[i].values[j].value);
        }
      }
    }
  }

  try {
    publisher_diagnostics.publish(diag);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_diagnostics.getTopic().c_str());
  }

  routine_diagnostics_callback->end();
}
//}

}  // namespace mrs_mavros_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mavros_interface::DiagnosticsParser, nodelet::Nodelet)
