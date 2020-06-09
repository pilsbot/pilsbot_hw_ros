#pragma once

#include <controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

// constexpr

constexpr int max_length = 1024;

class HoverboardAPI;

class PilsbotDriver : public hardware_interface::RobotHW
{
public:
  PilsbotDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv);
  ~PilsbotDriver();

  void read();
  void write();
  void update_diagnostics();
  void tick();

private:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
  double pos[2];
  double vel[2];
  double eff[2];
  double cmd[2];

  double wheel_radius;
  ros::Time last_read;
  HoverboardAPI *api;

  std::string port;

  ros::NodeHandle _nh;
  ros::NodeHandle _nh_priv;

  diagnostic_msgs::DiagnosticStatus current_status;
  realtime_tools::RealtimePublisherSharedPtr<diagnostic_msgs::DiagnosticStatus> diagnostics_pub;
};
