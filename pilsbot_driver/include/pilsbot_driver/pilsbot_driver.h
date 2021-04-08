#pragma once

#include <chrono>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include <rclcpp/rclcpp.hpp>

// #include <diagnostic_msgs/DiagnosticStatus.h>
// #include <diagnostic_msgs/DiagnosticArray.h>
// #include <diagnostic_msgs/KeyValue.h>

// constexpr
constexpr int max_length = 1024;

class HoverboardAPI;

namespace pilsbot_driver
{

class PilsbotDriver : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  //PilsbotDriver(double wheel_radius, std::string &port);
  ~PilsbotDriver();
  RCLCPP_SHARED_PTR_DEFINITIONS(PilsbotDriver)

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void update_diagnostics();
  void tick();

private:
  // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
  // double pos[2];
  // double vel[2];
  // double eff[2];
  // double cmd[2];

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  rclcpp::Clock clock;

  double wheel_radius;
  rclcpp::Time last_read;
  HoverboardAPI *api;

  std::string port;

  // ros::NodeHandle _nh;
  // ros::NodeHandle _nh_priv;

  // diagnostic_msgs::DiagnosticStatus current_status;
  // realtime_tools::RealtimePublisherSharedPtr<diagnostic_msgs::DiagnosticStatus> diagnostics_pub;
};

}
