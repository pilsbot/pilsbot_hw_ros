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

constexpr int max_length = 1024;

class HoverboardAPI;

namespace pilsbot_driver
{

//TODO: Make this Directly a SystemInterface, probably
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

  struct WheelStatus {
    double commanded_turning_rate = 0;
    double curr_position = 0;
    double curr_speed = 0;
  };

  struct Sensors {
      double voltage = 0;
      double avg_amperage_motor0 = 0;
      double avg_amperage_motor1 = 0;
      double txBufferLevel = 0;
  };

  struct Params {
    double wheel_radius = 0.0825;
    std::string device = "/dev/ttyS0";
    unsigned serial_connect_retries = 30;
    unsigned max_power = 100;   //limit is around 1000, I think
    unsigned min_speed = 40;    // Somehow convoluted with wheel_radius.
                                // Minimum calculated speed to have the wheels moving.
  } params_;

  std::vector<WheelStatus> wheels_;
  double current_steering_angle_ = 0;
  Sensors sensors_;

  rclcpp::Clock clock;

  rclcpp::Time last_read;
  HoverboardAPI *api;

  // ros::NodeHandle _nh;
  // ros::NodeHandle _nh_priv;

  // diagnostic_msgs::DiagnosticStatus current_status;
  // realtime_tools::RealtimePublisherSharedPtr<diagnostic_msgs::DiagnosticStatus> diagnostics_pub;
};

}
