#pragma once

#include "pid.h"
#include <chrono>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pilsbot_driver/linear_interpolation.hpp>

// #include <diagnostic_msgs/DiagnosticStatus.h>
// #include <diagnostic_msgs/DiagnosticArray.h>
// #include <diagnostic_msgs/KeyValue.h>

constexpr int max_length = 1024;
int hoverboard_fd = -1;

class HoverboardAPI;
using linear_interpolator::CalibrationListSerialized;

namespace pilsbot_driver
{
//TODO: Make this Directly a SystemInterface, probably perhaps maybe dunno
class PilsbotDriver : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
  typedef LinearInterpolator<unsigned, double> PotInterpolator;
public:
  ~PilsbotDriver();
  RCLCPP_SHARED_PTR_DEFINITIONS(PilsbotDriver)

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void tick();
  void read_from_head_mcu();

private:

  struct WheelStatus {
    double commanded_turning_rate = 0;
    double curr_position = 0;
    double curr_speed = 0;
  };

  struct HoverboardSensors {
      double voltage = 0;
      double avg_amperage_motor0 = 0;
      double avg_amperage_motor1 = 0;
      double txBufferLevel = 0;
  };

  struct SteeringAxleSensors {
      double steering_angle_raw;
      double endstop_l;     // stupid state interface only allows double
      double endstop_r;     // stupid state interface only allows double
      double steering_angle_normalized;
  };

  struct Params {
    double wheel_radius = 0.125;
    struct {
      std::string tty_device = "/dev/ttyHoverboard";
      PID::Settings pid = PID::Settings {
        .Kp = .28, .Ki = 1,  .Kd = .001,
        .dt = 1, .max = 600, .min = NAN, //min/max is PWM (0-2000)
        .max_dv = 200, .overshoot_integral_adaptation = .5
      };
    } hoverboard;
    struct {
      std::string tty_device = "/dev/ttyHeadMCU";
      unsigned baudrate = 115200;
      unsigned update_period_ms = 5;
      CalibrationListSerialized calibration_val = {
            // TODO: proper config loading
            50601, -1.56601,
            32432, 0.0,
            14142, 1.56601
      };
    } head_mcu;
    unsigned serial_connect_retries = 3;
                                // Minimum calculated speed to have the wheels moving.
  } params_;

  //int hoverboard_fd = -1; // declared globally because of ugly api
  int head_mcu_fd = -1;

  std::vector<WheelStatus> wheels_;
  PID wheel_controller_l;
  PID wheel_controller_r;
  HoverboardSensors hoverboard_sensors_;

  // head_mcu threading
  std::atomic<bool> stop_;
  PotInterpolator interpolator_;
  SteeringAxleSensors axle_sensors_;
  std::thread reading_function_;

  rclcpp::Clock clock;

  rclcpp::Time last_serial_read;   // last time read from serial
  rclcpp::Time last_write_tick;  // last time a 'write' tick happened
  HoverboardAPI *api;
};

}
