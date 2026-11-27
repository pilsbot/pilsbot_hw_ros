#pragma once

#include "pid.h"
#include <atomic>
#include <chrono>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

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
class PilsbotDriver : public hardware_interface::SystemInterface
{
  typedef LinearInterpolator<unsigned, double> PotInterpolator;
public:
  ~PilsbotDriver();
  RCLCPP_SHARED_PTR_DEFINITIONS(PilsbotDriver)

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void tick();


private:
  enum class WhichSerial
  {
    hoverboard,
    head_mcu
  };

  void read_from_head_mcu_continuously();

  bool set_head_mcu_pin(const bool pin);

  bool set_update_period_of_head_mcu(head_mcu::UpdatePeriodMs period_ms);

  bool try_setup_serial(const WhichSerial which);

  void try_toggle_hoverboard_power();

  enum class PowerButton
  {
    press_if_connected,  // clean shutdown: switch the hoverboard off if it talked
                         // since the activation and within the last 1 s
    leave                // error or failed activation: never touch the power button
  };

  // Closes the serial connections and joins the Head MCU thread. Safe to call twice.
  void shutdown_connections(PowerButton power_button);

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
    double wheel_radius = 0;  // from the robot profile
    struct {
      std::string tty_device;  // from the robot profile
      PID::Settings pid = PID::Settings {
        .Kp = .3, .Ki = 1.1,  .Kd = .001,
        .dt = 1, .max = 1000, .min = NAN, //min/max is PWM (0-2000)
        .max_dv = 200, .overshoot_integral_adaptation = .5
      };
    } hoverboard;
    struct {
      std::string tty_device;  // from the robot profile
      unsigned baudrate = 115200;
      unsigned update_period_ms = 5;
      CalibrationListSerialized calibration_val;  // from the robot profile
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
  std::atomic<bool> stop_{true};
  // Humble calls read()/write() on inactive hardware too; they do nothing until on_activate succeeded.
  std::atomic<bool> active_{false};
  // Set from read() on a hoverboard timeout; the Head MCU thread performs the
  // (blocking, ~1 s) power toggle so the control loop is never put to sleep.
  std::atomic<bool> power_toggle_requested_{false};
  // Set once the hoverboard sent bytes since on_activate. Without it a
  // shutdown within 1 s of activation would press power although the
  // hoverboard never talked (and may be off: the press would switch it on).
  std::atomic<bool> hoverboard_seen_{false};
  PotInterpolator interpolator_;
  SteeringAxleSensors axle_sensors_;
  std::thread reading_function_;

  rclcpp::Clock clock;

  rclcpp::Time last_serial_read;   // last time read from serial
  rclcpp::Time last_write_tick;  // last time a 'write' tick happened
  HoverboardAPI *api = nullptr;
};

}
