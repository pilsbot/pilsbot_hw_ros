// Humble's resource manager calls read() and write() on inactive hardware
// too (Foxy only did after start()). Without an open device the driver must
// then do nothing and report OK; an ERROR drops the hardware back to
// 'unconfigured', so it can never be activated once the devices appear.

#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

// The driver header relies on these being included first (as in pilsbot_driver.cpp).
#include "HoverboardAPI.h"
#include "../src/head_mcu/include/data_frame.hpp"
#include "pilsbot_driver/pilsbot_driver.h"

namespace
{
hardware_interface::HardwareInfo pilsbot_hardware_on(
  const std::string & hoverboard, const std::string & head_mcu)
{
  hardware_interface::HardwareInfo info;
  info.name = "PilsbotSystem";
  info.type = "system";
  info.hardware_class_type = "pilsbot_driver/PilsbotDriver";
  info.hardware_parameters = {
    {"hoverboard_tty_device", hoverboard},
    {"head_mcu_tty_device", head_mcu},
    {"wheel_radius", "0.125"},
    {"steering_calibration", "50702:-1.56601,32358:0.0,14017:1.56601"},
  };
  for (const auto * name : {"front_wheel_left_joint", "front_wheel_right_joint"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    hardware_interface::InterfaceInfo velocity;
    velocity.name = "velocity";
    joint.command_interfaces.push_back(velocity);
    joint.state_interfaces.push_back(velocity);
    hardware_interface::InterfaceInfo position;
    position.name = "position";
    joint.state_interfaces.push_back(position);
    info.joints.push_back(joint);
  }
  return info;
}

hardware_interface::HardwareInfo pilsbot_hardware_without_devices()
{
  return pilsbot_hardware_on("/dev/does-not-exist-hoverboard", "/dev/does-not-exist-head-mcu");
}

// A pseudo-terminal stands in for a serial device that opens but never answers.
struct SilentDevice
{
  SilentDevice() {openpty(&controller, &device, name, nullptr, nullptr);}
  ~SilentDevice() {close(device); close(controller);}
  int controller{-1};
  int device{-1};
  char name[64]{};
};

// True if the driver sent "power button pressed" (output bit 2 on) to the Head MCU within timeout_ms.
bool head_mcu_received_power_press(SilentDevice & head_mcu, int timeout_ms)
{
  head_mcu::Command cmd;
  size_t have = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct pollfd pfd{head_mcu.controller, POLLIN, 0};
    if (::poll(&pfd, 1, 50) <= 0) {
      continue;
    }
    const auto n = ::read(head_mcu.controller, reinterpret_cast<char *>(&cmd) + have, sizeof(cmd) - have);
    if (n <= 0) {
      return false;
    }
    have += static_cast<size_t>(n);
    if (have == sizeof(cmd)) {
      if (cmd.type == head_mcu::Command::setOutputFrame && cmd.frame.digital0_8.as_bit.bit2) {
        return true;
      }
      have = 0;
    }
  }
  return false;
}

class InactiveHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(driver_.on_init(pilsbot_hardware_without_devices()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  pilsbot_driver::PilsbotDriver driver_;
  const rclcpp::Time now_{1, 0};
  const rclcpp::Duration period_{0, 20000000};
};
}  // namespace

TEST_F(InactiveHardware, read_without_devices_is_ok)
{
  EXPECT_EQ(driver_.read(now_, period_), hardware_interface::return_type::OK);
}

TEST_F(InactiveHardware, write_without_devices_is_ok)
{
  EXPECT_EQ(driver_.write(now_, period_), hardware_interface::return_type::OK);
}

TEST_F(InactiveHardware, failed_activation_leaves_it_inactive_and_quiet)
{
  // Activation fails (no devices); read() and write() must stay harmless so
  // the activator can retry later.
  EXPECT_EQ(driver_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(driver_.read(now_, period_), hardware_interface::return_type::OK);
  EXPECT_EQ(driver_.write(now_, period_), hardware_interface::return_type::OK);
}

TEST(SilentHardware, hoverboard_timeout_presses_the_power_button_and_stays_ok)
{
  // Foxy behaviour: when the hoverboard is silent for over 1 s (e.g. switched
  // off), the driver presses its power button through the Head MCU and keeps
  // going. Humble would stop the hardware on an ERROR before the press runs,
  // so a timeout must report OK.
  SilentDevice hoverboard;
  SilentDevice head_mcu;
  pilsbot_driver::PilsbotDriver driver;
  const rclcpp::Time now{1, 0};
  const rclcpp::Duration period{0, 20000000};
  ASSERT_EQ(driver.on_init(pilsbot_hardware_on(hoverboard.name, head_mcu.name)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  EXPECT_EQ(driver.read(now, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(head_mcu_received_power_press(head_mcu, 2000));

  EXPECT_EQ(driver.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(SilentHardware, recovers_after_a_read_error)
{
  // Humble handles a read() error by calling on_error and leaving the
  // hardware unconfigured, without on_deactivate. The driver must close its
  // ports and stop the Head MCU thread there, or the next activation starts a
  // second thread and the controller manager dies in std::terminate.
  SilentDevice hoverboard;
  SilentDevice head_mcu;
  pilsbot_driver::PilsbotDriver driver;
  const rclcpp::Time now{1, 0};
  const rclcpp::Duration period{0, 20000000};
  ASSERT_EQ(driver.on_init(pilsbot_hardware_on(hoverboard.name, head_mcu.name)),
    hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  // A read() or write() ERROR makes Humble call on_error (not on_deactivate).
  EXPECT_EQ(driver.on_error(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  // Unconfigured again: harmless, and activation works a second time.
  EXPECT_EQ(driver.read(now, period), hardware_interface::return_type::OK);
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(driver.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}
