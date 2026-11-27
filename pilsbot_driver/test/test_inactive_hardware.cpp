// Humble's resource manager calls read() and write() on inactive hardware
// too (Foxy only did after start()). Without an open device the driver must
// then do nothing and report OK; an ERROR drops the hardware back to
// 'unconfigured', so it can never be activated once the devices appear.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fake_devices.hpp"

namespace
{
using fake_devices::pilsbot_hardware_on;
using fake_devices::PowerButtonWatch;
using fake_devices::SilentDevice;

hardware_interface::HardwareInfo pilsbot_hardware_without_devices()
{
  return pilsbot_hardware_on("/dev/does-not-exist-hoverboard", "/dev/does-not-exist-head-mcu");
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

  PowerButtonWatch power_button(head_mcu);
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  EXPECT_EQ(driver.read(now, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(power_button.wait_for_presses(1, 2000));

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
