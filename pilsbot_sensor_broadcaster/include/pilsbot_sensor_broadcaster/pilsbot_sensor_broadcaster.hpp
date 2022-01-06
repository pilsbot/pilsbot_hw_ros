#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pilsbot_sensor_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <pilsbot_driver_msgs/msg/steering_axle_sensors_stamped.hpp>
#include <pilsbot_driver_msgs/msg/hoverboard_api_sensors.hpp>

namespace pilsbot_sensor_broadcaster
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace pilsbot_driver_msgs::msg;

class PilsbotSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  PilsbotSensorBroadcaster();

  controller_interface::return_type init(const std::string & controller_name) override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  PILSBOT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update() override;

protected:
  typedef std::reference_wrapper<const hardware_interface::LoanedStateInterface> StateIF;

  struct HoverboardApiStates
  {
    StateIF voltage;
    StateIF avg_amperage_motor_0;
    StateIF avg_amperage_motor_1;
    StateIF tx_bufferlevel;
  };

  struct HeadMCUStates
  {
    StateIF steering_angle_raw;
    StateIF endstop_l;
    StateIF endstop_r;
  };

  std::vector<HoverboardApiStates> registered_hoverboard_sensors_;
  std::vector<HeadMCUStates> registered_head_mcu_sensors_;
  std::vector<StateIF> registered_steering_axle_handle_;

  CallbackReturn configure_hoverboard_api_sensors(std::vector<HoverboardApiStates>& handles);
  CallbackReturn configure_head_mcu_sensors(std::vector<HeadMCUStates>& handles);

  std::string steering_axle_name_;
  std::string hmcu_publish_target_;
  std::string hbapi_publish_target_;

  std::shared_ptr<rclcpp::Publisher<SteeringAxleSensorsStamped>> head_mcu_publisher_ =
      nullptr;
  std::shared_ptr<rclcpp::Publisher<HoverboardAPISensors>> hoverboard_api_publisher_ =
      nullptr;
};
}
