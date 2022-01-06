// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Authors: Subhas Das, Denis Stogl
 */

#include "pilsbot_sensor_broadcaster/pilsbot_sensor_broadcaster.hpp"
#include "rclcpp/logging.hpp"

#include <memory>
#include <string>

namespace pilsbot_sensor_broadcaster
{
PilsbotSensorBroadcaster::PilsbotSensorBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::return_type PilsbotSensorBroadcaster::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    auto_declare<std::string>("head_mcu_publish_target", "/head_mcu");
    auto_declare<std::string>("hoverboard_api_publish_target", "/hoverboard_api");
    auto_declare<std::string>("steering_axle_name", std::string());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn PilsbotSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hmcu_publish_target_ = node_->get_parameter("head_mcu_publish_target").as_string();
  hbapi_publish_target_ = node_->get_parameter("hoverboard_api_publish_target").as_string();
  steering_axle_name_ = node_->get_parameter("steering_axle_name").as_string();

  bool publishing_hmcu = true;
  bool publishing_hbapi = true;

  if (hmcu_publish_target_.empty())
  {
    RCLCPP_WARN(node_->get_logger(),
      "head_mcu publish target is empty, not publishing.");
    publishing_hmcu = false;
  }
  else if(steering_axle_name_.empty()){
    RCLCPP_ERROR(node_->get_logger(),
        "steering_axle_name is empty, can not register to nothing");
    return CallbackReturn::ERROR;
  }

  if (hbapi_publish_target_.empty())
  {
    RCLCPP_WARN(node_->get_logger(),
      "hoverboard_api publish target is empty, not publishing.");
    publishing_hbapi = false;
  }

  if (!publishing_hmcu && !publishing_hbapi)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "both sensor publish targets are unset, why did you even start this?");
    return CallbackReturn::ERROR;
  }

  steering_axle_name_ = node_->get_parameter("steering_axle_name").as_string();

  // TODO: Should this go in on_activate?
  if(publishing_hmcu) {
      RCLCPP_INFO(node_->get_logger(), "publishing " + hmcu_publish_target_);
      head_mcu_publisher_ = node_->create_publisher<SteeringAxleSensorsStamped>(
          hmcu_publish_target_, rclcpp::SystemDefaultsQoS());
  }
  if(publishing_hbapi) {
      RCLCPP_INFO(node_->get_logger(), "publishing " + hbapi_publish_target_);
      hoverboard_api_publisher_ = node_->create_publisher<HoverboardAPISensors>(
          hbapi_publish_target_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_DEBUG(node_->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PilsbotSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
PilsbotSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if(!hmcu_publish_target_.empty()) {
    state_interfaces_config.names.push_back(steering_axle_name_ + "/position");
    state_interfaces_config.names.push_back("head_mcu/steering_angle_raw");
    state_interfaces_config.names.push_back("head_mcu/endstop_l");
    state_interfaces_config.names.push_back("head_mcu/endstop_r");
  }

  if(!hbapi_publish_target_.empty()) {
    state_interfaces_config.names.push_back("hoverboard_api/voltage");
    state_interfaces_config.names.push_back("hoverboard_api/avg_amperage_motor.0");
    state_interfaces_config.names.push_back("hoverboard_api/avg_amperage_motor.1");
    state_interfaces_config.names.push_back("hoverboard_api/tx_bufferlevel");
  }

  return state_interfaces_config;
}

CallbackReturn PilsbotSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if(!hmcu_publish_target_.empty() &&
      configure_hoverboard_api_sensors(registered_hoverboard_sensors_) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  if(!hbapi_publish_target_.empty() &&
      configure_hoverboard_api_sensors(registered_hoverboard_sensors_) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // steering handle
  const auto steering_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [name=steering_axle_name_](const auto & interface) {
      return interface.get_name() == name &&
             interface.get_interface_name() == "position";
    });

  if (steering_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(node_->get_logger(),
        "Unable to obtain joint state handle for " + steering_axle_name_);
    return CallbackReturn::ERROR;
  }
  registered_steering_axle_handle_.emplace_back(std::ref(*steering_handle));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PilsbotSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PilsbotSensorBroadcaster::update()
{
  if(hoverboard_api_publisher_) {
    HoverboardAPISensors msg;
    msg.voltage =
        registered_hoverboard_sensors_[0].voltage.get().get_value();
    msg.avg_amperate_motor_0 =
        registered_hoverboard_sensors_[0].avg_amperage_motor_0.get().get_value();
    msg.avg_amperate_motor_1 =
        registered_hoverboard_sensors_[0].avg_amperage_motor_1.get().get_value();;
    msg.tx_bufferlevel =
        registered_hoverboard_sensors_[0].tx_bufferlevel.get().get_value();
    hoverboard_api_publisher_->publish(msg);
  }
  if(head_mcu_publisher_){
    SteeringAxleSensorsStamped msg;
    msg.stamp = node_->get_clock()->now();
    msg.sensors.steering_angle_normalized =
        registered_steering_axle_handle_[0].get().get_value();
    msg.sensors.endstop_l =
        registered_head_mcu_sensors_[0].endstop_l.get().get_value();
    msg.sensors.endstop_r =
        registered_head_mcu_sensors_[0].endstop_r.get().get_value();
    msg.sensors.steering_angle_raw =
        registered_head_mcu_sensors_[0].steering_angle_raw.get().get_value();
    head_mcu_publisher_->publish(msg);
  }

  return controller_interface::return_type::OK;
}


CallbackReturn PilsbotSensorBroadcaster::configure_hoverboard_api_sensors(
    std::vector<HoverboardApiStates>& handles) {
  const auto voltage = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "hoverboard_api" &&
             interface.get_interface_name() == "voltage";
    });

  const auto avg_amperage_motor_0 = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "hoverboard_api" &&
             interface.get_interface_name() == "avg_amperage_motor.0";
    });

  const auto avg_amperage_motor_1 = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "hoverboard_api" &&
             interface.get_interface_name() == "avg_amperage_motor.1";
    });

  const auto tx_bufferlevel = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "hoverboard_api" &&
             interface.get_interface_name() == "tx_bufferlevel";
    });

  if (voltage == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for voltage");
    return CallbackReturn::ERROR;
  }
  if (avg_amperage_motor_0 == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for avg_amperage_motor_0");
    return CallbackReturn::ERROR;
  }
  if (avg_amperage_motor_1 == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for avg_amperage_motor_1");
    return CallbackReturn::ERROR;
  }
  if (tx_bufferlevel == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for tx_bufferlevel");
    return CallbackReturn::ERROR;
  }

  handles.emplace_back( HoverboardApiStates {
    .voltage = std::ref(*voltage),
    .avg_amperage_motor_0 = std::ref(*avg_amperage_motor_0),
    .avg_amperage_motor_1 = std::ref(*avg_amperage_motor_1),
    .tx_bufferlevel = std::ref(*tx_bufferlevel),
  });
  return CallbackReturn::SUCCESS;
}

CallbackReturn PilsbotSensorBroadcaster::configure_head_mcu_sensors(
    std::vector<HeadMCUStates>& handles) {

  const auto steering_angle_raw = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "head_mcu" &&
             interface.get_interface_name() == "steering_angle_raw";
    });

  const auto endstop_l = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "head_mcu" &&
             interface.get_interface_name() == "endstop_l";
    });

  const auto endstop_r = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [](const auto & interface) {
      return interface.get_name() == "head_mcu" &&
             interface.get_interface_name() == "endstop_r";
    });

  if (steering_angle_raw == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for raw steering angle");
    return CallbackReturn::ERROR;
  }
  if (endstop_l == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for endstop_l");
    return CallbackReturn::ERROR;
  }
  if (endstop_r == state_interfaces_.cend()){
    RCLCPP_ERROR(node_->get_logger(), "Unable to obtain sensor state handle for endstop_r");
    return CallbackReturn::ERROR;
  }

  handles.emplace_back( HeadMCUStates {
    .steering_angle_raw = std::ref(*steering_angle_raw),
    .endstop_l = std::ref(*endstop_l),
    .endstop_r = std::ref(*endstop_r),
  });

  return CallbackReturn::SUCCESS;
}


}  // namespace force_torque_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pilsbot_sensor_broadcaster::PilsbotSensorBroadcaster,
  controller_interface::ControllerInterface)
