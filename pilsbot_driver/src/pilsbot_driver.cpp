#include "HoverboardAPI.h"
#include <pilsbot_driver/pilsbot_driver.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int port_fd = -1;

int serialWrite(unsigned char* data, int len)
{
  return (int)write(port_fd, data, len);
}

namespace pilsbot_driver
{

// PilsbotDriver::PilsbotDriver(double wheel_radius, std::string &port) : hardware_interface::BaseInterface<hardware_interface::SystemInterface>()
// {
//   this->wheel_radius = wheel_radius;
//   this->port = port;

//   // _nh_priv.param<double>("wheel_radius", wheel_radius, 0.0825);
//   // _nh_priv.param<std::string>("port", port, "/dev/ttyS0");

//   api = new HoverboardAPI(serialWrite);

//   // current_status.level = 0;
//   // current_status.name = "Pilsbot Driver";
//   // current_status.hardware_id = "pilsbot_serial1";

//   // diagnostic_msgs::KeyValue battery_level;
//   // battery_level.key = "Battery Level";
//   // battery_level.value = "";
//   // current_status.values.push_back(battery_level);

//   // diagnostics_pub.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticStatus>(_nh, "status", 5));
// }

PilsbotDriver::~PilsbotDriver()
{
  delete api;
}

std::vector<hardware_interface::StateInterface>
PilsbotDriver::export_state_interfaces()
{
  // at this point, we know there are just two joints (see on_configure)

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for(auto& joint : info_.joints) {
    if(joint.name.find("left") != std::string::npos) {
      RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
           "interpreting joint %s as left wheel", joint.name);

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &wheels_[0].curr_position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[0].curr_speed));
    }
    else if (joint.name.find("right") != std::string::npos) {
      RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
           "interpreting joint %s as right wheel", joint.name);

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &wheels_[1].curr_position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[1].curr_speed));
    }
    else if (joint.name == "steering_axle_joint") { // todo: Make this configurable
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &current_steering_angle_));
    }
  }

  // export sensor state interface
  for (auto& sens : info_.sensors)
  {
    if (sens.name == "hoverboard_api" &&
        sens.state_interfaces.size() <= 4) {
      for(auto& interface : sens.state_interfaces) {
        if(interface.name == "voltage") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &sensors_.voltage));
        }
        else if (interface.name == "avg_amperage_motor.0") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &sensors_.avg_amperage_motor0));
        }
        else if (interface.name == "avg_amperage_motor.1") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &sensors_.avg_amperage_motor1));
        }
        else if (interface.name == "tx_bufferlevel") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &sensors_.txBufferLevel));
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
               "Not offering interface %s in sensor %s",
               interface.name, sens.name);
        }
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
           "Not offering sensor %s", sens.name);
    }
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
PilsbotDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels_[i].commanded_turning_rate));
  }

  return command_interfaces;
}


hardware_interface::return_type PilsbotDriver::configure(const hardware_interface::HardwareInfo &info)
{
  clock = rclcpp::Clock();

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  else
  {
    if(info_.joints.size() != 3) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Pilsbot driver currently connects only to one board with two wheels!");
      return hardware_interface::return_type::ERROR;
    }

    if(info_.sensors.size() > 1) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Pilsbot driver currently only has 2 Sensors available (requested: %d)",
                 info_.sensors.size() );
      return hardware_interface::return_type::ERROR;
    }

    wheels_.resize(2, WheelStatus());
    sensors_ = Sensors();
    current_steering_angle_ = 0;

    //todo: something with info_.hardware_parameters
    // is this the only param possibility? Or also yaml-shit?
    params_ = Params();

    return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type PilsbotDriver::start()
{
  unsigned retries = 0;
  bool still_unsuccessful = true;
  while(retries < params_.serial_connect_retries) {
    if ((port_fd = open(params_.device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
                   "Cannot open serial device %s to pilsbot, retrying...",
                   params_.device);
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      still_unsuccessful = false;
    }
  }

  if(still_unsuccessful) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Could not open serial device %s to pilsbot controller board",
                 params_.device);
    return hardware_interface::return_type::ERROR;
  }

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see
  // http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  struct termios options;
  tcgetattr(port_fd, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port_fd, TCIFLUSH);
  tcsetattr(port_fd, TCSANOW, &options);

  // Doesn't work for some reason. Need to investigate
  // api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 20, PROTOCOL_SOM_NOACK);
  // api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 20, PROTOCOL_SOM_NOACK);

  for (auto& wheel : wheels_)
  {
    wheel = WheelStatus();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::stop()
{
  if (port_fd != -1)
    close(port_fd);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::read()
{
  api->requestRead(HoverboardAPI::Codes::sensHall);
  api->requestRead(HoverboardAPI::Codes::sensElectrical);

  if (port_fd != -1)
  {
    unsigned char c;
    int i = 0, r = 0;

    while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < max_length)
    {
      api->protocolPush(c);
    }

    if (i > 0)
    {
      last_read = clock.now();
    }

    if (r < 0 && errno != EAGAIN)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"), "Reading from serial %s failed: %d",
          params_.device.c_str(), r);
      return hardware_interface::return_type::ERROR;
    }
  }

  if ((clock.now() - last_read) > rclcpp::Duration(1, 0))
  {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"), "Timeout reading from serial %s failed",
        params_.device.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // sensory
  current_steering_angle_ = 0;  // TODO: Get this from a separate topic
  sensors_.voltage = api->getBatteryVoltage();
  sensors_.avg_amperage_motor0 = api->getMotorAmpsAvg(0);
  sensors_.avg_amperage_motor1 = api->getMotorAmpsAvg(1);
  sensors_.txBufferLevel = api->getTxBufferLevel();

  // Convert m/s to rad/s
  double sens_speed0 = api->getSpeed0_mms();
  double sens_speed1 = api->getSpeed1_mms();

  // Basic sanity check, speed should be less than 10 m/s
  // Sometimes, it seems during EMI peaks, we're getting ridiculous values here
  // Don't know what to do with it, just ignoring for now
  if (fabs(sens_speed0) < 10000 && fabs(sens_speed1) < 10000)
  {
    wheels_[0].curr_speed = (sens_speed0 / 1000.0) / params_.wheel_radius;
    wheels_[0].curr_position = (api->getPosition0_mm() / 1000.0) / params_.wheel_radius;
    wheels_[1].curr_speed = (sens_speed1 / 1000.0) / params_.wheel_radius;
    wheels_[1].curr_position = (api->getPosition1_mm() / 1000.0) / params_.wheel_radius;
    return hardware_interface::return_type::OK;
  }
  else
    return hardware_interface::return_type::ERROR;

}

hardware_interface::return_type PilsbotDriver::write()
{
  if (port_fd == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"), "Attempt to write on closed serial");
    return hardware_interface::return_type::ERROR;
  }

  // Convert rad/s to m/s
  double left_speed = wheels_[0].commanded_turning_rate * params_.wheel_radius;
  double right_speed = wheels_[1].commanded_turning_rate * params_.wheel_radius;

  api->sendSpeedData(left_speed, right_speed, params_.max_power, params_.min_speed);

  tick();

  return hardware_interface::return_type::OK;
}

// TODO: Update for ROS 2
// void PilsbotDriver::update_diagnostics()
// {
//   current_status.values[0].value = std::to_string(api->getBatteryVoltage());

//   if (diagnostics_pub->trylock())
//   {
//     diagnostics_pub->msg_= current_status;
//     diagnostics_pub->unlockAndPublish();
//   }
// }

void PilsbotDriver::tick()
{
  api->protocolTick();
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pilsbot_driver::PilsbotDriver,
  hardware_interface::SystemInterface
)
