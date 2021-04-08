#include "HoverboardAPI.h"
#include <pilsbot_driver/pilsbot_driver.h>

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

hardware_interface::return_type PilsbotDriver::configure(const hardware_interface::HardwareInfo &info)
{
  this->wheel_radius = 0.0825; // TODO: Get as param
  this->port = "/dev/ttyS0";

  clock = rclcpp::Clock();

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  else
  {
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type PilsbotDriver::start()
{
  if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"), "Cannot open serial port to pilsbot");
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

  for (uint i = 0; i < this->hw_states_.size(); i++) 
  {
    if (std::isnan(this->hw_states_[i])) 
    {
        this->hw_states_[i] = 0;
        this->hw_commands_[i] = 0;
    }
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
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"), "Reading from serial %s failed: %d", port.c_str(), r);
      return hardware_interface::return_type::ERROR;
    }
  }

  if ((clock.now() - last_read) > rclcpp::Duration(1, 0))
  {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"), "Timeout reading from serial %s failed", port.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // Convert m/s to rad/s
  double sens_speed0 = api->getSpeed0_mms();
  double sens_speed1 = api->getSpeed1_mms();

  // Basic sanity check, speed should be less than 10 m/s
  // Sometimes, it seems during EMI peaks, we're getting ridiculous values here
  // Don't know what to do with it, just ignoring for now
  if (fabs(sens_speed0) < 10000 && fabs(sens_speed1) < 10000)
  {
    hw_states_[0] = (sens_speed0 / 1000.0) / wheel_radius;
    hw_states_[1] = (sens_speed1 / 1000.0) / wheel_radius;
    hw_states_[2] = (api->getPosition0_mm() / 1000.0) / wheel_radius;
    hw_states_[3] = (api->getPosition1_mm() / 1000.0) / wheel_radius;
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
  double left_speed = hw_commands_[0] * wheel_radius;
  double right_speed = hw_commands_[1] * wheel_radius;

  // TODO: Cap according to dynamic_reconfigure!!
  const int max_power = 100;
  const int min_speed = 40;
  api->sendSpeedData(left_speed, right_speed, max_power, min_speed);

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
