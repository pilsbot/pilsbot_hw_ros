#include "HoverboardAPI.h"
#include "./head_mcu/include/data_frame.hpp"
#include <pilsbot_driver/pilsbot_driver.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int serialWrite(unsigned char* data, int len)
{
  return (int)write(hoverboard_fd, data, len);
}

namespace pilsbot_driver
{
using std::placeholders::_1;

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
    else if (joint.name == "steering_axle_joint") { // todo: Make this name configurable
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &axle_sensors_.steering_angle_normalized));
    }
  }

  // export sensor state interface
  for (auto& sens : info_.sensors)
  {
    if (sens.name == "hoverboard_api") {
      for(auto& interface : sens.state_interfaces) {
        if(interface.name == "voltage") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.voltage));
        }
        else if (interface.name == "avg_amperage_motor.0") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.avg_amperage_motor0));
        }
        else if (interface.name == "avg_amperage_motor.1") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.avg_amperage_motor1));
        }
        else if (interface.name == "tx_bufferlevel") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.txBufferLevel));
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
               "Not offering interface %s in sensor %s",
               interface.name, sens.name);
        }
      }
    } else if (sens.name == "head_mcu") {
      for(auto& interface : sens.state_interfaces) {
        if(interface.name == "steering_angle_raw") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.steering_angle_raw));
        }
        else if (interface.name == "endstop_l") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.endstop_l));
        }
        else if (interface.name == "endstop_r") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.endstop_r));
        }
        else {
          RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
               "Not offering interface %s in sensor %s",
               interface.name, sens.name);
        }
      }
    }else {
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

    wheels_.resize(2, WheelStatus());
    hoverboard_sensors_ = HoverboardSensors();

    //todo: something with info_.hardware_parameters
    // is this the only param possibility? Or also yaml-shit?
    params_ = Params();

    if(!interpolator_.deserialize(params_.head_mcu.calibration_val)) {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
             "calibration values could not be loaded!");
    }

    return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type PilsbotDriver::start()
{
  unsigned retries = 0;
  bool still_he_unsuccessful = true;
  bool still_ho_unsuccessful = true;
  while(retries < params_.serial_connect_retries) {
    if ((hoverboard_fd = open(params_.hoverboard.tty_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
                   "Cannot open serial device %s to pilsbot, retrying...",
                   params_.hoverboard.tty_device);
    } else {
      still_ho_unsuccessful = false;
    }
    if ((head_mcu_fd = ::open(params_.head_mcu.tty_device.c_str(), O_RDWR | O_NOCTTY)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
                   "Cannot open serial device %s to head_mcu!",
                   params_.head_mcu.tty_device.c_str());
    } else {
      still_he_unsuccessful = false;
    }
    if(still_he_unsuccessful && still_ho_unsuccessful)
      rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if(still_ho_unsuccessful) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Could not open serial device %s to pilsbot controller board",
                 params_.hoverboard.tty_device);
    if (head_mcu_fd != -1)
      ::close(head_mcu_fd);
    return hardware_interface::return_type::ERROR;
  }
  if(still_he_unsuccessful) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Could not open serial device %s to head mcu board",
                 params_.head_mcu.tty_device);
    if (hoverboard_fd != -1)
      ::close(hoverboard_fd);
    return hardware_interface::return_type::ERROR;
  }

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see
  // http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  struct termios options;
  tcgetattr(hoverboard_fd, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(hoverboard_fd, TCIFLUSH);
  tcsetattr(hoverboard_fd, TCSANOW, &options);

  // Doesn't work for some reason. Need to investigate
  // api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 20, PROTOCOL_SOM_NOACK);
  // api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 20, PROTOCOL_SOM_NOACK);


  // configuring head_mcu UART
  tcflush(head_mcu_fd, TCIOFLUSH); // flush previous bytes

  struct termios tio;
  if(tcgetattr(head_mcu_fd, &tio) < 0)
    perror("tcgetattr");

  tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  tio.c_oflag &= ~(ONLCR | OCRNL);
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  switch (params_.head_mcu.baudrate)
  {
  case 9600:   cfsetospeed(&tio, B9600);   break;
  case 19200:  cfsetospeed(&tio, B19200);  break;
  case 38400:  cfsetospeed(&tio, B38400);  break;
  case 115200: cfsetospeed(&tio, B115200); break;
  case 230400: cfsetospeed(&tio, B230400); break;
  case 460800: cfsetospeed(&tio, B460800); break;
  case 500000: cfsetospeed(&tio, B500000); break;
  default:
    RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
        "Baudrate of %d not supported, using 115200!", params_.head_mcu.baudrate);
    cfsetospeed(&tio, B115200);
    break;
  }
  cfsetispeed(&tio, cfgetospeed(&tio));

  if(tcsetattr(head_mcu_fd, TCSANOW, &tio) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
        "Could not set terminal attributes!");
    perror("tcsetattr");
  }

  // ok, go on little bird
  stop_ = false;
  reading_function_ = std::thread(&PilsbotDriver::read_from_head_mcu, this);

  for (auto& wheel : wheels_)
  {
    wheel = WheelStatus();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::stop()
{
  stop_ = true;
  if (hoverboard_fd != -1)
    close(hoverboard_fd);
  if (head_mcu_fd != -1)
    close(head_mcu_fd);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::read()
{
  api->requestRead(HoverboardAPI::Codes::sensHall);
  api->requestRead(HoverboardAPI::Codes::sensElectrical);

  if (hoverboard_fd != -1)
  {
    unsigned char c;
    int i = 0, r = 0;

    while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length)
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
          params_.hoverboard.tty_device.c_str(), r);
      return hardware_interface::return_type::ERROR;
    }
  }

  if ((clock.now() - last_read) > rclcpp::Duration(1, 0))
  {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"), "Timeout reading from serial %s failed",
        params_.hoverboard.tty_device.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // steering angle & co are updated in thread

  // sensory
  hoverboard_sensors_.voltage = api->getBatteryVoltage();
  hoverboard_sensors_.avg_amperage_motor0 = api->getMotorAmpsAvg(0);
  hoverboard_sensors_.avg_amperage_motor1 = api->getMotorAmpsAvg(1);
  hoverboard_sensors_.txBufferLevel = api->getTxBufferLevel();

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
  if (hoverboard_fd == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"), "Attempt to write on closed serial");
    return hardware_interface::return_type::ERROR;
  }

  // Convert rad/s to m/s
  double left_speed = wheels_[0].commanded_turning_rate * params_.wheel_radius;
  double right_speed = wheels_[1].commanded_turning_rate * params_.wheel_radius;

  api->sendSpeedData(left_speed, right_speed, params_.hoverboard.max_power, params_.hoverboard.min_speed);

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

void PilsbotDriver::read_from_head_mcu() {
  head_mcu::UpdatePeriodMs period_ms = params_.head_mcu.update_period_ms;
  if(::write(head_mcu_fd, &period_ms, sizeof(head_mcu::UpdatePeriodMs)) < 0){
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
        "could not set update period of %d ms", period_ms);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
      "Probably successfully set update period of %d ms", period_ms);

  RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
      "head_mcu serial connection thread started");
  while(!stop_) {
    head_mcu::Frame frame;
    int ret = ::read(head_mcu_fd, &frame, sizeof(head_mcu::Frame));
    if(ret == sizeof(head_mcu::Frame)) {
      axle_sensors_.steering_angle_raw = frame.analog0;
      axle_sensors_.endstop_l = frame.digital0_8.as_bit.bit0;
      axle_sensors_.endstop_r = frame.digital0_8.as_bit.bit1;
      axle_sensors_.steering_angle_normalized =
            interpolator_(frame.analog0);
    } else if(ret > 0) {
      RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
          "serial connection out of sync!");
      // todo: maybe reopen serial, this resets the controller
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
          "serial connection closed or something: %d", ret);
      return;
    }
  }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pilsbot_driver::PilsbotDriver,
  hardware_interface::SystemInterface
)
