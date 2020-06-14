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

PilsbotDriver::PilsbotDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
  _nh = nh;
  _nh_priv = _nh_priv;

  hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel", &pos[0], &vel[0], &eff[0]);
  hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel", &pos[1], &vel[1], &eff[1]);
  joint_state_interface.registerHandle(left_wheel_state_handle);
  joint_state_interface.registerHandle(right_wheel_state_handle);
  registerInterface(&joint_state_interface);

  hardware_interface::JointHandle left_wheel_vel_handle(joint_state_interface.getHandle("left_wheel"), &cmd[0]);
  hardware_interface::JointHandle right_wheel_vel_handle(joint_state_interface.getHandle("right_wheel"), &cmd[1]);
  velocity_joint_interface.registerHandle(left_wheel_vel_handle);
  velocity_joint_interface.registerHandle(right_wheel_vel_handle);
  registerInterface(&velocity_joint_interface);

  _nh_priv.param<double>("wheel_radius", wheel_radius, 0.0825);
  _nh_priv.param<std::string>("port", port, "/dev/ttyS0");

  if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
  {
    ROS_FATAL("Cannot open serial port to pilsbot");
    exit(-1);
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

  api = new HoverboardAPI(serialWrite);
  // Doesn't work for some reason.
  // api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 20, PROTOCOL_SOM_NOACK);
  // api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 20, PROTOCOL_SOM_NOACK);

  current_status.level = 0;
  current_status.name = "Pilsbot Driver";
  current_status.hardware_id = "pilsbot_serial1";

  diagnostic_msgs::KeyValue battery_level;
  battery_level.key = "Battery Level";
  battery_level.value = "";
  current_status.values.push_back(battery_level);

  diagnostics_pub.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticStatus>(_nh, "status", 5));
}

PilsbotDriver::~PilsbotDriver()
{
  if (port_fd != -1)
    close(port_fd);

  delete api;
}

void PilsbotDriver::read()
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
      last_read = ros::Time::now();
    }

    if (r < 0 && errno != EAGAIN)
    {
      ROS_ERROR("Reading from serial %s failed: %d", port.c_str(), r);
    }
  }

  if ((ros::Time::now() - last_read).toSec() > 1)
  {
    ROS_FATAL("Timeout reading from serial %s failed", port.c_str());
  }

  // Convert m/s to rad/s
  double sens_speed0 = api->getSpeed0_mms();
  double sens_speed1 = api->getSpeed1_mms();

  // Basic sanity check, speed should be less than 10 m/s
  // Sometimes, it seems during EMI peaks, we're getting ridiculous values here
  // Don't know what to do with it, just ignoring for now
  if (fabs(sens_speed0) < 10000 && fabs(sens_speed1) < 10000)
  {
    vel[0] = (sens_speed0 / 1000.0) / wheel_radius;
    vel[1] = (sens_speed1 / 1000.0) / wheel_radius;
    pos[0] = (api->getPosition0_mm() / 1000.0) / wheel_radius;
    pos[1] = (api->getPosition1_mm() / 1000.0) / wheel_radius;
  }

}

void PilsbotDriver::write()
{
  if (port_fd == -1)
  {
    ROS_ERROR("Attempt to write on closed serial");
    return;
  }

  // Convert rad/s to m/s
  double left_speed = cmd[0] * wheel_radius;
  double right_speed = cmd[1] * wheel_radius;

  // Cap according to dynamic_reconfigure
  const int max_power = 100;
  const int min_speed = 40;
  api->sendSpeedData(left_speed, right_speed, max_power, min_speed);
}

void PilsbotDriver::update_diagnostics()
{
  current_status.values[0].value = std::to_string(api->getBatteryVoltage());

  if (diagnostics_pub->trylock())
  {
    diagnostics_pub->msg_= current_status;
    diagnostics_pub->unlockAndPublish();
  }
}

void PilsbotDriver::tick()
{
  api->protocolTick();
}
