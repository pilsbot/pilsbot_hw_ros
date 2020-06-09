#include <pilsbot_driver/pilsbot_driver.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pislbot_driver");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  PilsbotDriver pilsbot(nh, nh_priv);
  controller_manager::ControllerManager cm(&pilsbot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;

    pilsbot.read();
    cm.update(time, period);
    pilsbot.write();
    pilsbot.tick();

    rate.sleep();
  }

  return 0;
}
