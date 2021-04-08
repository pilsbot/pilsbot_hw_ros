#include <pilsbot_driver/pilsbot_driver.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  PilsbotDriver pilsbot();
  controller_manager::ControllerManager cm(&pilsbot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  rclcpp::Time prev_time = rclcpp::Time::now();
  rclcpp::Rate rate(50.0);

  while (ros::ok())
  {
    const rclcpp::Time time = rclcpp::Clock::now();
    const rclcpp::Duration period = time - prev_time;

    pilsbot.read();
    pilsbot.update_diagnostics();
    cm.update(time, period);
    pilsbot.write();
    pilsbot.tick();

    rate.sleep();
  }

  return 0;
}
