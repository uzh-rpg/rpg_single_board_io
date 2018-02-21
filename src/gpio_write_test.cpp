#include <ros/ros.h>

#include "rpg_single_board_io/gpio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_write_test");
  ros::NodeHandle nh;

  const unsigned int num_gpio = 25; // Example GPIO for Odroid XU4
  //const unsigned int num_gpio = 200; // Example GPIO for Odroid U3
  //const unsigned int num_gpio = 3; // Example GPIO for Up Board

  rpg_single_board_io::GPIO gpio(num_gpio,
                                 rpg_single_board_io::GpioDirection::Out);

  if (!gpio.gpioIsOpen())
  {
    ROS_ERROR("[%s] Error while opening GPIO, exiting",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }

  ros::Rate loop_rate(2.0);

  bool set_gpio_high = true;
  while (nh.ok())
  {
    if (set_gpio_high)
    {
      gpio.gpioSetValue(rpg_single_board_io::GpioValue::High);
      set_gpio_high = false;
    }
    else
    {
      gpio.gpioSetValue(rpg_single_board_io::GpioValue::Low);
      set_gpio_high = true;
    }
    loop_rate.sleep();
  }

  return 0;
}
