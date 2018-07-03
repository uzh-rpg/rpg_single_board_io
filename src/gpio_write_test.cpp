#include <ros/ros.h>

#include "rpg_single_board_io/gpio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_write_test");
  ros::NodeHandle nh;

  const unsigned int num_gpio = 25; // Example GPIO for Odroid XU4
  // See: https://wiki.odroid.com/odroid-xu4/hardware/expansion_connectors

  //const unsigned int num_gpio = 200; // Example GPIO for Odroid U3
  // See: http://www.hardkernel.com/main/products/prdt_info.php?g_code=G138745696275&tab_idx=2

  //const unsigned int num_gpio = 3; // Example GPIO for Up Board
  // See: https://wiki.up-community.org/Pinout

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
