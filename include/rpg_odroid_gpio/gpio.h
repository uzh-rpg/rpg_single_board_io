/*
 * gpio.h
 *
 *  Created on: 6 Aug 2015
 *  Author: Raphael Meyer
 */

#ifndef GPIO_H_
#define GPIO_H_

/****************************************************************
* Constants
****************************************************************/
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */

#define MAX_BUF 64

#define T_CAPTURE_MARGIN 300

#define GPIO_OUT 1
#define GPIO_IN 0

namespace rpg_odroid_io
{
class GPIO
{
public:
  GPIO();
  GPIO(int gpio, int dir);
  ~GPIO();

  int gpioSetup(int gpio, int dir);
  bool gpioIsOpen();

  int gpioSetValue(unsigned int value);
  int gpioGetValue(unsigned int *value);

  int gpioSetEdge(char *edge);
  int gpioGetFd();
  int gpioClose();

private:
  int gpioExport_(unsigned int gpio);
  int gpioUnexport_(unsigned int gpio);
  int gpioSetDir_(unsigned int gpio, int dir);

  int fd_value_;
  int num_gpio_;
  int direction_;

};  // END class GPIO

};  // namespace rpg_odroid_io

#endif
