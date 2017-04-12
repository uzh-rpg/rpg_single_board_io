/*
 * gpio.cpp
 *
 *  Created on: 6 Aug 2015
 *      Author: rpg
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <rpg_odroid_io/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace rpg_odroid_io
{
GPIO::GPIO() :
    fd_value_(-1), num_gpio_(-1), direction_(0)
{
}

GPIO::GPIO(int gpio, int dir) :
    fd_value_(-1), num_gpio_(-1), direction_(0)
{
  gpioSetup(gpio, dir);
}

/* Set new GPIO pin */
int GPIO::gpioSetup(int gpio, int dir)
{
  /* try to unexport */
  gpioUnexport_(gpio);

  num_gpio_ = -1;

  /* init new GPIO: */
  if (gpioExport_(gpio))
  {
    perror("gpio/init: export");
    return -1;
  }

  // After exporting the GPIO pin, udev rules (if used) need a moment to
  // apply. So we will retry writing the direction for 2 seconds before
  // giving up.
  const size_t max_attempts = 10000;
  const size_t attempt_timeout_s = 2; // Timeout in seconds
  for (size_t attempt = 0; attempt <= max_attempts; attempt++)
  {
    if (!gpioSetDir_(gpio, dir))
    {
      // Direction set succesfully
      break;
    }
    // Chill for a moment
    usleep(attempt_timeout_s * 1000000 / max_attempts);
    if (attempt == max_attempts)
    {
      perror("gpio/init: direction");
      return -1;
    }
  }

  num_gpio_ = gpio;
  direction_ = dir;

  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", num_gpio_);

  if (direction_ == GPIO_IN)
  {
    fd_value_ = open(buf, O_RDONLY);
  }
  else
  {
    fd_value_ = open(buf, O_WRONLY);
  }
  if (fd_value_ < 0)
  {
    perror("gpio/init: Value");
    return fd_value_;
  }

  return 0;
}

GPIO::~GPIO()
{
  gpioClose();
}

bool GPIO::gpioIsOpen()
{
  return fd_value_ > -1;
};

int GPIO::gpioExport_(unsigned int gpio)
{
  int fd, len;
  char buf[MAX_BUF];

  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0)
  {
    perror("gpio/export: open failed");
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  int res;
  res = write(fd, buf, len);

  printf("Export res: %i\n\r", res);

  if (res < 0)
  {
    perror("gpio/export: write failed");
    return -1;
  }

  printf("Export successful!\n\r");

  close(fd);

  return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int GPIO::gpioUnexport_(unsigned int gpio)
{
  int fd, len;
  char buf[MAX_BUF];

  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0)
  {
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  if (write(fd, buf, len) < 0)
  {
    perror("gpio/unexport");
    return -1;
  }

  printf("Unexport successful!\n\r");

  close(fd);

  return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int GPIO::gpioSetDir_(unsigned int gpio, int dir)
{
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);

  // printf("open: %s\n\r", buf);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
    return fd;
  }

  if (dir)
  {
    if (write(fd, "out", 4) < 0)
    {
      perror("gpio/set-dir");
      return -1;
    }
  }
  else
  {
    if (write(fd, "in", 3) < 0)
    {
      perror("gpio/set-dir");
      return -1;
    }
  }

  printf("Set dir successful!\n\r");

  close(fd);
  return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int GPIO::gpioSetValue(unsigned int value)
{
  if (fd_value_ < 0)
  {
    return fd_value_;
  }

  if (value)
  {
    if (write(fd_value_, "1", 2) < 0)
      return -1;
  }
  else
  {
    if (write(fd_value_, "0", 2) < 0)
      return -1;
  }
  // printf("Set value successful successful!\n\r");
  return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int GPIO::gpioGetValue(unsigned int *value)
{
  char ch;
  if (fd_value_ < 0)
  {
    return fd_value_;
  }

  if (read(fd_value_, &ch, 1) < 0)
    return -1;

  if (ch != '0')
  {
    *value = 1;
  }
  else
  {
    *value = 0;
  }
  return 0;
}
/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int GPIO::gpioSetEdge(char *edge)
{
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", num_gpio_);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
    perror("gpio/set-edge");
    return fd;
  }

  if (write(fd, edge, strlen(edge) + 1) < 0)
  {
    return -1;
  }

  printf("Set edge successful!\n\r");

  close(fd);
  return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int GPIO::gpioGetFd()
{
  if (fd_value_ < 0)
    perror("gpio/fd_open");

  return fd_value_;
}

int GPIO::gpioClose()
{
  if (num_gpio_ != -1)
  {
    gpioUnexport_(num_gpio_);
    num_gpio_ = -1;
  }

  if (fd_value_ > -1)
  {
    close(fd_value_);
    fd_value_ = -1;
  }
  return 0;
}

} // namespace rpg_odroid_io
