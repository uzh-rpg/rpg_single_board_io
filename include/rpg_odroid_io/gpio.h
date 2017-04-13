#pragma once

#define SYSFS_GPIO_DIR "/sys/class/gpio"

namespace rpg_odroid_io
{

static constexpr int kMaxBufLen = 64;

enum class GpioDirection
{
  In, Out
};

enum class GpioEdge
{
  None, Rising, Falling, Both
};

enum class GpioValue
{
  High, Low
};

class GPIO
{
public:
  GPIO(const unsigned int gpio, const GpioDirection dir);
  GPIO(const unsigned int gpio, const GpioEdge edge);
  ~GPIO();

  int gpioSetup(const unsigned int gpio, const GpioDirection dir);
  int gpioSetup(const unsigned int gpio, const GpioEdge edge);

  int gpioSetValue(const GpioValue value) const;
  int gpioGetValue(GpioValue *value) const;

  bool gpioIsOpen() const;
  int gpioGetFd() const;
  int gpioGetGpioNum() const;
  GpioDirection gpioGetDirection() const;
  GpioEdge gpioGetEdge() const;

private:
  int gpioExport(const unsigned int gpio) const;
  int gpioUnexport(const unsigned int gpio) const;
  int gpioSetDir(const unsigned int gpio, const GpioDirection dir) const;
  int gpioSetEdge(const GpioEdge edge) const;

  int gpioOpen();
  int gpioClose();

  int fd_value_;
  int num_gpio_;
  GpioDirection direction_;
  GpioEdge edge_;

}; // END class GPIO

}// END NAMESPACE rpg_odroid_io

