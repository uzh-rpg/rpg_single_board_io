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
  Rising, Falling, Both
};

enum class GpioValue
{
  High, Low
};

class GPIO
{
public:
  GPIO();
  GPIO(const int gpio, const GpioDirection dir);
  GPIO(const int gpio, const GpioEdge edge);
  ~GPIO();

  int gpioSetup(const int gpio, const GpioDirection dir);
  int gpioSetup(const int gpio, const GpioEdge edge);

  int gpioSetValue(const GpioValue value) const;
  int gpioGetValue(GpioValue *value) const;

  bool gpioIsOpen() const;
  int gpioGetFd() const;
  int gpioGetGpioNum() const;
  GpioDirection gpioGetDirection() const;

  int gpioClose();

private:
  int gpioExport(const unsigned int gpio) const;
  int gpioUnexport(const unsigned int gpio) const;
  int gpioSetDir(const unsigned int gpio, const GpioDirection dir) const;
  int gpioSetEdge(const GpioEdge edge) const;

  int gpioOpen();

  int fd_value_;
  int num_gpio_;
  GpioDirection direction_;

}; // END class GPIO

}// END NAMESPACE rpg_odroid_io

