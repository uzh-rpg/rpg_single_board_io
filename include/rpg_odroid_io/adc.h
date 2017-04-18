#pragma once

#include <ros/ros.h>

#define XU4_ADC0_PATH "/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw"
#define XU4_ADC3_PATH "/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw"

namespace rpg_odroid_io
{

static constexpr int kMaxAdcValue = 4095;
static constexpr double kMaxAdcVoltage = 1.8;

class ADCReader
{
public:
  ADCReader(const unsigned int adc_id);
  ADCReader(const unsigned int adc_id, const double upper_res, const double lower_res);
  ADCReader();
  ~ADCReader();

  int setVoltageDividerValues(const double upper_res, const double lower_res);
  int adcReadRaw(unsigned int& value) const;
  int adcReadScaled(double& value) const;

  int adcSetup(const unsigned int adc_id);
  int adcSetup(const unsigned int adc_id, const double upper_res, const double lower_res);

private:
  int adcConnect(const unsigned int adc_id);
  int adcDisconnect();

  int fd_;
  double voltage_divider_upper_res_;
  double voltage_divider_lower_res_;
  bool is_setup_ = false;
};

} // namespace rpg_odroid_io
