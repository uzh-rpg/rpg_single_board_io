#pragma once

#include <ros/ros.h>

namespace rpg_odroid_io
{

class ADCReader
{
public:
  ADCReader(const unsigned int adc_id);
  ADCReader(const unsigned int adc_id, const double upper_res, const double lower_res);
  ~ADCReader();

  int setVoltageDividerValues(const double upper_res, const double lower_res);
  int adcReadRaw(unsigned int& value) const;
  int adcReadScaled(double& value) const;

private:

  int adcSetup(const unsigned int adc_id);
  int adcSetup(const unsigned int adc_id, const double upper_res, const double lower_res);

  int adcConnect(const unsigned int adc_id);
  int adcDisconnect();

  int fd_;
  double voltage_divider_upper_res_;
  double voltage_divider_lower_res_;
};

} // namespace rpg_odroid_io
