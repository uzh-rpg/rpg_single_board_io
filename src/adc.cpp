#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include "rpg_odroid_io/adc.h"

namespace rpg_odroid_io
{

ADCReader::ADCReader(const unsigned int adc_id) :
    fd_(-1), voltage_divider_upper_res_(0.0), voltage_divider_lower_res_(0.0)
{
  adcSetup(adc_id);
}

ADCReader::ADCReader(const unsigned int adc_id, const double upper_res, const double lower_res) :
    fd_(-1), voltage_divider_upper_res_(0.0), voltage_divider_lower_res_(0.0)
{
  adcSetup(adc_id, upper_res, lower_res);
}

ADCReader::ADCReader() :
    fd_(-1), voltage_divider_upper_res_(0.0), voltage_divider_lower_res_(0.0)
{
}

ADCReader::~ADCReader()
{
  adcDisconnect();
}

int ADCReader::adcSetup(const unsigned int adc_id)
{
  if (adcConnect(adc_id) < 0)
  {
    return -1;
  }

  return 0;
}

int ADCReader::adcSetup(const unsigned int adc_id, const double upper_res, const double lower_res)
{
  if (adcConnect(adc_id) < 0)
  {
    return -1;
  }

  if (setVoltageDividerValues(upper_res, lower_res) < 0)
  {
    return -1;
  }
  
  is_setup_ = true;
  return 0;
}


int ADCReader::setVoltageDividerValues(const double upper_res, const double lower_res)
{
  if (!is_setup_){
    perror("ADC has not been setup!");
    return -1;
  }
  
  if (upper_res <= 0.0 || lower_res <= 0.0)
  {
    perror("Voltage divider resistor values must be strictly positive!");
    return -1;
  }

  voltage_divider_upper_res_ = upper_res;
  voltage_divider_lower_res_ = lower_res;

  return 0;
}

int ADCReader::adcReadRaw(unsigned int& value) const
{
  if (!is_setup_){
    perror("ADC has not been setup!");
    return -1;
  }
  
  if (fd_ < 0)
  {
    perror("ADC is not open");
    return fd_;
  }

  // Make sure we read from the beginning of the file
  lseek(fd_, 0, SEEK_SET);

  char buf[4];
  for (int i = 0; i < 4; i++)
  {
    char ch[1];
    int ret = read(fd_, ch, sizeof(ch));
    if (ret > 0)
    {
      buf[i] = ch[0];
    }
    else if (ret == 0)
    {
      // If we read less than 4 bytes, the last red byte seem to be rubbish
      buf[i - 1] = 0;
      break;
    }
    else
    {
      return -1;
    }
  }

  value = atoi(buf);

  return 0;
}

int ADCReader::adcReadScaled(double& value) const
{
  if (!is_setup_){
    perror("ADC has not been setup!");
    return -1;
  }
  
  unsigned int raw_value;
  if (adcReadRaw(raw_value) < 0)
  {
    perror("Could not read ADC");
    return -1;
  }

  if (voltage_divider_lower_res_ <= 1e-4)
  {
    perror("Voltage divider resistor values not set");
    return -1;
  }

  value = raw_value / double(kMaxAdcValue) * kMaxAdcVoltage * (voltage_divider_upper_res_ + voltage_divider_lower_res_)
      / voltage_divider_lower_res_;

  return 0;
}

int ADCReader::adcConnect(const unsigned int adc_id)
{
  if (!is_setup_){
    perror("ADC has not been setup!");
    return -1;
  }
  
  if (adc_id != 0 && adc_id != 3)
  {
    perror("On the Odroid XU4, the ADC ID must be either 0 or 3!");
    return -1;
  }

  std::string adc_path;

  if (adc_id == 0)
  {
    adc_path = XU4_ADC0_PATH;
  }
  if (adc_id == 3)
  {
    adc_path = XU4_ADC3_PATH;
  }

  fd_ = open(adc_path.c_str(), O_RDONLY);
  if (fd_ < 0)
  {
    perror("Could not open adc!");
    return fd_;
  }

  return 0;
}

int ADCReader::adcDisconnect()
{
  if (!is_setup_){
    perror("ADC has not been setup!");
    return -1;
  }
  
  if (fd_ > -1)
  {
    close(fd_);
    fd_ = -1;
  }

  voltage_divider_upper_res_ = 0.0;
  voltage_divider_lower_res_ = 0.0;

  return 0;
}

} // namespace rpg_odroid_io
