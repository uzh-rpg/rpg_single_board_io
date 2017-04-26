GPIO Library
============

Attention!!!
------------

Odroids use 1.8V for all teir IO signals, so make sure that you use a level shifter (e.g. [this one](https://www.digikey.ch/product-detail/de/sparkfun-electronics/BOB-11771/1568-1208-ND/5673794)) when using GPIOs with external signals of different voltage levels and use an appropriate [voltage divider](https://en.wikipedia.org/wiki/Voltage_divider) when using an ADC port!

Use with odroid XU3/XU4
-----------------------
For GPIO pins to be used, they need to be exported with `echo $PIN_NUMBER > /sys/class/gpio/export`. This is taken care of by rpg_odroid_io. However, the newly exported GPIO pins have their permissions set to root-access only. We can change that by adding
corresponding udev rules:

First create `/etc/udev/rules.d/10-gpio.rules`, for instance by using `touch`.

Then add the following two lines to `/etc/udev/rules.d/10-gpio.rules`:

```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
```

[source](http://forum.odroid.com/viewtopic.php?f=80&t=15000)

For the changes to take effect, either reboot the odroid or run

`udevadm control --reload-rules`

or simply reboot the odroid. 


Use with Odroid U3
------------------
TODO

Alternative
-----------

To give a single executable root privileges you can do the following steps:

```
cd catkin_ws/devel/lib/YOUR_PACKAGE
sudo chown root:root YOUR_EXECUTABLE && sudo chmod a+rx YOUR_EXECUTABLE && sudo chmod u+s YOUR_EXECUTABLE
```

**Note:** You have to redo this step each time you recompile the executable
