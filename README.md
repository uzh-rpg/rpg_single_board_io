GPIO Library
============

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
