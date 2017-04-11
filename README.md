GPIO Library
============

Use with odroid
---------------
By default, GPIO pins need to be exported with
`echo $PIN_NUMBER > /sys/class/gpio/export`
which is taken care by this library. However, the newly exported GPIO pins
have their permissions set to root-access only. We can change that by adding
corresponding udev rules:

First create `/etc/udev/rules.d/10-gpio.rules`.

Then add the following two lines to the udev rules:
```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
```

For the changes to take effect, either reboot the odroid or run
`udevadm control --reload-rules`
