This is a simple egg timer for the STM32L-DISCOVERY and the 32L152CDISCOVERY
boards:


After power on or reset, it counts down from a hard coded time (1 hour by
default). When the time reaches zero, the two LEDs start flashing and the time
starts counting upwards. Hours, minutes and seconds are shown on the LCD. The
USER button delays the countdown by adding a hard coded amount of time (15
minutes by default).

RTC interrupts are used for decreasing the time every second.

The touch slider / 4 touch buttons are not used.

The project compiles on Linux using the GCC open source toolchain and is flashed
to the board using Texane's stlink software: https://github.com/texane/stlink.

The seconds are stored in a volatile variable that gets updated by the RTC
interrupt. In main, a tight loop runs continously that checks the number of
seconds remaining and flashes the LEDs if it's negative. This is not a power
efficient solution. To adjust this project for battery operated usage, the LEDs
should also be flashed using interrupts and the device should be put to sleep
between interrupts.

This project is based on the following project on GitHub:

https://github.com/tomvdb/stm32l1-discovery-basic-template

See the instructions there on how to set up an open source development
environment on Linux. When things are set up properly, build and flash with:

make flash


# Simple relay control for the $11 STM32L-DISCOVERY and 32L152CDISCOVERY boards

http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00027566.pdf

This is the program for this contraption:

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer.jpg)

The program cycles through the following states:

* relay 1 on,  relay 2 off, blue LED on,       green LED off
* relay 1 on,  relay 2 on,  blue LED on,       green LED flashing
* relay 1 off, relay 2 on,  blue LED off,      green LED on
* relay 2 off, relay 2 off, blue LED flashing, green LED on

In other words, the LEDs indicate which relay is on and if a switch will soon happen. While a LED is flashing, both relays are on.

The time to stay in each state is configurable and the remaining time for the current state is displayed on the LCD.

Pressing the USER button causes both relays to turn on and the device to remain in that state until Reset or one of the two custom buttons is pressed.

Pressing the left custom button causes the device to go directly to the state where relay 1 and the green LED are on.

Pressing the right custom button causes the device to go directly to the state where relay 2 and the blue LED are on.

The left button has been connected to PA4 and the right button to PD2. For the left button to work, the SB2 solder bridge on the board must be removed.

# Build and Flash

This project is based on:

https://github.com/rogerdahl/stm32l-discovery-egg-timer

Start with getting that project working using the instructions there. Then replace `main.c` with the one from this project.
