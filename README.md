### Simple relay control for the $11 STM32L-DISCOVERY and 32L152CDISCOVERY boards

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

#### Build and Flash

This project is based on:

https://github.com/rogerdahl/stm32l-discovery-egg-timer

Start with getting that project working using the instructions there. Then replace `main.c` with the one from this project.
