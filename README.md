### Simple relay control for the STM32L-DISCOVERY and 32L152CDISCOVERY boards

http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00027566.pdf

This is the program for this contraption:

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer.jpg)

The program switches 2 relays alternately on and off with a period of overlap where both relays are on. Only 2 of the 4 relays are in use. The LEDs indicate active relay and, in the period of overlap, which relay will become active. The amount of time for each relay to be enabled and for the overlap period is configured in the code. The remaining time for the current state is counted down on the LCD. 

Pressing the USER button once causes both relays to turn on and pressing it again causes both relays to turn off. In both cases, the device remains in that state until Reset or one of the two custom buttons is pressed.

Pressing the left and right custom buttons causes relay 1 or relay 2 to be enabled and normal cycling to resume.

#### Wiring

* Relay 5V: Discovery 5V
* Relay GND: Discovery GND
* Relay 1: PA5
* Relay 2: PA11
* Relay 3: PA12
* Relay 3: PC12
* Left button: PA4
* Right button: PD2

For the left button to work, the SB2 solder bridge on the board must be removed.

The relay module is a 5V, 4 channel module with optocouplers purchased on eBay. There are also boards without optocouplers available. I'm not sure if those will work, since the STM32L1 drives the relays with 3.3V.

#### Implementation

RTC interrupts are used for decreasing the time every second. The time is stored in seconds in a volatile variable that gets updated by the RTC interrupt. In main, a simple state machine runs in a tight loop. State switches occur when the time countdown reaches zero or when one of the buttons are pressed.

The program cycles through the following states:

* relay 1 on, relay 2 off, blue LED on, green LED off
* relay 1 on, relay 2 on, blue LED on, green LED flashing
* relay 1 off, relay 2 on, blue LED off, green LED on
* relay 2 off, relay 2 off, blue LED flashing, green LED on

The touch slider / 4 touch buttons are not used.

#### Build and Flash

This project is based on:

https://github.com/rogerdahl/stm32l-discovery-egg-timer

Start with getting that project working using the instructions there. Then replace `main.c` with the one from this project.
