### Simple relay control for the [STM32L-DISCOVERY and 32L152CDISCOVERY boards](http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00027566.pdf)

This is firmware for this contraption:

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer.jpg)

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer-side.jpg)

#### Firmware

The program switches 2 relays alternately on and off with a period of overlap where both relays are on. Only 2 of the 4 relays are in use. The LEDs indicate active relay and, in the period of overlap, which relay will become active. The amount of time for each relay to be enabled and for the overlap period is configured in the code. The remaining time for the current state is counted down on the LCD. 

Pressing the USER button once causes both relays to turn on and pressing it again causes both relays to turn off. In both cases, the device remains in that state until Reset or one of the two custom buttons is pressed.

Pressing the left and right custom buttons causes relay 1 or relay 2 to be enabled and normal cycling to resume.

#### Hardware

The STM32L board was soldered onto a breadboard since it doesn't have mounting holes. The breadboard and the relay board were mounted onto a piece of plywood with through-hole machine screws with vinyl spacers and nuts on the back. The cables were clamped onto the board at the back to handle mechanical stress.

The relay module is a 5V, 4 channel module with optocouplers purchased on eBay. There are also boards without optocouplers available. I'm not sure if those will work, since the STM32L1 drives the relays with 3.3V.

#### Wiring

| STM32L1 | Relay |
|:--------|:------|
| EXT_5V  | VCC   |
| GND     | GND   |
| PA5     | IN1   |
| PA11    | IN2   |
| PA12    | IN3   |
| PC12    | IN4   |

| STM32L1 | Button |
|:--------|:------|
| PA4     | Left  |
| PD2     | Right |

For the left button to work, the SB2 solder bridge on the board must be removed.

#### Implementation

RTC interrupts are used for decreasing the time every second. The time is stored in seconds in a volatile variable that gets updated by the RTC interrupt. In main, a simple state machine runs in a tight loop. State switches occur when the time countdown reaches zero or when one of the buttons are pressed.

The program cycles through the following states:

| Relay 1 | Relay 2 | Blue LED | Green LED |
|---------|---------|----------|-----------|
| on      | off     | on       | off       |
| on      | on      | on       | flashing  |
| off     | on      | off      | on        |
| on      | on      | flashing | on        |

The touch slider / 4 touch buttons are not used.

#### Build and Flash

This project is based on:

https://github.com/rogerdahl/stm32l-discovery-egg-timer

Start with getting that project working using the instructions there. Then replace `main.c` with the one from this project.
