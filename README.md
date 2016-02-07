### Simple relay control for the [STM32L-DISCOVERY and 32L152CDISCOVERY](http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF250990?sc=internet/evalboard/product/250990.jsp) boards.

This is firmware for this contraption:

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer.jpg)

![](https://github.com/rogerdahl/stm32l1-relay-timer/blob/master/relay-timer-side.jpg)

#### Firmware

The program switches relay 1 and 2 alternately on and off with a period of overlap where both relays are on. The LEDs indicate active relay and, in the period of overlap, which relay will become active. The amount of time for each relay to be on and for the overlap period is configured in the code. The remaining time for the current state is counted down on the LCD. During the last few seconds before a switch, relay 4 is toggled to provide an audible indication. Relay 3 is not in use.  

Touching the left or right capacitive buttons causes relay 1 or 2 to be switched on and normal cycling to resume. Touching one of the center capacitive buttons causes a pause before normal cycling resumes. Pressing the User button switches both relays off and pressing it again switches both relays on. When relays are switched off with the User button, operations are halted until the left or right capacitive button is touched. When relays are switched on with the User button, operations are resumed after a configurable period.

#### Hardware

The STM32L152 Discovery board was soldered onto a breadboard since it doesn't have mounting holes. The breadboard and relay boards were mounted onto a piece of plywood with machine screws, using vinyl spacers and nuts on the back. The cables were clamped onto the board at the back to handle mechanical stress.

The relay module is a 5V, 4 channel module with optocouplers from eBay. There are also boards without optocouplers available. Note that the relay module must handle the 3.3V signal level of the STM32L1 unless level shifters are used.

#### Wiring

| STM32L1 | Relay |
|:--------|:------|
| EXT_5V  | VCC   |
| GND     | GND   |
| PA5     | IN1   |
| PA11    | IN2   |
| PA12    | IN3   |
| PC12    | IN4   |

The User button will not work correctly unless solder bridges are removed as detailed in the template project (see below).

#### Implementation

This project is based on:

https://github.com/rogerdahl/stm32l-discovery-timer-template

The time is stored in seconds in a volatile variable that gets updated by an interrupt that is triggered every second. In main(), a simple state machine runs in a tight loop. State switches occur when the time countdown reaches zero or when one of the buttons is pressed.

#### Build and Flash

Start with getting the template project working by following the instructions there. Then replace `main.c` with the one from this project.
