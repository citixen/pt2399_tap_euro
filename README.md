# pt2399_tap_euro
A eurorack focused design for an accurate(ish) way to tap tempo/externally clocked delay using the PT2399 delay chip, a Teensy 4.1, MCP41100, MCP41010, 25LC256 EEPROM and an Adafruit 128x64 OLED SPI display

The code is very messy as this is an old unfinished (but working) project that I just want to make available as an idea. Code was built in the Arduino IDE, so some includes may be specific to that.

This works in a similar way to some other simple PT2399 tap solutions - but it tries to give more accuracy and the capability to account for variations between PT2399 chips, between different digital potentiometers tolerances of the same model, and also other possible variables in the circuit (like small changes in resistance to ground from pin 6 of the PT2399 caused by other components in-circuit). And to do this, i am trying to use simple readily available off the shelf components rather than hard to find and/or expensive high-accuracy digital pots.

In code we have an LFO running which allows for internal modulation of the delay time with a variety of LFO shapes.
We have potentiometer controls for LFO Rate, LFO Depth, LFO Shape, and also the delay time division.
A future improvement here would be to use one of the unused Teensy inputs to add a switch that allowed to switch between the LFO rate being just a range value, and it being set up as a tempo division of the delay time or straight tempo. 

In this euroracky design it also provides the ability to plug in an external modulation source for delay time, and to have pulse outputs matching the tapped tempo and also the expected tempo of the delayed signal. It is essential this input is disconnected when initialising.


The mode of operation is as follows : 

When put into the initialisation mode at startup, we iterate through all the possible values for combinations of the 100k and 10k resistor settings.
At each combination setting, we measure the frequency output by pin 5 of the PT2399 by using the FreqCount include. This then allows us to calculate the delay time the chip is operating at using the equations built by Electric Druid - https://electricdruid.net/useful-design-equations-for-the-pt2399/

Though in reality, because there is a lot of possible overlap here we don't measure *all* combinations. 1 step on the MCP41010 is ~40ohms. 1 step on the MCP41100 is ~390ohms. And so we instead just measure the first 12 steps of the MCP41010 at the first 255 values of the MCP41100 - then we also then measure 256 steps of MCP41010 at the highest resistance setting of the MCP41100. As a result instead of 65,536 delay time measurements with lots of duplication, we end up getting 3316 measurements with low levels of duplication.

These values are all entered to an array of objects containing both resistor values and the delay time calculated.
This array is sorted based on delay time, and then is deduplicated based on delay time. 

So we end up with an array of objects containing the required potentiometer value settings for each possible delay time that can be achieved by the hardware.

Once this is defined, it gets written to the EEPROM - so on next load, the array of values can be loaded and is available so we don't need to initialise every time.

After this initialisation is completed, or skipped, we enter the main loop which performs actions as follows : 
- Checks the state of the pulse output pins, and sets them high or low as needed.
- Reads the analog inputs - LFO Rate, depth, shape and delay division
- If the division selected has changed, calculate any required modifiers to the pulse outputs to ensure they stay in sync (with some divisions, the pulse outputs will drift apart without this because the time divisions leave remainders), identify what the nearest possible delay time from the initialised array is, and re-trigger the pulse outputs
- Check the LFO to get a modification value for the delay time to be output
- Write to set the digital pots to the relevant values for the closes possible delay time as defined above (modified by the internal LFO).
- Display the current values on the screen

In addition, an interrupt is enabled on a pin for a tap input. This is debounced in code. When the interrupt is issued, the delay time gets set to the time since the last tap, output pulses get retriggered, and relevant modifications are recalculated for output drift, and the best matching delay resistance settings are identified.
Future improvement for this could be to only reset the delay time after x taps?


TEENSY PINS : 
- PIN 0 - Divided clock output. 5ms pulse output at same rate as the calculated delay time
- PIN 1 - Clock output. 5ms pulse output as same rate as tap input
- PIN 2 - Tap input. Rising edge detection.
- PIN 3-7 - SPI OLED pins
- PIN 9 - Connects to pin 5 on the PT2399, which provides a measurable frequency output
- PIN 10 - select for 1 digital pot
- PIN 11 - MOSI for digital pots & EEPROM
- PIN 12 - MISO for EEPROM
- PIN 13 - SCK for digital pots and EEPROM
- PIN 14 - select for 2nd digital pot
- PIN 15 - select for EEPROM
- PIN 18 - analog in for beat division selection - potentiometer divider between 3v3 and GND
- PIN 19 - analog in for lfo shape selection - potentiometer divider between 3v3 and GND
- PIN 20 - analog in for lfo depth - potentiometer divider between 3v3 and GND
- PIN 22 - analog in for lfo rate - potentiometer divider between 3v3 and GND

