# Teensy Smoker Controller
Hysteretic temperature controller for my smoker

Calculations based on US Sensor 103JG1K Thermistor.

Spreadsheet contains formulas and optimizations made.

The final target will be a TEENSY LC microprocessor connected to a
solid-state relay used to drive the coil on a 30A contactor.

The 30A contactor is connected to the smoker heating element.

A Thermistor is placed in the desired control location in the smoker and 
measured by the Teensy 12-bit ADC.

The Teensy is programmed to compute the temperature and compare it against the set point.  If deasserted and lower than set_point+(hysteresis/2), then the Teensy asserts the LED pin (13).  If asserted and greater than set_point+(hysteresis/2), the Teensy de-asserts the LED pin (13) and will not re-assert until temperature drops below set_point-(hysteresis/2).  Hysteresis can be adjusted to balance the maximum temperature swing with the wear/stress on the contactor (for example, a tight control range can be achieved if it is acceptable to slam the contactor every few seconds).

A small-signal FET gate is driven by LED pin (13).  The FET is used to switch 5V to a solid-state relay input.  The solid-state relay is used for electrical isolation between the 5V circuits (user-accessible parts), and the 120V contactor coil.  

The 30A contactor is energized by switching its coil with with the solid-state relay, which energizes a standard outlet.  The heating element would then be powered from the switched outlet.

Teensy code includes interface to Adafruit USB+Serial LCD backpack controller 
and 16x2 character LCD for user feedback and calibration entry.  Furthermore 2 pushbutton switches and a 10k pot are used for setting the operation mode, adjusting the set-point, and for calibration.

src/thermistor_conversion.c contains the final formulas and a crude simulation of the system
used to validate basic operation before trying to run this on a Teensy. The Teensy .ino file then serves as an example for how to use the code for other thermistor projects.

## Future Development Ideas
An outter (slower) feedback loop may be implemented to adjust the set point linearly to directly control the internal temperature of the meat/cheese/whatever high-thermal-mass item.  Since the thermal time constant is typically much longer inside of, say, a 3-lb beef roast, then one can control the thermal profile at the core of the roast on a much slower outer control loop.  This can be used to estimate cook time and terminate heating when core temperature has reached the desired final temperature.

The objective would be for a repeatable automated means to deliver a succulent smoked roast perfectly cooked to medium rare.
