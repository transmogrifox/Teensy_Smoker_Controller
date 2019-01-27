A more advanced control algorithm.

This uses the analog prototype circuit known most commonly in switch-mode power supply design, example here:
http://www.ti.com/lit/an/slva662/slva662.pdf

The analog circuit for the op-amp Type III compensator is digitized by Bilinear Transform method (Tustin's method). (Note the resistor and capacitor references in the code are different than the TI paper because I drew my own schematic and derived it independently).

This is similar to a PID controller and offers very little (if any) advantage over a traditional PID implementation.  The main purpose for developing this style of controller is educational in nature.

For most cooking purposes a simple hysteretic controller (like my first implementation) is sufficient as long as the smoker/cooker doesn't have particularly large thermal masses.  For example, many electric ovens are based upon a bi-metallic thermostat, cycling temperature up and down.  Very few ovens use anything like PID control.

The more advanced type of controller (whether the Type III compensator or tradition PID)becomes useful when the heating/cooling system continues to conduct and radiate energy out of a large thermal mass even when the active power source has been shut down. A controller with a derivative function can help prevent the overshoot.

In my own experience I get a degree or two overshoot in the hysteretic controller while the more advanced controller can keep ripple to within +/-0.5 Fahrenheit (limit-cycle around resolution of the PWM output used to cycle power on and off).

That said, this implementation is a novelty, and a fun educational approach in which I get to tune a control loop response. The point is the set of plots I get off the temperature control process.  I don't have an expectation that my smoked jerky will taste different when smoked at a temperature tightly controlled to within +/-0.5 degrees of the set point compared to jerky cycled +/- 3 degrees.
