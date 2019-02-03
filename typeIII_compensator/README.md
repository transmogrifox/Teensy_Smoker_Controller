A more advanced control algorithm.

This uses the analog typical analog circuit used to implement a PID controller.  Example here:
http://www.ti.com/lit/an/slva662/slva662.pdf

The full PID controller is the circuit identified as the Type III compensator.

In this repository is code implementing the digitized analog circuit for the op-amp Type III compensator.  Method used for digitization by Bilinear Transform method (Tustin's method). (Note the resistor and capacitor references in the code are different than the TI paper because I drew my own schematic and derived it independently -- this will be updated as time allows, probably removing reference to the TI app note).

The main advantages to the Type III compensator vs traditional software PID implementation are as follows:
1) Control system stability analysis can be performed using a SPICE simulator.  The RC components from the analog PID controller can then be submitted directly to the setup function after a system with desired performance is simulated.
2) Additional pole for noise filtering is included in the Type III compensator analog prototype circuit.  This improves high-frequency noise rejection and can be used to avoid high-frequency instability due to higher order effects that encroach on the gain margin above cross-over.  It may be the anti-aliasing filter used in the application already serves this purpose, but this pole has been included because it does no harm if set high enough, and might be useful in some applications.
3) Gain limiting effect (low-frequency pole) typically characteristic of an op-amp finite gain when configured as an integrator.  The low-frequency gain can be reduced to coordinate with ADC resolution if the theoretical infinite gain of an integrator causes registers to saturate, leading to unwanted nonlinear behavior.  Normally this pole is set low enough to be of no measurable effect (like an op amp).  It was captured in the derivation because it might be useful in some applications. 

For most cooking purposes a simple hysteretic controller (like my first implementation) is sufficient as long as the smoker/cooker doesn't have particularly large thermal masses leading to unacceptable overshoot.  In a smoker, this is rarely the case.

For example, many commercial electric ovens are based upon a bi-metallic thermostat, cycling temperature up and down.  Very few ovens use anything like PID control.

The more advanced type of controller (PID or adaptive controllers) becomes useful when the heating/cooling system continues to conduct and radiate energy out of a large thermal mass even when the active power source has been shut down and large amounts of gain (high-power heating element) is required to change the temperature in the mass sufficiently quickly for the requirements. A controller with a derivative function can help prevent the overshoot.

In my own experience I get a degree or two overshoot in the hysteretic controller while the more advanced controller can keep ripple to within +/-0.5 Fahrenheit (limit-cycle around resolution of the PWM output used to cycle power on and off).

That said, this implementation is a novelty, and a fun educational approach in which I get to tune a control loop response. The point is the set of plots I get off the temperature control process.  I don't have an expectation that my smoked jerky will taste different when smoked at a temperature tightly controlled to within +/-0.5 degrees of the set point compared to jerky cycled +/- 3 degrees.
