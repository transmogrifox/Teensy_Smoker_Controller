Released to the public domain under Unlicense

The file "iir_compensator.cpp" is the code implementing the Type III compensator.  The remainder of the code in this directory supports testing and evaluating it in a simulated control loop.

Note sim.ods (sim output) limit cycles at a much slower duty than the controller direct duty.  This response is hypothesized to be a side effect of quantization error.

A higher resolution pulse generator and a higher resolution ADC would likely make this go away.  In the target application a higher resolution pulse generator is not possible because I am using a solid-state relay that only switches at zero crossings.  Thus the minimum pulse width is 8ms for 60Hz power input.

Increasing resolution to, say, 12 bits, makes for a 33-second sampling period.  Unless the thermal system is exceedingly slow, the temperature change over the duration of 33 seconds is several bits.  Thus we cannot achieve high enough control loop bandwidth for very tight control.

Backing the sampling period off to a duration where 9-bit output resolution is achieved makes a fair balance where the temperature can be tightly controlled about the set-point with a well-behaved outer control loop.  The inner-loop limit-cycle effect is bounded and defined.
