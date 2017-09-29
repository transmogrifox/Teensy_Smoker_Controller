# Teensy_Smoker_Controller
Hysteretic temperature controller for my smoker

Calculations based on US Sensor 103JG1K Thermistor.

Spreadsheet contains formulas and optimizations made.

The final target will be a TEENSY LC microprocessor connected to a
solid-state relay used to drive the coil on a 30A contactor.

The 30A contactor is connected to the smoker heating element.

A Thermistor is placed in the desired control location in the smoker and 
measured by the Teensy 12-bit ADC.

Further development will include interface to Adafruit USB+Serial LCD backpack controller 
and 16x2 character LCD for user feedback and calibration entry.

thermistor_conversion.c contains the final formulas and a crude simulation of the system
used to validate basic operation before trying to run this on a Teensy.
