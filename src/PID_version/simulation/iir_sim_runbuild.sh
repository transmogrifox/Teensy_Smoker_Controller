#!/bin/bash
g++ test_iir.cpp ../lib/iir_compensator.cpp -lm -lfftw3 -o iir_sim
./iir_sim
mousepad frequency_response.txt &
mousepad impulse_response.txt &
