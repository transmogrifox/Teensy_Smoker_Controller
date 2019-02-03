#!/bin/bash
g++ control_sim.c ../lib/iir_compensator.cpp -lm -o sim
./sim > output.txt
mousepad output.txt
