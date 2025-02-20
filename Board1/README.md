# YAORP (Yet Another Open Reflow Plate)

## Status:

Prototype. Pending build. Untested in its fully built form, but largely tested on breadboard and very basic PoC firmware written.

## Introduction:

A design for a solder reflow plate for SMD rework using commonly available aluminium 400W PTC (positive-temperature-coefficient) heating plate from AliExpress. The PTC temperature is determined using a K type thermocouple. The PTC is driven by an SSR (solid state relay) using PWM (pulse width modulation) and temperature controlled by a PID (proportional, integral and derivative). The UI is presented via a 1.5" OLED screen and rotary encoder. An ESP32 S3 Wroom 1 module is at the heart of the design and firmware is written in C++ using Arduino IDE. 

## Project Goals:

- Learn KiCad.
- Enhance my knowledge of electronics
- Make the move away from hand soldered components.
- Build a useful addition to my lab.
- Make it as professional as possible.

![Prototype](Images/pcb.png)