
This is code that belongs to the project at http://spritesmods.com/?art=frekvens

This firmware is intended for an ESP-Cam, but will possibly work on any board connected to
an ov2640 camera. It does not use or require PSRAM. It's built for ESP-IDF 4.0, but may or may
not also work with earlier or later versions.

Connections from the ESP-Cams GPIOs to the Frekvens board:

LAK - GPIO14
CLK - GPIO15
DA - GPIO13
EN - GPIO12 (or connect to gnd)

Button 1 - GPIO2
Button 2 - GPIO4 (note: unused)
Button common - GND

