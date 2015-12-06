# Lumazoid

Firmware for the Lumazoid realtime music visualizer board from nootropic design.
Detailed technical information about Lumazoid is at http://nootropicdesign.com/lumazoid

Install libraries/ffft in your Arduino sketchbook libraries folder.

Requires the [Adafruit NeoPixel library](https://github.com/adafruit/Adafruit_NeoPixel). 
Your directory structure should look like this:

your_sketchbook
 |
 +--libraries
 |   |
 |   +--ffft
 |   |
 |   +--Adafruit_NeoPixel
 |
 +--Lumazoid
     |
     +--Lumazoid.h
     +--Lumazoid.ino

In the Arduino IDE, select board type "Arduino Uno" to upload to the Lumazoid.
