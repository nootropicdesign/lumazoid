/*
  Lumazoid realtime music visualizer board firmware
  Copyright (C) 2015 Michael Krumpus, nootropic design, LLC

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LUMAZOID_H
#define LUMAZOID_H

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

typedef struct {
  uint8_t baseColor;
  uint8_t age;
  uint8_t magnitude;
  uint8_t rnd;
} peak_t;


#endif

