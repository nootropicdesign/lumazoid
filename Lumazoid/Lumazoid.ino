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

#include "Lumazoid.h"
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <ffft.h>
#include <math.h>

//#define DEBUG
#define ADC_CHANNEL 0
#define N_BANDS 8
#define N_FRAMES 5
#define N_PEAKS 25
#define N_MODES 9
#define PATTERN_DANCE_PARTY 0
#define PATTERN_SINGLE_DIR_DANCE_PARTY 1
#define PATTERN_PULSE 2
#define PATTERN_LIGHT_BAR 3
#define PATTERN_COLOR_BARS 4
#define PATTERN_COLOR_BARS2 5
#define PATTERN_FLASHBULBS 6
#define PATTERN_FIREFLIES 7
#define PATTERN_RANDOM 8
#define COLOR_RANDOM 0
#define COLOR_CYCLE 1
#define COLOR_BAND 2
#define N_COLOR_MODES 3
#define LEDCONFIG_60 0
#define LEDCONFIG_120 1
#define LEDCONFIG_180 2
#define MAX_COLOR_BARS 22
#define MAXCOLORINDEX 256
#define EEPROM_MAGIC_NUMBER 0xbad1 // just a 16 bit number that is not likely to randomly be found in EEPROM.
#define LED_BRIGHTNESS 25
#define UNUSED 255

#define PATTERN_BUTTON_PIN 2
#define COLOR_BUTTON_PIN 3
#define LED_PIN 5
#define LED_STRIP_PIN 6
#define PARM_POT 1

uint8_t N_LEDS = 120;
uint8_t MAX_AGE = 0;

#ifndef DEBUG
Adafruit_NeoPixel strip = Adafruit_NeoPixel(180, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(120, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
#endif

peak_t peaks[N_PEAKS];
uint8_t peakIndex = 0;
uint8_t newPeakFlags = 0;
uint8_t mode = 0;
uint8_t pattern = 0;
uint8_t cutoffFreqBand = 2;
uint8_t colorMode = COLOR_RANDOM;
uint8_t colorIndexIncFreq = 100;
uint8_t colorIndex = 0;
uint8_t randColor[] = {0, 0};
uint8_t randColorCount[] = {0, 0};
uint8_t randColorChangeParm = 10;
uint8_t ledConfig = LEDCONFIG_120;
uint16_t parm = 1024;
uint16_t lastParm;
uint8_t colorBars[MAX_COLOR_BARS];
uint16_t loopCounter = 0;
uint16_t loopCounterMax = 500;
boolean randomized = false;
uint8_t recentPatterns[3];
uint8_t recentPatternIndex = 0;
uint8_t transitionWaitTime = 0;

// FFT_N = 128
int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N / 2]; // Spectrum output buffer

volatile byte samplePos = 0;     // Buffer position counter
byte maxBrightness = 255;
float brightnessScale = 1.0;

byte
bandPeakLevel[8],      // Peak level of each band
              bandPeakCounter = 0, // Frame counter for delaying the fall of the band peak
              bandPeakDecay = 6, // peak decreases by 1 every bandPeakDecay frames. Larger value is slower decay
              bandCount = 0; // Frame counter for storing past band data
int
band[8][N_FRAMES],   // band levels for the prior N_FRAMES frames
     minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
     maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
     bandDiv[8];    // Used when filtering FFT output to 8 bands

// declare all control variables here so we can more easily measure memory consumption
uint8_t i, j, noiseThreshold, *data, nBins, binNum, weighting, c;
uint16_t minLvl, maxLvl;
int16_t level, sum;

const uint8_t PROGMEM noiseFloor[64] = {8, 6, 6, 5, 3, 4, 4, 4, 3, 4, 4, 3, 2, 3, 3, 4, 2, 1, 2, 1, 3, 2, 3, 2, 1, 2, 3, 1, 2, 3, 4, 4, 3, 2, 2, 2, 2, 2, 2, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4};
const uint8_t PROGMEM eq[64] = {255, 175, 218, 225, 220, 198, 147, 99, 68, 47, 33, 22, 14, 8, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t PROGMEM bandBinCounts[] = {2, 4, 5, 8, 11, 17, 25, 37};
const uint8_t PROGMEM bandBinStarts[] = {1, 1, 2, 3, 5, 7, 11, 16};
const uint8_t PROGMEM band0weights[] = {181, 40};
const uint8_t PROGMEM band1weights[] = {19, 186, 38, 2};
const uint8_t PROGMEM band2weights[] = {11, 176, 118, 16, 1};
const uint8_t PROGMEM band3weights[] = {5, 55, 165, 164, 71, 18, 4, 1};
const uint8_t PROGMEM band4weights[] = {3, 24, 89, 139, 148, 118, 54, 20, 6, 2, 1};
const uint8_t PROGMEM band5weights[] = {2, 9, 29, 70, 125, 172, 185, 162, 118, 74, 41, 21, 10, 5, 2, 1, 1};
const uint8_t PROGMEM band6weights[] = {1, 4, 11, 25, 49, 83, 121, 156, 180, 185, 174, 149, 118, 87, 60, 40, 25, 16, 10, 6, 4, 2, 1, 1, 1};
const uint8_t PROGMEM band7weights[] = {1, 2, 5, 10, 18, 30, 46, 67, 92, 118, 143, 164, 179, 185, 184, 174, 158, 139, 118, 97, 77, 60, 45, 34, 25, 18, 13, 9, 7, 5, 3, 2, 2, 1, 1, 1, 1};
const uint8_t PROGMEM * const bandWeights[] = {band0weights, band1weights, band2weights, band3weights, band4weights, band5weights, band6weights, band7weights};


void setup() {
  randomSeed(analogRead(2));
  memset(bandPeakLevel, 0, sizeof(bandPeakLevel));
  memset(band, 0, sizeof(band));

  for (i = 0; i < N_BANDS; i++) {
    binNum = pgm_read_byte(&bandBinStarts[i]);
    nBins = pgm_read_byte(&bandBinCounts[i]);

    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&bandWeights[i]);
    for (bandDiv[i] = 0, j = 0; j < nBins; j++) {
      bandDiv[i] += pgm_read_byte(&data[j]);
    }
  }

  for (i = 0; i < N_PEAKS; i++) {
    peaks[i].age = 0;
    peaks[i].magnitude = 0;
  }
  for (i = 0; i < MAX_COLOR_BARS; i++) {
    colorBars[i] = UNUSED;
  }
  for (i = 0; i < 3; i++) {
    recentPatterns[i] = UNUSED;
  }

  lastParm = 0;
  parm = analogRead(PARM_POT);

  setADCFreeRunning();
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin

  pinMode(PATTERN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(COLOR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  for (i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
  analogWrite(LED_PIN, LED_BRIGHTNESS);

  sei(); // Enable interrupts

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Check to see if the ledConfig (60 LEDs, 120, or 180 LEDs) and mode are stored in EEPROM.
  if (EEPROMValid()) {
    ledConfig = EEPROM.read(2);
    if (ledConfig > LEDCONFIG_180) {
      ledConfig = LEDCONFIG_180;
    }
    mode = EEPROM.read(3);
    if (mode > N_MODES) {
      mode = 0;
    }
    pattern = mode;
    if (pattern == PATTERN_RANDOM) {
      randomized = true;
      pattern = chooseRandomPattern();
      loopCounter = 0;
    } else {
      randomized = false;
    }
    maxBrightness = EEPROM.read(4);
    brightnessScale = maxBrightness / 255.0;
    colorMode = EEPROM.read(5);
    if (colorMode > COLOR_BAND) colorMode = COLOR_BAND;
    cutoffFreqBand = EEPROM.read(6);
    if (cutoffFreqBand > 7) cutoffFreqBand = 2;
  } else {
    test(); // run diagnostics on first time startup
    // EEPROM not written, so default to 120 LEDs and mode 0.
    ledConfig = LEDCONFIG_120;
    mode = 0;
    maxBrightness = 255;
    colorMode = COLOR_RANDOM;
    cutoffFreqBand = 2;
    saveConfig();
  }

  configure();

  setConfig();
  setParameters();

  TIMSK0 = 0;                // Timer0 off

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(getMemory());
#endif
}

void loop() {
  // While the ADC interrupt is enabled, wait. The program is still gathering
  // audio samples in the capture buffer.
  while (ADCSRA & _BV(ADIE));

  // The sampling interrupt has finished filling the buffer, so show the
  // output we computed last time around the loop. This sends the data to the LED strip.
  strip.show();

  // Perform the FFT algorithm to convert samples to complex numbers.
  fft_input(capture, bfly_buff);

  // Get analog input from user
  setADCDefault(); // configure the ADC so that we can read pot parameters
  lastParm = parm;
  parm = analogRead(PARM_POT);
  if (parm != lastParm) {
    // only change parameters if we read a different value than last time.
    setParameters();
  }


  if ((colorMode == COLOR_CYCLE) && ((loopCounter % colorIndexIncFreq) == 0)) {
    colorIndex++;
  }
  loopCounter++;
  if ((randomized) && (loopCounter > loopCounterMax)) {
    loopCounter = 0;
    loopCounterMax = random(500, 1000);
    transitionWaitTime = determineWaitTime();
  }
  if (transitionWaitTime > 0) {
    transitionWaitTime--;
    if (transitionWaitTime == 0) {
      pattern = chooseRandomPattern();
      setParameters();
      reset();
    }
  }


  // If the pattern button is pressed...
  if ((digitalRead(PATTERN_BUTTON_PIN) == LOW) && (digitalRead(COLOR_BUTTON_PIN) == HIGH)) {
    digitalWrite(LED_PIN, HIGH);
    setMode();
    analogWrite(LED_PIN, LED_BRIGHTNESS);
  }

  if ((digitalRead(COLOR_BUTTON_PIN) == LOW) && (digitalRead(PATTERN_BUTTON_PIN) == HIGH)) {
    digitalWrite(LED_PIN, HIGH);
    setColorMode();
    analogWrite(LED_PIN, LED_BRIGHTNESS);
  }

  if ((digitalRead(COLOR_BUTTON_PIN) == LOW) && (digitalRead(PATTERN_BUTTON_PIN) == LOW)) {
    // Both buttons held. Set cutoff frequency band.
    digitalWrite(LED_PIN, HIGH);
    setCutoffFreqBand();
    analogWrite(LED_PIN, LED_BRIGHTNESS);
  }



  // Now that we've updated the LED strip and processed the audio samples with FFT,
  // we can resume the collection of audio samples in the sample buffer. The interrupt
  // service routine (ISR) will run while we compute the next LED output based
  // on the audio samples we captured.
  setADCFreeRunning();             // Resume sampling interrupt


  // The rest of the FFT computation:
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Now call this to analyze the audio. See comments in this function for details.
  analyzeAudioSamples();


  if (transitionWaitTime == 0) { // only do this if not waiting to transition to new mode

    // The peak values for each of the 8 bands has been computed. A bit in the 8-bit
    // value newPeakFlags indicates whether the analysis found a *new* peak in the band.
    for (i = 0; i <= cutoffFreqBand; i++) {

      // If a new peak was found in band i...
      if (newPeakFlags & (1 << i)) {
        // Map the peak value to a magnitude in range [0,255]. We pass in the band
        // number because the mapping is different for different bands.
        uint8_t magnitude = getMagnitude(i, bandPeakLevel[i]);

        // A nonzero magnitude means that the peak value is large enough to do
        // something visually with it. We ignore small peaks.
        if (magnitude > 0) {
          // We want to store the information about the peak in a peak_t structure.
          // When we actually draw a visualization, the peak_t structures in peaks[]
          // represent the "visually active" band peaks.

          if (pattern != PATTERN_PULSE) {
            // Look through the list of peak structures 'peaks' for an unused one.
            for (j = 0; j < N_PEAKS; j++) {
              if (peaks[j].magnitude == 0) {
                // unused peak found
                peakIndex = j;
                break;
              }
            }
          } else {
            peakIndex = i; // for pulse mode, just use a peak for each band
          }
          // If an unused one not found, we use the last one that was used (peakIndex).

          // Initialize the structure.
          peaks[peakIndex].age = 0;
          peaks[peakIndex].rnd = random(255); // a random component for a visualzation to use. For example, to shift the color a bit.
          if (colorMode == COLOR_BAND) {
            peaks[peakIndex].baseColor = i * 32;
          }
          if (colorMode == COLOR_RANDOM) {
            peaks[peakIndex].baseColor = getRandomBaseColor(peaks[peakIndex].rnd);
          }
          if (colorMode == COLOR_CYCLE) {
            peaks[peakIndex].baseColor = colorIndex;
          }
          peaks[peakIndex].magnitude = magnitude;
        }
      }
    }
  }

  // Clear LED strip buffer. This doesn't actually affect the strip itself.
  if (pattern != PATTERN_LIGHT_BAR) {
    strip.clear();
  }

  // Now call the visualization routine based on the mode.
  doVisualization();
}

void analyzeAudioSamples() {
  // The peaks in each band need to decay so that new peaks can happen later.
  // The value bandPeakDecay determines how fast they decay. Currently set to 6,
  // so every 6th time through the main loop, each band peak decays by 1.
  // If it decays too fast (original value was 3), then there are too many peaks
  // detected and display is messy. Too few and you'll miss some real audio peaks
  // that should be displayed.
  if (++bandPeakCounter >= bandPeakDecay) {
    bandPeakCounter = 0;
    for (i = 0; i <= cutoffFreqBand; i++) {
      if (bandPeakLevel[i] > 0) {
        bandPeakLevel[i]--;
      }
    }
  }

  // Remove noise and apply EQ levels. This is from the Piccolo project.
  for (i = 0; i < FFT_N / 2; i++) {
    noiseThreshold = pgm_read_byte(&noiseFloor[i]);
    if (spectrum[i] < noiseThreshold) {
      spectrum[i] = 0;
    } else {
      spectrum[i] -= noiseThreshold;
      uint8_t eqVal = pgm_read_byte(&eq[i]);
      if (eqVal > 0) {
        spectrum[i] = (  ( spectrum[i] * (256L - eqVal)) >> 8);
      }
    }
  }

  // Downsample spectrum output to 8 bands.
  newPeakFlags = 0;
  for (i = 0; i <= cutoffFreqBand; i++) {
    data   = (uint8_t *)pgm_read_word(&bandWeights[i]);
    binNum = pgm_read_byte(&bandBinStarts[i]);
    nBins = pgm_read_byte(&bandBinCounts[i]);
    for (sum = 0, j = 0; j < nBins; j++) {
      sum += spectrum[binNum++] * pgm_read_byte(&data[j]);
    }
    band[i][bandCount] = sum / bandDiv[i];
    minLvl = maxLvl = band[i][0];
    for (j = 1; j < N_FRAMES; j++) { // Get range of prior frames
      if (band[i][j] < minLvl)      minLvl = band[i][j];
      else if (band[i][j] > maxLvl) maxLvl = band[i][j];
    }
    if ((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    minLvlAvg[i] = (minLvlAvg[i] * 7 + minLvl) >> 3; // Dampen min/max levels
    maxLvlAvg[i] = (maxLvlAvg[i] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 5L * (band[i][bandCount] - minLvlAvg[i]) /
            (long)(maxLvlAvg[i] - minLvlAvg[i]);

    if (level < 0L)      c = 0;
    else if (level > 20) c = 20;
    else                 c = (uint8_t)level;

    if (c > bandPeakLevel[i]) {
      bandPeakLevel[i] = c; // New peak found for this frequency band.
      newPeakFlags = newPeakFlags | (1 << i); // set new peak flag for this band
    }
  }

  if (++bandCount >= N_FRAMES) bandCount = 0;

}

void doVisualization() {
  float ageScale, b;
  uint32_t color;
  uint8_t r;

  // Bright peaks emanating from center moving outward.
  if ((pattern == PATTERN_DANCE_PARTY) ||
      (pattern == PATTERN_SINGLE_DIR_DANCE_PARTY)) {

    for (i = 0; i < N_PEAKS; i++) {
      if (peaks[i].magnitude > 0) {
        // peak is visually active

        // The age of the visual peak will be used to determine brightness.
        ageScale = (float)(1.0 - ((float)peaks[i].age / (float)MAX_AGE));

        // If brightness has been altered by a parameter, scale everything further.
        // Get the right color for this peak's band and adjust the brightness.
        color = adjustBrightness(getColor(peaks[i].baseColor, peaks[i].rnd), ageScale * brightnessScale);

        // Calculate horizontal position relative to starting point.
        // The first term is a value from 0.0 to 1.0 indicating speed.
        // It is a function of the peak's magnitude.
        // Minimum speed is 0.5 (127.0 / 255.0). Maximum is 1.0 (255.0 / 255.0);
        // So speed is how many LEDs does it "move" every step. Multiply this by
        // the peak's age to get the position.

        uint8_t pos;
        if (peaks[i].magnitude > 150) {
          pos = peaks[i].age;
        } else {
          pos = ((127 + (peaks[i].magnitude / 2)) / 255.0) * peaks[i].age;
        }

        switch (pattern) {
          case PATTERN_DANCE_PARTY:
            // Draw one right of center and one left of center.
            strip.setPixelColor((N_LEDS / 2) + pos, color);
            strip.setPixelColor(((N_LEDS / 2) - 1) - pos, color);
            break;
          case PATTERN_SINGLE_DIR_DANCE_PARTY:
            // Draw pixel relative to 0
            strip.setPixelColor(pos, color);
            break;
        }

        // Increment age
        peaks[i].age++;

        // If too old, retire this peak, making it available again.
        if (peaks[i].age > MAX_AGE) {
          peaks[i].magnitude = 0;
          continue;
        }

        if ((pos >= N_LEDS / 2) && (pattern != PATTERN_SINGLE_DIR_DANCE_PARTY)) {
          // Off the edge of the strip. This peak can now be returned to the pool
          // of peak structures that are available for use.
          peaks[i].magnitude = 0;
        }
        if ((pos >= N_LEDS) && (pattern == PATTERN_SINGLE_DIR_DANCE_PARTY)) {
          // Off the edge of the strip. This peak can now be returned to the pool
          // of peak structures that are available for use.
          peaks[i].magnitude = 0;
        }
      }
    }
    return;
  }


  // Pulse from center of strip.
  if (pattern == PATTERN_PULSE) {
    uint8_t maxWidth = N_LEDS / 2;
    uint8_t sortedBand[8];
    uint8_t sortedVal[8];
    uint8_t v;
    for (i = 0; i < N_BANDS; i++) {
      sortedBand[i] = 255;
      sortedVal[i] = 255;
    }

    for (i = 0; i < N_BANDS; i++) {
      v = map(peaks[i].magnitude, 0, 255, 0, maxWidth);

      ageScale = (float)(1.0 - ((float)peaks[i].age / (float)MAX_AGE));
      v = v * ageScale;

      // Insert into sorted list of bands.
      for (j = 0; j < N_BANDS; j++) {
        if (sortedVal[j] > v) {
          for (uint8_t k = 7; k > j; k--) {
            sortedVal[k] = sortedVal[k - 1];
            sortedBand[k] = sortedBand[k - 1];
          }
          sortedVal[j] = v;
          sortedBand[j] = i;
          break;
        }
      }

      // Increment age
      peaks[i].age++;

      // If too old, retire this peak, making it available again.
      if (peaks[i].age > MAX_AGE) {
        peaks[i].magnitude = 0;
        continue;
      }
    }

    i = 8;
    while (i > 0) {
      i--;
      v = sortedVal[i];
      if (v == 0) continue;
      color = getColor(peaks[sortedBand[i]].baseColor, peaks[sortedBand[i]].rnd);
      b = 0.5 + (float)v / (float)(2 * maxWidth);

      for (j = 0; j < v; j++) {
        uint32_t c2 = adjustBrightness(color, b * (float)(1.0 - (float)j / (float)v));
        strip.setPixelColor((N_LEDS / 2) + j, c2);
        strip.setPixelColor(((N_LEDS / 2) - 1) - j, c2);
      }
    }
    return;
  }

  // Entire strip is same color displaying most recent peak color.
  if (pattern == PATTERN_LIGHT_BAR) {

    uint32_t color;
    float ageScale;
    if (peaks[peakIndex].magnitude > 0) {
      if (peaks[peakIndex].age == 0) {
        byte baseColor = peaks[peakIndex].baseColor;
        // Since the light bar is so bright, scale down the max brightness
        // for this mode to avoid blindness.
        byte tmp = maxBrightness;
        float tmpF = brightnessScale;
        maxBrightness = maxBrightness >> 2; // divide by 4
        brightnessScale = maxBrightness / 255.0;
        for (i = 0; i < N_LEDS; i++) {
          color = getColor(baseColor, random(255));
          strip.setPixelColor(i, color);
        }
        // restore brightness settings
        maxBrightness = tmp;
        brightnessScale = tmpF;
      } else {
        // Adjust brightness for age
        ageScale = (float)(1.0 - ((float)peaks[peakIndex].age / (float)MAX_AGE));
        uint32_t tmpColor ;
        if ((peaks[peakIndex].rnd % 2) == 0) {
          // shift to right
          tmpColor = strip.getPixelColor(N_LEDS - 1);
          for (int p = (N_LEDS - 1); p >= 0; p--) {
            if (p == 0) {
              color = tmpColor;
            } else {
              color = strip.getPixelColor(p - 1);
            }
            color = adjustBrightness(color, ageScale);
            strip.setPixelColor(p, color);
          }
        } else {
          // shift to left
          tmpColor = strip.getPixelColor(0);
          for (int p = 0; p < N_LEDS; p++) {
            if (p == (N_LEDS - 1)) {
              color = tmpColor;
            } else {
              color = strip.getPixelColor(p + 1);
            }
            color = adjustBrightness(color, ageScale);
            strip.setPixelColor(p, color);
          }
        }

      }

      // age peak
      peaks[peakIndex].age++;

      if (peaks[peakIndex].age > MAX_AGE) {
        peaks[peakIndex].magnitude = 0;
      }
    }
    return;
  }

  // Visual peaks are assigned one of 15 color bars.
  if ((pattern == PATTERN_COLOR_BARS) || (pattern == PATTERN_COLOR_BARS2)) {
    uint8_t k = 0;
    uint8_t oldest;
    uint8_t nbars;
    switch (N_LEDS) {
      case 60:
      case 120:
        nbars = 15;
        break;
      case 180:
        nbars = 22;
        break;
    }
    if ((N_LEDS == 60) && (pattern == PATTERN_COLOR_BARS2)) {
      nbars = nbars / 2;
    }
    for (i = 0; i < N_PEAKS; i++) {
      if ((peaks[i].magnitude > 0) && (peaks[i].age == 0)) {
        // New peak (age == 0).
        // Find an unused color bar.
        j = random(nbars);
        k = 0;
        oldest = j;
        while (k < nbars) {
          if (colorBars[j] == UNUSED) {
            colorBars[j] = i;
            break;
          } else {
            if (peaks[colorBars[j]].age > peaks[colorBars[oldest]].age) {
              oldest = j;
            }
          }
          // keep looking for an unused bar.
          if ((i % 2) == 0) {
            j = (j + 1) % nbars;
          } else {
            if (j == 0) {
              j = nbars;
            } else {
              j = j - 1;
            }
          }
          k++;
        }
        if (k == nbars) {
          // did not find an unused color bar, so use oldest.
          colorBars[oldest] = i;
        }
      }
    }

    uint8_t maxWidth = N_LEDS / nbars;
    uint8_t width;
    for (i = 0; i < nbars; i++) {
      j = colorBars[i];
      if (j == UNUSED) continue;
      if (peaks[j].age == 0) {
        // If peak is brand new, make it max brightness!
        ageScale = 1.0;
      } else {
        // Otherwise, in range [0.0,0.5] as function of age.
        ageScale = 0.5 * (float)(1.0 - ((float)peaks[j].age / (float)MAX_AGE));
      }

      color = adjustBrightness(getColor(peaks[j].baseColor, peaks[j].rnd), ageScale);
      if (peaks[j].age == MAX_AGE) {
        // mark the assigned color bar as unused.
        colorBars[i] = UNUSED;
      }

      if (pattern == PATTERN_COLOR_BARS) {
        width = maxWidth;
      } else {
        // Width is function of age.
        if (peaks[j].age < 5) {
          // If young, force to max width
          width = maxWidth;
        } else {
          // Then shrink based on age.
          width = map(peaks[j].age - 5, 0, MAX_AGE - 4, maxWidth, 1);
          width -= width % 2; // force width to be multiple of 2
        }
      }

      for (j = 0; j < (width / 2); j++) {
        // Adjust brightness based on how far LED is from "center" of the bar.
        // Brighter at center, dimmer outside.
        color = adjustBrightness(color, (float)(1.0 - (float)((j * 2) / width)));

        k = (i * maxWidth) + ((maxWidth / 2) - 1 - j);
        strip.setPixelColor(k % N_LEDS, color);
        k = k + 1 + (j * 2);
        strip.setPixelColor(k % N_LEDS, color);
      }

    }

    // Age all the peaks.
    for (i = 0; i < N_PEAKS; i++) {
      if (peaks[i].magnitude > 0) {
        peaks[i].age++;
        if (peaks[i].age > MAX_AGE) {
          peaks[i].magnitude = 0;
        }
      }
    }

    return;
  }

  if ((pattern == PATTERN_FLASHBULBS) || (pattern == PATTERN_FIREFLIES)) {
    uint8_t pos, width;
    for (i = 0; i < N_PEAKS; i++) {
      if (peaks[i].magnitude > 0) {
        if (peaks[i].age == 0) {
          // If peak is brand new, make it max brightness!
          ageScale = 1.0;
        } else {
          // Otherwise, in range [0.0,0.5] as function of age.
          ageScale = 0.5 * (float)(1.0 - ((float)peaks[i].age / (float)MAX_AGE));
        }

        if ((peaks[i].age < 3) && (pattern == PATTERN_FLASHBULBS)) {
          // make it white for the first 3 times through the loop
          color = adjustBrightness(strip.Color(maxBrightness, maxBrightness, maxBrightness), ageScale);
        } else {
          color = adjustBrightness(getColor(peaks[i].baseColor, peaks[i].rnd), ageScale);
        }
        pos = (peaks[i].rnd * 7) % N_LEDS;
        if (pattern == PATTERN_FIREFLIES) {
          uint8_t delta = (min(255, 64 + peaks[i].magnitude) / 255.0) * peaks[i].age;
          if (pos % 2 == 0) {
            pos += delta;
          } else {
            pos -= delta;
          }
        }
        if (pos < N_LEDS) {
          strip.setPixelColor(pos, color);
        }
      }
    }

    // Age all the peaks.
    for (i = 0; i < N_PEAKS; i++) {
      if (peaks[i].magnitude > 0) {
        peaks[i].age++;
        if (peaks[i].age > MAX_AGE) {
          peaks[i].magnitude = 0;
        }
      }
    }
    return;
  }

}

void reset() {
  for (i = 0; i < N_PEAKS; i++) {
    peaks[i].age = 0;
    peaks[i].magnitude = 0;
  }
  for (i = 0; i < MAX_COLOR_BARS; i++) {
    colorBars[i] = UNUSED;
  }
}

// Configure the ADC to be free-running. It samples constantly and calls the ISR when
// a new conversion is ready to be recorded.
void setADCFreeRunning() {
  samplePos = 0; // Reset sample counter
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  // Start a conversion. We want to discard the first conversion after
  // changing the voltage reference reference.
  ADCSRB = 0;
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC);   // ADC start

  // Wait until conversion finishes.
  while (ADCSRA & _BV(ADSC));

  // Now start auto-triggered conversions with interrupt enabled.
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
}

// Configure ADC for normal Arduino operation so we can read pots.
void setADCDefault() {
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
  ADMUX  = 0;
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  analogRead(PARM_POT); // discard first conversion
}

// Change parameters based on config value.
void setConfig() {
  switch (ledConfig) {
    case 0:
      N_LEDS = 60;
      break;
    case 1:
      N_LEDS = 120;
      break;
    case 2:
      N_LEDS = 180;
      break;
  }
}

uint8_t determineWaitTime() {
  // determine how long to wait for the current visualization to "finish"
  // so we can move to a new visualization
  uint8_t youngest = MAX_AGE;
  if (pattern == PATTERN_LIGHT_BAR) {
    youngest = peaks[peakIndex].age;
  } else {
    for (i = 0; i < N_PEAKS; i++) {
      if (peaks[i].magnitude > 0) {
        if (peaks[i].age < youngest) {
          youngest = peaks[i].age;
        }
      }
    }
  }
  return (MAX_AGE - youngest) + 3;
}


uint8_t chooseRandomPattern() {
  boolean found = false;
  uint8_t p;
  while (!found) {
    p = random(PATTERN_RANDOM);
    found = true; // assume we've chosen a pattern we haven't used recently
    for (i = 0; i < 3; i++) {
      if (p == recentPatterns[i]) {
        // we used this pattern recently, so try again
        found = false;
        break;
      }
    }
  }
  recentPatterns[recentPatternIndex] = p;
  recentPatternIndex++;
  if (recentPatternIndex == 3) {
    recentPatternIndex = 0;
  }
  return p;
}

// Change parameters based on mode.
void setParameters() {
  switch (pattern) {
    case PATTERN_DANCE_PARTY:
      MAX_AGE = N_LEDS / 2 + N_LEDS / 8;
      break;
    case PATTERN_PULSE:
      MAX_AGE = 15;
      break;
    case PATTERN_SINGLE_DIR_DANCE_PARTY:
      MAX_AGE = N_LEDS + N_LEDS / 4;
      break;
    case PATTERN_LIGHT_BAR:
      MAX_AGE = 60;
      break;
    case PATTERN_COLOR_BARS:
    case PATTERN_FLASHBULBS:
      MAX_AGE = 30;
      break;
    case PATTERN_FIREFLIES:
      MAX_AGE = 30;
      break;
    case PATTERN_COLOR_BARS2:
      MAX_AGE = 25;
      break;
  }

  if (colorMode == COLOR_CYCLE) {
    colorIndexIncFreq = map(parm, 0, 1024, 20, 0);
  }
  if (colorMode == COLOR_RANDOM) {
    randColorChangeParm = (parm >> 5) & 0x1F;
    int diff = parm - lastParm;
    if (abs(diff) > 1) {
      randColorCount[0] = 31 - randColorChangeParm;
      randColorCount[1] = (31 - randColorChangeParm) >> 1;
    }
  }
}

// Get a random color. There are 2 random colors that change over
// time depending on randColorChangeParm.
// This gives a 2-color scheme that changes over time.
uint8_t getRandomBaseColor(uint8_t r) {
  if ((r % 2) == 0) {
    if (randColorChangeParm > 0) {
      if (randColorCount[0]-- == 0) {
        randColorCount[0] = 31 - randColorChangeParm;
        randColor[0] = random(255);
      }
    }
    return randColor[0];
  } else {
    if (randColorChangeParm > 0) {
      if (randColorCount[1]-- == 0) {
        randColorCount[1] = 31 - randColorChangeParm;
        randColor[1] = random(255);
      }
    }
    return randColor[1];
  }

}

// Map a color index in range [0-255] to a color.
// This is a uniform colorspace over 8 base colors:
// [red, orange, yellow, green, cyan, blue, magenta, white]
uint32_t getColor(uint8_t index, uint8_t rnd) {
  byte r, g, b;

  // advance the color by the random amount for variation
  if (colorMode != COLOR_RANDOM) {
    index += (rnd >> 4);
  } else {
    index += (rnd >> 5); // not as much random variation in 2-color scheme mode
  }
  byte f = (index % 32) * 8; // fractional part of color component
  switch (index / 32) {
    case 0: // red
      r = 255;
      g = f >> 1;
      b = 0;
      break;
    case 1: // orange
      r = 255;
      g = 127 + (f >> 1);
      b = 0;
      break;
    case 2: // yellow
      r = 255 - f;
      g = 255;
      b = 0;
      break;
    case 3: // green
      r = 0;
      g = 255;
      b = f;
      break;
    case 4: // cyan
      r = 0;
      g = 255 - f;
      b = 255;
      break;
    case 5: // blue
      r = f;
      g = 0;
      b = 255;
      break;
    case 6: // magenta
      r = 255 - f / 2;
      g = f / 2;
      b = 255 - f / 2;
      break;
    case 7: // white
      r = 127 + f / 2;
      g = 127 - f / 2;
      b = 127 - f / 2;
      break;
  }

  if (maxBrightness < 255) {
    // scale by brightnessScale;
    // optimize for 255 and 0 cases to avoid floating point multiply
    if (r == 255) {
      r = maxBrightness;
    } else {
      if (r > 0) {
        r = ((float)r * brightnessScale);
      }
    }
    if (g == 255) {
      g = maxBrightness;
    } else {
      if (g > 0) {
        g = ((float)g * brightnessScale);
      }
    }
    if (b == 255) {
      b = maxBrightness;
    } else {
      if (b > 0) {
        b = ((float)b * brightnessScale);
      }
    }
  }
  return strip.Color(r, g, b);
}


// Map the band peak level to a magnitude in range [0,255].
// This mapping depends on band number because different bands have different
// characteristics.
uint8_t getMagnitude(uint8_t band, uint8_t peakValue) {
  uint8_t peakMin, peakMax;
  switch (band) {
    case 0:
      // bass bands are really strong, so don't show anything with peak value
      // smaller than 8. They can have high values, so anything bigger than 20 is
      // maximum magnitude.
      peakMin = 8;
      peakMax = 20;
      break;
    case 1:
      peakMin = 8;
      peakMax = 20;
      break;
    case 2: // fall through...
    case 3:
    case 4:
    case 5:
      // These middle bands need a little more sensitivity, so allow values as low
      // as 4 to be represented visually.
      // These bands never go that high, so let's say that value of 8 is the high end.
      peakMin = 4;
      peakMax = 8;
      break;
    case 6:
    case 7:
      // High bands are not so sensitive and go a bit higher in practice.
      peakMin = 6;
      peakMax = 12;
      break;
  }

  if (pattern == PATTERN_PULSE) {
    // This mode needs to be more sensitive.
    if (band != 0) {
      peakMin = 1;
    } else {
      peakMin = 3;
    }
  }

  // If below min, then return 0 so we don't do anything visually.
  if (peakValue < peakMin) {
    return 0;
  }
  // Constrain to range [peakMin,peakMax]
  peakValue = constrain(peakValue, peakMin, peakMax);

  // Map to range [1,255] for all bands.
  return map(peakValue, peakMin, peakMax, 1, 255);
}



// Audio-sampling interrupt. This is invoked automatically whenever an ADC
// conversion is ready.
ISR(ADC_vect) {
  static const int16_t noiseThreshold = 4; // ignore small voltage variations.

  // the sample is available in the ADC register
  int16_t sample = ADC; // 0-1023

  capture[samplePos] =
    ((sample > (512 - noiseThreshold)) &&
     (sample < (512 + noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if (++samplePos >= FFT_N) {
    // The sample buffer is full, so disable the interrupt
    ADCSRA &= ~_BV(ADIE);
  }
}


// Scale brightness of a color.
uint32_t adjustBrightness(uint32_t c, float amt) {

  // pull the R,G,B components out of the 32bit color value
  uint8_t r = (uint8_t)(c >> 16);
  uint8_t g = (uint8_t)(c >> 8);
  uint8_t b = (uint8_t)c;

  // Scale
  r = r * amt;
  g = g * amt;
  b = b * amt;

  // Pack them into a 32bit color value again.
  return strip.Color(r, g, b);

}

// See how much SRAM is available
// Allocate memory until it fails. Then free it.
int getMemory() {
  int size = 2000;
  byte *buf;
  while ((size > 0) && ((buf = (byte *) malloc(--size)) == NULL));
  free(buf);
  return size;
}

// Determine if the EEPROM has ever been written by this firmware
// so we can determine if the values can be trusted.
// If it has been written, then there is a magic number in bytes [0,1].
boolean EEPROMValid() {
  unsigned int magic = EEPROM.read(0);
  magic = magic << 8;
  magic |= EEPROM.read(1);
  return (magic == EEPROM_MAGIC_NUMBER);
}

// Write the magic number into bytes [0,1].
void setEEPROMValid() {
  EEPROM.write(0, EEPROM_MAGIC_NUMBER >> 8);
  EEPROM.write(1, (EEPROM_MAGIC_NUMBER & 0xFF));
}

void saveConfig() {
  // Mark EEPROM as valid
  setEEPROMValid();
  EEPROM.write(2, ledConfig);
  EEPROM.write(3, mode);
  EEPROM.write(4, maxBrightness);
  EEPROM.write(5, colorMode);
  EEPROM.write(6, cutoffFreqBand);
}

void configure() {
  boolean buttonReleased = false;

  // If user is holding color button at startup, read the parameter pot and map the
  // value to the brightness.
  if ((digitalRead(COLOR_BUTTON_PIN) == LOW) && (digitalRead(PATTERN_BUTTON_PIN) == HIGH)) {
    delay(20); // debounce
    // Configure the ADC (analog to digital converter) to be in normal Arduino mode:
    setADCDefault();

    // show colors to adjust brightness
    while ((!buttonReleased) || (digitalRead(COLOR_BUTTON_PIN) == HIGH)) {
      if (digitalRead(COLOR_BUTTON_PIN) == HIGH) {
        buttonReleased = true;
        delay(20); // debounce
      }

      maxBrightness = analogRead(PARM_POT) >> 2;
      brightnessScale = maxBrightness / 255.0;

      // Draw each color to allow user to see brightness.
      strip.clear();
      for (i = 0; i < N_BANDS; i++) {
        strip.setPixelColor(i, getColor(i * 32, 0));
      }
      strip.show();
    }
    strip.clear();
    // Were'd done reading the pot, so configure the ADC back to free-running mode
    // for fast audio sampling.
    setADCFreeRunning();
    saveConfig();
    delay(20); // debounce
    while (digitalRead(COLOR_BUTTON_PIN) == LOW); // wait for release
    return;
  }

  // If user is holding pattern button at startup, read the parameter pot and map the
  // value to the LED config setting (0, 1, 2). 0 means 60 LEDs, 1 means 120 LEDs, 2 means 180 LEDs.
  if ((digitalRead(PATTERN_BUTTON_PIN) == LOW) && (digitalRead(COLOR_BUTTON_PIN) == HIGH)) {
    delay(20); // debounce
    // Configure the ADC (analog to digital converter) to be in normal Arduino mode:
    setADCDefault();
    buttonReleased = false;
    while ((!buttonReleased) || (digitalRead(PATTERN_BUTTON_PIN) == HIGH)) {
      if (digitalRead(PATTERN_BUTTON_PIN) == HIGH) {
        buttonReleased = true;
        delay(20); // debounce
      }

      // This maps [0,1023] to [0,2]. For mappings with a small number of possible
      // values, the map() function works best if you add 1 to the max input and max
      // output. That's why it's 1024 instead of 1023, and 3 instead of 2.
      ledConfig = map(analogRead(PARM_POT), 0, 1024, LEDCONFIG_60, LEDCONFIG_180+1);

      // Draw 1, 2, or 3 red pixels to indicate ledConfig
      strip.clear();
      for (i = 0; i <= ledConfig; i++) {
        strip.setPixelColor(i, strip.Color(64, 0, 0));
      }
      strip.show();
    }
    strip.clear();
    // Were'd done reading the pot, so configure the ADC back to free-running mode
    // for fast audio sampling.
    setADCFreeRunning();
    saveConfig();
    delay(20); // debounce
    while (digitalRead(PATTERN_BUTTON_PIN) == LOW); // wait for release
    return;
  }

  // If user is holding both buttons at startup, enter diagnostic mode.
  if ((digitalRead(COLOR_BUTTON_PIN) == LOW) && (digitalRead(PATTERN_BUTTON_PIN) == LOW)) {
    delay(20); // debounce
    test();
    return;
  }
}

void setMode() {
  byte tmp = mode;
  mode++;
  if (mode >= N_MODES) {
    mode = 0;
    transitionWaitTime = 0;
    randomized = false;
  }
  pattern = mode;
  if (pattern == PATTERN_RANDOM) {
    randomized = true;
    pattern = chooseRandomPattern();
    loopCounter = 0;
  } else {
    randomized = false;
  }

  strip.clear();
  // Light up an LED in the mode position as an indicator.
  if (!randomized) {
    strip.setPixelColor(mode, strip.Color(64, 64, 64));
  } else {
    strip.setPixelColor(mode, strip.Color(64, 0, 0)); // red for random pattern
  }
  strip.show();

  // Set parameters for the mode.
  setParameters();

  reset();

  // wait until button release
  while ((digitalRead(PATTERN_BUTTON_PIN) == LOW) && (digitalRead(COLOR_BUTTON_PIN) == HIGH));
  strip.clear();

  saveConfig();
}

void setColorMode() {
  byte tmp = colorMode;
  colorMode++;
  if (colorMode >= N_COLOR_MODES) {
    colorMode = 0;
  }
  strip.clear();
  strip.setPixelColor(colorMode, strip.Color(0, 64, 64));
  strip.show();
  while ((digitalRead(COLOR_BUTTON_PIN) == LOW) && (digitalRead(PATTERN_BUTTON_PIN) == HIGH));
  if (digitalRead(PATTERN_BUTTON_PIN) == LOW) {
    // other button was pressed, so undo this operation
    colorMode = tmp;
  }
  strip.clear();
  saveConfig();
}

void setCutoffFreqBand() {
  // Configure the ADC (analog to digital converter) to be in normal Arduino mode:
  setADCDefault();
  uint16_t reading1 = map(analogRead(PARM_POT), 0, 1024, 0, 8);
  uint8_t tmpColorMode = colorMode;
  colorMode = COLOR_BAND;

  // show bands to adjust cutoff frequency band
  while ((digitalRead(PATTERN_BUTTON_PIN) == LOW) && (digitalRead(COLOR_BUTTON_PIN) == LOW)) {
    strip.clear();
    for (i = 0; i <= cutoffFreqBand; i++) {
      strip.setPixelColor(i, getColor(i * 32, 0));
    }
    strip.show();
    uint16_t reading2 = map(analogRead(PARM_POT), 0, 1024, 0, 8);
    if (reading2 != reading1) {
      reading1 = reading2;
      cutoffFreqBand = reading2;
    }
  }

  colorMode = tmpColorMode;

  saveConfig();
  while ((digitalRead(PATTERN_BUTTON_PIN) == LOW) || (digitalRead(COLOR_BUTTON_PIN) == LOW)); // wait for release of both buttons
  strip.clear();
}

void test() {
  // diagnostics for first time startup
  setADCDefault();
  TIMSK0 = 1;
  boolean buttonReleased = false;

  for (i = 0; i < 10; i++) {
    strip.clear();
    strip.setPixelColor(i, strip.Color(64, 64, 64));
    strip.show();
    delay(20);
  }

  strip.clear();
  strip.show();

  while ((!buttonReleased) || (digitalRead(PATTERN_BUTTON_PIN) == HIGH)) {
    if (digitalRead(PATTERN_BUTTON_PIN) == HIGH) {
      buttonReleased = true;
      delay(20); // debounce
    }

    // set LED brightness to parm pot reading.
    analogWrite(LED_PIN, map(analogRead(PARM_POT), 0, 1023, 5, 255));
  }

  TIMSK0 = 0;
  analogWrite(LED_PIN, LED_BRIGHTNESS);
  setADCFreeRunning();
  while (digitalRead(PATTERN_BUTTON_PIN) == LOW); // wait for release
}

