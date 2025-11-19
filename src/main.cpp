#include "MotionProcessor.h"

MotionProcessor motion(MotionProcessor::RATE_BALANCED);

unsigned long lastProcess = 0;
const unsigned long processInterval = 5000; // process every 5s

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!motion.begin()) {
        Serial.println("MPU init failed!");
        while (true);
    }
}

void loop() {
    motion.update(); // handles timing internally

    if (millis() - lastProcess > processInterval) {
        motion.computeFFT();
        motion.computeAutocorrelation();

        Serial.print("Peak Freq (FFT): ");
        Serial.print(motion.getPeakFrequency());
        Serial.print(" Hz | Autocorr Freq: ");
        Serial.print(motion.getDominantAutocorr());
        Serial.println(" Hz");

        lastProcess = millis();
    }
}
