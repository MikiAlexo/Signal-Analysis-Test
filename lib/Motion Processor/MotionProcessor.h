#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SimpleDSP.h"

class MotionProcessor {
public:
    // Actual sampling rate presets (Hz)
    enum Rate {
        RATE_LOW = 25,
        RATE_BALANCED = 100,
        RATE_HIGH = 200
    };

    MotionProcessor(Rate rate = RATE_BALANCED);

    bool begin();
    void update();                  // Call this inside loop()

    // DSP computations
    void computeFFT();
    void computeAutocorrelation();

    // Data getters
    float getAccX() const { return acc_X; }
    float getAccY() const { return acc_Y; }
    float getAccZ() const { return acc_Z; }
    float getGyrX() const { return gyr_X; }
    float getGyrY() const { return gyr_Y; }
    float getGyrZ() const { return gyr_Z; }

    // Forwarding getters to DSP results
    float getPeakFrequency() const { return dsp.getPeakFrequency(); }
    float getDominantAutocorr() const { return dsp.getDominantAutocorr(); }

private:
    Adafruit_MPU6050 mpu;
    Adafruit_Sensor *mpu_accel;
    Adafruit_Sensor *mpu_gyro;

    // Raw readings
    float acc_X, acc_Y, acc_Z;
    float gyr_X, gyr_Y, gyr_Z;

    // Sampling control
    Rate sampleRate;
    unsigned long sampleInterval;   // milliseconds per sample
    unsigned long lastSampleTime;   // timing control

    // DSP
    SimpleDSP dsp;
    static const int N = 1024;
    float signalBuffer[N];
    int bufIndex;
};
