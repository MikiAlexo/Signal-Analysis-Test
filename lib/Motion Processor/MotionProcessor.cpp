#include "MotionProcessor.h"
#include <Arduino.h>
#include <math.h>

MotionProcessor::MotionProcessor(Rate rate)
    : sampleRate(rate),
      dsp(static_cast<float>(rate)),   // give real Hz to DSP
      bufIndex(0),
      acc_X(0), acc_Y(0), acc_Z(0),
      gyr_X(0), gyr_Y(0), gyr_Z(0)
{
    sampleInterval = 1000UL / static_cast<unsigned long>(sampleRate);
    lastSampleTime = 0;
}

bool MotionProcessor::begin() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050!");
        return false;
    }

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_gyro  = mpu.getGyroSensor();

    Serial.print("MPU6050 initialized at ");
    Serial.print(sampleRate);
    Serial.println(" Hz sampling.");
    return true;
}

void MotionProcessor::update() {
    unsigned long now = millis();
    if (now - lastSampleTime < sampleInterval) return;  // enforce sampling rate
    lastSampleTime = now;

    sensors_event_t accel, gyro;
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    acc_X = accel.acceleration.x;
    acc_Y = accel.acceleration.y;
    acc_Z = accel.acceleration.z;

    gyr_X = gyro.gyro.x;
    gyr_Y = gyro.gyro.y;
    gyr_Z = gyro.gyro.z;

    // Acceleration magnitude
    float mag = sqrtf(acc_X * acc_X + acc_Y * acc_Y + acc_Z * acc_Z);

    // Add to DSP buffer
    dsp.addSample(mag);
}

void MotionProcessor::computeFFT() {
    dsp.computeFFT();
}

void MotionProcessor::computeAutocorrelation() {
    dsp.computeAutocorrelation();
}
