#pragma once
#include <Arduino.h>
#include <math.h>

class SimpleDSP {
public:
    static const int N = 1024;       
    static const int OVERLAP = 512;  
    float fftMagnitudes[N/2];        

    SimpleDSP(float sampleRate = 100.0f);


    void addSample(float x);

    
    void computeFFT();               
    void computeAutocorrelation();   


    void getFFTArray(float* outArray) const;
    float getMagnitude(int bin) const;
    float getPeakFrequency() const;  
    float getDominantAutocorr() const;

private:
    float Fs;             
    float buffer[N];      
    int index = 0;        
    bool bufferReady = false;

   
    float fftReal[N];
    float fftImag[N];
    float fftMag[N/2];
    float window[N];

   
    float autocorr[N];

    float peakFreq;        

    void applyHammingWindow();
    void computeMagnitude();
};
