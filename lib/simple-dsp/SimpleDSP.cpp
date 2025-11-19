#include "SimpleDSP.h"


SimpleDSP::SimpleDSP(float sampleRate)
    : Fs(sampleRate), index(0), bufferReady(false), peakFreq(0.0f)
{
    
    for (int i = 0; i < N; i++) {
        buffer[i] = 0.0f;
        fftReal[i] = 0.0f;
        fftImag[i] = 0.0f;
        fftMag[i / 2] = 0.0f;
        autocorr[i] = 0.0f;
        window[i] = 0.54f - 0.46f * cosf((2.0f * M_PI * i) / (N - 1)); // Hamming window
    }
}


void SimpleDSP::addSample(float x)
{
    buffer[index++] = x;
    if (index >= N) {
        index = 0;
        bufferReady = true;
    }
}


void SimpleDSP::applyHammingWindow()
{
    for (int i = 0; i < N; i++)
        fftReal[i] = buffer[i] * window[i];
}


// Basic Cooley–Tukey FFT implementation (in-place)
void SimpleDSP::computeFFT()
{
    if (!bufferReady) return;

    applyHammingWindow();

    
    for (int i = 0; i < N; i++)
        fftImag[i] = 0.0f;

    
    int j = 0;
    for (int i = 0; i < N; i++) {
        if (i < j) {
            float tmp = fftReal[i];
            fftReal[i] = fftReal[j];
            fftReal[j] = tmp;
        }
        int m = N >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * M_PI / len;
        float wlen_cos = cosf(ang);
        float wlen_sin = sinf(ang);

        for (int i = 0; i < N; i += len) {
            float w_real = 1.0f;
            float w_imag = 0.0f;

            for (int j = 0; j < len / 2; j++) {
                int even = i + j;
                int odd = i + j + len / 2;

                float u_real = fftReal[even];
                float u_imag = fftImag[even];
                float v_real = fftReal[odd] * w_real - fftImag[odd] * w_imag;
                float v_imag = fftReal[odd] * w_imag + fftImag[odd] * w_real;

                fftReal[even] = u_real + v_real;
                fftImag[even] = u_imag + v_imag;
                fftReal[odd]  = u_real - v_real;
                fftImag[odd]  = u_imag - v_imag;

                float next_w_real = w_real * wlen_cos - w_imag * wlen_sin;
                w_imag = w_real * wlen_sin + w_imag * wlen_cos;
                w_real = next_w_real;
            }
        }
    }

    computeMagnitude();
}


void SimpleDSP::computeMagnitude()
{
    float maxMag = 0.0f;
    int peakBin = 0;

    for (int i = 0; i < N / 2; i++) {
        fftMag[i] = sqrtf(fftReal[i] * fftReal[i] + fftImag[i] * fftImag[i]);
        fftMagnitudes[i] = fftMag[i]; 
        if (fftMag[i] > maxMag) {
            maxMag = fftMag[i];
            peakBin = i;
        }
    }

    peakFreq = (peakBin * Fs) / N; 
}


void SimpleDSP::computeAutocorrelation()
{
    if (!bufferReady) return;

    float mean = 0.0f;
    for (int i = 0; i < N; i++) mean += buffer[i];
    mean /= N;

    for (int i = 0; i < N; i++)
        fftReal[i] = buffer[i] - mean;

    for (int lag = 0; lag < N; lag++) {
        float sum = 0.0f;
        for (int i = 0; i < N - lag; i++)
            sum += fftReal[i] * fftReal[i + lag];
        autocorr[lag] = sum;
    }

    float maxVal = autocorr[0];
    for (int i = 0; i < N; i++)
        autocorr[i] /= maxVal;
}

void SimpleDSP::getFFTArray(float* outArray) const
{
    for (int i = 0; i < N / 2; i++)
        outArray[i] = fftMagnitudes[i];
}

float SimpleDSP::getMagnitude(int bin) const
{
    if (bin < 0 || bin >= N/2) return 0.0f;
    return fftMagnitudes[bin];
}

float SimpleDSP::getPeakFrequency() const
{
    return peakFreq;
}

float SimpleDSP::getDominantAutocorr() const
{
    int bestLag = 1;
    float maxVal = 0.0f;
    for (int i = 1; i < N / 2; i++) {
        if (autocorr[i] > maxVal) {
            maxVal = autocorr[i];
            bestLag = i;
        }
    }

    return Fs / bestLag; // return dominant frequency (Hz)
}
