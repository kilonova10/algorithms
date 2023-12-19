#include <iostream>
#include <sndfile.h>
#include <vector>
#include <cmath>
const double DECAY_THRESHOLD_DB = -60.0;

double calculateRT60(const std::vector<int16_t>& samples, double sampleRate) {
    // Find the peak amplitude in the signal
    double peakAmplitude = 0.0;
    for (double sample : samples) {
        peakAmplitude = std::max(peakAmplitude, std::abs(sample));
    }

    // Find the index of the peak amplitude
    size_t peakIndex = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (std::abs(samples[i]) == peakAmplitude) {
            peakIndex = i;
            break;
        }
    }
    std::cout << peakAmplitude <<  " " <<peakIndex << std::endl;
    double peakAmplitudeDB = 20.0 * std::log10(peakAmplitude);
    std::cout << peakAmplitudeDB << std::endl;
    // Calculate the decay threshold level
    double decayThreshold = peakAmplitudeDB + DECAY_THRESHOLD_DB;
    std::cout << "dt" << decayThreshold << std::endl;
    // Find the index where the signal falls below the decay threshold
    size_t decayIndex = peakIndex+1;
    while (decayIndex < samples.size() && 20.0 * std::log10(std::abs(samples[decayIndex])) >= decayThreshold) {
        ++decayIndex;
    }
    std::cout << 20.0 * std::log10(std::abs(samples[decayIndex]))<< std::endl;
    std::cout << decayIndex << " " << peakIndex << std::endl;

    // Calculate the RT60 in seconds
    double rt60 = static_cast<double>(decayIndex - peakIndex) / sampleRate;

    return rt60;
}

int main() {
    const char* filename = "/home/stargazer/Desktop/SYDE/new_doa_data/trial_2.wav"; // Change this to the path of your audio file

    // Open the audio file using libsndfile
    SF_INFO sf_info;
    SNDFILE* sndfile = sf_open(filename, SFM_READ, &sf_info);
    if (!sndfile) {
        std::cerr << "Error: Couldn't open the audio file." << std::endl;
        return 1;
    }

    // Read audio data into a vector of int16_t
    std::vector<int16_t> int16Samples(sf_info.frames * sf_info.channels);
    sf_readf_short(sndfile, int16Samples.data(), sf_info.frames);

    // Calculate RT60
    double sampleRate = sf_info.samplerate;
    double rt60 = calculateRT60(int16Samples, sampleRate);

    std::cout << "RT60: " << rt60 << " seconds" << std::endl;

    // Clean up
    sf_close(sndfile);

    return 0;
}
