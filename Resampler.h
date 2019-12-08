#ifndef resampler_h_
#define resampler_h_


#include "Arduino.h"
#include "AudioStream.h"        //needed for AUDIO_BLOCK_SAMPLES

//#define DEBUG_RESAMPLER

#define MAX_FILTER_SAMPLES 40961 //=1024*20 +1
#define NO_EXACT_KAISER_SAMPLES 1025
#define MAX_HALF_FILTER_LENGTH 80
class Resampler {
    public:
        void init();
        void configure(float fs, float newFs);
        void resample(float* input0, float* input1, uint16_t inputLength, uint16_t& processedLength, float* output0, float* output1,uint16_t outputLength, uint16_t& outputCount);
        bool addToSampleDiff(double diff);
        double getXPos() const;
        double getStep() const;
        void fixStep();

        bool initialized() const;
    private:
        void getKaiserExact(float beta);
        void setKaiserWindow(float beta, int32_t noSamples);
        void setFilter(int32_t halfFiltLength,int32_t overSampling, float cutOffFrequ, float kaiserBeta);
        float filter[MAX_FILTER_SAMPLES];
        double kaiserWindowSamples[NO_EXACT_KAISER_SAMPLES];
        double tempRes[NO_EXACT_KAISER_SAMPLES-1];
        double kaiserWindowXsq[NO_EXACT_KAISER_SAMPLES-1];
        float _buffer0[MAX_HALF_FILTER_LENGTH*2];
        float _buffer1[MAX_HALF_FILTER_LENGTH*2];
        float* _endOfBuffer;

        int32_t _overSamplingFactor;
        int32_t _halfFilterLength;
        int32_t _filterLength;     
        bool _initialized=false;  
        
        double _stepAdaptedOld;
        double _settledThrs = 1e-12;
        double _step;
        double _amplitude;
        double _stepTransission[AUDIO_BLOCK_SAMPLES];
        int32_t _stepTransissionCounter;
        double _cPos;
        double _pControlParam= 0.05;
        double _oldCorrection;   
        double _oldDiff;  
};

#endif