#ifndef quantizer_h_
#define quantizer_h_

#include "Arduino.h"

//#define DEBUG_QUANTIZER

#define NOISE_SHAPE_F_LENGTH 9  //order of filter is 10, but the first coefficient equals 1 and doesn't need to be stored

class Quantizer {
public:
    ///@param audio_sample_rate currently only 44.1kHz and 48kHz are supported
    Quantizer(float audio_sample_rate);
    void configure(bool noiseShaping, bool dither, float factor);
    void quantize(float* input, int16_t* output, uint16_t length);
    //attention outputInterleaved must have length 2*length
    void quantize(float* input0, float* input1, int32_t* outputInterleaved, uint16_t length);
    void reset();
        
private:

bool _noiseShaping=true;
bool _dither=true;
float _fOutputLastIt0=0.f;
float _fOutputLastIt1=0.f;
float _buffer0[NOISE_SHAPE_F_LENGTH];
float _buffer1[NOISE_SHAPE_F_LENGTH];
float* _bPtr0=_buffer0;
float* _bufferEnd0;
float* _bPtr1=_buffer1;
float _noiseSFilter[NOISE_SHAPE_F_LENGTH ];
float _factor;

};

#endif