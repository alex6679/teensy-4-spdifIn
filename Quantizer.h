#ifndef quantizer_h_
#define quantizer_h_

#include "Arduino.h"

//#define DEBUG_QUANTIZER

#define NOISE_SHAPE_F_LENGTH 9  //order of filter is 10, but the first coefficient equals 1 and doesn't need to be stored

class Quantizer {
public:
    Quantizer();
    void configure(bool noiseShaping, bool dither);
    void quantize(float* input, int16_t* output, uint16_t length);
        
private:

bool _noiseShaping=true;
bool _dither=true;
float _fOutputLastIt=0.f;
float _buffer[NOISE_SHAPE_F_LENGTH]={0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
float* _bPtr=_buffer;
float* _bufferEnd=&_buffer[NOISE_SHAPE_F_LENGTH];
float _noiseSFilter[NOISE_SHAPE_F_LENGTH ]={-0.06935825f,  0.52540845f, -1.20537028f,  2.09422811f, -3.2177438f,  4.04852027f, -3.83872701f,  3.30584589f, -2.38682527f};
//  all coefficients in correct order:
//      {1.        , -2.38682527,  3.30584589, -3.83872701,  4.04852027,
//       -3.2177438 ,  2.09422811, -1.20537028,  0.52540845, -0.06935825};
float _maxVal=powf(2,15)-1.;
float _minVal=-powf(2,15);

};

#endif