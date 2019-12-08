#include "Quantizer.h"

Quantizer::Quantizer(){
#ifdef DEBUG_QUANTIZER
    while(!Serial);
#endif
    randomSeed(1);
}

void Quantizer::configure(bool noiseShaping, bool dither){
     _noiseShaping=noiseShaping;
     _dither=dither;
     _fOutputLastIt=0.;
}
void Quantizer::quantize(float* input, int16_t* output, uint16_t length){
    float xn, xnD, error;
    const float f2 = 1.f/1000000.f;
#ifdef DEBUG_QUANTIZER
    float debugFF=1024.f;
    const float factor=(powf(2.f, 15.f)-1.f)/debugFF;
#else
    const float factor=(powf(2.f, 15.f)-1.f);
#endif

    for (uint16_t i =0; i< length; i++){
        xn=(*input++)*factor;    //-_fOutputLastIt according to paper
        if (_noiseShaping){
            xn+=_fOutputLastIt;
        }
        if(_dither){
            const uint32_t r0=random(1000000);
            const uint32_t r1=random(1000000);
            xnD=xn + (r0 + r1)*f2-1.f;
        }
        else {
            xnD=xn;
        }
        float xnDR=round(xnD);
        if (_noiseShaping){
            //compute quatization error:
            error=xnDR- xn;
            *_bPtr++=error;
            if (_bPtr==_bufferEnd){
                _bPtr=_buffer;
            }
            _fOutputLastIt=0;
            float* f=_noiseSFilter;
            for (uint16_t j =0 ; j< NOISE_SHAPE_F_LENGTH; j++){
                _fOutputLastIt+=(*_bPtr++ * *f++);
                if (_bPtr==_bufferEnd){
                    _bPtr=_buffer;
                }
            }
        }
#ifdef DEBUG_QUANTIZER
        xnDR*=debugFF;
#endif
        if (xnDR > _maxVal){
            *output=(int16_t)_maxVal;
        }
        else if (xnDR < _minVal){
            *output=(int16_t)_minVal;
        }
        else {
            *output=(int16_t)xnDR;
        }

        ++output;
    }
}