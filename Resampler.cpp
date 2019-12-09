#include "Resampler.h"
#include <math.h>

void Resampler::init(){
#ifdef DEBUG_RESAMPLER
	while (!Serial);
#endif
    kaiserWindowSamples[0]=1.;
    double step=1./(NO_EXACT_KAISER_SAMPLES-1);
    double* xSq=kaiserWindowXsq;
    for (uint16_t i = 1; i <NO_EXACT_KAISER_SAMPLES; i++){
        double x=(double)i*step;
        *xSq++=(1.-x*x);
    }
    for (uint16_t i = 1; i< AUDIO_BLOCK_SAMPLES; i++){
        _stepTransission[i]=0.5-0.5*cos(M_PI*(double)i/AUDIO_BLOCK_SAMPLES);
    }
    _stepTransission[AUDIO_BLOCK_SAMPLES-1]=1.;
    _stepTransissionCounter =AUDIO_BLOCK_SAMPLES;
    _amplitude=0;
}
void Resampler::getKaiserExact(float beta){
    const double thres=1e-10;   
    double* winS=&kaiserWindowSamples[1];
    double* t=tempRes;
    for (uint16_t i = 1; i <NO_EXACT_KAISER_SAMPLES; i++){
        *winS++=1.;
        *t++=1.;
    }
    double denomLastSummand=1.;
    const double halfBetaSq=beta*beta/4.;
    double denom=1.;
    double i=1.;
    while(i < 1000){
        denomLastSummand*=(halfBetaSq/(i*i));
        i+=1.;
        denom+=denomLastSummand;
        t=tempRes;
        winS=&kaiserWindowSamples[1];
        double* xSq=kaiserWindowXsq;
        for (uint16_t j=0; j<  NO_EXACT_KAISER_SAMPLES-1;j++){
            (*t)*=(*xSq);
            double summand=(denomLastSummand*(*t));
            (*winS)+=summand;
            if (summand< thres){
                break;
            }
            ++winS;
            ++t;
            ++xSq;
        }  
        if (denomLastSummand< thres){
            break;
        }
    }
    winS=&kaiserWindowSamples[1];
    for (int32_t i = 0; i <NO_EXACT_KAISER_SAMPLES-1; i++){
        *winS++/=denom;
    }
}
    
void Resampler::setKaiserWindow(float beta, int32_t noSamples){
    getKaiserExact(beta);
    double step=(float)(NO_EXACT_KAISER_SAMPLES-1.)/(noSamples-1.);
    double xPos=step;
    float* filterCoeff=filter;
    *filterCoeff=1.;
    ++filterCoeff;
    int32_t lower=(int)(xPos);
    double* windowLower=&kaiserWindowSamples[lower];
    double* windowUpper=&kaiserWindowSamples[lower+1];
    for (int32_t i =0; i< noSamples-2; i++){
        double lambda=xPos-lower;
        if (lambda > 1.){
            lambda-=1.;
            ++windowLower;
            ++windowUpper;
            lower++;
        }
        *filterCoeff++=(float)(lambda*(*windowUpper)+(1.-lambda)*(*windowLower));
        xPos+=step;
        if (xPos>=NO_EXACT_KAISER_SAMPLES-1 || lower >=NO_EXACT_KAISER_SAMPLES-1){
            break;
        }
    }
    *filterCoeff=*windowUpper;
}

void Resampler::setFilter(int32_t halfFiltLength,int32_t overSampling, float cutOffFrequ, float kaiserBeta){

    const int32_t noSamples=halfFiltLength*overSampling+1;
    setKaiserWindow(kaiserBeta, noSamples);  
    
    float* filterCoeff=filter;
    *filterCoeff++=cutOffFrequ;
    double step=halfFiltLength/(noSamples-1.);
    double xPos=step;
    double factor=M_PI*cutOffFrequ;
    for (int32_t i = 0; i<noSamples-1; i++ ){
        *filterCoeff++*=(float)((sin(xPos*factor)/(xPos*M_PI)));        
        xPos+=step;
    } 
}

double Resampler::getStep() const {
    return _step;
}

void Resampler::configure(float fs, float newFs){
    if (fs<=0. || newFs <=0.){
        _initialized=false;
        return;
    }
    _step=(double)fs/newFs;
    _stepAdaptedOld=_step;
    _stepTransissionCounter =AUDIO_BLOCK_SAMPLES;
    _amplitude=0;
    _oldCorrection=0;
    _oldDiff=0;
    float* b =_buffer0;
    for (uint8_t i =0; i< MAX_HALF_FILTER_LENGTH * 2; i++){
        *b=0.;
        ++b;
    }

    float cutOffFrequ, kaiserBeta;
    const int32_t minHalfLength=20;
    _overSamplingFactor=1024;
    if (fs <= newFs){
        cutOffFrequ=1.;
        kaiserBeta=10;
        _halfFilterLength=minHalfLength;
    }
    else{
        cutOffFrequ=newFs/fs;
        double b=2.*(0.5*newFs-20000)/fs;   //this transition band width causes aliasing. However the generated frequencies are above 20kHz
        double attenuation;
#ifdef DEBUG_RESAMPLER
        Serial.print("b: ");
        Serial.println(b);
#endif
        attenuation=100;    //100db 
        double hfl=(int32_t)((attenuation-8)/(2.*2.285*TWO_PI*b)+0.5);
        if (hfl >= minHalfLength && hfl <= MAX_HALF_FILTER_LENGTH){
            _halfFilterLength=hfl;
#ifdef DEBUG_RESAMPLER
            Serial.print("Attenuation: ");
#endif
        }
        else if (hfl < minHalfLength){
            _halfFilterLength=minHalfLength;
            attenuation=((2*_halfFilterLength+1)-1)*(2.285*TWO_PI*b)+8;            
#ifdef DEBUG_RESAMPLER
            Serial.println("Resmapler: sinc filter length increased");
            Serial.print("Attenuation increased to ");
#endif
        }
        else{
            _halfFilterLength=MAX_HALF_FILTER_LENGTH;
            attenuation=((2*_halfFilterLength+1)-1)*(2.285*TWO_PI*b)+8;
#ifdef DEBUG_RESAMPLER
            Serial.println("Resmapler: needed sinc filter length too long");
            Serial.print("Attenuation decreased to ");
#endif
        }
#ifdef DEBUG_RESAMPLER
        Serial.print(attenuation);
        Serial.println("dB");
#endif
        if (attenuation>50.){
            kaiserBeta=0.1102*(attenuation-8.7);
        }
        else if (21<=attenuation && attenuation<=50){
            kaiserBeta=0.5842*(float)pow(attenuation-21.,0.4)+0.07886*(attenuation-21.);
        }
        else{
            kaiserBeta=0.;
        }
        int32_t noSamples=_halfFilterLength*_overSamplingFactor+1;
        if (noSamples > MAX_FILTER_SAMPLES){
            int32_t f = (noSamples-1)/(MAX_FILTER_SAMPLES-1)+1;
            _overSamplingFactor/=f;
        }
    }

#ifdef DEBUG_RESAMPLER
    Serial.print("fs: ");
    Serial.println(fs);
    Serial.print("cutOffFrequ: ");
    Serial.println(cutOffFrequ);
    Serial.print("_halfFilterLength: ");
    Serial.println(_halfFilterLength);
    Serial.print("overSampling: ");
    Serial.println(_overSamplingFactor);
    Serial.print("kaiserBeta: ");
    Serial.println(kaiserBeta, 12);
    Serial.print("_step: ");
    Serial.println(_step, 12);
#endif
    setFilter(_halfFilterLength, _overSamplingFactor, cutOffFrequ, kaiserBeta);
    _filterLength=_halfFilterLength*2;
    _endOfBuffer=&_buffer0[_filterLength];
    _cPos=-_halfFilterLength;   //marks the current center position of the filter
    _initialized=true;
}
bool Resampler::initialized() const {
    return _initialized;
}

void Resampler::resample(float* input0, float* input1, uint16_t inputLength, uint16_t& processedLength, float* output0, float* output1,uint16_t outputLength, uint16_t& outputCount) {
    outputCount=0;
    double stepAdapted;
    if(_stepTransissionCounter >= AUDIO_BLOCK_SAMPLES){
        stepAdapted=_stepAdaptedOld+_amplitude;
    }
    int32_t successorIndex=(int32_t)(ceil(_cPos));  //negative number -> currently the _buffer0 of the last iteration is used
    float* ip0, *ip1, *fPtr;
    while (floor(_cPos + _halfFilterLength) < inputLength && outputCount < outputLength){
        float dist=successorIndex-_cPos;
            
        float si0[]={0,0};
        float si1[]={0,0};
        float distScaled=dist*_overSamplingFactor;
        int32_t rightIndex=abs((int32_t)(ceil(distScaled))-_overSamplingFactor*_halfFilterLength);   
        const int32_t indexData=successorIndex-_halfFilterLength;
        if (indexData>=0){
            ip0=input0+indexData;
            ip1=input1+indexData;
        }  
        else {
            ip0=_buffer0+indexData+_filterLength; 
            ip1=_buffer1+indexData+_filterLength; 
        }       
        fPtr=filter+rightIndex;
        if (rightIndex==_overSamplingFactor*_halfFilterLength){
            si1[0]+=*ip0++**fPtr;
            si1[1]+=*ip1++**fPtr;   
            fPtr-=_overSamplingFactor;          
            rightIndex=(int32_t)(ceil(distScaled))+_overSamplingFactor;     //needed below  
        }
        else {
            rightIndex=(int32_t)(ceil(distScaled));     //needed below
        }
        for (uint16_t i =0 ; i<_halfFilterLength; i++){
            if(ip0==_endOfBuffer){
                ip0=input0;
                ip1=input1;
            }
            si1[0]+=*ip0**fPtr;            
            si1[1]+=*ip1**fPtr;
            ++fPtr;
            si0[0]+=*ip0**fPtr;
            si0[1]+=*ip1**fPtr;       
            --fPtr;    
            fPtr-=_overSamplingFactor;      
            ++ip0;
            ++ip1;
        }
        fPtr=filter+rightIndex-1;
        for (uint16_t i =0 ; i<_halfFilterLength; i++){  
            if(ip0==_endOfBuffer){
                ip0=input0;
                ip1=input1;
            }
            si0[0]+=*ip0**fPtr;
            si0[1]+=*ip1**fPtr;
            ++fPtr;
            si1[0]+=*ip0**fPtr;            
            si1[1]+=*ip1**fPtr;
            --fPtr;
            fPtr+=_overSamplingFactor;
            ++ip0;
            ++ip1;
        }
        const float w0=ceil(distScaled)-distScaled;
        const float w1=1.-w0;
        *output0++=si0[0]*w0 + si1[0]*w1;
        *output1++=si0[1]*w0 + si1[1]*w1;

        outputCount++;

        if(_stepTransissionCounter < AUDIO_BLOCK_SAMPLES){
            stepAdapted=_stepAdaptedOld+_amplitude*_stepTransission[_stepTransissionCounter];
            _stepTransissionCounter++;
        }
        _cPos+=stepAdapted;
        while (_cPos >successorIndex){
            successorIndex++;
        }
    }
    if(outputCount < outputLength){
        //ouput vector not full -> we ran out of input samples
        processedLength=inputLength;
    }
    else{
        processedLength=min(inputLength, (int16_t)floor(_cPos + _halfFilterLength));
    }
    //fill _buffer
    const int32_t indexData=processedLength-_filterLength;
    if (indexData>=0){
        ip0=input0+indexData;
        ip1=input1+indexData;
        const unsigned long long bytesToCopy= _filterLength*sizeof(float);
        memcpy((void *)_buffer0, (void *)ip0, bytesToCopy);
        memcpy((void *)_buffer1, (void *)ip1, bytesToCopy);    
    }  
    else {
        float* b0=_buffer0;
        float* b1=_buffer1;
        ip0=_buffer0+indexData+_filterLength; 
        ip1=_buffer1+indexData+_filterLength;
        for (uint16_t i =0; i< _filterLength; i++){
            if(ip0==_endOfBuffer){
                ip0=input0;
                ip1=input1;
            }        
            *b0++ = *ip0++;
            *b1++ = *ip1++;
        } 
    }
    _cPos-=processedLength;
    if (_cPos < -_halfFilterLength){
        _cPos=-_halfFilterLength;
    }
}

void Resampler::fixStep(){
    _step=_stepAdaptedOld;
    _oldCorrection=0;
    _oldDiff=0;
}

bool Resampler::addToSampleDiff(double diff){
    double correction=diff*_pControlParam;
    if (abs(diff) > 1e-5){
        //increase correction if the error is still quite large
        correction = diff > 0 ? correction*diff/1e-5 : (-1.)*correction*diff/1e-5;
    }

    _amplitude=correction-_oldCorrection;
    bool settled=false;
    if (abs(_oldDiff-diff)*_pControlParam/(_stepAdaptedOld*AUDIO_SAMPLE_RATE_EXACT) <_settledThrs){
        _step+=_oldCorrection;
        _oldCorrection=0.;
        settled=true;

// #ifdef DEBUG_RESAMPLER
//         Serial.println(_amplitude/(_stepAdaptedOld*AUDIO_SAMPLE_RATE_EXACT)*1e12);
// #endif
    }

#ifdef DEBUG_RESAMPLER
    if (abs(_amplitude)/(_stepAdaptedOld*AUDIO_SAMPLE_RATE_EXACT)*1e12 >1.){
        Serial.println("jitter");
    }
#endif
    _oldDiff=diff;
    _stepTransissionCounter=0;
    _stepAdaptedOld=_step+_oldCorrection;
    _oldCorrection=correction;
    return settled;
}

double Resampler::getXPos() const{
    return _cPos+(double)_halfFilterLength;
}