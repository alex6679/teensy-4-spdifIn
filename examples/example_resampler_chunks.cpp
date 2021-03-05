#include "Resampler.h"
#include "Arduino.h"

void setup() {
  while(!Serial){}
  
  double fs=48000;
  double newFs=44100;
  Resampler resampler;
  resampler.configure(fs, newFs);


  const uint8_t noChannels=2;
  uint16_t noInputSamples=128;
  float inputSamples[noChannels][noInputSamples];
  uint16_t usedInputSamples = noInputSamples; //the buffer should be filled at the first iteration

  
  uint16_t noOutputSamples=128;	// it's not necessary that the input and output buffer have the same length.
  float outputSamples[noChannels][noOutputSamples];
  uint16_t computedOutputSamples = 0;

  // parameters or the resampled signal
  double frequ=4000;
  uint32_t x = 0;
  //===================================

  for (int32_t i =0; i< 20; i++){
    //filling the input buffer if necessary =============
    if (usedInputSamples == noInputSamples){
      // all inpur samples are processed -> fill the input buffer
        for (uint32_t i=0; i< noInputSamples; i++){
          inputSamples[0][i]=sin(x*frequ/fs*TWO_PI);
          x++;
          for (uint8_t j=1; j< noChannels; j++){
            inputSamples[j][i]=inputSamples[0][i];
          }
        }
        usedInputSamples = 0;
    }
    //====================================================

    //resampling =========================================
    uint16_t usedInputSamplesCurrentIteration;
    uint16_t computedOutputSamplesCurrentIteration;
	//This function changes the inner state of the resampler.
	//The exact position (somerwhere between the samples of the incoming data) of the last interpolated sample is stored, so that the
	//sequence of interpolated samples can be correctly extended at the next call.
    resampler.resample(
        inputSamples[0] + usedInputSamples,
        inputSamples[1] + usedInputSamples,
        noInputSamples - usedInputSamples,
        usedInputSamplesCurrentIteration,
        outputSamples[0] + computedOutputSamples,
        outputSamples[1] + computedOutputSamples,
        noOutputSamples - computedOutputSamples,
        computedOutputSamplesCurrentIteration);
    usedInputSamples+=usedInputSamplesCurrentIteration;
    computedOutputSamples+=computedOutputSamplesCurrentIteration;
    //=====================================================

    //clearing the output buffer ==========================
    if (computedOutputSamples == noOutputSamples){
      //output buffer is full -> send/ clear ... the buffer
        for (uint16_t i=0; i< computedOutputSamples; i++){
            Serial.print(outputSamples[0][i],10);
            Serial.println(",");
        }
        computedOutputSamples = 0;
    }
    //======================================================

  }

}


void loop() {
}
