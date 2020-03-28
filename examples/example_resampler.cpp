#include "Resampler.h"
#include "Arduino.h"

void setup() {
  while(!Serial){}

  double fs=48000;
  double newFs=44100;

  uint16_t noInputSamples=4000;
  uint16_t noOutputSamples=(uint16_t)(noInputSamples*newFs/fs);
  const uint8_t noChannels=2;
  float inputSamples[noChannels][noInputSamples];
  double frequ=4000;//441;

  for (uint16_t i=0; i< noInputSamples; i++){
    inputSamples[0][i]=sin(i*frequ/fs*TWO_PI);
    for (uint8_t j=1; j< noChannels; j++){
      inputSamples[j][i]=inputSamples[0][i];
    }
  }

  Resampler resampler;
  resampler.configure(fs, newFs);
  float outputSamples[noChannels][noOutputSamples];
  uint16_t usedInputSamples;
  uint16_t computedOutputSamples;
  uint32_t t0=micros();
  resampler.resample(
      inputSamples[0],
      inputSamples[1],
      noInputSamples,
      usedInputSamples,
      outputSamples[0],
      outputSamples[1],
      noOutputSamples,
      computedOutputSamples);

  uint32_t t1=micros();
  Serial.print("ellapsed time: ");
  Serial.println(t1-t0);
  //the data from the serial monitor is used in the python script 'eval_example_resampler.py' 
  for (uint32_t i =0 ; i< computedOutputSamples; i++){
    Serial.print(outputSamples[0][i],10);
    Serial.println(",");
  }

}


void loop() {
}
