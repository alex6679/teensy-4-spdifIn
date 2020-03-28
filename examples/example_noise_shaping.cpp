#include "Quantizer.h"
#include "Arduino.h"

void setup() {
  while(!Serial){}
  float fs=44100;
  Quantizer quantizer(fs);

  const bool noiseShaping=true;
  const bool dither=true;

  float factor = powf(2, 15)-1.f; // to 16 bit audio
  quantizer.configure(noiseShaping, dither, factor);
  double frequ=4000;
  double samplesPerPeriod=44100/frequ;
  uint16_t noSamples=(uint16_t)round(200*samplesPerPeriod);
  float samples[noSamples];


  for (uint16_t i=0; i< noSamples; i++){
    samples[i]=sin(i*frequ/fs*TWO_PI)*0.9;
  }
  int16_t outputSamples[noSamples];
  uint32_t t0=micros();
  quantizer.quantize(samples, outputSamples, noSamples);
  uint32_t t1=micros();
  Serial.print("ellapsed time: ");
  Serial.println(t1-t0);

  for (uint32_t i =0 ; i< noSamples; i++){
      //Serial.print(outputSamples[2*i+1]);     
      Serial.print(outputSamples[i]);     
      Serial.println(",");
      delayMicroseconds(50);
  }
}
void loop() {

}
