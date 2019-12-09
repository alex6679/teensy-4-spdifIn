
#include "output_spdif3.h"
#include "input_spdif.h"
#include <Audio.h>
#include <SerialFlash.h>



AudioOutputSPDIF3   spdifOut;
AudioInputSPDIF     spdifIn;
//

AudioConnection          patchCord1(spdifIn, 0, spdifOut, 0);
AudioConnection          patchCord2(spdifIn, 1, spdifOut, 1);


void setup() {

  // put your setup code here, to run once:
  AudioMemory(12);

}


void loop() {

}
