
#include "output_spdif3.h"
//#define DEBUG_RESAMPLER	//activates more debug output
//#define DEBUG_SPDIF_IN
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
  while (!Serial);

}


void loop() {
	double bufferedTine=spdifIn.getBufferedTime();
	Serial.print("buffered time [micro seconds]: ");
	Serial.println(bufferedTine*1e6,8);
	
	double pUsageIn=spdifIn.processorUsage(); 
	Serial.print("processor usage [%]: ");
	Serial.println(pUsageIn);

	delay(250);
}
