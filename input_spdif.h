
#ifndef input_spdif_nativ_h_
#define input_spdif_nativ_h_
#include "Resampler.h"
#include "Quantizer.h"
#include "FrequencyMeasurement.h"
#include "Arduino.h"
#include "AudioStream.h"
#include "DMAChannel.h"
#include "BiQuad.h"

//#define DEBUG_SPDIF_IN

class AudioInputSPDIF : public AudioStream
{
public:
	AudioInputSPDIF(void) : AudioStream(0, NULL) { begin(); }
	virtual void update(void);
	void begin(void);
	double getBufferedTime() const;
	static double getInputFrequ();
protected:	
	static DMAChannel dma;
	static void isr(void);
	static void resample();
	static void monitorResampleBuffer();
	static void configure();
private:
	static audio_block_t *block_left;
	static audio_block_t *block_right;
	static int32_t block_offset;

	static void allocateBlocks();

	static volatile int32_t buffer_offset;
	static int32_t resample_offset;

	static volatile bool locked;
	static volatile bool lockChanged;
	static volatile bool firstHalfProcessed;
  	static void spdif_interrupt();
	static Resampler resampler;
	static BiQuad bufferLPFilter;
	static Quantizer quantizer[2];

	static volatile double bufferedTime;
	static volatile uint32_t inputFrequOld;
	static double inputFrequency;
	static double targetLatencyS;	//target latency [seconds]
	static int32_t maxLatency;
	void config_spdifIn();

#ifdef DEBUG_SPDIF_IN
	static volatile bool bufferOverflow;
#endif
};

#endif
